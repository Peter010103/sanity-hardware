"""Local CTBR policy runner. ROS is confined to the separate TF adapter."""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
from udp_broadcaster.ctbr_protocol import CtbrMapping
from sanity_zenoh.client import SanitySender
from sanity_zenoh.mocap import state_age


def default_socket():
    return str(Path(os.environ.get("XDG_RUNTIME_DIR", f"/tmp/sanity-{os.getuid()}")) / "sanity01/state.sock")


class StateInbox:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        stat = self.path.parent.stat()
        if stat.st_uid != os.getuid() or stat.st_mode & 0o077:
            raise ValueError("State socket needs a private directory owned by this user (mode 700)")
        self.lock_file = open(str(self.path) + ".lock", "a")
        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.lock_file.close()
            raise ValueError("Another local runner owns this state socket")
        self.path.unlink(missing_ok=True)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.bind(str(self.path))
        os.chmod(self.path, 0o600)
        self.socket.settimeout(.05)
        self.packet = None
        self.last_stamp = 0
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.receive, name="mocap_state", daemon=True)
        self.thread.start()

    def receive(self):
        while not self.stopping.is_set():
            try:
                payload = self.socket.recv(4096)
                packet = json.loads(payload)
                if not isinstance(packet, dict):
                    raise ValueError("State must be an object")
                if packet.get("ready") is True:
                    state_age(packet, time.monotonic_ns())
                    stamp = int(packet["stamp_ns"])
                    if stamp <= self.last_stamp:
                        continue
                    self.last_stamp = stamp
                with self.lock:
                    self.packet = packet
            except socket.timeout:
                pass
            except (ValueError, TypeError, KeyError, OverflowError) as error:
                with self.lock:
                    self.packet = {"ready": False, "error": str(error)}

    def snapshot(self, tracked_frame="sanity09"):
        with self.lock:
            packet = dict(self.packet or {})
        state, age = state_age(packet, time.monotonic_ns())
        if packet.get("tracked_frame") != tracked_frame:
            raise ValueError("Unexpected tracked rigid body")
        return state, age, packet

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=1)
        self.socket.close()
        self.path.unlink(missing_ok=True)
        self.lock_file.close()


class ActorProcess:
    def __init__(self, python, source, checkpoint, log, realtime=False, residual_model=None, residual_source=None, controller="policy", mpc_model=None):
        self.log = open(log, "w")
        environment = {**os.environ, "JAX_PLATFORMS": "cpu", "XLA_PYTHON_CLIENT_PREALLOCATE": "false",
                       "OMP_NUM_THREADS": "2", "OPENBLAS_NUM_THREADS": "2", "MKL_NUM_THREADS": "2"}
        command = [str(python), "-u", str(source)]
        if realtime:
            command = [str(python), "-u", str(Path(__file__).with_name("policy_process.py")), "--worker-source", str(source)]
        if residual_model:
            if not realtime or not residual_source:
                raise ValueError("Residual requires the realtime worker and frozen source")
            command += ["--residual-model",str(residual_model),"--residual-source",str(residual_source)]
        if controller == "mpc":
            if residual_model or mpc_model is None or not realtime:
                raise ValueError("MPC requires its XML and realtime process, without a residual policy")
            environment.update(OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
            command = [str(python), "-u", str(Path(__file__).with_name("mpc_process.py")),
                "--worker-source", str(source), "--model", str(mpc_model), "--config", str(checkpoint)]
        elif controller == "policy":
            command += ["--checkpoint", str(checkpoint)]
        else:
            raise ValueError("Expected policy or mpc controller")
        self.process = subprocess.Popen(command,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log, env=environment)
        self.buffer = b""
        try:
            self.info = self.read(time.monotonic() + 180)
        except BaseException:
            self.close()
            raise

    def read(self, deadline):
        while b"\n" not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                raise TimeoutError("Local policy deadline missed; output must stop")
            block = os.read(self.process.stdout.fileno(), 65536)
            if not block:
                raise RuntimeError("Policy worker exited; see policy_worker.log")
            self.buffer += block
            if len(self.buffer) > 1024 * 1024:
                raise ValueError("Oversized policy reply")
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line)

    def call(self, op, deadline, **values):
        self.process.stdin.write((json.dumps({"op": op, **values}, allow_nan=False) + "\n").encode())
        self.process.stdin.flush()
        return self.read(deadline)

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.process.stdin.close()
        self.process.stdout.close()
        self.log.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--loco-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--policy-python", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory for log and run receipt")
    parser.add_argument("--state-socket", default=default_socket())
    parser.add_argument("--tracked-frame", default="sanity09")
    parser.add_argument("--drone-id", type=int, default=9)
    parser.add_argument("--listen", default="tcp/10.0.0.200:7447")
    parser.add_argument("--connect", help="Use an existing Zenoh router instead of a local listener")
    parser.add_argument("--no-zenoh", action="store_true", help="Offline shadow; no network commands")
    parser.add_argument("--execute", action="store_true", help="Explicitly enable physical output; default is disabled shadow")
    parser.add_argument("--ctbr-map", type=Path, help="Measured collective RC table and verified FC rate mapping, JSON")
    parser.add_argument("--mode", choices=["pd", "policy", "hybrid"], default="hybrid")
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--height", type=float, default=1., help="Climb above initial base_link height, metres")
    parser.add_argument("--climb-time", type=float, default=10.)
    args = parser.parse_args(argv)
    if not np.isfinite([args.duration, args.height, args.climb_time]).all() or not 0 < args.duration <= 120 or not 0 < args.height <= 2 or args.climb_time < 5:
        parser.error("Require finite duration (0,120], height (0,2] and climb time >=5 s")
    if args.execute and (args.no_zenoh or not args.ctbr_map):
        parser.error("Execution requires Zenoh and a measured --ctbr-map")
    source = args.loco_root.resolve() / "experiments/sanity_ctbr_sim2sim"
    for path in (source / "policy_worker.py", source / "takeoff_controller.py", args.checkpoint, args.policy_python):
        if not path.is_file():
            parser.error(f"Missing file: {path}")
    params = json.loads(args.ctbr_map.read_text()) if args.ctbr_map else {}
    mapping = CtbrMapping(**{k: tuple(v) if isinstance(v, list) else v for k, v in params.items()})
    args.output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(source))
    from takeoff_controller import PositionPD, TakeoffReference, TakeoffSupervisor
    inbox = sender = actor = None
    error = None
    rows = 0
    receipt = {"execute": args.execute, "zenoh": not args.no_zenoh, "mode": args.mode,
               "tracked_frame": args.tracked_frame, "drone_id": args.drone_id,
               "arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
               "end_behavior": "STOP/disarm, not an autonomous landing"}
    try:
        inbox = StateInbox(args.state_socket)
        if not args.no_zenoh:
            sender = SanitySender(device=args.drone_id, listen=args.listen, connect=args.connect, ctbr_mapping=mapping)
        print(json.dumps({"state": "loading_policy", "zenoh_endpoint": (args.connect or args.listen) if sender else None}), flush=True)
        actor = ActorProcess(args.policy_python.absolute(), source / "policy_worker.py", args.checkpoint.resolve(), args.output / "policy_worker.log", realtime=True)
        info = actor.info
        receipt["policy"] = info
        dt = float(info["policy_dt_s"])
        if not math.isclose(dt, .02):
            raise ValueError("This live runner requires the trained 50 Hz history contract")
        bounds = np.asarray(info["ctrlrange"])
        if args.execute:
            if not mapping.thrust_newtons or mapping.thrust_newtons[-1] < bounds[0, 1] or mapping.max_thrust_newtons < bounds[0, 1]:
                raise ValueError("Measured collective table must cover the policy thrust range")
            if np.any(np.asarray(mapping.model_rate_limits_rad_s) < np.max(abs(bounds[1:]), axis=1)):
                raise ValueError("CTBR mapping must cover model body-rate limits")
        print(json.dumps({"state": "waiting_for_mocap_and_receiver", "tracked_frame": args.tracked_frame}), flush=True)
        deadline = time.monotonic() + 20
        while True:
            if sender:
                sender.ctbr(False, 0., (0., 0., 0.))
            try:
                state, age, packet = inbox.snapshot(args.tracked_frame)
                if args.execute and packet.get("frames_confirmed") is not True:
                    raise RuntimeError("Confirm measured marker/base and map/world alignment in the TF adapter before execution")
                status = sender.receiver_status() if sender else None
                if not sender or (status and status.session == sender.session_id and not status.enabled and not status.failsafe):
                    break
            except ValueError:
                pass
            if time.monotonic() >= deadline:
                raise TimeoutError("No fresh selected mocap and/or disabled Zenoh receiver acknowledgement")
            time.sleep(.02)
        rotation = Rotation.from_quat(np.r_[state[4:7], state[3]])
        yaw = rotation.as_euler("xyz")[2]
        if args.execute and (np.linalg.norm(state[7:10]) > .15 or rotation.as_matrix()[2, 2] < math.cos(math.radians(10))):
            raise ValueError("Takeoff requires a stationary, approximately level aircraft")
        initial = state[:3].copy()
        reference = TakeoffReference(initial[2], initial[2] + args.height, climb_s=args.climb_time)
        pd = PositionPD(info["mass_kg"])
        supervisor = TakeoffSupervisor(args.mode, reference, bounds, info["history_length_steps"], dt=dt)
        imu_offset = np.asarray(info["imu_position"])
        reference_offset = Rotation.from_euler("z", yaw).apply(imu_offset)
        # The model envelope bottom is used only for the handover clearance gate.
        vertices = np.array([[float(v) for v in line.split()[1:4]] for line in
            (args.loco_root / "loco_mjx/environments/robots/sanity_ctbr/data/assets/meshes/collision/envelope.obj").read_text().splitlines() if line.startswith("v ")])
        floor_height = initial[2] + rotation.apply(vertices)[:, 2].min()
        actor.call("reset", deadline=time.monotonic() + 1)
        if args.execute:
            deadline = time.monotonic() + 3
            while True:
                inbox.snapshot(args.tracked_frame)
                sender.ctbr(True, 0., (0., 0., 0.))
                status = sender.receiver_status()
                if status and status.session == sender.session_id and status.enabled and not status.failsafe:
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError("Zero-thrust enable handshake failed")
                time.sleep(.02)
        started = next_tick = time.monotonic()
        previous_tick = started
        print(json.dumps({"state": "running", "execute": args.execute, "zenoh": bool(sender), "tracked_frame": args.tracked_frame}), flush=True)
        with (args.output / "control.jsonl").open("w", buffering=1) as log:
            while time.monotonic() - started < args.duration:
                tick = time.monotonic()
                if tick - previous_tick > .04:
                    raise TimeoutError("Control loop stalled beyond 40 ms")
                previous_tick = tick
                state, age, packet = inbox.snapshot(args.tracked_frame)
                if args.execute and packet.get("frames_confirmed") is not True:
                    raise ValueError("Frame calibration confirmation lost")
                step_deadline = tick + min(.04 - age, .04)
                receipt["last_cycle"] = {"time_s": tick - started, "initial_state_age_s": age, "stage": "state"}
                rotation = Rotation.from_quat(np.r_[state[4:7], state[3]])
                offset = rotation.apply(imu_offset)
                position = state[:3] + offset
                velocity = state[7:10] + np.cross(state[10:13], offset)
                t = tick - started
                ref = reference.sample(t)
                ref["position"] = (np.asarray(ref["position"]) + [initial[0], initial[1], 0.] + reference_offset).tolist()
                pd_command = pd.command(position, velocity, rotation, ref, yaw=yaw)
                desired_velocity = np.asarray(ref["velocity"]) + np.array([.75, .75, 1.]) * (np.asarray(ref["position"]) - position)
                desired_velocity[:2] += .5 * (np.asarray(ref["velocity"])[:2] - velocity[:2]) + .15 * np.asarray(ref["acceleration"])[:2]
                desired_velocity[2] = np.clip(desired_velocity[2], -.8, .8)
                desired_velocity *= min(1., .8 / max(np.linalg.norm(desired_velocity), 1e-9))
                yaw_error = math.atan2(math.sin(yaw - rotation.as_euler("xyz")[2]), math.cos(yaw - rotation.as_euler("xyz")[2]))
                goal = [*rotation.inv().apply(desired_velocity), float(np.clip(1.5 * yaw_error, -1., 1.))]
                result = actor.call("infer", deadline=step_deadline, state=state.tolist(), goal=goal, commit=False)
                receipt["last_cycle"].update(stage="inferred", inference_ms=result["inference_ms"], elapsed_ms=(time.monotonic()-tick)*1000)
                clearance = state[2] + rotation.apply(vertices)[:, 2].min() - floor_height
                tilt = float(np.degrees(np.arccos(np.clip(rotation.as_matrix()[2, 2], -1., 1.))))
                command, transition = supervisor.step(t, np.asarray(ref["position"]) - position, velocity, tilt, clearance,
                    pd_command, result["ctbr"], result["history_age"], policy_age_s=time.monotonic() - tick,
                    state_age_s=age + time.monotonic() - tick)
                if tilt > 45 or np.linalg.norm(state[:2] - initial[:2]) > .5 or state[2] > initial[2] + args.height + .5:
                    raise ValueError("Takeoff/hover flight envelope exceeded")
                # A disabled shadow has zero authority: its action history must record zero output.
                sent = command if args.execute else np.zeros(4)
                raw_action = args.execute and transition["policy_weight"] == 1 and not transition["command_limited"] and t >= reference.climb_start
                committed = actor.call("commit", deadline=step_deadline,
                    **({"action": result["action"]} if raw_action else {"ctbr": sent.tolist()}))
                receipt["last_cycle"].update(stage="committed", elapsed_ms=(time.monotonic()-tick)*1000)
                _, dispatch_age = state_age(packet, time.monotonic_ns())
                if time.monotonic() >= step_deadline:
                    raise TimeoutError("Policy/commit exceeded state validity budget")
                # Also detect invalidation/new tracking loss during inference.
                inbox.snapshot(args.tracked_frame)
                if sender:
                    sender.ctbr(args.execute, float(sent[0]), tuple(sent[1:]))
                    status = sender.receiver_status()
                    if status is None or (args.execute and (not status.enabled or status.failsafe or sender.rearm_required)):
                        raise RuntimeError("Zenoh receiver freshness/enable interlock lost; restart explicitly")
                row = {"time_s": t, "stamp_ns": packet["stamp_ns"], "state_age_s": dispatch_age,
                       "state": state.tolist(), "goal": goal, "reference": ref, "pd_ctbr": pd_command.tolist(),
                       "policy_ctbr": result["ctbr"], "selected_ctbr": command.tolist(), "sent_ctbr": sent.tolist(),
                       "inference_ms": result["inference_ms"], "cycle_work_ms": (time.monotonic()-tick)*1000,
                       "history": committed, **transition}
                log.write(json.dumps(row, allow_nan=False) + "\n")
                rows += 1
                next_tick += dt
                if next_tick < time.monotonic():
                    next_tick = time.monotonic()
                time.sleep(max(0., next_tick - time.monotonic()))
    except KeyboardInterrupt:
        error = "Interrupted"
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        for resource in (sender, actor, inbox):
            if resource:
                try:
                    resource.close()
                except Exception as exc:
                    error = (error + "; " if error else "") + f"Cleanup: {exc}"
        receipt.update(error=error, control_steps=rows, finished_wall_time_ns=time.time_ns())
        (args.output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
        print(json.dumps({"error": error, "control_steps": rows, "output": str(args.output)}), flush=True)
    if error and error != "Interrupted":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
