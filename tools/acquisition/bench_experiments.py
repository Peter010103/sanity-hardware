#!/usr/bin/env python3
"""Sanity bench experiments v2: separate RPM, force, and command timestamps.

Default mode prints the plan without opening devices. See BENCH_EXPERIMENTS.txt.
One thread owns HX711; the main thread alone owns the MSP port. No new deps.
"""
import argparse
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import queue
import select
import signal
import statistics
import struct
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "hx711_jetson"))
from betaflight_msp import USBSerial, MSP, motor_telemetry, battery, status, inspect_controller
from betaflight_motors import motor_packet, validate_setup
from hx711_jetson import RAILS, TimingError, load_calibration, force_from_counts
from hx711_native import NativeHX711

VERSION = "2.0.0"
STOP = (1000,) * 4
CEILING = 1450
G = 9.80665
STALE_FORCE_S = .35
STALE_HEALTH_S = .45
HEALTH_PERIOD_S = .20
PRESETS = ("repeatability", "small_steps", "multilevel", "individual", "reference", "weights", "probe")


def mono():
    return time.monotonic_ns()


def positive(value):
    v = float(value)
    if not math.isfinite(v) or v <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return v


def atomic_json(path, data):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, allow_nan=False)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def plan_for(preset, motor=None):
    """Finite, deterministic plans. Unequal dwells avoid fixed sensor phase."""
    out = []
    def add(name, commands, seconds, role="identification", cycle=0):
        if isinstance(commands, int):
            commands = [commands] * 4
        if len(commands) != 4 or any(type(c) is not int or not 1000 <= c <= CEILING for c in commands):
            raise ValueError("Invalid experiment command vector")
        out.append(dict(name=name, commands=list(commands), duration_s=seconds, role=role, cycle=cycle))
    if preset in ("repeatability", "small_steps"):
        low, high, pairs = (1250, 1450, 4) if preset == "repeatability" else (1300, 1400, 3)
        add("warmup", low, 2.5, "warmup")
        dwells = [1.57, 1.83, 1.69, 1.91, 1.76, 1.63, 1.88, 1.72]
        for i in range(pairs):
            role = "validation" if i == pairs - 1 else "identification"
            add("up_%d" % (i+1), high, dwells[2*i], role, i+1)
            add("down_%d" % (i+1), low, dwells[2*i+1], role, i+1)
    elif preset == "multilevel":
        for i, value in enumerate([1350, 1250, 1350, 1450, 1350, 1300, 1350, 1400, 1350]):
            add("level_%02d_%d" % (i+1, value), value, 2.0,
                "validation" if i else "warmup")
    elif preset == "reference":
        add("reference_low_before", 1250, 2.5, "reference")
        add("reference_high", 1450, 2.0, "reference")
        add("reference_low_after", 1250, 2.0, "reference")
    elif preset == "individual":
        motors = [motor] if motor is not None else [1, 2, 3, 4]
        for m in motors:
            for c in [1250, 1350, 1450]:
                vector = list(STOP)
                vector[m-1] = c
                add("M%d_%d" % (m, c), vector, 2.0, "individual_static")
            add("stop_after_M%d" % m, STOP, 2.0, "stopped")
        if motor is None:
            add("M1_reference_repeat", [1350, 1000, 1000, 1000], 2.0, "reference")
    return out


FIELDS = {
    "commands": ["event", "phase", "request_start_ns", "ack_ns", "acknowledged", "M1", "M2", "M3", "M4"],
    "phases": ["phase", "role", "cycle", "command_event", "request_start_ns", "ack_ns", "end_ns", "requested_duration_s", "completed"],
    "rpm": ["request_start_ns", "reply_ns", "unix_time_ns", "last_command_event"] +
        ["rpm_M%d" % i for i in range(1, 5)] + ["omega_reported_rad_s_M%d" % i for i in range(1, 5)] +
        ["erpm_derived_M%d" % i for i in range(1, 5)] + ["invalid_percent_M%d" % i for i in range(1, 5)] +
        ["esc_voltage_V_M%d" % i for i in range(1, 5)] + ["esc_current_A_M%d" % i for i in range(1, 5)] +
        ["esc_temperature_C_M%d" % i for i in range(1, 5)] + ["raw_payload_hex"],
    "force": ["sequence", "read_begin_ns", "ready_observed_ns", "read_complete_ns", "unix_time_ns", "raw_counts",
              "load_N", "baseline_N", "force_delta_N", "thrust_axis_force_N", "max_clock_high_bound_us", "valid"],
    "battery": ["request_start_ns", "reply_ns", "detected_cells", "voltage_V", "current_A", "consumed_mAh", "battery_state_code"],
    "status": ["request_start_ns", "reply_ns", "armed", "raw_payload_hex"],
    "weight_checks": ["cycle", "position", "mass_g", "expected_delta_N", "unloaded_before_N", "loaded_N",
                      "unloaded_after_N", "measured_delta_N", "error_N", "zero_return_shift_N"],
}


class Log:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.files, self.writers, self.counts = {}, {}, {}
        try:
            for name, fields in FIELDS.items():
                f = open(self.directory / (name + ".csv"), "x", newline="", encoding="utf-8")
                self.files[name] = f
                w = csv.DictWriter(f, fieldnames=fields)
                w.writeheader()
                self.writers[name] = w
                self.counts[name] = 0
            self.events_file = open(self.directory / "events.jsonl", "x", encoding="utf-8")
        except BaseException:
            self.close()
            raise

    def row(self, name, data):
        self.writers[name].writerow(data)
        self.files[name].flush()  # no fsync in the acquisition loop
        self.counts[name] += 1

    def event(self, kind, **details):
        self.events_file.write(json.dumps(dict(kind=kind, monotonic_ns=mono(), unix_time_ns=time.time_ns(), **details)) + "\n")
        self.events_file.flush()

    def close(self):
        for f in list(getattr(self, "files", {}).values()) + [getattr(self, "events_file", None)]:
            if f is not None:
                f.close()


class ForceReader(threading.Thread):
    """The worker owns the sensor for its entire lifetime, including cleanup."""
    def __init__(self, factory=None):
        super().__init__(daemon=True, name="hx711-acquisition")
        self.factory = factory or (lambda: NativeHX711(timeout=.25, timing_error=TimingError))
        self.samples = queue.Queue(maxsize=256)
        self.cancel = threading.Event()
        self.error = None

    def run(self):
        sensor = None
        try:
            sensor = self.factory()
            while not self.cancel.is_set():
                began = mono()
                row = sensor.read()
                row["read_begin_ns"] = began
                self.samples.put_nowait(row)
        except BaseException as exc:
            self.error = "%s: %s" % (type(exc).__name__, exc)
        finally:
            if sensor is not None:
                try:
                    sensor.close()
                except BaseException as exc:
                    self.error = self.error or "sensor close: %s" % exc


class Bench:
    def __init__(self, args, log, reader, calibration, client=None, live=None, box_ids=None):
        self.args, self.log, self.reader, self.calibration = args, log, reader, calibration
        self.client, self.live, self.box_ids = client, live, box_ids
        self.powered = args.preset not in ("weights", "probe")
        self.baseline = None
        self.force_history = []
        self.latest_force = None
        self.force_time = None
        self.latest_rpm = None
        self.latest_battery = None
        self.latest_status = None
        self.battery_time = self.status_time = 0
        self.next_battery = self.next_status = self.next_rpm = 0
        self.next_print = 0
        self.may_have_sent = False
        self.command_event = 0
        self.vector = STOP
        self.command_ack_ns = 0
        self.stop_attempts = self.stop_acks = 0
        self.state = "initializing"
        self.error = None
        self.baseline_sd = None
        self.final_baseline = None
        self.force_range = calibration.get("measured_force_range_N") or calibration.get("fit", {}).get("measured_force_range_N")

    def drain_force(self):
        while True:
            try:
                raw = self.reader.samples.get_nowait()
            except queue.Empty:
                break
            counts = raw["raw_counts"]
            force = force_from_counts(counts, self.calibration)
            good = counts not in RAILS and math.isfinite(force) and self.force_range[0] <= force <= self.force_range[1]
            delta = None if self.baseline is None else force - self.baseline
            self.log.row("force", dict(sequence=self.log.counts["force"], read_begin_ns=raw["read_begin_ns"],
                ready_observed_ns=raw["ready_observed_monotonic_ns"], read_complete_ns=raw["read_complete_monotonic_ns"],
                unix_time_ns=raw["unix_time_ns"], raw_counts=counts, load_N=force, baseline_N=self.baseline,
                force_delta_N=delta, thrust_axis_force_N=None if delta is None else self.args.thrust_sign*delta,
                max_clock_high_bound_us=raw["max_clock_high_bound_us"], valid=good))
            self.latest_force = force
            self.force_time = raw["read_complete_monotonic_ns"]
            self.force_history.append((self.force_time, force))
            if not good:
                raise RuntimeError("HX711 at rail or outside measured calibration force range")
            if delta is not None and abs(delta) > self.args.force_limit_n:
                raise RuntimeError("Force delta %.3f N exceeds %.3f N cutoff" % (delta, self.args.force_limit_n))
        if self.reader.error is not None:
            raise RuntimeError("HX711 worker failed: " + self.reader.error)

    def force_guard(self):
        self.drain_force()
        if self.force_time is None or (mono() - self.force_time)/1e9 > STALE_FORCE_S:
            raise RuntimeError("HX711 data missing/stale; stopping rather than running without force monitoring")

    def query(self, number):
        start = mono()
        try:
            payload = self.client.query(number)
        except BaseException as exc:
            self.log.event("query_failure", command=number, request_start_ns=start, error=str(exc))
            raise
        return start, mono(), payload

    def poll_battery(self):
        a, b, payload = self.query(130)
        data = battery(payload)
        self.latest_battery, self.battery_time = data, b
        self.log.row("battery", dict(request_start_ns=a, reply_ns=b,
            **{k:data[k] for k in FIELDS["battery"] if k not in ("request_start_ns", "reply_ns")}))
        self.next_battery = b + int(HEALTH_PERIOD_S*1e9)
        if self.powered and (data["detected_cells"] not in (2, 3) or data["battery_state_code"] != 0):
            raise RuntimeError("Battery not OK: %s" % data)

    def poll_status(self):
        a, b, payload = self.query(101)
        data = status(payload, self.box_ids)
        self.latest_status, self.status_time = data, b
        self.log.row("status", dict(request_start_ns=a, reply_ns=b, armed=data["armed"], raw_payload_hex=data["raw_payload_hex"]))
        self.next_status = b + int(HEALTH_PERIOD_S*1e9)
        if data["armed"] is not False:
            raise RuntimeError("FC is armed or armed state is unknown")

    def poll_rpm(self):
        a, b, payload = self.query(139)
        data = motor_telemetry(payload, 4)
        self.latest_rpm = data
        row = dict(request_start_ns=a, reply_ns=b, unix_time_ns=time.time_ns(),
                   last_command_event=self.command_event, raw_payload_hex=data["raw_payload_hex"])
        poles = self.live["motor_config"]["configured_motor_poles"]
        for i, m in enumerate(data["motors"], 1):
            row["rpm_M%d" % i] = m["rpm"]
            row["omega_reported_rad_s_M%d" % i] = m["rpm"] * 2*math.pi/60
            row["erpm_derived_M%d" % i] = m["rpm"] * poles/2
            row["invalid_percent_M%d" % i] = m["telemetry_invalid_percent"]
            for key in ("esc_voltage_V", "esc_current_A", "esc_temperature_C"):
                row[key+"_M%d" % i] = m[key]
        self.log.row("rpm", row)
        self.next_rpm = max(a + int(1e9/self.args.rpm_hz), b)
        if self.powered and self.command_ack_ns and b-self.command_ack_ns > 1_000_000_000:
            missing = [i for i, (command, motor) in enumerate(zip(self.vector, data["motors"]), 1)
                       if command > 1000 and motor["rpm"] <= 0]
            if missing:
                raise RuntimeError("No reported RPM after one second on commanded motors %s" % missing)
        if self.args.preset == "probe" and any(m["rpm"] != 0 for m in data["motors"]):
            raise RuntimeError("Read-only probe observed nonzero RPM; leaving outputs untouched")

    def pump(self, display=True):
        self.force_guard()
        if self.client is not None:
            now = mono()
            if now >= self.next_battery:
                self.poll_battery()
            elif now >= self.next_status:
                self.poll_status()
            elif now >= self.next_rpm:
                self.poll_rpm()
            self.force_guard()
            if self.powered and max(mono()-self.battery_time, mono()-self.status_time)/1e9 > STALE_HEALTH_S:
                raise RuntimeError("Battery/status monitoring stale")
        if display and mono() >= self.next_print:
            f = self.latest_force
            delta = None if self.baseline is None else self.args.thrust_sign*(f-self.baseline)
            batt = self.latest_battery
            rpm = None if self.latest_rpm is None else [m["rpm"] for m in self.latest_rpm["motors"]]
            print("%s: load=%.3f N delta=%s RPM=%s battery=%s" % (
                self.state, f, "n/a" if delta is None else "%.3f N" % delta, rpm,
                "n/a" if batt is None else "%.2f V / %.2f A" % (batt["voltage_V"], batt["current_A"])), flush=True)
            self.next_print = mono() + 500_000_000
        time.sleep(.001)

    def healthy_before_command(self):
        self.force_guard()
        self.poll_status()
        self.poll_battery()
        self.force_guard()

    def send(self, vector, phase):
        if not self.powered:
            raise RuntimeError("Motor command prohibited in read-only/weight mode")
        vector = tuple(vector)
        packet = motor_packet(list(vector), 4, 1000, 2000, CEILING)
        if vector != STOP:
            if self.baseline is None:
                raise RuntimeError("No stopped force baseline")
            self.healthy_before_command()
            if self.args.preset == "individual" and self.vector == STOP:
                self.poll_rpm()
                if any(m["rpm"] for m in self.latest_rpm["motors"]):
                    raise RuntimeError("RPM did not return to zero before isolated-motor test")
        self.command_event += 1
        self.may_have_sent = True  # an absent ACK does not mean an unaccepted command
        a = mono()
        b, acknowledged = None, False
        try:
            self.client._exchange(packet, 214)
            b, acknowledged = mono(), True
            self.vector = vector
            self.command_ack_ns = b
        finally:
            self.log.row("commands", dict(event=self.command_event, phase=phase, request_start_ns=a,
                ack_ns=b, acknowledged=acknowledged, **dict(zip(("M1", "M2", "M3", "M4"), vector))))
        return self.command_event, a, b

    def stop(self):
        """Stop attempts remain independent of file logging, sensor state and health."""
        if not self.powered or not self.may_have_sent:
            return True
        packet = motor_packet(list(STOP), 4, 1000, 2000, CEILING)
        got_ack = False
        for _ in range(3):
            self.stop_attempts += 1
            a = mono()
            b = None
            error = None
            try:
                self.client._exchange(packet, 214)
                b = mono()
                self.stop_acks += 1
                got_ack = True
                self.vector = STOP
            except BaseException as exc:
                error = "%s: %s" % (type(exc).__name__, exc)
                print("STOP attempt failed: " + error, file=sys.stderr, flush=True)
            try:
                self.log.event("cleanup_stop", request_start_ns=a, ack_ns=b, acknowledged=b is not None, error=error)
            except BaseException:
                pass
        return got_ack

    def hold(self, spec):
        self.state = spec["name"]
        if self.powered:
            event, a, b = self.send(spec["commands"], self.state)
        else:
            event, a, b = 0, mono(), mono()
        completed = False
        try:
            # First RPM sample immediately after command ACK, before slow safety polling.
            if self.client is not None:
                self.poll_rpm()
            end = b + int(spec["duration_s"]*1e9)
            while mono() < end:
                self.pump()
            completed = True
        finally:
            self.log.row("phases", dict(phase=self.state, role=spec["role"], cycle=spec.get("cycle", 0),
                command_event=event, request_start_ns=a, ack_ns=b, end_ns=mono(),
                requested_duration_s=spec["duration_s"], completed=completed))

    def recent_force_mean(self, seconds=1.):
        end = mono()
        values = [f for t, f in self.force_history if end-int(seconds*1e9) <= t <= end]
        if len(values) < 6:
            raise RuntimeError("Too few force samples for stopped/settled baseline")
        return statistics.mean(values), statistics.stdev(values)

    def initialize_acquisition(self):
        self.reader.start()
        deadline = mono() + 3_000_000_000
        while self.force_time is None and mono() < deadline:
            self.drain_force()
            time.sleep(.005)
        self.force_guard()
        if self.client is not None:
            self.poll_status()
            self.poll_battery()
            self.poll_rpm()
            outputs = struct.unpack("<8H", self.client.query(104))
            if tuple(outputs[:4]) != STOP or any(m["rpm"] for m in self.latest_rpm["motors"]):
                raise RuntimeError("Start requires observed stopped motor outputs and zero RPM")
            # Stagger status and battery deadlines to reduce RPM gaps.
            self.next_status = mono() + 100_000_000
            self.next_battery = mono() + 200_000_000

    def run_plan(self, plan):
        self.hold(dict(name="stopped_baseline", commands=STOP, duration_s=2.5, role="stopped"))
        self.baseline, self.baseline_sd = self.recent_force_mean()
        self.log.event("baseline_established", mean_N=self.baseline, sd_N=self.baseline_sd)
        print("Stopped baseline %.5f N (SD %.5f N)" % (self.baseline, self.baseline_sd), flush=True)
        for spec in plan:
            self.hold(spec)
        self.hold(dict(name="stopped_after", commands=STOP, duration_s=2.5, role="stopped"))
        self.final_baseline, _ = self.recent_force_mean()

    def prompt(self, message):
        print(message, flush=True)
        self.log.event("operator_prompt", message=message)
        deadline = mono() + 180_000_000_000
        while mono() < deadline:
            self.pump(display=False)
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line or line.strip().lower() == "q":
                    raise KeyboardInterrupt
                self.log.event("operator_enter", note="Manual marker, not precise physical force-step time")
                return
        raise RuntimeError("Operator prompt timed out")

    def run_weights(self):
        self.hold(dict(name="weight_unloaded_baseline", duration_s=3., role="weight_unloaded"))
        self.baseline, self.baseline_sd = self.recent_force_mean()
        before = self.baseline
        for cycle in range(1, 4):
            self.state = "weight_adding_%d" % cycle
            self.prompt("Cycle %d/3: add %.1f g at %s, remove your hand, then press Enter (q exits)." %
                        (cycle, self.args.mass_g, self.args.position))
            self.hold(dict(name="weight_loaded_%d" % cycle, duration_s=3., role="weight_loaded", cycle=cycle))
            loaded, _ = self.recent_force_mean()
            self.state = "weight_removing_%d" % cycle
            self.prompt("Remove the reference weight, remove your hand, then press Enter (q exits).")
            self.hold(dict(name="weight_unloaded_%d" % cycle, duration_s=3., role="weight_unloaded", cycle=cycle))
            after, _ = self.recent_force_mean()
            expected = self.args.mass_g/1000*G
            self.log.row("weight_checks", dict(cycle=cycle, position=self.args.position, mass_g=self.args.mass_g,
                expected_delta_N=expected, unloaded_before_N=before, loaded_N=loaded, unloaded_after_N=after,
                measured_delta_N=loaded-before, error_N=loaded-before-expected, zero_return_shift_N=after-before))
            print("Weight change %.4f N; expected %.4f N; zero return %+.4f N" % (loaded-before, expected, after-before), flush=True)
            before = after
        self.final_baseline = before


def parser(preset=None):
    p = argparse.ArgumentParser(description=__doc__)
    if preset is None:
        p.add_argument("preset", choices=PRESETS)
    else:
        p.set_defaults(preset=preset)
    p.add_argument("--port", default="/dev/serial/by-id/usb-Betaflight_Betaflight_STM32F405_0x8000000-if00")
    p.add_argument("--profile", type=Path, default=HERE / "fc_profile.json")
    p.add_argument("--calibration", type=Path, default=HERE / "loadcell_calibration/calibration.json")
    p.add_argument("--output", type=Path)
    p.add_argument("--battery-id", default="unrecorded", help="label the pack used for this run")
    p.add_argument("--note", default="", help="optional pack/configuration note")
    p.add_argument("--force-limit-n", type=positive, default=2.0)
    p.add_argument("--thrust-sign", type=int, choices=(-1, 1), default=1)
    p.add_argument("--rpm-hz", type=positive, default=60., help="requested poll rate; actual achieved rate is logged")
    p.add_argument("--motor", type=int, choices=(1, 2, 3, 4), help="individual preset only; default tests all four")
    p.add_argument("--mass-g", type=positive, default=100., help="weights preset: known reference mass")
    p.add_argument("--position", default="centre", help="weights preset: where reference force is applied")
    p.add_argument("--seconds", type=positive, default=5., help="read-only acquisition probe duration")
    p.add_argument("--execute", action="store_true", help="without this flag only print the plan; no device access")
    p.add_argument("--bench-ready", action="store_true", help="props installed, secured thrust fixture, reachable physical battery cut")
    p.add_argument("--off-axis-checked", action="store_true", help="individual preset: mounting restrains moments and off-centre force response has been checked")
    p.add_argument("--battery-disconnected", action="store_true", help="weights preset: physically disconnected motor battery")
    return p


def preview(args, plan):
    print("%s — %s" % (args.preset, "EXECUTION requested" if args.execute else "PREVIEW ONLY; no devices opened"))
    if args.preset == "weights":
        print("Motor battery disconnected. Three manual add/remove cycles of %.1f g; three-second settled windows." % args.mass_g)
    elif args.preset == "probe":
        print("Read-only FC + HX711 acquisition for %.1f s; no motor commands." % args.seconds)
    else:
        print("Battery label: %s. 2.5 s stopped baseline before and after. Force cutoff %.2f N." % (args.battery_id, args.force_limit_n))
        for spec in plan:
            print("  %-24s %s  %.2f s  [%s]" % (spec["name"], spec["commands"], spec["duration_s"], spec["role"]))
        print("Total planned capture: %.2f s plus preflight/serial overhead." % (5+sum(s["duration_s"] for s in plan)))


def run(args, plan):
    directory = args.output or Path("bench_%s_%s" % (args.preset, time.strftime("%Y%m%d_%H%M%S", time.gmtime())))
    directory.mkdir(parents=True, exist_ok=False)
    log = transport = reader = bench = lock_file = None
    result = 0
    failure = None
    final_status = "failed"
    cleanup_ack = None
    old_handlers = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old_handlers[sig] = signal.signal(sig, interrupted)
        lock_file = open("/tmp/sanity_bench_experiments.lock", "a")
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        calibration = load_calibration(str(args.calibration))
        bounds = calibration.get("measured_force_range_N") or calibration.get("fit", {}).get("measured_force_range_N")
        if (not bounds or len(bounds) != 2 or not all(math.isfinite(v) for v in bounds)
                or not bounds[0] <= 0 < bounds[1] or args.force_limit_n > bounds[1]-bounds[0]):
            raise ValueError("Measured calibration range including zero and adequate force span required")
        log = Log(directory)
        client = live = box_ids = None
        if args.preset != "weights":
            transport = USBSerial(args.port)
            fcntl.flock(transport.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            client = MSP(transport, timeout=.25)
            live = inspect_controller(client)
            profile = json.loads(args.profile.read_text())
            validate_setup(live, profile)
            if not live["motor_config"]["dshot_telemetry_enabled"]:
                raise RuntimeError("These experiments require DShot RPM telemetry")
            box_ids = list(client.query(119))
        reader = ForceReader()
        bench = Bench(args, log, reader, calibration, client, live, box_ids)
        atomic_json(directory/"metadata.json", dict(schema_version=2, logger_version=VERSION, preset=args.preset,
            started_unix_time_ns=time.time_ns(), started_monotonic_ns=mono(), battery_id=args.battery_id,
            operator_note=args.note, argv=sys.argv, python=sys.version, platform=platform.platform(),
            settings=dict(force_limit_N=args.force_limit_n, thrust_sign=args.thrust_sign, rpm_requested_hz=args.rpm_hz,
                          health_period_s=HEALTH_PERIOD_S, force_stale_timeout_s=STALE_FORCE_S, health_stale_timeout_s=STALE_HEALTH_S),
            plan=plan, controller=live, calibration_id=calibration.get("calibration_id"),
            calibration_sha256=hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
            source_sha256={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in
                           ("bench_experiments.py", "betaflight_msp.py", "betaflight_motors.py")},
            units=dict(commands="MSP external motor counts", rpm="FC reported mechanical RPM; physical pole count unverified", force="newtons"),
            timing=dict(clock="CLOCK_MONOTONIC ns; shared across threads", command="host request start / acknowledgement",
                        rpm="request start / reply arrival; FC sample time and age unavailable", force="native ready-observed / read-complete"),
            limitations=["Host timestamps do not separate USB/FC/ESC/motor delays.",
                         "HX711 nominal 10 SPS; force includes sensor/fixture dynamics.",
                         "Unchanging RPM values may reflect quantization or cached data; reply rate alone is not freshness.",
                         "FC voltage/current and physical motor pole count remain independently unverified.",
                         "Zero-valued ESC auxiliary fields do not establish working ESC current/voltage/temperature sensors.",
                         "STOP delivery is best effort; USB/host loss can retain persistent motor-test output."],
            operator_declarations=dict(bench_ready=args.bench_ready, off_axis_checked=args.off_axis_checked,
                                       battery_disconnected=args.battery_disconnected)))
        atomic_json(directory/"calibration_reference.json", calibration)
        bench.initialize_acquisition()
        if args.preset == "weights":
            bench.run_weights()
        elif args.preset == "probe":
            bench.state = "read_only_probe"
            bench.hold(dict(name="read_only_probe", duration_s=args.seconds, role="read_only"))
        else:
            print("Test starts automatically after stopped baseline. Ctrl+C requests STOP; keep battery cut reachable.", flush=True)
            bench.run_plan(plan)
        final_status = "completed"
    except KeyboardInterrupt:
        result, final_status = 130, "interrupted"
        failure = "Operator interrupt or termination signal"
    except BaseException as exc:
        result, failure = 1, "%s: %s" % (type(exc).__name__, exc)
        print("ERROR: " + failure, file=sys.stderr, flush=True)
    finally:
        # Finish stop attempts even if Ctrl+C is pressed again. No sensor/file waits first.
        for sig in old_handlers:
            signal.signal(sig, signal.SIG_IGN)
        if bench is not None and bench.may_have_sent:
            cleanup_ack = bench.stop()
            if not cleanup_ack:
                result, final_status = 1, "stop_not_acknowledged"
                print("STOP NOT ACKNOWLEDGED: disconnect motor battery now.", file=sys.stderr, flush=True)
            else:
                print("FC acknowledged STOP; verify physical rotor stop.", flush=True)
        if reader is not None:
            reader.cancel.set()
            if reader.ident is not None:
                reader.join(timeout=1.5)
                if reader.is_alive():
                    result = 1
                    final_status = "sensor_cleanup_failed"
                    failure = (failure or "") + " HX711 thread did not exit in time"
            if reader.error is not None and final_status == "completed":
                result, final_status = 1, "sensor_cleanup_failed"
                failure = "HX711 worker error during completion: " + reader.error
        try:
            atomic_json(directory/"summary.json", dict(status=final_status, error=failure,
                ended_unix_time_ns=time.time_ns(), rows={} if log is None else log.counts,
                stopped_baseline_N=None if bench is None else bench.baseline,
                stopped_baseline_sd_N=None if bench is None else bench.baseline_sd,
                final_stopped_baseline_N=None if bench is None else bench.final_baseline,
                baseline_return_shift_N=None if bench is None or bench.final_baseline is None or bench.baseline is None else bench.final_baseline-bench.baseline,
                last_battery=None if bench is None else bench.latest_battery,
                last_rpm=None if bench is None or bench.latest_rpm is None else [m["rpm"] for m in bench.latest_rpm["motors"]],
                stop_attempts=0 if bench is None else bench.stop_attempts,
                stop_acknowledgements=0 if bench is None else bench.stop_acks,
                cleanup_stop_acknowledged=cleanup_ack, physical_stop_confirmed=False,
                HX711_worker_error=None if reader is None else reader.error,
                no_motor_commands_sent=bench is None or not bench.may_have_sent))
        except BaseException as exc:
            print("Could not write summary: %s" % exc, file=sys.stderr)
            result = 1
        for resource in (log, transport, lock_file):
            if resource is not None:
                try:
                    resource.close()
                except BaseException as exc:
                    print("Cleanup failed: %s" % exc, file=sys.stderr)
                    result = 1
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        print("Data directory: " + str(directory), flush=True)
    return result


def main(argv=None, preset=None):
    p = parser(preset)
    args = p.parse_args(argv)
    if args.motor is not None and args.preset != "individual":
        p.error("--motor applies only to the individual preset")
    if args.rpm_hz > 100 or args.seconds > 30:
        p.error("Requested RPM poll rate must be <=100 Hz; probe duration <=30 s")
    if args.preset == "weights" and args.mass_g/1000*G >= args.force_limit_n:
        p.error("Reference-weight force must be below the force cutoff")
    plan = plan_for(args.preset, args.motor)
    preview(args, plan)
    if not args.execute:
        return 0
    if args.preset not in ("probe", "weights"):
        if not args.bench_ready:
            p.error("Powered execution requires --bench-ready")
        if args.battery_id == "unrecorded" or not args.battery_id.strip():
            p.error("Provide --battery-id to identify the pack used in this run")
        if args.preset == "individual" and not args.off_axis_checked:
            p.error("Individual-motor tests require --off-axis-checked: fixture must restrain moments and have checked off-centre force response")
    if args.preset == "weights" and not args.battery_disconnected:
        p.error("Weight handling requires --battery-disconnected")
    return run(args, plan)


if __name__ == "__main__":
    sys.exit(main())
