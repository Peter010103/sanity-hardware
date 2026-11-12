"""Sanity15 USB-only, single-setting INDI gain plan/apply/rollback utility.

Default is a read-only configuration plan. Battery must be disconnected.
Never arms, drives motors, flashes firmware, or erases Blackbox data.
"""
from __future__ import annotations

import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import struct
import sys
import time

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sanity_zenoh import fc_blackbox as bb, paper_status

BASELINE = "100,100,80"
CANDIDATE = "150,150,80"
GAIN_KEY = "indi_rate_gains"
REQUIRED = {"paper_input": "1", "paper_backend": "1", "paper_servo": "1",
            "paper_cutoff_hz": "12", "pid_process_denom": "2",
            "indi_enabled": "ON", "force_observer_enabled": "OFF"}


class TuneError(RuntimeError):
    pass


def normalized_commands(text):
    """Preserve command order and profile scope; strip only CLI presentation.

    A dump contains a final 'save' recipe; it is evidence, never replayed.
    """
    commands = []
    for line in text.replace("\r", "").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line in ("dump all", "save"):
            continue
        commands.append(" ".join(line.split()))
    if not commands or commands[0] != "batch start" or commands[-1] != "rateprofile 0":
        raise TuneError("Incomplete/unrecognized dump all; expected batch start and restored rateprofile 0")
    return commands


def setting(commands, name):
    prefix = f"set {name} = "
    found = [line[len(prefix):] for line in commands if line.startswith(prefix)]
    if len(found) != 1:
        raise TuneError(f"Expected exactly one global {name} setting")
    return found[0]


def validate_configuration(commands, expected_gain):
    for key, expected in {**REQUIRED, GAIN_KEY: expected_gain}.items():
        if setting(commands, key) != expected:
            raise TuneError(f"Unexpected {key}: expected {expected}, got {setting(commands, key)}")


def expected_commands(before, target):
    prefix = f"set {GAIN_KEY} = "
    return [prefix + target if line.startswith(prefix) else line for line in before]


def durable_text(path, text):
    with path.open("x") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    bb._sync_dir(path.parent)


class Session:
    def __init__(self, port, factory, timeout, receipt):
        self.port, self.factory, self.timeout = port, factory, timeout
        self.receipt = receipt
        self.transport = None
        self.in_cli = False

    def connect(self):
        self.transport = self.factory(self.port)
        self.receipt["USB_closed"] = False

    def close(self):
        if self.transport is not None:
            self.transport.close()
            self.receipt["USB_closed"] = self.transport.fd is None

    def snapshot(self):
        client = bb.MSP(self.transport, self.timeout)
        snapshot = bb.safe_snapshot(client)
        body = struct.pack("<BHH", 0, paper_status.COMMAND, 0)
        payload = client._exchange(b"$X<" + body + bytes((bb.crc8(body),)), paper_status.COMMAND, 2)
        paper = paper_status.decode(payload)
        if (paper["backend"] != 1 or paper["control_period_us"] != 1000
                or not paper["servo_enabled"] or paper["armed"]):
            raise TuneError("Expected disarmed paper backend 1, servo enabled, nominal 1000us period")
        snapshot["paper"] = paper
        return snapshot

    def _write(self, data):
        self.transport.write(data, time.monotonic() + self.timeout)

    def _response(self, *, reboot=False, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        data = bytearray()
        while True:
            data.extend(self.transport.read(deadline))
            if len(data) > 2_000_000:
                raise TuneError("Oversized CLI response")
            if reboot and b"Rebooting" in data:
                self.in_cli = False
                return data.decode("ascii", errors="strict")
            if not reboot and (data.endswith(b"\n# ") or data.endswith(b"\r# ")):
                text = data.decode("ascii", errors="strict")
                if any(word in text.lower() for word in ("###error", "unknown command", "invalid name", "parse error")):
                    raise TuneError("FC CLI rejected command: " + text[-500:])
                return text

    def enter_cli(self):
        # Mark before writing: even a lost response must trigger an exit attempt.
        self.in_cli = True
        self._write(b"#")
        self._response()

    def dump(self):
        self._write(b"dump all\n")
        return self._response(timeout=max(self.timeout, 30.0))

    def set_gain(self, target):
        if target not in (BASELINE, CANDIDATE):
            raise TuneError("Only the fixed baseline and candidate gains are permitted")
        self._write(f"set {GAIN_KEY} = {target}\n".encode("ascii"))
        return self._response()

    def leave(self, *, save=False):
        self._write(b"save\n" if save else b"exit\n")
        try:
            return self._response(reboot=True)
        except (OSError, TimeoutError, ConnectionError) as exc:
            # Native F405 USB can reset before the text reaches the host. This
            # is not evidence of a successful save: reconnect and full readback
            # below are mandatory. A failure to WRITE is deliberately uncaught.
            self.receipt.setdefault("reset_response_missing", []).append(
                dict(command="save" if save else "exit", error=str(exc)))
            return None

    def reconnect(self, *, timeout=20.0):
        self.close()
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.connect()
                result = self.snapshot()
                self.in_cli = False  # A valid MSP response proves CLI has exited.
                return result
            except (OSError, TimeoutError, ConnectionError):
                self.close()
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.25)


def perform(session, output, mode, receipt):
    source, target = (CANDIDATE, BASELINE) if mode == "rollback" else (BASELINE, CANDIDATE)
    receipt.update(mode=mode, source_gains=source, target_gains=target,
                   physical_target_rate_gains_per_s=[int(x) / 10 for x in target.split(",")],
                   save_requested=False, save_acknowledged=False, configuration_verified=False)
    session.connect()
    before = session.snapshot()
    receipt["safety_before"] = before
    bb._json(output / "safety_before.json", before)
    session.enter_cli()
    dump = session.dump()
    durable_text(output / "before_dump_all.txt", dump)
    receipt["before_dump_sha256"] = hashlib.sha256(dump.encode()).hexdigest()
    commands = normalized_commands(dump)
    validate_configuration(commands, source)
    desired = expected_commands(commands, target)
    receipt["normalized_command_count"] = len(commands)
    bb._json(output / "plan.json", dict(mode=mode, expected_source=source, target=target,
                                      setting=GAIN_KEY, requirements=REQUIRED,
                                      configuration_before_sha256=receipt["before_dump_sha256"]))
    bb._json(output / "receipt.json", receipt)
    if mode != "plan":
        # The full backup and pending receipt are durable before the single mutation.
        receipt["set_requested"] = True
        bb._json(output / "receipt.json", receipt)
        durable_text(output / "set_response.txt", session.set_gain(target))
        staged = session.dump()
        durable_text(output / "staged_dump_all.txt", staged)
        if normalized_commands(staged) != desired:
            raise TuneError("Staged configuration differs beyond the sole approved gain field; not saved")
        receipt["staged_single_field_verified"] = True
        receipt["save_requested"] = True
        bb._json(output / "receipt.json", receipt)
        response = session.leave(save=True)
        durable_text(output / "save_response.txt", response or "USB reset before text acknowledgement; full saved readback required.\n")
        receipt["save_acknowledged"] = response is not None
    else:
        durable_text(output / "exit_response.txt", session.leave() or "USB reset before text acknowledgement.\n")
    receipt["safety_after_reboot"] = session.reconnect()
    bb._same_recording(before, receipt["safety_after_reboot"])
    session.enter_cli()
    after_dump = session.dump()
    durable_text(output / "after_dump_all.txt", after_dump)
    expected = commands if mode == "plan" else desired
    if normalized_commands(after_dump) != expected:
        raise TuneError("Reboot readback differs from the complete expected configuration")
    validate_configuration(expected, source if mode == "plan" else target)
    durable_text(output / "final_exit_response.txt", session.leave() or "USB reset before text acknowledgement.\n")
    after = session.reconnect()
    bb._same_recording(before, after)
    receipt.update(safety_final=after, configuration_verified=True, log_size_unchanged=True)


def main(argv=None, *, transport_factory=bb.USBSerial):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="new receipt/backup directory")
    parser.add_argument("--port", help="known Sanity USB alias by default")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--apply", action="store_true", help="100,100,80 -> 150,150,80")
    mode.add_argument("--rollback", action="store_true", help="150,150,80 -> 100,100,80")
    parser.add_argument("--timeout", type=bb.positive, default=3.0)
    args = parser.parse_args(argv)
    receipt = dict(host=socket.gethostname(), pid=os.getpid(), started_unix=time.time(),
                   expected_uid=bb.UID, device=bb.DEVICE, status="running", USB_closed=True,
                   services_started=False, automatic_rollback_attempted=False)
    session = None
    output = None
    old_handlers = {}

    def interrupted(signum, _frame):
        raise KeyboardInterrupt(f"Signal {signum}")

    try:
        port = bb.resolve_port(args.port)
        args.output.mkdir(parents=True, exist_ok=False)
        output = args.output.resolve()
        receipt["port"] = port
        with contextlib.ExitStack() as stack:
            lock = stack.enter_context((output / ".tune.lock").open("a"))
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # Held across USB disconnect/reconnect, in addition to USBSerial's device lock.
            device_lock = stack.enter_context(Path(f"/tmp/sanity15-{bb.UID}-gain-tune.lock").open("a"))
            fcntl.flock(device_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                old_handlers[sig] = signal.signal(sig, interrupted)
            bb._json(output / "receipt.json", receipt)
            session = Session(port, transport_factory, args.timeout, receipt)
            try:
                perform(session, output, "rollback" if args.rollback else "apply" if args.apply else "plan", receipt)
                receipt["status"] = "complete"
            finally:
                # exit explicitly discards unsaved changes and reboots; never save in cleanup.
                if session.in_cli:
                    receipt["cleanup_cli_exit_attempted"] = True
                    try:
                        response = session.leave()
                        receipt["cleanup_cli_exit_acknowledged"] = response is not None
                        current = session.reconnect()
                        receipt["cleanup_safety"] = current
                        if "safety_before" in receipt:
                            bb._same_recording(receipt["safety_before"], current)
                    except BaseException as exc:
                        receipt["cleanup_error"] = f"{type(exc).__name__}: {exc}"
                        # A save may have rebooted successfully while its ACK was
                        # lost. Observe that state without claiming save success
                        # or issuing another configuration write.
                        if receipt.get("save_requested"):
                            try:
                                current = session.reconnect()
                                session.in_cli = False
                                receipt["cleanup_safety"] = current
                                bb._same_recording(receipt["safety_before"], current)
                            except BaseException as probe_exc:
                                receipt["cleanup_reconnect_error"] = f"{type(probe_exc).__name__}: {probe_exc}"
                session.close()
        print(f"PASS: {receipt['mode']} verified; full configuration preserved; log size unchanged; USB closed")
        return 0
    except BaseException as exc:
        receipt.update(status="interrupted" if isinstance(exc, KeyboardInterrupt) else "failed",
                       error=f"{type(exc).__name__}: {exc}")
        if isinstance(exc, (SystemExit, GeneratorExit)):
            raise
        print(f"ERROR: {exc}; inspect receipt and backups. No automatic rollback was performed.", file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        if session is not None:
            session.close()
        receipt["completed_unix"] = time.time()
        if output is not None:
            bb._json(output / "receipt.json", receipt)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
