"""Sanity15 USB Blackbox archive reader. No ROS, Zenoh or pyserial dependency.

Downloads once with packet integrity checks, then erases the FC log by default.
Use --no-erase to retain it. Identity/disarm/battery/motor checks still apply.
"""
from __future__ import annotations

import argparse
import contextlib
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import struct
import sys
import termios
import time

DEVICE = 15
UID = "56003f001451333231343731"
CAPACITY = 16 * 1024 * 1024
CHUNK = 1024
DEFAULT_PORTS = (
    "/dev/serial/by-id/usb-Betaflight_Sanity_INDI_HOVER_Betaflight_STM32F405_0x8000000-if00",
    "/dev/serial/by-id/usb-Betaflight_Betaflight_STM32F405_0x8000000-if00",
)
READ_ONLY = frozenset((2, 70, 101, 104, 110, 119, 160))


class ArchiveError(RuntimeError):
    pass


def resolve_port(port=None, *, candidates=None):
    """Inspect known USB aliases only; never probe other connected FCs."""
    if port:
        return port
    devices = {}
    for name in DEFAULT_PORTS if candidates is None else candidates:
        path = Path(name)
        if path.exists():
            devices.setdefault(path.resolve(), str(path))
    if len(devices) != 1:
        raise ArchiveError("Expected one Sanity F405 USB device; connect it or specify --port explicitly")
    return next(iter(devices.values()))


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 255
    return crc


class USBSerial:
    """Raw 115200 8N1 transport with a nonblocking advisory device lock."""

    def __init__(self, port: str):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.original = None
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.original = termios.tcgetattr(self.fd)
            attrs = list(self.original)
            attrs[6] = list(attrs[6])
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except BaseException:
            self.close()
            raise

    def write(self, packet: bytes, deadline: float):
        view = memoryview(packet)
        while view:
            wait = deadline - time.monotonic()
            if wait <= 0 or not select.select([], [self.fd], [], wait)[1]:
                raise TimeoutError("USB write timed out")
            try:
                count = os.write(self.fd, view)
            except BlockingIOError:
                continue
            if not count:
                raise ConnectionError("USB write made no progress")
            view = view[count:]

    def read(self, deadline: float) -> bytes:
        while True:
            wait = deadline - time.monotonic()
            if wait <= 0 or not select.select([self.fd], [], [], wait)[0]:
                raise TimeoutError("MSP response timed out")
            try:
                data = os.read(self.fd, 8192)
            except BlockingIOError:
                continue
            if not data:
                raise ConnectionError("FC USB disconnected")
            return data

    def close(self):
        if self.fd is not None:
            try:
                if self.original is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
            except (termios.error, OSError):
                pass
            finally:
                os.close(self.fd)
                self.fd = None


class MSP:
    def __init__(self, transport, timeout: float = 3.0):
        self.transport, self.timeout = transport, timeout
        self.buffer = bytearray()

    def query(self, command: int) -> bytes:
        if command not in READ_ONLY:
            raise ValueError("This reader permits only its fixed read-only MSP queries")
        return self._v1(command)

    def _v1(self, command: int) -> bytes:
        return self._exchange(b"$M<" + bytes((0, command, command)), command, 1)

    def read_block(self, address: int, count: int, used: int) -> bytes:
        if not 0 <= address < used <= CAPACITY or not 1 <= count <= CHUNK or address + count > used:
            raise ValueError("DATAFLASH_READ address/count exceeds verified recording bounds")
        request = struct.pack("<BHHIHB", 0, 71, 7, address, count, 0)
        payload = self._exchange(b"$X<" + request + bytes((crc8(request),)), 71, 2)
        if len(payload) != count + 7:
            raise ArchiveError("DATAFLASH_READ returned the wrong byte count")
        if struct.unpack_from("<IHB", payload) != (address, count, 0):
            raise ArchiveError("DATAFLASH_READ address/count/compression mismatch")
        return payload[7:]

    def _erase(self):
        self._v1(72)  # Only called after a complete checked archive and fresh safety checks.

    def _exchange(self, packet: bytes, command: int, version: int) -> bytes:
        deadline = time.monotonic() + self.timeout
        self.transport.write(packet, deadline)
        while True:
            while len(self.buffer) >= 3:
                starts = [i for i in (self.buffer.find(b"$M"), self.buffer.find(b"$X")) if i >= 0]
                if not starts:
                    del self.buffer[:-1]
                    break
                start = min(starts)
                if start:
                    del self.buffer[:start]
                native = self.buffer[1] == ord("X")
                header = 8 if native else 5
                if len(self.buffer) < header:
                    break
                if native:
                    flags, reply, length = struct.unpack_from("<BHH", self.buffer, 3)
                    if flags != 0 or length > CHUNK + 7:
                        raise ArchiveError("Invalid MSP2 flags or oversized payload")
                else:
                    length, reply = self.buffer[3:5]
                    if length == 255:
                        raise ArchiveError("Unexpected MSP1 jumbo frame")
                total = header + length + 1
                if len(self.buffer) < total:
                    break
                frame = bytes(self.buffer[:total])
                del self.buffer[:total]
                checksum = crc8(frame[3:-1]) if native else 0
                if not native:
                    for value in frame[3:-1]:
                        checksum ^= value
                if checksum != frame[-1]:
                    raise ArchiveError("MSP response checksum/CRC failed; partial archive retained")
                if frame[2] not in (ord(">"), ord("!"), ord("<")):
                    raise ArchiveError("Invalid MSP response direction")
                if reply != command or native != (version == 2) or frame[2] == ord("<"):
                    continue
                if frame[2] == ord("!"):
                    raise ArchiveError(f"FC rejected MSP command {command}")
                return frame[header:-1]
            self.buffer.extend(self.transport.read(deadline))


def _need(payload: bytes, size: int, name: str):
    if len(payload) < size:
        raise ArchiveError(f"Short {name} response")


def safe_snapshot(client: MSP, *, ready: bool = True) -> dict:
    if client.query(160).hex() != UID:
        raise ArchiveError(f"Wrong FC UID: expected Sanity{DEVICE} {UID}")
    if client.query(2) != b"BTFL":
        raise ArchiveError("Expected Betaflight FC")
    boxes = list(client.query(119))
    if boxes.count(0) != 1 or boxes.index(0) >= 32:
        raise ArchiveError("Cannot locate ARM in MSP_BOXIDS; refusing unsafe status interpretation")
    status = client.query(101)
    _need(status, 11, "STATUS")
    if struct.unpack_from("<I", status, 6)[0] & (1 << boxes.index(0)):
        raise ArchiveError("FC is armed; disarm before downloading")
    motor_payload = client.query(104)
    if len(motor_payload) != 16:
        raise ArchiveError("Expected eight MSP_MOTOR values")
    motors = struct.unpack("<8H", motor_payload)[:4]
    if any(value > 1000 for value in motors):
        raise ArchiveError("Motors are not stopped")
    analog = client.query(110)
    _need(analog, 1, "ANALOG")
    if analog[0] > 5:
        raise ArchiveError("Flight battery is present; disconnect it before USB download")
    payload = client.query(70)
    if len(payload) != 13:
        raise ArchiveError("Unexpected DATAFLASH_SUMMARY layout")
    flags, sectors, capacity, used = struct.unpack("<BIII", payload)
    if not flags & 2 or (ready and not flags & 1):
        raise ArchiveError("FC Blackbox flash is unsupported or not ready")
    if capacity != CAPACITY or not sectors or not 0 <= used <= capacity:
        raise ArchiveError("Unexpected Sanity15 flash geometry or invalid used length")
    return dict(uid=UID, device=DEVICE, flags=flags, sectors=sectors,
                capacity_bytes=capacity, used_bytes=used,
                battery_voltage_V=analog[0] / 10, motor_values=list(motors), armed=False)


def _same_recording(before: dict, current: dict):
    keys = ("uid", "device", "sectors", "capacity_bytes", "used_bytes")
    if not isinstance(before, dict) or any(key not in before for key in keys):
        raise ArchiveError("Invalid saved download summary; use a fresh output directory")
    if any(before[key] != current[key] for key in keys):
        raise ArchiveError("FC identity/recording size changed; use a fresh output directory")


def _json(path: Path, value):
    temp = path.with_name(path.name + ".tmp")
    with temp.open("w") as stream:
        json.dump(value, stream, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)
    _sync_dir(path.parent)


def _sync_dir(path: Path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _digest(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@contextlib.contextmanager
def output_directory(output: Path, resume: bool):
    output = output.resolve()
    existing = False
    if resume:
        if not output.is_dir():
            raise ArchiveError("--resume requires an existing download directory")
    else:
        try:
            output.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            existing = True
    with (output / ".download.lock").open("a") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        if resume:
            if not (output / "summary_before.json").is_file():
                raise ArchiveError("No saved download summary; if preflight failed, retry without --resume")
        elif existing:
            # Preflight failures create ownership records but no saved summary
            # or raw data. Reuse only that exact, closed-attempt state. The lock
            # covers inspection and the whole retry; preserve every old record.
            entries = [p for p in output.iterdir() if p.name != ".download.lock"]
            retryable = bool(entries)
            for path in entries:
                if not path.name.startswith("process_") or path.suffix != ".json" or path.is_symlink() or not path.is_file():
                    retryable = False
                    break
                try:
                    previous = json.loads(path.read_text())
                    valid = (isinstance(previous, dict) and previous.get("device") == DEVICE
                             and previous.get("expected_uid") == UID
                             and previous.get("status") in ("failed", "interrupted")
                             and previous.get("port_closed") is True)
                except (ValueError, OSError):
                    valid = False
                if not valid:
                    retryable = False
                    break
            if not retryable:
                action = "Use --resume to verify saved data" if (output / "summary_before.json").is_file() else "Choose a new output directory"
                raise ArchiveError(f"Output directory already exists and cannot be reused. {action}; existing files were preserved")
        yield output


def _read_copy(client, output: Path, number: int, before: dict, progress):
    used = before["used_bytes"]
    target, partial = output / f"read{number}.bbl", output / f"read{number}.partial"
    if target.exists() and partial.exists():
        raise ArchiveError(f"Both complete and partial read{number} exist; inspect the output directory")
    existing = target if target.exists() else partial
    done = existing.stat().st_size if existing.exists() else 0
    if done > used or (target.exists() and done != used):
        raise ArchiveError("Existing copy length does not match the verified flash summary")
    # A process killed during a write can leave an incomplete final block. Keep
    # that tail as evidence, then re-read the whole block from its boundary.
    if done != used and done % CHUNK:
        boundary = done // CHUNK * CHUNK
        tail_path = output / f"read{number}.interrupted_tail.{time.time_ns()}.bin"
        with partial.open("rb+") as stream, tail_path.open("xb") as tail:
            stream.seek(boundary)
            tail.write(stream.read())
            tail.flush()
            os.fsync(tail.fileno())
            stream.truncate(boundary)
            stream.flush()
            os.fsync(stream.fileno())
        done = boundary
    # Revalidate EVERY existing byte against the FC, not just a prefix/header.
    # Equal length alone cannot distinguish a new recording from an old one.
    if done:
        with existing.open("rb") as stream:
            for address in range(0, done, CHUNK):
                count = min(CHUNK, done - address)
                if client.read_block(address, count, used) != stream.read(count):
                    raise ArchiveError(f"Resume copy{number} differs from FC at byte {address}")
        progress(dict(read=number, resumed_verified_bytes=done, total=used))
    if target.exists():
        return
    with partial.open("ab") as stream:
        for address in range(done, used, CHUNK):
            count = min(CHUNK, used - address)
            stream.write(client.read_block(address, count, used))
            # Each acknowledged partial prefix survives a clean interrupt.
            stream.flush()
            os.fsync(stream.fileno())
            end = address + count
            if end == used or end // 524288 != address // 524288:
                progress(dict(read=number, bytes=end, total=used))
    os.replace(partial, target)
    _sync_dir(output)


def download(client: MSP, output: Path, *, resume=False, erase_after_verify=True,
             erase_timeout=120.0, progress=print) -> dict:
    """Caller owns/locks the output directory and closes the serial transport."""
    before = safe_snapshot(client)
    if before["used_bytes"] == 0:
        raise ArchiveError("FC Blackbox is empty; nothing to archive or erase")
    saved = output / "summary_before.json"
    if resume:
        _same_recording(json.loads(saved.read_text()), before)
    else:
        _json(saved, before)
    _read_copy(client, output, 1, before, progress)
    _same_recording(before, safe_snapshot(client))
    first = output / "read1.bbl"
    digest = _digest(first)
    if first.stat().st_size != before["used_bytes"]:
        raise ArchiveError("Incomplete archive; refusing erase")
    receipt = dict(success=True, device=DEVICE, uid=UID, bytes=before["used_bytes"],
                   sha256=digest, download_passes=1, independent_reads_equal=None,
                   verification="MSP2 packet CRC, address, length and complete saved size; local SHA256 recorded",
                   all_addresses_lengths_CRCs_valid=True, native_MSP2=True,
                   chunk_bytes=CHUNK, firmware_changed=False, configuration_changed=False,
                   arm_or_motor_commands_sent=False, erased=False, USB_closed=False)
    _json(output / "receipt.json", receipt)
    if erase_after_verify:
        # Keep a durable verified archive receipt before issuing the optional
        # erase, and recheck all interlocks immediately before that command.
        _same_recording(before, safe_snapshot(client))
        receipt["erase_requested"] = True
        receipt["erase_completion_confirmed"] = False
        _json(output / "receipt.json", receipt)
        client._erase()
        deadline = time.monotonic() + erase_timeout
        while True:
            current = safe_snapshot(client, ready=False)
            if current["sectors"] != before["sectors"]:
                raise ArchiveError("Flash geometry changed during erase")
            if current["flags"] & 1 and current["used_bytes"] == 0:
                receipt.update(erased=True, erase_completion_confirmed=True,
                               summary_after_erase=current)
                _json(output / "receipt.json", receipt)
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Erase completion not confirmed; checked raw archive retained")
            time.sleep(0.25)
    return receipt


def positive(value):
    x = float(value)
    if not math.isfinite(x) or x <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return x


def main(argv=None, *, transport_factory=USBSerial) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path,
                        help="new directory, preferably FLIGHT_RUN/blackbox")
    parser.add_argument("--port", help="USB path; default detects the known Sanity/generic F405 aliases")
    parser.add_argument("--resume", action="store_true", help="verify and resume this output directory")
    erase = parser.add_mutually_exclusive_group()
    erase.add_argument("--no-erase", dest="erase_after_verify", action="store_false",
                       help="retain FC logs after the single checked download")
    erase.add_argument("--erase-after-verify", dest="erase_after_verify", action="store_true",
                       help="erase after the single checked download (default)")
    parser.set_defaults(erase_after_verify=True)
    parser.add_argument("--timeout", type=positive, default=3.0)
    parser.add_argument("--erase-timeout", type=positive, default=120.0)
    args = parser.parse_args(argv)
    ownership = dict(host=socket.gethostname(), pid=os.getpid(), pgid=os.getpgrp(),
                     started_unix=time.time(), device=DEVICE, expected_uid=UID,
                     port=args.port, argv=list(argv if argv is not None else sys.argv[1:]),
                     status="running", services_started=False)
    transport = None
    old_handlers = {}

    def interrupted(signum, _frame):
        raise KeyboardInterrupt(f"Signal {signum}; partial raw files retained")

    try:
        args.port = resolve_port(args.port)
        ownership["port"] = args.port
        with output_directory(args.output, args.resume) as output:
            # Keep each invocation's ownership record instead of overwriting a
            # previous interrupted run's history.
            owner_path = output / f"process_{os.getpid()}_{time.time_ns()}.json"
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                old_handlers[sig] = signal.signal(sig, interrupted)
            _json(owner_path, ownership)
            try:
                transport = transport_factory(args.port)
                result = download(MSP(transport, args.timeout), output, resume=args.resume,
                                  erase_after_verify=args.erase_after_verify,
                                  erase_timeout=args.erase_timeout,
                                  progress=lambda value: print(json.dumps(value), flush=True))
                ownership["status"] = "complete"
            except BaseException as exc:
                ownership.update(status="interrupted" if isinstance(exc, KeyboardInterrupt) else "failed",
                                 error=f"{type(exc).__name__}: {exc}")
                raise
            finally:
                try:
                    if transport is not None:
                        transport.close()
                finally:
                    ownership.update(port_opened=transport is not None,
                                     port_closed=transport is None or transport.fd is None,
                                     completed_unix=time.time())
                    _json(owner_path, ownership)
            result.update(USB_closed=True, completed_unix=time.time())
            _json(output / "receipt.json", result)
            print("PASS: single download complete; packet CRC/address/length checks passed; USB closed; " +
                  ("FC log erased." if result["erased"] else "FC log retained."), flush=True)
        return 0
    except KeyboardInterrupt:
        print("Interrupted: verified partial files retained; rerun with the same --output and --resume.", file=sys.stderr)
        return 130
    except (OSError, ArchiveError, ValueError, TimeoutError, ConnectionError) as exc:
        print(f"ERROR: {exc}. FC log was not erased unless receipt.json confirms/requested erase.", file=sys.stderr)
        return 1
    finally:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
