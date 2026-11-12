"""Record exact observation bytes and replay the portable C ESKF on a host.

python -m sanity_zenoh.estimator_tools --help
Recording subscribes only; it cannot send commands or arm hardware.
"""

import argparse
import ctypes as C
import hashlib
import json
import math
import queue
from pathlib import Path
import struct
import subprocess
import threading
import time
from .estimator_protocol import decode_record, split_transport_records

MAGIC = b"SELOG1\r\n"


def read_log(path):
    with Path(path).open("rb") as source:
        if source.read(8) != MAGIC:
            raise ValueError("not an observation log")
        size = source.read(4)
        if len(size) != 4:
            raise ValueError("truncated metadata")
        n = struct.unpack("<I", size)[0]
        if n > 1024 * 1024:
            raise ValueError("metadata too large")
        metadata = json.loads(source.read(n))
        records = []
        while header := source.read(12):
            if len(header) != 12:
                raise ValueError("truncated record header")
            n, host_ns = struct.unpack("<IQ", header)
            if not 28 <= n <= 256:
                raise ValueError("invalid record size")
            raw = source.read(n)
            item = decode_record(raw)
            item["host_received_ns"] = host_ns
            item["raw"] = raw
            records.append(item)
    return metadata, records


def write_log(path, metadata, records):
    """Offline fixture writer; records are original encoded bytes."""
    header = json.dumps(metadata, sort_keys=True).encode()
    with Path(path).open("xb") as target:
        target.write(MAGIC + struct.pack("<I", len(header)) + header)
        for raw, stamp in records:
            decode_record(raw)
            target.write(struct.pack("<IQ", len(raw), stamp) + raw)


def record(args):
    import zenoh

    metadata = json.loads(args.firmware_manifest.read_text())
    metadata.update(
        format="SELOG1",
        created_unix_ns=time.time_ns(),
        device=args.device,
        endpoint=args.endpoint,
        qualification="observation only; hardware commissioning gates remain separate",
    )
    header = json.dumps(metadata, sort_keys=True).encode()
    lock = threading.Lock()
    counts = {
        "received": 0,
        "malformed": 0,
        "sequence_gaps": 0,
        "writer_drops": 0,
        "writer_errors": 0,
    }
    last = None
    pending = queue.Queue(maxsize=4096)
    stopping = threading.Event()
    with args.output.open("xb") as target:
        target.write(MAGIC + struct.pack("<I", len(header)) + header)

        def writer():
            while not stopping.is_set() or not pending.empty():
                try:
                    raw, host = pending.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    target.write(struct.pack("<IQ", len(raw), host) + raw)
                except OSError:
                    with lock:
                        counts["writer_errors"] += 1
                finally:
                    pending.task_done()

        worker = threading.Thread(
            target=writer, name="observation_log_writer", daemon=True
        )
        worker.start()

        def received(sample):
            nonlocal last
            host = time.monotonic_ns()
            try:
                bundle = split_transport_records(bytes(sample.payload))
            except ValueError:
                with lock:
                    counts["malformed"] += 1
                return
            with lock:
                for raw in bundle:
                    decoded = decode_record(raw)
                    if last is not None:
                        delta = (decoded["sequence"] - last) & 0xFFFFFFFF
                        if 1 < delta < 0x80000000:
                            counts["sequence_gaps"] += delta - 1
                    last = decoded["sequence"]
                    counts["received"] += 1
                    try:
                        pending.put_nowait((raw, host))
                    except queue.Full:
                        counts["writer_drops"] += 1

        try:
            config = zenoh.Config()
            config.insert_json5("mode", json.dumps("client"))
            config.insert_json5("connect/endpoints", json.dumps([args.endpoint]))
            with zenoh.open(config) as session:
                subscriber = session.declare_subscriber(
                    f"sanity/{args.device:02d}/observation/records", received
                )
                try:
                    deadline = (
                        time.monotonic() + args.duration if args.duration else math.inf
                    )
                    while time.monotonic() < deadline:
                        time.sleep(0.2)
                except KeyboardInterrupt:
                    pass
                finally:
                    subscriber.undeclare()
        except Exception:
            counts["session_errors"] = counts.get("session_errors", 0) + 1
            raise
        finally:
            stopping.set()
            worker.join()
            args.output.with_suffix(args.output.suffix + ".receipt.json").write_text(
                json.dumps(counts, indent=2) + "\n"
            )
    args.output.with_suffix(args.output.suffix + ".receipt.json").write_text(
        json.dumps(counts, indent=2) + "\n"
    )
    print(json.dumps(counts))


def build(args):
    args.output.mkdir(parents=True, exist_ok=True)
    sources = [args.core / name for name in ("eskf.c", "sensor_wire.c", "host_api.c")]
    hashes = {
        str(p.name): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [*sources, args.core / "eskf.h", args.core / "sensor_wire.h"]
    }
    for precision in ("float", "double"):
        command = [
            "cc",
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-shared",
            "-fPIC",
            *[str(p) for p in sources],
            "-lm",
            "-o",
            str(args.output / f"estimator_{precision}.so"),
        ]
        if precision == "double":
            command.insert(1, "-DSE_DOUBLE")
        subprocess.run(command, check=True)
    (args.output / "host_build.json").write_text(
        json.dumps(
            dict(
                sources=hashes,
                compiler=subprocess.check_output(
                    ["cc", "--version"], text=True
                ).splitlines()[0],
            ),
            indent=2,
        )
        + "\n"
    )


class ReplayCore:
    def __init__(self, library):
        self.lib = C.CDLL(str(Path(library).resolve()))
        D = C.POINTER(C.c_double)
        U64 = C.c_uint64
        U32 = C.c_uint32
        self.lib.se_host_reset.argtypes = [U64]
        self.lib.se_host_clock.argtypes = [C.c_void_p, U64]
        self.lib.se_host_map.argtypes = [U64, U32, U64, C.POINTER(U64), C.POINTER(U32)]
        self.lib.se_host_imu.argtypes = [
            U64,
            U32,
            U32,
            C.c_uint16,
            D,
            D,
            D,
            U64,
            U32,
            U32,
        ]
        self.lib.se_host_pose.argtypes = [U64, U32, U32, D]
        self.lib.se_host_state.argtypes = [D, C.POINTER(U64), C.POINTER(U32)]

    def reset(self, epoch):
        self.lib.se_host_reset(epoch)

    def mapped(self, epoch, raw, now):
        stamp = C.c_uint64()
        u = C.c_uint32()
        if not self.lib.se_host_map(epoch, raw, now, C.byref(stamp), C.byref(u)):
            return None
        return stamp.value, u.value

    def snapshot(self):
        values = (C.c_double * 241)()
        stamp = C.c_uint64()
        stats = (C.c_uint32 * 15)()
        flags = self.lib.se_host_state(values, C.byref(stamp), stats)
        return dict(
            time_us=stamp.value,
            initialized=bool(flags & 1),
            pending=bool(flags & 6),
            state=list(values[:16]),
            covariance=list(values[16:]),
            fault=stats[0],
            fused=stats[1],
            replayed=stats[2],
            pending_replaced=stats[3],
            rejected=list(stats[4:]),
        )


def _replay_segment(records, library, epoch):
    core = ReplayCore(library)
    if epoch is None:
        raise ValueError("log has no observation epoch")
    core.reset(epoch)
    poses = []
    results = []
    mapping_rejected = 0
    pose_session = None
    last_source_ns = 0
    last_submitted = 0
    last_imu_time = 0
    # Original ESP reception times determine scheduling; host arrival is never an IMU acquisition time.
    events = sorted(
        (r for r in records if r["kind"] in (1, 2, 4)), key=lambda r: r["received_us"]
    )
    if not events:
        return [], dict(mapping_rejected=0)
    index = 0
    now = events[0]["received_us"] // 1000 * 1000
    end = events[-1]["received_us"] + 100000
    next_output = now
    while now <= end:
        while index < len(events) and events[index]["received_us"] <= now:
            r = events[index]
            index += 1
            if r["kind"] == 2:
                clock_result = core.lib.se_host_clock(r["raw"][24:72], r["t4"])
                if not clock_result:
                    mapping_rejected += 1
                elif clock_result == 2:
                    poses = []
                    last_submitted = 0
                    last_imu_time = 0
                    pose_session = None
                    last_source_ns = 0
            elif r["kind"] == 4:
                if r["epoch"] != epoch:
                    continue
                lo, hi = r["capture_lower_us"], r["capture_upper_us"]
                if (
                    hi < lo
                    or hi - lo > 4000
                    or r["received_us"] < hi
                    or r["received_us"] - lo > 250000
                ):
                    continue
                if not r["session"]:
                    continue
                if pose_session is not None and pose_session != r["session"]:
                    core.reset(epoch)
                    poses = []
                    last_source_ns = 0
                if r["source_ros_ns"] <= last_source_ns:
                    continue
                pose_session = r["session"]
                last_source_ns = r["source_ros_ns"]
                pose = (C.c_double * 7)(*r["position"], *r["quaternion"])
                stamp = (lo + hi) // 2
                uncertainty = (hi - lo) // 2
                seq = r["pose_sequence"]
                poses.append((stamp, seq, uncertainty, pose))
                poses = poses[-32:]

            else:
                s = r["sensor"]
                g = core.mapped(s["epoch"], s["gyro_us"], r["received_us"])
                a = core.mapped(s["epoch"], s["accel_us"], r["received_us"])
                if not g or not a or s["flags"] & 3 != 3:
                    mapping_rejected += 1
                    continue
                gt, at = g[0] - s["gyro_delay_us"], a[0] - s["accel_delay_us"]
                if min(gt, at) < 0:
                    mapping_rejected += 1
                    continue
                stamp = (gt + at) // 2
                uncertainty = (
                    max(g[1], a[1])
                    + max(s["gyro_uncertainty_us"], s["accel_uncertainty_us"])
                    + abs(gt - at) // 2
                )
                p = next((p for p in reversed(poses) if p[0] <= stamp), None)
                core.lib.se_host_imu(
                    stamp,
                    s["sequence"],
                    uncertainty,
                    s["flags"],
                    (C.c_double * 3)(*s["gyro"]),
                    (C.c_double * 3)(*s["accel"]),
                    p[3] if p else None,
                    p[0] if p else 0,
                    p[1] if p else 0,
                    p[2] if p else 0,
                )
                last_imu_time = stamp
        if core.snapshot()["initialized"]:
            eligible = next((p for p in reversed(poses) if p[0] <= last_imu_time), None)
            if eligible and eligible[0] > last_submitted:
                core.lib.se_host_pose(
                    eligible[0], eligible[1], eligible[2], eligible[3]
                )
                last_submitted = eligible[0]
        core.lib.se_host_service()
        if now >= next_output:
            output = core.snapshot()
            output["processing_us"] = now
            results.append(output)
            next_output = now + 20000
        now += 1000
    return results, dict(
        mapping_rejected=mapping_rejected,
        records=len(records),
        outputs=len(results),
        final=core.snapshot(),
    )


def replay_records(records, library):
    # Capture order is authoritative across monotonic-clock resets. Explicit
    # epoch records split sessions before sorting reception times within one boot.
    segments = []
    current = []
    epoch = None
    for r in records:
        if r["kind"] == 6:
            if current:
                segments.append((epoch, current))
                current = []
            epoch = r["epoch"]
        elif epoch is None and r["kind"] in (3, 4):
            epoch = r["epoch"]
        current.append(r)
    if current:
        segments.append((epoch, current))
    results = []
    receipts = []
    for index, (epoch, segment) in enumerate(segments):
        if epoch is None:
            raise ValueError("segment lacks explicit observation epoch")
        output, receipt = _replay_segment(segment, library, epoch)
        for item in output:
            item["epoch"] = epoch
            item["segment"] = index
        results.extend(output)
        receipts.append(receipt)
    return results, dict(
        segments=receipts,
        records=len(records),
        outputs=len(results),
        final=receipts[-1].get("final") if receipts else None,
    )


def replay(args):
    metadata, records = read_log(args.input)
    results, receipt = replay_records(records, args.library)
    with args.output.open("x") as target:
        for result in results:
            target.write(json.dumps(result, allow_nan=False) + "\n")
    receipt.update(
        input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
        library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),
        original_metadata=metadata,
    )
    args.output.with_suffix(args.output.suffix + ".receipt.json").write_text(
        json.dumps(receipt, indent=2, allow_nan=False) + "\n"
    )
    print(
        json.dumps(
            {
                k: v
                for k, v in receipt.items()
                if k not in ("final", "original_metadata")
            }
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    p = commands.add_parser("record")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--firmware-manifest", type=Path, required=True)
    p.add_argument("--device", type=int, default=15)
    p.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
    p.add_argument("--duration", type=float, default=0)
    p.set_defaults(run=record)
    p = commands.add_parser("build-host")
    p.add_argument("--core", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.set_defaults(run=build)
    p = commands.add_parser("replay")
    p.add_argument("input", type=Path)
    p.add_argument("--library", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.set_defaults(run=replay)
    args = parser.parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
