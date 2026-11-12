"""Offline estimator milestone tests; no serial/device access."""

import ctypes as C
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import pytest

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "Sanity").is_dir())
HOST = ROOT / "Sanity/src/ros2_sanity_ws/src/sanity_zenoh"
sys.path.insert(0, str(HOST))
from sanity_zenoh.estimator_protocol import (
    sensor_packet,
    decode_sensor,
    decode_record,
    frame_record,
    crc32c,
)
from sanity_zenoh.estimator_tools import read_log, write_log, replay_records, ReplayCore

CORE = ROOT / "Sanity/src/sanity_receiver/zenoh_receiver/main/estimator"
TEST = Path(__file__).parent
BF = ROOT / "outputs/sanity_hover_integration_20261030/betaflight-4.5.1/src/main"


@pytest.fixture(scope="session")
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp("estimator")
    for precision in ("float", "double"):
        flags = ["-DSE_DOUBLE"] if precision == "double" else []
        common = [
            "cc",
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            *flags,
            "-I",
            str(CORE.parent),
        ]
        subprocess.run(
            [
                *common,
                str(TEST / "estimator_native.c"),
                str(CORE / "eskf.c"),
                str(CORE / "sensor_wire.c"),
                "-lm",
                "-o",
                str(out / precision),
            ],
            check=True,
        )
        subprocess.run(
            [
                *common,
                "-shared",
                "-fPIC",
                str(CORE / "host_api.c"),
                str(CORE / "eskf.c"),
                str(CORE / "sensor_wire.c"),
                "-lm",
                "-o",
                str(out / f"{precision}.so"),
            ],
            check=True,
        )
    subprocess.run(
        [
            "cc",
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(TEST / "estimator_fakes"),
            "-I",
            str(CORE.parent),
            str(TEST / "estimator_runtime_native.c"),
            *[str(CORE / f"{name}.c") for name in ("eskf", "sensor_wire", "runtime")],
            "-lm",
            "-o",
            str(out / "runtime"),
        ],
        check=True,
    )
    subprocess.run(
        [
            "cc",
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(TEST / "estimator_fc_fakes"),
            "-I",
            str(BF),
            str(TEST / "estimator_fc_native.c"),
            str(BF / "flight/sanity_sensor.c"),
            str(BF / "flight/sanity_sensor/sensor_wire.c"),
            "-lm",
            "-o",
            str(out / "fc"),
        ],
        check=True,
    )
    return out


@pytest.mark.parametrize("delay", [0, 20, 50, 100, 200])
@pytest.mark.parametrize("stress", [0, 1])
def test_delayed_3d_motion_float_double(built, delay, stress):
    results = [
        json.loads(
            subprocess.check_output(
                [str(built / p), str(delay), str(stress)], text=True
            )
        )
        for p in ("float", "double")
    ]
    f, d = results
    assert f["rmse_m"] <= 1.1 * d["rmse_m"] and f["rmse_m"] < 0.025
    assert f["attitude_rmse_rad"] < 0.025
    assert max(abs(a - b) for a, b in zip(f["state"], d["state"])) < 2e-5
    assert max(abs(a - b) for a, b in zip(f["covariance"], d["covariance"])) < 2e-5
    assert f["max_replay"] <= 8 and not f["pending"] and f["memory_bytes"] < 96 * 1024
    assert (
        abs(f["state"][13] - 0.03) < 0.01
        and abs(f["state"][14] + 0.02) < 0.01
        and abs(f["state"][15] - 0.01) < 0.01
    )
    if stress:
        assert f["gate_rejections"] > 0


@pytest.mark.parametrize("pose_delay", [500, 200000, 249900])
def test_runtime_parser_epochs_freshness_and_overflow(built, pose_delay):
    output = subprocess.check_output(
        [str(built / "runtime"), str(pose_delay)], text=True
    )
    assert "500 sensors" in output and "queueoverflow=68" in output


@pytest.mark.parametrize("frequency", [5, 40, 100, 200, 400])
def test_fc_interleaved_wrap_filter_queue(built, frequency):
    result = json.loads(
        subprocess.check_output([str(built / "fc"), str(frequency)], text=True)
    )
    assert (
        result["samples"] == 4000
        and result["wrap_and_interleaved_acquisition"]
        and result["drop_oldest"] > 0
    )
    if frequency == 5:
        assert 0.85 < result["gain"] < 1
    if frequency >= 100:
        assert result["gain"] < 0.025


def test_c_python_packet_fixture(built):
    raw = sensor_packet(
        0x123456789ABCDEF,
        0xFFFFFFFF,
        0xFFFFFFF0,
        0x10,
        [0.1, -0.2, 0.3],
        [0, 0, 9.80665],
    )
    lib = C.CDLL(str(built / "float.so"))
    lib.se_sensor_decode.argtypes = [C.c_void_p, C.c_size_t, C.c_void_p]
    lib.se_sensor_decode.restype = C.c_bool
    lib.se_sensor_encode.argtypes = [C.c_void_p, C.c_void_p]
    decoded = (C.c_uint8 * 128)()
    encoded = (C.c_uint8 * 76)()
    assert lib.se_sensor_decode(raw, len(raw), decoded)
    lib.se_sensor_encode(decoded, encoded)
    assert bytes(encoded) == raw
    for n in range(76):
        corrupted = bytearray(raw)
        corrupted[n] ^= 1
        with pytest.raises(ValueError):
            decode_sensor(corrupted)
        assert not lib.se_sensor_decode(bytes(corrupted), 76, decoded)
    with pytest.raises(ValueError):
        decode_sensor(raw[:-1])
    with pytest.raises(ValueError):
        sensor_packet(42, 1, 1, 1, [float("nan"), 0, 0], [0, 0, 9.8])


def fixture_records(epoch=42, fc_epoch=99, base=1000000, fc_reset=False):
    records = []
    seq = 0

    def emit(kind, payload, t):
        nonlocal seq
        seq += 1
        records.append((frame_record(kind, payload, t, seq), t * 1000))

    emit(6, struct.pack("<Q", epoch), base)
    for n in range(1, 501):
        now = base + n * 5000
        if n % 4 == 1 or fc_reset and n == 301: # Match the 50 Hz hardware clock-probe profile.
            if fc_reset and n == 301:
                fc_epoch += 1
            c = struct.pack("<5QI", fc_epoch, n, now - 1410, now - 710, now - 700, n)
            c += struct.pack("<I", crc32c(c))
            emit(2, c + struct.pack("<Q", now), now)
        if fc_reset and n == 302:
            emit(
                1,
                sensor_packet(
                    fc_epoch - 1,
                    n,
                    now - 400,
                    now - 400,
                    [0, 0, 0],
                    [0, 0, 9.80665],
                    gyro_delay_us=0,
                    accel_delay_us=0,
                ),
                now,
            )
        # Bodyposeprovidedaftermarker-to-bodycalibration; velocitydeliberatelywrong.
        p = struct.pack(
            "<4Q2I10f8x",
            epoch,
            n * 5000000,
            now - 600,
            now - 400,
            7,
            n,
            0,
            0,
            0,
            1,
            0,
            0,
            0,
            123,
            456,
            789,
        )
        emit(4, p, now)
        emit(
            1,
            sensor_packet(
                fc_epoch,
                n,
                now - 400,
                now - 400,
                [0, 0, 0],
                [0, 0, 9.80665],
                gyro_delay_us=0,
                accel_delay_us=0,
            ),
            now,
        )
    return records


def test_host_record_replay_deterministic_restart(tmp_path, built):
    first = fixture_records()
    second = fixture_records(epoch=43, base=1000, fc_reset=True)
    second.insert(1, first[2])  # stale pre-reboot pose after the new explicit epoch
    records = first + second
    path = tmp_path / "fixture.slog"
    write_log(path, {"synthetic": True, "hardware": False}, records)
    metadata, decoded = read_log(path)
    a, receipt = replay_records(decoded, built / "float.so")
    b, _ = replay_records(decoded, built / "float.so")
    d, _ = replay_records(decoded, built / "double.so")
    assert a == b and len(receipt["segments"]) == 2
    assert any(s["initialized"] for s in a if s["epoch"] == 42)
    assert any(s["initialized"] for s in a if s["epoch"] == 43)
    assert receipt["segments"][1]["final"]["initialized"]
    assert receipt["segments"][1]["mapping_rejected"] >= 1
    assert {s["epoch"] for s in a} == {42, 43}
    assert all(math.isfinite(x) for s in a for x in s["state"] + s["covariance"])
    assert (
        max(abs(x - y) for s, t in zip(a, d) for x, y in zip(s["state"], t["state"]))
        < 1e-5
    )
    assert metadata["synthetic"]
    # Truncation/corruption is rejected, never silently treated as end-of-log.
    corrupted = tmp_path / "truncated.slog"
    corrupted.write_bytes(path.read_bytes()[:-1])
    with pytest.raises(ValueError):
        read_log(corrupted)


def test_tilted_stationary_gravity_and_missing_pose_covariance(built):
    core = ReplayCore(built / "float.so")
    core.reset(42)
    angle = 0.4
    q = [math.cos(angle / 2), math.sin(angle / 2), 0, 0]
    p = (C.c_double * 7)(0, 0, 0, *q)
    g = (C.c_double * 3)(0, 0, 0)
    acc = (C.c_double * 3)(0, math.sin(angle) * 9.80665, math.cos(angle) * 9.80665)
    for n in range(1, 201):
        core.lib.se_host_imu(n * 5000, n, 100, 15, g, acc, p, n * 5000, n, 100)
    first = core.snapshot()
    assert first["initialized"]
    for n in range(201, 601):
        core.lib.se_host_imu(n * 5000, n, 100, 15, g, acc, None, 0, 0, 0)
    last = core.snapshot()
    assert max(map(abs, last["state"][:6])) < 1e-4
    assert last["covariance"][0] > first["covariance"][0] * 2


def test_source_frame_identity_and_command_isolation():
    source = (BF / "flight/imu.c").read_text()
    assert (
        "DEGREES_TO_RADIANS(gyroAverage[X]), DEGREES_TO_RADIANS(gyroAverage[Y]), DEGREES_TO_RADIANS(gyroAverage[Z])"
        in source
    )
    runtime = (CORE / "runtime.c").read_text()
    assert "sz_motors" not in runtime and "sz_rc_" not in runtime
    fc = (BF / "flight/sanity_sensor.c").read_text()
    assert "dps[a]" not in fc or "filtered[a]" in fc
    assert (CORE / "sensor_wire.h").read_bytes() == (
        BF / "flight/sanity_sensor/sensor_wire.h"
    ).read_bytes()
    assert (CORE / "sensor_wire.c").read_bytes() == (
        BF / "flight/sanity_sensor/sensor_wire.c"
    ).read_bytes()


def test_recorder_session_failure_joins_writer(tmp_path, monkeypatch):
    import threading
    import types
    from sanity_zenoh.estimator_tools import record

    class Config:
        def insert_json5(self, *args):
            pass

    def failed_open(config):
        raise RuntimeError("synthetic connection failure")

    monkeypatch.setitem(
        sys.modules, "zenoh", types.SimpleNamespace(Config=Config, open=failed_open)
    )
    manifest = tmp_path / "manifest.json"
    manifest.write_text("{}")
    args = types.SimpleNamespace(
        firmware_manifest=manifest,
        device=15,
        endpoint="tcp/localhost:7447",
        output=tmp_path / "failed.slog",
        duration=0.01,
    )
    with pytest.raises(RuntimeError, match="synthetic connection"):
        record(args)
    assert not any(
        t.name == "observation_log_writer" and t.is_alive()
        for t in threading.enumerate()
    )
    assert (
        json.loads(args.output.with_suffix(".slog.receipt.json").read_text())[
            "session_errors"
        ]
        == 1
    )


@pytest.mark.parametrize('clock_ratio',[1.0002,.9986])
def test_clock_drift_fit_handles_bounded_uart_asymmetry(built,clock_ratio):
    class ClockMap(C.Structure):
        _fields_ = [(name, C.c_uint64) for name in
                    ('epoch','remote_anchor','local_anchor','last_local','fit_remote','fit_local')]+[
                    ('scale',C.c_double),('uncertainty_us',C.c_uint32),
                    ('samples',C.c_uint),('valid',C.c_bool)]
    lib=C.CDLL(str(built/'float.so'));m=ClockMap()
    lib.se_clock_observe.argtypes=[C.POINTER(ClockMap),*[C.c_uint64]*5]
    lib.se_clock_observe.restype=C.c_bool
    lib.se_clock_convert.argtypes=[C.POINTER(ClockMap),C.c_uint64,C.c_uint64,C.c_uint64,C.POINTER(C.c_uint64),C.POINTER(C.c_uint32)]
    lib.se_clock_convert.restype=C.c_bool
    # Fixed 200 ppm clock skew, with bounded UART asymmetry changing each second.
    # A one-second point-to-point slope confuses this latency jitter with skew.
    for n in range(120):
        t1=60_000_000+n*100_000
        forward=100 if n//10%2==0 else 1800
        reverse=2000-forward
        t2=round((t1+forward)*clock_ratio)+800_000;t3=t2+10
        t4=t1+forward+reverse+10
        assert lib.se_clock_observe(C.byref(m),42,t1,t2,t3,t4)
        stamp=C.c_uint64();u=C.c_uint32()
        acquisition=t4-300;remote=round(acquisition*clock_ratio)+800_000
        assert lib.se_clock_convert(C.byref(m),42,remote,t4,C.byref(stamp),C.byref(u))
        assert abs(stamp.value-acquisition)<=u.value<=2000
    assert m.samples>=3 and m.valid
    anchor=m.remote_anchor;uncertainty=m.uncertainty_us
    t1=t4+100_000;remote=lambda t: round(t*clock_ratio)+800_000
    assert lib.se_clock_observe(C.byref(m),42,t1,remote(t1+1800),remote(t1+1810),t1+3810)
    assert m.remote_anchor==anchor and m.uncertainty_us==uncertainty
    # A packet that cannot satisfy the 2 ms limit must not replace the map.
    assert not lib.se_clock_observe(C.byref(m),42,t1+100_000,remote(t1+101800),remote(t1+101810),t1+104010)
    assert m.valid and m.remote_anchor==anchor
    lib.se_clock_observe_uart.argtypes=lib.se_clock_observe.argtypes
    lib.se_clock_observe_uart.restype=C.c_bool
    for forward,reverse in ((700,700),(600,2400),(2400,600)):
        u_map=ClockMap();t1=1_000_000;offset=800_000
        t2=t1+forward+offset;t3=t2+10;t4=t1+forward+reverse+10
        assert lib.se_clock_observe_uart(C.byref(u_map),42,t1,t2,t3,t4)
        stamp=C.c_uint64();u=C.c_uint32()
        assert lib.se_clock_convert(C.byref(u_map),42,t4-300+offset,t4,C.byref(stamp),C.byref(u))
        assert abs(stamp.value-(t4-300))<=u.value
        assert u.value==(forward+reverse-1200)//2+50
    assert not lib.se_clock_observe_uart(C.byref(ClockMap()),42,1000,1100,1110,1210)


def test_observation_transport_single_bundles_and_atomic_rejection():
    from sanity_zenoh.estimator_protocol import split_transport_records
    a=frame_record(6,struct.pack('<Q',42),1000,1)
    b=frame_record(6,struct.pack('<Q',43),2000,2)
    assert split_transport_records(a)==[a]
    assert split_transport_records(a+b)==[a,b]
    assert split_transport_records(a*16)==[a]*16
    for bad in (a*17,a+b[:-1],a+b'xx',b'xx',a+bytes(bytearray(b[:-1])+bytes([b[-1]^1]))):
        with pytest.raises(ValueError):split_transport_records(bad)
