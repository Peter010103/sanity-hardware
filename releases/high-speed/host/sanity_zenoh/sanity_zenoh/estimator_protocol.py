"""Version1 observation-only FC/S3 stream. No commands or actuator interface."""

import math
import struct

SENSOR_SIZE = 76
SENSOR_MSP = 0x7F12
CLOCK_MSP = 0x7F13


def crc32c(data):
    value = 0xFFFFFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x82F63B78 if value & 1 else 0)
    return value ^ 0xFFFFFFFF


def sensor_packet(
    epoch,
    sequence,
    gyro_us,
    accel_us,
    gyro,
    accel,
    *,
    flags=15,
    dropped=0,
    gyro_uncertainty_us=125,
    accel_uncertainty_us=125,
    gyro_source=2,
    accel_source=2,
    gyro_delay_us=15915,
    accel_delay_us=15915,
):
    b = bytearray(76)
    struct.pack_into(
        "<2sBBQ6I6fHBB2I",
        b,
        0,
        b"SI",
        1,
        76,
        epoch,
        sequence,
        gyro_us,
        accel_us,
        gyro_uncertainty_us,
        accel_uncertainty_us,
        dropped,
        *gyro,
        *accel,
        flags,
        gyro_source,
        accel_source,
        gyro_delay_us,
        accel_delay_us,
    )
    struct.pack_into("<I", b, 72, crc32c(b[:72]))
    decode_sensor(b)
    return bytes(b)


def decode_sensor(raw):
    if (
        len(raw) != 76
        or raw[:4] != b"SI\x01L"
        or struct.unpack_from("<I", raw, 72)[0] != crc32c(raw[:72])
    ):
        raise ValueError("sensor version/length/checksum")
    values = struct.unpack_from("<Q6I6fHBB2I", raw, 4)
    epoch, sequence, gyro_us, accel_us, gu, au, dropped, *tail = values
    gyro = tail[:3]
    accel = tail[3:6]
    flags, gs, acs, gd, ad = tail[6:]
    if (
        not epoch
        or flags & ~15
        or max(gd, ad) > 100000
        or not all(map(math.isfinite, gyro + accel))
    ):
        raise ValueError("invalid sensor fields")
    return dict(
        epoch=epoch,
        sequence=sequence,
        gyro_us=gyro_us,
        accel_us=accel_us,
        gyro_uncertainty_us=gu,
        accel_uncertainty_us=au,
        dropped=dropped,
        gyro=gyro,
        accel=accel,
        flags=flags,
        gyro_source=gs,
        accel_source=acs,
        gyro_delay_us=gd,
        accel_delay_us=ad,
    )


def decode_record(raw):
    if (
        len(raw) < 28
        or raw[:5] != b"SE01\x01"
        or struct.unpack_from("<H", raw, 6)[0] != len(raw)
        or struct.unpack_from("<I", raw, len(raw) - 4)[0] != crc32c(raw[:-4])
    ):
        raise ValueError("observation record version/length/checksum")
    kind = raw[5]
    sequence, received, dropped = struct.unpack_from("<IQI", raw, 8)
    p = raw[24:-4]
    result = dict(
        kind=kind, sequence=sequence, received_us=received, record_drops=dropped
    )
    if kind == 1:
        result["sensor"] = decode_sensor(p)
    elif kind == 2:
        if len(p) != 56 or struct.unpack_from("<I", p, 44)[0] != crc32c(p[:44]):
            raise ValueError("clock length/checksum")
        result["clock"] = dict(
            zip(
                ("epoch", "nonce", "t1", "t2", "t3", "sequence"),
                struct.unpack_from("<5QI", p),
            )
        )
        result["t4"] = struct.unpack_from("<Q", p, 48)[0]
    elif kind == 3:
        if len(p) != 228:
            raise ValueError("estimate length")
        result.update(
            zip(("epoch", "time_us", "flags", "fault"), struct.unpack_from("<QQII", p))
        )
        state = struct.unpack_from("<16f", p, 24)
        result.update(
            position=state[:3],
            velocity=state[3:6],
            quaternion=state[6:10],
            gyro_bias=state[10:13],
            accel_bias=state[13:16],
            covariance_diag=struct.unpack_from("<15f", p, 88),
        )
        result.update(
            zip(
                (
                    "event_drops",
                    "record_drops",
                    "max_task_us",
                    "fused",
                    "replayed",
                    "pending_replaced",
                    "clock_rejected",
                    "timing_rejected",
                    "static_memory_bytes",
                ),
                struct.unpack_from("<9I", p, 148),
            )
        )
        result["last_pose_us"], result["latest_imu_us"] = struct.unpack_from(
            "<QQ", p, 184
        )
        result["nis"] = struct.unpack_from("<f", p, 200)[0]
        result["innovation"] = struct.unpack_from("<6f", p, 204)
    elif kind == 4:
        if len(p) != 88:
            raise ValueError("pose length")
        result.update(
            zip(
                (
                    "epoch",
                    "source_ros_ns",
                    "capture_lower_us",
                    "capture_upper_us",
                    "session",
                    "pose_sequence",
                ),
                struct.unpack_from("<4Q2I", p),
            )
        )
        state = struct.unpack_from("<10f", p, 40)
        result.update(
            position=state[:3], quaternion=state[3:7], source_velocity=state[7:]
        )
    elif kind == 5:
        if len(p) != 80:
            raise ValueError("health length")
        result.update(
            rejected=struct.unpack_from("<11I", p),
            imu_age_us=struct.unpack_from("<Q", p, 44)[0],
            pose_age_us=struct.unpack_from("<Q", p, 52)[0],
        )
        result.update(
            zip(
                (
                    "max_replay",
                    "clock_uncertainty_us",
                    "propagated",
                    "history_count",
                    "remaining",
                ),
                struct.unpack_from("<5I", p, 60),
            )
        )
    elif kind == 6:
        if len(p) != 8:
            raise ValueError("epoch length")
        result["epoch"] = struct.unpack("<Q", p)[0]
    else:
        raise ValueError("unknown observation kind")
    return result


def frame_record(kind, payload, received_us, sequence=1, dropped=0):
    raw = bytearray(
        struct.pack(
            "<4sBBHIQI",
            b"SE01",
            1,
            kind,
            len(payload) + 28,
            sequence,
            received_us,
            dropped,
        )
        + payload
    )
    raw += struct.pack("<I", crc32c(raw))
    decode_record(raw)
    return bytes(raw)


def split_transport_records(payload):
    """Validate a legacy single record or bounded bundle before returning bytes."""
    raw=bytes(payload)
    if not 28 <= len(raw) <= 4096:
        raise ValueError("observation transport length")
    records=[];cursor=0
    while cursor<len(raw):
        if len(records)>=16 or len(raw)-cursor<28:
            raise ValueError("observation transport record count/truncation")
        size=struct.unpack_from('<H',raw,cursor+6)[0]
        if not 28<=size<=256 or cursor+size>len(raw):
            raise ValueError("observation transport record length")
        record=raw[cursor:cursor+size];decode_record(record)
        records.append(record);cursor+=size
    return records
