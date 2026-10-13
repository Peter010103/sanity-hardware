#!/usr/bin/env python3
"""Read-only Betaflight MSP v1 USB telemetry, using Python's Linux standard library."""
import argparse
import functools
import glob
import json
import os
from pathlib import Path
import select
import struct
import sys
import termios
import time

COMMANDS = {1: "API_VERSION", 2: "FC_VARIANT", 3: "FC_VERSION", 4: "BOARD_INFO", 5: "BUILD_INFO",
            36: "FEATURE_CONFIG", 42: "MIXER_CONFIG", 90: "ADVANCED_CONFIG",
            101: "STATUS", 104: "MOTOR", 108: "ATTITUDE", 110: "ANALOG",
            119: "BOXIDS", 124: "MOTOR_3D_CONFIG", 130: "BATTERY_STATE", 131: "MOTOR_CONFIG",
            139: "MOTOR_TELEMETRY", 160: "UID"}


class MSPError(RuntimeError):
    pass


def request_frame(command):
    if command not in COMMANDS:
        raise ValueError("Only read-only telemetry queries are permitted")
    return b"$M<" + bytes((0, command, command))


def ports():
    result = []
    for device in sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))):
        links = [str(p) for p in Path("/dev/serial/by-id").glob("*")
                 if p.resolve() == Path(device).resolve()]
        result.append({"device": device, "stable_paths": links,
                       "readable": os.access(device, os.R_OK),
                       "writable": os.access(device, os.W_OK)})
    return result


class USBSerial:
    """115200, 8N1, raw tty; no pyserial dependency, no motor/set commands."""
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self.fd)
            self.original = attrs
            attrs = list(attrs)
            attrs[6] = list(attrs[6])
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except BaseException:
            os.close(self.fd)
            raise

    def write(self, packet, deadline):
        view = memoryview(packet)
        while view:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                raise TimeoutError("USB write timeout")
            try:
                count = os.write(self.fd, view)
            except BlockingIOError:
                continue
            if count == 0:
                raise ConnectionError("USB write made no progress")
            view = view[count:]

    def read(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([self.fd], [], [], remaining)[0]:
            raise TimeoutError("No MSP reply: check port, MSP configuration and other serial clients")
        try:
            data = os.read(self.fd, 1024)
        except BlockingIOError:
            return b""
        if not data:
            raise ConnectionError("USB disconnected or tty closed")
        return data

    def close(self):
        if self.fd is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
            except termios.error:
                pass
            os.close(self.fd)
            self.fd = None


class MSP:
    def __init__(self, transport, timeout=1.0):
        self.transport, self.timeout = transport, timeout
        self.buffer = bytearray()

    def query(self, command):
        packet = request_frame(command)
        return self._exchange(packet, command)

    def _exchange(self, packet, command):
        """Shared frame exchange; the telemetry query interface remains read-only."""
        deadline = time.monotonic() + self.timeout
        self.transport.write(packet, deadline)
        while True:
            while len(self.buffer) >= 3:
                start = self.buffer.find(b"$M")
                if start < 0:
                    del self.buffer[:-1]
                    break
                if start:
                    del self.buffer[:start]
                if len(self.buffer) < 5:
                    break
                if self.buffer[2] not in (ord(">"), ord("!")):
                    del self.buffer[0]
                    continue
                size, reply_command = self.buffer[3:5]
                if size == 255:
                    raise MSPError("Jumbo MSP frames are unsupported by this small telemetry client")
                if len(self.buffer) < size + 6:
                    break
                frame = bytes(self.buffer[:size + 6])
                del self.buffer[:size + 6]
                checksum = functools.reduce(lambda a, b: a ^ b, frame[3:-1], 0)
                if checksum != frame[-1] or reply_command != command:
                    continue
                if frame[2] == ord("!"):
                    raise MSPError("FC rejected/does not support " + COMMANDS.get(command, str(command)))
                return frame[5:-1]
            self.buffer.extend(self.transport.read(deadline))


def need(payload, minimum, label):
    if len(payload) < minimum:
        raise MSPError("Short %s response: %d bytes" % (label, len(payload)))


def battery(payload):
    need(payload, 9, "BATTERY_STATE")
    cells, capacity, legacy_voltage, consumed, current, state = struct.unpack_from("<BHBHhB", payload)
    voltage = struct.unpack_from("<H", payload, 9)[0] / 100 if len(payload) >= 11 else legacy_voltage / 10
    return {"detected_cells": cells, "voltage_V": voltage, "current_A": current / 100,
            "consumed_mAh": consumed, "configured_capacity_mAh": capacity,
            "battery_state_code": state,
            "voltage_resolution_V": 0.01 if len(payload) >= 11 else 0.1,
            "source": "FC-configured voltage/current meters; calibration not independently verified"}


def motor_telemetry(payload, expected_count=4):
    """Decode MSP_MOTOR_TELEMETRY (139): per motor RPM, invalid %, and ESC fields."""
    if not payload:
        raise MSPError("Empty MOTOR_TELEMETRY response")
    count = payload[0]
    record_size = 13  # U32 rpm, U16 invalid%, U8 temp, U16 V, U16 A, U16 mAh
    if count != expected_count or len(payload) != 1 + count * record_size:
        raise MSPError("Unexpected MOTOR_TELEMETRY payload: count=%d bytes=%d" % (count, len(payload)))
    motors = []
    for i in range(count):
        rpm, invalid_hundredths, temperature, voltage_centi, current_centi, consumption = \
            struct.unpack_from("<IHBHHH", payload, 1 + i * record_size)
        motors.append({"rpm": rpm, "telemetry_invalid_percent": invalid_hundredths / 100.0,
                       "esc_temperature_C": temperature, "esc_voltage_V": voltage_centi / 100.0,
                       "esc_current_A": current_centi / 100.0,
                       "esc_consumption_mAh": consumption})
    return {"motor_count": count, "motors": motors, "raw_payload_hex": payload.hex()}


def motor_config(payload):
    need(payload, 6, "MOTOR_CONFIG")
    minimum, maximum, stop = struct.unpack_from("<HHH", payload)
    return {"min_throttle_setting": minimum, "max_throttle_setting": maximum,
            "min_command_setting": stop,
            "motor_count": payload[6] if len(payload) > 6 else None,
            "configured_motor_poles": payload[7] if len(payload) > 7 else None,
            "dshot_telemetry_enabled": bool(payload[8]) if len(payload) > 8 else None,
            "esc_sensor_feature": bool(payload[9]) if len(payload) > 9 else None,
            "raw_payload_hex": payload.hex()}


def board_info(payload):
    need(payload, 9, "BOARD_INFO")
    info = {"board_identifier": payload[:4].decode("ascii", errors="replace"),
            "hardware_revision": struct.unpack_from("<H", payload, 4)[0]}
    offset = 8
    for name in ("target_name", "board_name", "manufacturer_id"):
        need(payload, offset + 1, "BOARD_INFO")
        length = payload[offset]
        need(payload, offset + 1 + length, "BOARD_INFO")
        info[name] = payload[offset + 1:offset + 1 + length].decode("ascii", errors="replace")
        offset += 1 + length
    info["raw_payload_hex"] = payload.hex()
    return info


def inspect_controller(client):
    info = {"identity": identify(client), "board": board_info(client.query(4)),
            "motor_config": motor_config(client.query(131))}
    feature = client.query(36)
    need(feature, 4, "FEATURE_CONFIG")
    mask = struct.unpack_from("<I", feature)[0]
    advanced = client.query(90)
    need(advanced, 8, "ADVANCED_CONFIG")
    protocols = ["PWM", "ONESHOT125", "ONESHOT42", "MULTISHOT", "BRUSHED",
                 "DSHOT150", "DSHOT300", "DSHOT600", "PROSHOT1000", "DISABLED"]
    code = advanced[3]
    info["features"] = {"mask": mask, "reversible_3d_enabled": bool(mask & (1 << 12))}
    info["motor_protocol"] = {"code": code, "name": protocols[code] if code < len(protocols) else "UNKNOWN",
                              "raw_advanced_payload_hex": advanced.hex()}
    for command in (5, 42, 124):
        info[COMMANDS[command]] = {"raw_payload_hex": client.query(command).hex()}
    uid = client.query(160)
    need(uid, 12, "UID")
    info["uid_payload_hex"] = uid.hex()
    return info


def status(payload, box_ids):
    need(payload, 11, "STATUS")
    cycle, i2c, sensors, mode_flags, profile = struct.unpack_from("<HHHIB", payload)
    armed = None
    if box_ids is not None and 0 in box_ids:
        arm_bit = box_ids.index(0)  # BOXARM permanent ID; status bit positions are dynamic.
        if arm_bit < 32:
            armed = bool(mode_flags & (1 << arm_bit))
    return {"pid_cycle_time_us": cycle, "i2c_error_count": i2c,
            "sensor_mask": sensors, "mode_flags_first32": mode_flags,
            "pid_profile_index": profile, "armed": armed,
            "raw_payload_hex": payload.hex()}


def identify(client):
    api, variant, version = client.query(1), client.query(2), client.query(3)
    need(api, 3, "API_VERSION")
    need(version, 3, "FC_VERSION")
    variant = variant.decode("ascii", errors="replace")
    if variant != "BTFL":
        raise MSPError("Expected Betaflight BTFL identifier; received " + repr(variant))
    if api[1] != 1:
        raise MSPError("Unsupported MSP API major: %d; inspect protocol before decoding" % api[1])
    return {"firmware_identifier": variant,
            "firmware_version": ".".join(str(x) for x in version[:3]),
            "msp_protocol_version": api[0], "msp_api_version": "%d.%d" % (api[1], api[2])}


def positive(value):
    x = float(value)
    if not 0 < x < float("inf"):
        raise argparse.ArgumentTypeError("must be finite and positive")
    return x


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    sub.add_parser("list", help="list USB serial paths without opening them")
    p = sub.add_parser("status", help="read FC identity, battery and controller status")
    p.add_argument("--port", required=True, help="prefer /dev/serial/by-id/... path")
    p.add_argument("--rate", type=positive, default=2.0, help="requested polls/s; not a sensor update rate")
    p.add_argument("--timeout", type=positive, default=1.0)
    p.add_argument("--duration", type=positive)
    p.add_argument("--once", action="store_true")
    p.add_argument("--inspect", action="store_true", help="print full read-only controller configuration and exit")
    args = parser.parse_args()
    if args.mode == "list":
        print(json.dumps(ports(), indent=2))
        return 0
    transport = None
    try:
        transport = USBSerial(args.port)
        client = MSP(transport, args.timeout)
        if args.inspect:
            print(json.dumps(inspect_controller(client), indent=2, sort_keys=True), flush=True)
            return 0
        print(json.dumps({"identity": identify(client)}, sort_keys=True), flush=True)
        try:
            box_ids = list(client.query(119))
        except MSPError:
            box_ids = None
        for command in (4, 131):
            try:
                payload = client.query(command)
                decoded = board_info(payload) if command == 4 else motor_config(payload)
                print(json.dumps({COMMANDS[command]: decoded}, sort_keys=True), flush=True)
            except MSPError as exc:
                print("NOTICE: " + str(exc), file=sys.stderr)
        start = time.monotonic()
        while True:
            began = time.monotonic()
            stat = status(client.query(101), box_ids)
            batt = battery(client.query(130))
            record = {"host_unix_time_ns": time.time_ns(), "host_monotonic_ns": time.monotonic_ns(),
                      "request_to_reply_s": time.monotonic() - began,
                      "status": stat, "battery": batt}
            print(json.dumps(record, sort_keys=True), flush=True)
            if args.once or (args.duration is not None and time.monotonic() - start >= args.duration):
                break
            time.sleep(max(0, 1 / args.rate - (time.monotonic() - began)))
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, MSPError, ValueError) as exc:
        print("ERROR: " + str(exc), file=sys.stderr)
        return 1
    finally:
        if transport is not None:
            transport.close()


if __name__ == "__main__":
    sys.exit(main())
