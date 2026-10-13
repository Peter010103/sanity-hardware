#!/usr/bin/env python3
"""Supervised, finite-duration Betaflight motor bench test; defaults to dry run."""
import argparse
import functools
import json
from pathlib import Path
import signal
import struct
import sys
import time

from betaflight_msp import USBSerial, MSP, MSPError, inspect_controller, status, battery, positive


def validate_setup(live, profile):
    identity = live["identity"]
    if (identity["firmware_version"], identity["msp_api_version"]) != ("4.5.1", "1.46"):
        raise ValueError("This bench implementation was checked for Betaflight 4.5.1 / API 1.46")
    if live["board"]["board_name"] != "BETAFPVF405":
        raise ValueError("Expected the identified BETAFPVF405 board")
    for key in ("uid_payload_hex", "identity", "board", "motor_config", "features", "motor_protocol"):
        if live[key] != profile[key]:
            raise ValueError("Live controller/configuration differs from profile: " + key)
    if live["features"]["reversible_3d_enabled"]:
        raise ValueError("3D motor mode is unsupported by this bench tool")
    if live["motor_protocol"]["name"] != "DSHOT300":
        raise ValueError("Expected the identified DSHOT300 protocol")
    motors = live["motor_config"]
    if (motors["motor_count"], motors["min_command_setting"], motors["max_throttle_setting"]) != (4, 1000, 2000):
        raise ValueError("Unexpected motor count, stop value or external command range")
    return 4, 1000, 2000


def motor_packet(values, count, stop, maximum, ceiling):
    if len(values) != count:
        raise ValueError("Provide exactly %d FC motor outputs" % count)
    if not stop <= ceiling <= maximum:
        raise ValueError("Ceiling must lie within the identified external command range")
    if any(not isinstance(v, int) or not stop <= v <= ceiling for v in values):
        raise ValueError("Motor commands must be integers between stop and your ceiling")
    payload = struct.pack("<" + "H" * count, *values)
    body = bytes((len(payload), 214)) + payload
    checksum = functools.reduce(lambda a, b: a ^ b, body, 0)
    return b"$M<" + body + bytes((checksum,))


def require_disarmed(client, box_ids):
    current = status(client.query(101), box_ids)
    if current["armed"] is not False:
        raise RuntimeError("FC must be positively identified as disarmed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--profile", required=True, help="read-only --inspect JSON from this exact FC")
    parser.add_argument("--values", nargs=4, type=int, required=True, metavar="COMMAND")
    parser.add_argument("--ceiling", type=int, required=True, help="your chosen maximum for this supervised run")
    parser.add_argument("--duration", type=positive, required=True, help="seconds, maximum 60")
    parser.add_argument("--execute", action="store_true", help="actually send motor-test commands")
    parser.add_argument("--bench-ready", action="store_true",
                        help="confirm props removed OR secured thrust rig, plus accessible power cut")
    args = parser.parse_args()
    if args.duration > 60:
        parser.error("A supervised run is limited to 60 seconds; this is a tool limit, not a motor specification")
    if args.execute and not args.bench_ready:
        parser.error("Powered execution requires --bench-ready; read README_BETAFLIGHT.md")
    transport = None
    stop_packet = None
    may_have_sent = False
    exit_code = 0
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        profile = json.loads(Path(args.profile).read_text())
        transport = USBSerial(args.port)
        client = MSP(transport, timeout=0.5)
        live = inspect_controller(client)
        count, stop, maximum = validate_setup(live, profile)
        desired = motor_packet(args.values, count, stop, maximum, args.ceiling)
        stop_packet = motor_packet([stop] * count, count, stop, maximum, maximum)
        box_ids = list(client.query(119))
        require_disarmed(client, box_ids)
        print(json.dumps({"mode": "execute" if args.execute else "dry_run",
                          "FC_output_order": ["M1", "M2", "M3", "M4"],
                          "external_commands": args.values, "stop_commands": [stop] * count,
                          "duration_s": args.duration, "units": "MSP external motor command counts; not RPM or N"}), flush=True)
        if not args.execute:
            return 0
        batt = battery(client.query(130))
        if batt["detected_cells"] not in (2, 3):
            raise RuntimeError("FC must detect the board's supported 2S/3S motor battery before execution")
        print("Bench motor test starting. USB/process failure may retain motor commands; keep the power cut accessible.",
              file=sys.stderr, flush=True)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            began = time.monotonic()
            require_disarmed(client, box_ids)
            may_have_sent = True  # even a failed ACK may follow an accepted command
            client._exchange(desired, 214)
            batt = battery(client.query(130))
            print(json.dumps({"host_monotonic_ns": time.monotonic_ns(), "host_unix_time_ns": time.time_ns(),
                              "requested_MSP_motor_commands": args.values, "battery": batt}), flush=True)
            time.sleep(max(0, min(0.1 - (time.monotonic() - began), deadline - time.monotonic())))
    except KeyboardInterrupt:
        exit_code = 130
    except (OSError, ValueError, KeyError, MSPError) as exc:
        print("ERROR: " + str(exc), file=sys.stderr)
        exit_code = 1
    except RuntimeError as exc:
        print("ERROR: " + str(exc), file=sys.stderr)
        exit_code = 1
    finally:
        if may_have_sent and transport is not None:
            stopped = False
            for _ in range(3):
                try:
                    client._exchange(stop_packet, 214)
                    stopped = True
                except (OSError, MSPError) as exc:
                    print("STOP DELIVERY ERROR: " + str(exc), file=sys.stderr, flush=True)
            print("Stop command acknowledged; verify physical stop." if stopped else
                  "STOP NOT CONFIRMED: cut motor battery power now.", file=sys.stderr, flush=True)
            if not stopped:
                exit_code = 1
        if transport is not None:
            transport.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
