"""Finite-duration physical command example. Starts disabled unless --enable is supplied."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

from udp_broadcaster.ctbr_protocol import CtbrMapping
from sanity_zenoh.actuator import DirectThrustMapping
from sanity_zenoh.client import SanitySender
from sanity_zenoh.protocol import CTBR, DIRECT


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["ctbr", "direct_thrust"], default="ctbr")
    parser.add_argument("--drone-id", type=int, default=9)
    parser.add_argument("--listen", default="tcp/0.0.0.0:7447")
    parser.add_argument(
        "--connect", help="Optional existing Zenoh peer/router endpoint"
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--enable", action="store_true")
    parser.add_argument(
        "--bench-ready",
        action="store_true",
        help="Direct mode: secured bench setup with accessible power cut",
    )
    parser.add_argument("--thrust", type=float, default=0.0, help="CTBR collective N")
    parser.add_argument(
        "--rates", type=float, nargs=3, default=[0.0, 0.0, 0.0], help="FLU rad/s"
    )
    parser.add_argument(
        "--rotor-thrusts",
        type=float,
        nargs=4,
        default=[0.0] * 4,
        help="RR FR FL RL thrusts, N",
    )
    parser.add_argument("--ctbr-map", help="JSON object of CtbrMapping parameters")
    parser.add_argument("--actuator-model", help="Supplied schema-v2 actuators.json")
    parser.add_argument(
        "--model-to-output",
        type=int,
        nargs=4,
        help="Hardware outputs (1..4) corresponding to RR FR FL RL",
    )
    args = parser.parse_args(argv)
    if not 0 < args.duration <= 60:
        parser.error("duration must be in (0, 60] seconds")
    mode = CTBR if args.mode == "ctbr" else DIRECT
    if mode == DIRECT and args.enable and not args.bench_ready:
        parser.error("Enabled direct output requires --bench-ready")
    params = json.loads(Path(args.ctbr_map).read_text()) if args.ctbr_map else {}
    mapping = CtbrMapping(
        **{k: tuple(v) if isinstance(v, list) else v for k, v in params.items()}
    )
    if bool(args.actuator_model) != bool(args.model_to_output):
        parser.error("Supply both --actuator-model and --model-to-output")
    direct = (
        DirectThrustMapping(args.actuator_model, args.model_to_output)
        if args.actuator_model
        else None
    )
    # Validate the requested CTBR command before starting an enable sequence.
    if mode == CTBR:
        mapping.convert(args.drone_id, args.enable, args.thrust, tuple(args.rates))
    elif args.enable and any(args.rotor_thrusts) and direct is None:
        parser.error(
            "Positive direct thrust requires the actuator model and motor order"
        )
    sender = SanitySender(
        args.drone_id, mode, args.listen, args.connect, mapping, direct
    )

    def update(enabled, zero=False):
        if mode == CTBR:
            sender.ctbr(
                enabled,
                0.0 if zero else args.thrust,
                (0.0, 0.0, 0.0) if zero else args.rates,
            )
        else:
            sender.direct_thrust(enabled, (0.0,) * 4 if zero else args.rotor_thrusts)

    def phase(enabled, require_ack):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            update(enabled, True)
            status = sender.receiver_status()
            if (
                status
                and status.session == sender.session_id
                and status.enabled == enabled
                and not status.failsafe
            ):
                if not require_ack or mode != DIRECT or status.fc_ready:
                    return
            time.sleep(0.02)
        raise RuntimeError(
            "Receiver did not acknowledge the disable/zero-enable sequence"
        )

    try:
        deadline = time.monotonic() + 10
        while sender.receiver_status() is None:
            if time.monotonic() >= deadline:
                raise RuntimeError("No matching receiver status within 10 seconds")
            time.sleep(0.02)
        phase(False, False)
        if args.enable:
            if mode == DIRECT and any(args.rotor_thrusts):
                ready_deadline = time.monotonic() + 5
                while not ((status := sender.receiver_status()) and status.fc_ready):
                    update(False, True)
                    if time.monotonic() >= ready_deadline:
                        raise RuntimeError(
                            "FC did not become ready for a disarmed bench test"
                        )
                    time.sleep(0.02)
                direct.convert(args.rotor_thrusts, status.voltage_mv / 1000)
            phase(True, True)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            update(args.enable)
            status = sender.receiver_status()
            if status is None:
                raise RuntimeError("Receiver connection lost")
            time.sleep(0.02)
        print(json.dumps(asdict(status), indent=2))
    except KeyboardInterrupt:
        pass
    finally:
        sender.close()


if __name__ == "__main__":
    main()
