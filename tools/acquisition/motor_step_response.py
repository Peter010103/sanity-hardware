#!/usr/bin/env python3
"""Record one supervised collective command step with HX711 and Betaflight telemetry.

This is a low-rate bench logger, not a fast transient acquisition system. On the
current Grove HX711 10 SPS configuration, expect roughly 0.1 s samples.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "hx711_jetson"))

from betaflight_msp import (USBSerial, MSP, MSPError, battery, inspect_controller,
                            motor_telemetry, status)
from betaflight_motors import motor_packet, require_disarmed, validate_setup
from hx711_jetson import RAILS, RecoveringHX711, force_from_counts, load_calibration, TimingError
from hx711_native import LIBRARY, NativeHX711, supported_board

STOP = 1000
MOTORS = ("M1", "M2", "M3", "M4")


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def command_value(value):
    number = int(value)
    if not 1000 <= number <= 2000:
        raise argparse.ArgumentTypeError("must be a Betaflight external command from 1000 to 2000")
    return number


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--profile", required=True)
    p.add_argument("--calibration", required=True)
    p.add_argument("--start-command", required=True, type=command_value,
                   help="steady collective command before the step")
    p.add_argument("--step-command", required=True, type=command_value,
                   help="new steady command after the step; must differ from start")
    p.add_argument("--pre-step-seconds", type=positive_float, default=3.0)
    p.add_argument("--post-step-seconds", type=positive_float, default=4.0)
    p.add_argument("--force-limit-n", required=True, type=positive_float,
                   help="stop if absolute change from stopped baseline exceeds this")
    p.add_argument("--thrust-sign", required=True, type=int, choices=(-1, 1))
    p.add_argument("--output", help="new output directory; defaults to timestamped name")
    p.add_argument("--execute", action="store_true", help="required to send motor-test commands")
    p.add_argument("--bench-ready", action="store_true",
                   help="confirm props installed, frame secured on thrust stand, and battery cutoff reachable")
    return p


def create_sensor():
    if not LIBRARY.is_file() or not supported_board():
        raise RuntimeError("HX711 native GPIO backend unavailable on this Jetson")
    try:
        import Jetson.GPIO as GPIO
    except ImportError as exc:
        raise RuntimeError("Jetson.GPIO is required") from exc
    sensor = RecoveringHX711(NativeHX711(timing_error=TimingError), retries=3)
    info = {"backend": "native_gpio_cdev", "gpio_version": GPIO.VERSION,
            "jetson_info": GPIO.JETSON_INFO, "channel": "A", "gain": 128,
            "physical_pins": {"dout": 29, "sck": 31}, "nominal_rate_hz": 10}
    return sensor, info


def safe_json(path, data):
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)


class Capture:
    def __init__(self, args, client, sensor, calibration, profile, live, box_ids,
                 directory, sensor_info):
        self.args, self.client, self.sensor = args, client, sensor
        self.calibration, self.profile, self.live, self.box_ids = calibration, profile, live, box_ids
        self.directory = directory
        self.may_have_sent = False
        self.stop_acks = 0
        self.sequence = 0
        self.started_mono_ns = time.monotonic_ns()
        self.csv_file = open(directory / "samples.csv", "x", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=[
            "sequence", "phase", "command", "command_send_monotonic_ns", "sample_monotonic_ns",
            "sample_unix_time_ns", "elapsed_s", "hx_unix_time_ns", "hx_raw_counts", "load_N",
            "thrust_delta_N", "collective_thrust_N", "battery_voltage_V", "battery_current_A",
            "battery_state_code", "fc_armed", "hx_clock_bound_us", "timing_rejections_total",
            "dshot_payload_hex", *[f"rpm_{m}" for m in MOTORS],
            *[f"esc_voltage_V_{m}" for m in MOTORS], *[f"esc_current_A_{m}" for m in MOTORS],
            *[f"esc_temperature_C_{m}" for m in MOTORS]])
        self.writer.writeheader()
        safe_json(directory / "metadata.json", {
            "description": "Collective motor-command step response using FC motor telemetry and load cell",
            "commands_units": "Betaflight MSP external motor command counts; not RPM, PWM microseconds, or newtons",
            "motor_order": list(MOTORS), "stop_command": STOP,
            "step_time_definition": "host monotonic timestamp immediately before first acknowledged step-command packet",
            "channels_sequential_not_simultaneous": True,
            "hx711_nominal_rate_hz": 10,
            "sampling_limit": "approximately 0.1 s minimum interval; sub-0.3 s delays are not resolved reliably",
            "setup_operator_confirmed": "props installed; aircraft secured on thrust stand; physical battery cut reachable",
            "configuration": {"start_command": args.start_command, "step_command": args.step_command,
                              "pre_step_seconds": args.pre_step_seconds,
                              "post_step_seconds": args.post_step_seconds,
                              "force_limit_N": args.force_limit_n, "thrust_sign": args.thrust_sign},
            "sensor": sensor_info,
            "calibration_file": str(Path(args.calibration).name),
            "calibration_force_range_N": calibration.get("measured_force_range_N",
                calibration.get("fit", {}).get("measured_force_range_N")),
            "flight_controller": live,
            "interpretation": "Measured collective thrust is not an individual motor curve. RPM is FC-reported mechanical RPM if supported. This is a bench characterization, not sim-to-real validation."
        })
        self.baseline = None
        self.status = "ready"
        self.error = None
        self.last_battery = None

    def send(self, value):
        cfg = self.live["motor_config"]
        packet = motor_packet([value] * 4, 4, STOP, cfg["max_throttle_setting"], 2000)
        self.may_have_sent = True
        sent_ns = time.monotonic_ns()
        self.client._exchange(packet, 214)
        return sent_ns

    def stop(self):
        cfg = self.live["motor_config"]
        packet = motor_packet([STOP] * 4, 4, STOP, cfg["max_throttle_setting"], cfg["max_throttle_setting"])
        any_ack = False
        for _ in range(3):
            try:
                self.client._exchange(packet, 214)
                self.stop_acks += 1
                any_ack = True
            except KeyboardInterrupt:
                continue
            except (OSError, MSPError, TimeoutError) as exc:
                print(f"Stop packet failed: {exc}", file=sys.stderr, flush=True)
        return any_ack

    def sample_once(self, phase, command, send_ns=None):
        live_status = status(self.client.query(101), self.box_ids)
        if live_status["armed"] is not False:
            raise RuntimeError("FC armed state is unknown or armed")
        batt = battery(self.client.query(130))
        self.last_battery = batt
        if batt["detected_cells"] not in (2, 3) or batt["battery_state_code"] != 0:
            raise RuntimeError(f"Battery state changed; refusing to continue: {batt}")
        # Reissue the desired motor command to maintain it through the FC motor-test interface.
        if send_ns is None:
            send_ns = self.send(command)
        try:
            telemetry = motor_telemetry(self.client.query(139), 4)
        except MSPError as exc:
            if "FC rejected/does not support" in str(exc):
                telemetry = None
            else:
                raise
        hx = self.sensor.read()
        raw = hx["raw_counts"]
        load = None if raw in RAILS else force_from_counts(raw, self.calibration)
        delta = None if load is None or self.baseline is None else load - self.baseline
        thrust = None if delta is None else self.args.thrust_sign * delta
        limits = self.calibration.get("measured_force_range_N") or self.calibration.get("fit", {}).get("measured_force_range_N")
        if raw in RAILS:
            raise RuntimeError("HX711 reached ADC rail")
        if limits and (load < limits[0] or load > limits[1]):
            raise RuntimeError(f"Load reading outside calibration range {limits} N")
        if thrust is not None and abs(thrust) > self.args.force_limit_n:
            raise RuntimeError("Configured force limit exceeded")
        if telemetry is not None:
            motor_cols = {}
            for name, esc in zip(MOTORS, telemetry["motors"]):
                motor_cols[f"rpm_{name}"] = esc["rpm"]
                motor_cols[f"esc_voltage_V_{name}"] = esc["esc_voltage_V"]
                motor_cols[f"esc_current_A_{name}"] = esc["esc_current_A"]
                motor_cols[f"esc_temperature_C_{name}"] = esc["esc_temperature_C"]
            payload_hex = telemetry["raw_payload_hex"]
        else:
            motor_cols = {key: "" for m in MOTORS for key in
                          (f"rpm_{m}", f"esc_voltage_V_{m}", f"esc_current_A_{m}", f"esc_temperature_C_{m}")}
            payload_hex = ""
        now_ns = time.monotonic_ns()
        row = {"sequence": self.sequence, "phase": phase, "command": command,
               "command_send_monotonic_ns": send_ns, "sample_monotonic_ns": now_ns,
               "sample_unix_time_ns": time.time_ns(), "elapsed_s": (now_ns-self.started_mono_ns)/1e9,
               "hx_unix_time_ns": hx.get("unix_time_ns"), "hx_raw_counts": raw, "load_N": load,
               "thrust_delta_N": delta, "collective_thrust_N": thrust,
               "battery_voltage_V": batt["voltage_V"], "battery_current_A": batt["current_A"],
               "battery_state_code": batt["battery_state_code"], "fc_armed": live_status["armed"],
               "hx_clock_bound_us": hx.get("max_clock_high_bound_us"),
               "timing_rejections_total": hx.get("timing_rejections_total", 0),
               "dshot_payload_hex": payload_hex, **motor_cols}
        self.writer.writerow(row)
        self.csv_file.flush()
        os.fsync(self.csv_file.fileno())
        self.sequence += 1
        rpm = "n/a" if telemetry is None else ",".join(str(x["rpm"]) for x in telemetry["motors"])
        print(f"{phase}: command={command} load={load if load is not None else 'n/a'} N RPM={rpm} "
              f"battery={batt['voltage_V']:.2f} V/{batt['current_A']:.2f} A", flush=True)

    def collect(self, phase, command, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            # The event is timestamped by first step command packet, after command preflight checks.
            self.sample_once(phase, command)
            time.sleep(0.002)

    def run(self):
        print("Starting with STOP command; keep the physical battery cutoff reachable.", flush=True)
        if not self.stop():
            raise RuntimeError("Could not establish acknowledged motor stop")
        input("Verify fixture, propellers, and load-cell zero. Press Enter to continue: ")
        # Baseline is stopped fixture reading; command is held only during the two response phases.
        baseline_values = []
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            armed = status(self.client.query(101), self.box_ids)
            if armed["armed"] is not False:
                raise RuntimeError("FC armed or state unknown")
            hx = self.sensor.read()
            if hx["raw_counts"] in RAILS:
                raise RuntimeError("HX711 reached ADC rail while reading baseline")
            baseline_values.append(force_from_counts(hx["raw_counts"], self.calibration))
        if len(baseline_values) < 3:
            raise RuntimeError("Too few stopped baseline samples")
        self.baseline = statistics.mean(baseline_values)
        print(f"Stopped-load baseline: {self.baseline:.4f} N from {len(baseline_values)} samples", flush=True)
        print(f"Holding {self.args.start_command} for {self.args.pre_step_seconds:g}s, then stepping to "
              f"{self.args.step_command} for {self.args.post_step_seconds:g}s.", flush=True)
        self.collect("pre_step", self.args.start_command, self.args.pre_step_seconds)
        # Send the change now; timestamp is immediately before the packet write/ack.
        event_ns = self.send(self.args.step_command)
        self.sample_once("post_step", self.args.step_command, event_ns)
        self.collect("post_step", self.args.step_command, self.args.post_step_seconds)
        self.status = "completed"

    def close(self):
        self.csv_file.close()
        safe_json(self.directory / "summary.json", {
            "status": self.status, "error": self.error, "samples_written": self.sequence,
            "stop_acknowledgements": self.stop_acks, "physical_spin_down_verified": False,
            "last_battery_telemetry": self.last_battery,
            "stopped_baseline_N": self.baseline,
            "nominal_force_sampling_hz": 10,
            "interpretation_limit": "The existing 10 SPS HX711 and sequential FC polling cannot robustly measure sub-0.3 s response delays.",
            "data_file": "samples.csv"})


def main(argv=None):
    args = parser().parse_args(argv)
    if not args.execute or not args.bench_ready:
        print("ERROR: motor step requires --execute --bench-ready; dry-run only checks arguments.", file=sys.stderr)
        return 2
    if args.start_command == STOP or args.step_command == STOP or args.start_command == args.step_command:
        print("ERROR: start and step commands must be distinct running commands above 1000", file=sys.stderr)
        return 2
    directory = Path(args.output or time.strftime("motor_step_%Y%m%d_%H%M%S", time.gmtime()))
    directory.mkdir(parents=True, exist_ok=False)
    transport = client = sensor = capture = None
    result = 0
    try:
        profile = json.loads(Path(args.profile).read_text())
        calibration = load_calibration(args.calibration)
        force_range = calibration.get("measured_force_range_N") or calibration.get("fit", {}).get("measured_force_range_N")
        if not force_range or len(force_range) != 2 or not force_range[0] <= 0 <= force_range[1]:
            raise ValueError("Calibration must contain a measured force range that includes zero")
        if args.force_limit_n > force_range[1] - force_range[0]:
            raise ValueError("Force limit exceeds the calibrated force span")
        sensor, sensor_info = create_sensor()
        transport = USBSerial(args.port)
        client = MSP(transport, timeout=0.5)
        live = inspect_controller(client)
        validate_setup(live, profile)
        if max(args.start_command, args.step_command) > live["motor_config"]["max_throttle_setting"]:
            raise ValueError("A requested command exceeds the live FC configured throttle limit")
        box_ids = list(client.query(119))
        require_disarmed(client, box_ids)
        initial_battery = battery(client.query(130))
        if initial_battery["detected_cells"] not in (2, 3) or initial_battery["battery_state_code"] != 0:
            raise RuntimeError(f"Battery preflight failed: {initial_battery}")
        capture = Capture(args, client, sensor, calibration, profile, live, box_ids, directory, sensor_info)
        capture.last_battery = initial_battery
        print("Connected, disarmed, battery state OK. This logger samples at ~10 Hz; it cannot resolve a very short delay.", flush=True)
        capture.run()
    except KeyboardInterrupt:
        result = 130
        if capture:
            capture.status = "interrupted_by_user"
    except EOFError:
        if capture:
            capture.status = "operator_input_closed"
    except BaseException as exc:
        result = 1
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        if capture:
            capture.status = "failed"
            capture.error = f"{type(exc).__name__}: {exc}"
    finally:
        if capture:
            if capture.may_have_sent:
                if not capture.stop():
                    print("STOP NOT ACKNOWLEDGED: disconnect motor battery now.", file=sys.stderr, flush=True)
                    result = 1
                else:
                    print("FC acknowledged stop; confirm physical rotor stop.", file=sys.stderr, flush=True)
            capture.close()
        if sensor:
            sensor.close()
        if transport:
            transport.close()
        print(f"Step-response data directory: {directory}", flush=True)
    return result


if __name__ == "__main__":
    sys.exit(main())
