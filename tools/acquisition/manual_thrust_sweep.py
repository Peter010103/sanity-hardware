#!/usr/bin/env python3
"""Interactive four-rotor thrust sweep with HX711 and Betaflight telemetry."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time
import uuid
_SCRIPT_DIR = Path(__file__).resolve().parent
for _module_dir in (_SCRIPT_DIR, _SCRIPT_DIR.parent / "hx711_jetson"):
    if (_module_dir / "hx711_jetson.py").is_file():
        sys.path.insert(0, str(_module_dir))
        break

from betaflight_msp import (USBSerial, MSP, MSPError, battery, inspect_controller,
                            motor_telemetry, status)
from betaflight_motors import motor_packet, require_disarmed, validate_setup
from hx711_jetson import RAILS, RecoveringHX711, force_from_counts, load_calibration, TimingError
from hx711_native import LIBRARY, NativeHX711, supported_board


STOP_VALUE = 1000  # Betaflight external command mapped to DShot stop (DShot value 0).
MOTOR_ORDER = ["M1", "M2", "M3", "M4"]
G0 = 9.80665


def positive_float(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return value


def positive_int(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return value


def make_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True, help="FC serial path, preferably /dev/serial/by-id/...")
    p.add_argument("--profile", required=True, help="read-only FC profile from betaflight_msp.py --inspect")
    p.add_argument("--calibration", required=True, help="HX711 multi-weight calibration.json")
    p.add_argument("--output", help="new run directory; default thrust_sweep_YYYYmmdd_HHMMSS")
    p.add_argument("--step", type=positive_int, default=100, help="external FC command-count increment (default 100)")
    p.add_argument("--interval", type=positive_float, default=3.0,
                   help="seconds to hold and sample each command (default 3)")
    p.add_argument("--baseline-seconds", type=positive_float, default=3.0,
                   help="stopped baseline collection time (default 3)")
    p.add_argument("--max-command", type=positive_int, default=2000,
                   help="last command; cannot exceed the FC configured maximum (default 2000)")
    p.add_argument("--force-limit-n", type=positive_float, required=True,
                   help="maximum absolute thrust delta allowed for this run; choose below rig/cell limits")
    p.add_argument("--thrust-sign", type=int, choices=(-1, 1), required=True,
                   help="+1 if thrust increases cell load; -1 if thrust decreases cell load")
    p.add_argument("--execute", action="store_true", help="required: send motor-test commands")
    p.add_argument("--bench-ready", action="store_true",
                   help="confirm props installed, aircraft secured on thrust stand, physical battery cut reachable")
    return p


def write_json(path, data):
    temp = path.with_suffix(path.suffix + ".tmp")
    with open(temp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, allow_nan=False)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)


def new_run_directory(requested):
    if requested:
        path = Path(requested)
    else:
        path = Path(time.strftime("thrust_sweep_%Y%m%d_%H%M%S", time.gmtime()))
    path.mkdir(parents=True, exist_ok=False)
    return path


def create_sensor():
    if not LIBRARY.is_file() or not supported_board():
        raise RuntimeError("HX711 native backend unavailable; verified Jetson/P3768 helper required")
    try:
        import Jetson.GPIO as GPIO
    except ImportError as exc:
        raise RuntimeError("Jetson.GPIO is required to record target hardware metadata") from exc
    sensor = RecoveringHX711(NativeHX711(timing_error=TimingError), retries=3)
    info = {"backend": "native_gpio_cdev", "gpio_version": GPIO.VERSION,
            "jetson_info": GPIO.JETSON_INFO, "channel": "A", "gain": 128,
            "physical_pins": {"dout": 29, "sck": 31}}
    return sensor, info


class Sweep:
    def __init__(self, args, directory, client, transport, box_ids, sensor, calibration,
                 fc_profile, live_fc, sensor_info, dshot_telemetry_supported):
        self.args, self.directory = args, directory
        self.client, self.transport, self.box_ids = client, transport, box_ids
        self.sensor, self.calibration = sensor, calibration
        self.fc_profile, self.live_fc, self.sensor_info = fc_profile, live_fc, sensor_info
        self.dshot_telemetry_supported = dshot_telemetry_supported
        self.csv_path = directory / "samples.csv"
        self.csv_file = open(self.csv_path, "x", newline="", encoding="utf-8")
        self.fields = [
            "sequence", "stage", "stage_command", "motor_command_M1", "motor_command_M2",
            "motor_command_M3", "motor_command_M4", "command_units", "hx_unix_time_ns",
            "fc_unix_time_ns", "row_unix_time_ns", "elapsed_s", "hx_raw_counts",
            "hx_force_N", "baseline_force_N", "force_delta_from_baseline_N", "collective_thrust_N",
            "mass_equivalent_g", "battery_cells", "battery_voltage_V", "battery_current_A",
            "battery_consumed_mAh", "battery_state_code", "fc_armed", "hx_clock_bound_us",
            "timing_rejections_total", "adc_recovery_resets", "calibration_id", "fc_uid",
            "out_of_calibration_range", "force_limit_exceeded"
        ]
        for motor in MOTOR_ORDER:
            self.fields.extend(["rpm_reported_" + motor, "erpm_derived_" + motor,
                                "dshot_invalid_percent_" + motor])
        self.fields.append("dshot_telemetry_raw_payload_hex")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fields)
        self.writer.writeheader()
        self.csv_file.flush()
        self.start_unix_ns = time.time_ns()
        self.start_mono = time.monotonic()
        self.baseline_force_N = None
        self.baseline_sd_N = None
        self.sequence = 0
        self.may_have_sent = False
        self.stop_attempts = 0
        self.stop_acks = 0
        self.status = "running"
        self.error = None
        self.command_acknowledgements = {}
        self.rail_samples = 0
        self.out_of_range_samples = 0
        self.force_limit_events = 0
        self.last_battery = None
        self.last_force = None
        self.last_armed = None
        self.metadata_path = directory / "metadata.json"
        self.summary_path = directory / "summary.json"
        self.meta = {
            "schema_version": 1,
            "run_id": str(uuid.uuid4()),
            "started_unix_time_ns": self.start_unix_ns,
            "operator_declared_setup": "props installed; aircraft secured on thrust stand; physical battery cut reachable",
            "execution": "manual step advance; four motors commanded collectively; all stopped between steps",
            "board_profile": fc_profile,
            "live_controller": live_fc,
            "live_battery_preflight": self.last_battery,
            "hx711_hardware": sensor_info,
            "calibration": {
                "calibration_id": calibration.get("calibration_id"),
                "sha256": hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest(),
                "counts_per_N": calibration["counts_per_N"],
                "zero_counts": calibration["zero_counts"],
                "measured_force_range_N": calibration.get("measured_force_range_N"),
                "quality_status": calibration.get("quality_status"),
                "rated_capacity": "unverified"
            },
            "motor_outputs": MOTOR_ORDER,
            "motor_protocol": "DSHOT300",
            "MSP_MOTOR_TELEMETRY_139_supported": dshot_telemetry_supported,
            "DShot_speed_units": "MSP reports mechanical RPM converted from DShot eRPM using FC configured motor pole count",
            "configured_motor_poles": live_fc["motor_config"]["configured_motor_poles"],
            "derived_eRPM_note": "erpm_derived = reported mechanical RPM * configured pole count / 2; physical pole count not independently verified",
            "stop_external_command": STOP_VALUE,
            "stop_dshot_value": 0,
            "command_units": "Betaflight MSP external motor command counts; not RPM, PWM microseconds, or newtons",
            "thrust_sign": args.thrust_sign,
            "thrust_delta_formula": "thrust_N = thrust_sign * (measured_load_force_N - stopped_baseline_force_N)",
            "settings": {"step_command_counts": args.step, "interval_s": args.interval,
                         "baseline_seconds": args.baseline_seconds, "max_command": args.max_command,
                         "force_limit_delta_N": args.force_limit_n},
            "measurement_notes": [
                "Thrust values are load-cell force changes from the stopped aircraft baseline.",
                "Linear interpolation is only supported within the weight-sweep calibration range.",
                "Force is invalid outside calibrated force range or at ADC rails.",
                "FC battery meter calibration is unverified; DShot RPM/eRPM is telemetry, not an independent tachometer reading.",
                "MSP RPM is mechanical RPM converted by FC using its configured pole count; eRPM columns are derived and pole count is unverified.",
                "Host acquisition timestamps are sequential, not simultaneous across HX711 and FC."
            ]
        }
        write_json(self.metadata_path, self.meta)

    def write_sample(self, stage, command, hx, batt, armed, baseline=None):
        raw = hx["raw_counts"]
        rail = raw in RAILS
        force = None if rail else force_from_counts(raw, self.calibration)
        limits = self.calibration.get("measured_force_range_N")
        if limits is None:
            fit = self.calibration.get("fit", {})
            limits = fit.get("measured_force_range_N")
        if not limits or len(limits) != 2:
            raise ValueError("Calibration has no measured force range; refusing an unbounded sweep")
        out_of_range = force is not None and not limits[0] <= force <= limits[1]
        delta = None if force is None or baseline is None else force - baseline
        thrust = None if delta is None else self.args.thrust_sign * delta
        force_limit = thrust is not None and abs(thrust) > self.args.force_limit_n
        self.rail_samples += int(rail)
        self.out_of_range_samples += int(out_of_range)
        self.force_limit_events += int(force_limit)
        self.last_battery, self.last_force, self.last_armed = batt, force, armed
        row = {
            "sequence": self.sequence, "stage": stage, "stage_command": command,
            "motor_command_M1": command, "motor_command_M2": command,
            "motor_command_M3": command, "motor_command_M4": command,
            "command_units": "MSP external motor command counts",
            "hx_unix_time_ns": hx.get("unix_time_ns"), "fc_unix_time_ns": time.time_ns(),
            "row_unix_time_ns": time.time_ns(),
            "elapsed_s": time.monotonic() - self.start_mono, "hx_raw_counts": raw,
            "hx_force_N": "" if rail or out_of_range else force,
            "baseline_force_N": "" if baseline is None else baseline,
            "force_delta_from_baseline_N": "" if delta is None or out_of_range else delta,
            "collective_thrust_N": "" if thrust is None or out_of_range else thrust,
            "mass_equivalent_g": "" if force is None or out_of_range else force / G0 * 1000,
            "battery_cells": batt.get("detected_cells"), "battery_voltage_V": batt.get("voltage_V"),
            "battery_current_A": batt.get("current_A"), "battery_consumed_mAh": batt.get("consumed_mAh"),
            "battery_state_code": batt.get("battery_state_code"), "fc_armed": armed,
            "hx_clock_bound_us": hx.get("max_clock_high_bound_us"),
            "timing_rejections_total": hx.get("timing_rejections_total", 0),
            "adc_recovery_resets": self.sensor.resets_total,
            "calibration_id": self.calibration.get("calibration_id"),
            "fc_uid": self.live_fc.get("uid_payload_hex"),
            "out_of_calibration_range": out_of_range, "force_limit_exceeded": force_limit
        }
        # Motor telemetry is attached by collect_stage after this common force/battery row is built.
        motor_data = getattr(self, "current_motor_data", None)
        if motor_data is not None:
            poles = self.live_fc["motor_config"]["configured_motor_poles"]
            for name, motor in zip(MOTOR_ORDER, motor_data["motors"]):
                row["rpm_reported_" + name] = motor["rpm"]
                row["erpm_derived_" + name] = motor["rpm"] * poles / 2
                row["dshot_invalid_percent_" + name] = motor["telemetry_invalid_percent"]
            row["dshot_telemetry_raw_payload_hex"] = motor_data["raw_payload_hex"]
        else:
            for name in MOTOR_ORDER:
                row["rpm_reported_" + name] = ""
                row["erpm_derived_" + name] = ""
                row["dshot_invalid_percent_" + name] = ""
            row["dshot_telemetry_raw_payload_hex"] = ""
        self.writer.writerow(row)
        self.csv_file.flush()
        os.fsync(self.csv_file.fileno())
        self.sequence += 1
        if rail:
            raise RuntimeError("HX711 ADC reached a rail; stopping the sweep")
        if out_of_range:
            raise RuntimeError("Load reading left calibrated force range %s N; stopping the sweep" % limits)
        if armed is not False:
            raise RuntimeError("Flight controller is not positively confirmed disarmed")
        if force_limit:
            raise RuntimeError("Configured thrust-force limit exceeded; stopping the sweep")

    def issue(self, command):
        packet = motor_packet([command] * 4, 4, STOP_VALUE,
                              self.live_fc["motor_config"]["max_throttle_setting"],
                              self.args.max_command)
        self.may_have_sent = True
        self.client._exchange(packet, 214)
        self.command_acknowledgements[command] = self.command_acknowledgements.get(command, 0) + 1

    def stop(self, tries=3):
        previous_acks = self.stop_acks
        packet = motor_packet([STOP_VALUE] * 4, 4, STOP_VALUE,
                              self.live_fc["motor_config"]["max_throttle_setting"],
                              self.live_fc["motor_config"]["max_throttle_setting"])
        for _ in range(tries):
            self.stop_attempts += 1
            try:
                self.client._exchange(packet, 214)
                self.stop_acks += 1
            except KeyboardInterrupt:
                print("Interrupted during stop attempt; retrying stop.", file=sys.stderr, flush=True)
            except (OSError, MSPError, TimeoutError) as exc:
                print("STOP delivery attempt failed: %s" % exc, file=sys.stderr, flush=True)
        return self.stop_acks > previous_acks

    def telemetry(self, command):
        armed = status(self.client.query(101), self.box_ids)["armed"]
        if armed is not False:
            raise RuntimeError("FC armed state unknown or armed; stopping immediately")
        batt = battery(self.client.query(130))
        if batt["detected_cells"] not in (2, 3):
            raise RuntimeError("FC lost 2S/3S battery detection; stopping immediately")
        if batt["battery_state_code"] != 0:
            state = {1: "WARNING", 2: "CRITICAL", 3: "NOT PRESENT", 4: "INIT"}.get(
                batt["battery_state_code"], "UNKNOWN(%s)" % batt["battery_state_code"])
            raise RuntimeError("FC battery state is %s; stopping immediately" % state)
        self.issue(command)
        motor_data = motor_telemetry(self.client.query(139), 4) if self.dshot_telemetry_supported else None
        self.current_motor_data = motor_data
        return batt, armed, motor_data

    def collect_stage(self, stage, command, seconds, baseline=None, stopped=False):
        end = time.monotonic() + seconds
        count = 0
        while time.monotonic() < end:
            if stopped:
                batt, armed, motor_data = self.telemetry(STOP_VALUE)
            else:
                batt, armed, motor_data = self.telemetry(command)
            hx = self.sensor.read()
            self.write_sample(stage, command, hx, batt, armed, baseline)
            count += 1
            if count == 1 or count % 5 == 0:
                value = "n/a" if self.last_force is None else "%.3f N" % self.last_force
                delta = "" if baseline is None or self.last_force is None else \
                    " dF=%+.3fN thrust=%+.3fN" % (self.last_force - baseline,
                    self.args.thrust_sign * (self.last_force - baseline))
                rpm = "" if motor_data is None else " RPM=" + ",".join(str(m["rpm"]) for m in motor_data["motors"])
                print("stage=%s command=%d samples=%d load=%s%s V=%.2f I=%.2f%s" %
                      (stage, command, count, value, delta, batt["voltage_V"], batt["current_A"], rpm),
                      flush=True)
            if count > 1:
                time.sleep(0.005)
        return count

    def run(self):
        self.stop()  # establish known stopped baseline before sampling
        input("Confirm the frame is secure and all propellers are installed. Press Enter to record stopped baseline: ")
        self.collect_stage("baseline", STOP_VALUE, self.args.baseline_seconds, stopped=True)
        baseline_rows = []
        # Baseline is the mean of calibrated stopped-load samples from CSV just collected.
        with open(self.csv_path, newline="", encoding="utf-8") as f:
            baseline_rows = [float(row["hx_force_N"]) for row in csv.DictReader(f)
                             if row["stage"] == "baseline" and row["hx_force_N"]]
        if len(baseline_rows) < 2:
            raise RuntimeError("Insufficient valid stopped baseline samples")
        self.baseline_force_N = statistics.mean(baseline_rows)
        self.baseline_sd_N = statistics.stdev(baseline_rows)
        self.meta["stopped_baseline"] = {"sample_count": len(baseline_rows),
                                           "mean_force_N": self.baseline_force_N,
                                           "sample_sd_N": self.baseline_sd_N}
        write_json(self.metadata_path, self.meta)
        print("Stopped baseline %.5f N (SD %.5f N); readings are calibrated to the weight sweep." %
              (self.baseline_force_N, self.baseline_sd_N), flush=True)

        command = STOP_VALUE
        while command < self.args.max_command:
            next_command = min(command + self.args.step, self.args.max_command)
            # Motors remain at DShot stop while the operator reviews and decides.
            if self.may_have_sent:
                if not self.stop():
                    raise RuntimeError("FC did not acknowledge stop; use the physical battery disconnect")
            reply = input("Press Enter for command %d/%d (%.1f s), or type q to stop: " %
                          (next_command, self.args.max_command, self.args.interval)).strip().lower()
            if reply == "q":
                self.status = "stopped_by_operator"
                break
            if reply:
                print("Enter advances one fixed step; q exits.", flush=True)
                continue
            self.collect_stage("command", next_command, self.args.interval,
                               baseline=self.baseline_force_N, stopped=False)
            command = next_command
            if command >= self.args.max_command:
                self.status = "completed_to_max_command"
                break

    def finalize(self):
        self.csv_file.close()
        ended = time.time_ns()
        summary = {
            "status": self.status, "error": self.error,
            "started_unix_time_ns": self.start_unix_ns, "ended_unix_time_ns": ended,
            "samples_written": self.sequence,
            "motor_command_acknowledgements_by_value": self.command_acknowledgements,
            "ADC_rail_samples": self.rail_samples, "out_of_calibration_range_samples": self.out_of_range_samples,
            "force_limit_events": self.force_limit_events,
            "timing_rejected_conversions": self.sensor.rejected_total,
            "ADC_recovery_resets": self.sensor.resets_total,
            "stop_attempts": self.stop_attempts, "stop_acknowledgements": self.stop_acks,
            "physical_spin_down_verified": False,
            "last_FC_battery": self.last_battery,
            "last_calibrated_load_force_N": self.last_force,
            "last_FC_armed": self.last_armed,
            "output_csv": "samples.csv", "metadata": "metadata.json"
        }
        write_json(self.summary_path, summary)


def main(argv=None):
    args = make_parser().parse_args(argv)
    if not args.execute or not args.bench_ready:
        print("ERROR: powered sweep requires --execute --bench-ready; see README_BETAFLIGHT.md",
              file=sys.stderr)
        return 2
    if args.step > 999:
        print("ERROR: command step must be between 1 and 999", file=sys.stderr)
        return 2
    run_dir = new_run_directory(args.output)
    transport = None
    client = None
    sensor = None
    sweep = None
    result = 0
    try:
        profile = json.loads(Path(args.profile).read_text())
        calibration = load_calibration(args.calibration)
        limits = calibration.get("measured_force_range_N")
        if limits is None:
            limits = calibration.get("fit", {}).get("measured_force_range_N")
        if not limits or len(limits) != 2 or not (limits[0] <= 0 <= limits[1]):
            raise ValueError("Calibration must include its measured force range, including zero")
        max_force_delta = limits[1] - limits[0]
        if args.force_limit_n > max_force_delta:
            raise ValueError("--force-limit-n exceeds calibration force span %.4f N" % max_force_delta)
        if args.max_command <= STOP_VALUE:
            raise ValueError("--max-command must be greater than the verified stop value 1000")
        if args.max_command > 2000:
            raise ValueError("Maximum command cannot exceed the verified DShot external range 2000")
        transport = USBSerial(args.port)
        client = MSP(transport, timeout=0.5)
        live_fc = inspect_controller(client)
        validate_setup(live_fc, profile)
        if args.max_command > live_fc["motor_config"]["max_throttle_setting"]:
            raise ValueError("--max-command exceeds the live FC configured limit")
        box_ids = list(client.query(119))
        require_disarmed(client, box_ids)
        initial_battery = battery(client.query(130))
        if initial_battery["detected_cells"] not in (2, 3):
            raise RuntimeError("FC must detect the supported 2S/3S battery before sweep")
        if initial_battery["battery_state_code"] != 0:
            raise RuntimeError("FC battery state must be OK before sweep (state code %s)" %
                               initial_battery["battery_state_code"])
        try:
            motor_telemetry(client.query(139), 4)
            dshot_telemetry_supported = True
        except MSPError as exc:
            if "FC rejected/does not support" not in str(exc):
                raise
            dshot_telemetry_supported = False
            print("WARNING: FC does not expose MSP motor telemetry; RPM columns will be blank.",
                  file=sys.stderr, flush=True)
        sensor, sensor_info = create_sensor()
        sweep = Sweep(args, run_dir, client, transport, box_ids, sensor, calibration,
                      profile, live_fc, sensor_info, dshot_telemetry_supported)
        sweep.last_battery = initial_battery
        sweep.meta["live_battery_preflight"] = initial_battery
        write_json(sweep.metadata_path, sweep.meta)
        print("Connected and disarmed. Four outputs will be stepped together from %d to %d in steps of %d."
              % (STOP_VALUE, args.max_command, args.step), flush=True)
        print("Each Enter advances one step; motors are stopped between intervals. Ctrl+C attempts three stop packets.",
              flush=True)
        sweep.run()
    except KeyboardInterrupt:
        result = 130
        if sweep:
            sweep.status = "interrupted_by_user"
    except EOFError:
        if sweep:
            sweep.status = "stopped_by_operator"
        else:
            result = 1
    except BaseException as exc:
        result = 1
        print("ERROR: %s: %s" % (type(exc).__name__, exc), file=sys.stderr, flush=True)
        if sweep:
            sweep.status = "failed"
            sweep.error = "%s: %s" % (type(exc).__name__, exc)
    finally:
        if sweep is not None:
            # Always try to stop after execution began, including Ctrl+C and sensor/FC failures.
            if sweep.may_have_sent:
                acknowledged = sweep.stop(tries=3)
                if not acknowledged:
                    print("STOP NOT ACKNOWLEDGED: disconnect motor battery power now.",
                          file=sys.stderr, flush=True)
                    result = 1
                else:
                    print("FC acknowledged stop command (%d total acknowledgements); verify all rotors physically stopped."
                          % sweep.stop_acks, file=sys.stderr, flush=True)
            try:
                sweep.finalize()
            except BaseException as exc:
                print("ERROR writing run summary: %s" % exc, file=sys.stderr, flush=True)
                result = 1
        try:
            if sensor is not None:
                sensor.close()
        finally:
            if transport is not None:
                transport.close()
        print("Sweep data directory: %s" % run_dir, flush=True)
    return result


if __name__ == "__main__":
    sys.exit(main())
