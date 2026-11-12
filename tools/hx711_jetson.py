#!/usr/bin/env python3
"""HX711 channel A / gain 128 reader for Jetson.GPIO; see README.md."""
import argparse
import csv
import gc
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time
import uuid

VERSION = "1.2.0"
G0 = 9.80665
RAILS = (-8388608, 8388607)


class TimingError(RuntimeError):
    pass


class RecoveringHX711:
    """Discard timing-failed transactions; reset and settle before retrying."""

    def __init__(self, sensor, retries=3):
        self.sensor, self.retries = sensor, retries
        self.rejected_total = 0
        self.resets_total = 0

    def read(self):
        before = self.rejected_total
        for attempt in range(self.retries + 1):
            try:
                sample = self.sensor.read()
                sample["timing_rejections_total"] = self.rejected_total
                sample["timing_rejections_before_sample"] = self.rejected_total - before
                return sample
            except TimingError as exc:
                self.rejected_total += 1
                if attempt == self.retries:
                    raise TimingError("Timing failures continued after %d reset retries; "
                                      "stopping with no value for the failed sample" % self.retries) from exc
                print("WARNING: %s. Resetting ADC and waiting 0.5 s; "
                      "retry %d/%d (rejected total %d)." %
                      (exc, attempt + 1, self.retries, self.rejected_total),
                      file=sys.stderr, flush=True)
                self.sensor.reset()
                self.resets_total += 1

    def reset(self):
        self.sensor.reset()

    def close(self):
        self.sensor.close()


def signed24(value):
    return value - (1 << 24) if value & (1 << 23) else value


class HX711:
    """Single-threaded GPIO driver. Fails closed on suspect clock timing."""

    def __init__(self, gpio, dout=29, sck=31, timeout=2.0, clock=time):
        if dout == sck:
            raise ValueError("DOUT and SCK must be different pins")
        self.gpio, self.dout, self.sck = gpio, dout, sck
        self.timeout, self.clock = timeout, clock
        self.initialized = []
        gpio.setmode(gpio.BOARD)
        try:
            gpio.setup(sck, gpio.OUT, initial=gpio.LOW)
            self.initialized.append(sck)
            gpio.setup(dout, gpio.IN)
            self.initialized.append(dout)
            self.reset()
        except BaseException:
            self.close()
            raise

    def reset(self):
        # Intentional power-down/reset, selecting channel A / gain 128.
        try:
            self.gpio.output(self.sck, self.gpio.HIGH)
            self.clock.sleep(0.001)
        finally:
            self.gpio.output(self.sck, self.gpio.LOW)
        # >=400 ms settling at 10 SPS, also sufficient at 80 SPS.
        self.clock.sleep(0.5)

    def _delay_us(self, us=1):
        until = self.clock.perf_counter_ns() + us * 1000
        while self.clock.perf_counter_ns() < until:
            pass

    def _pulse(self):
        # Bound the entire HIGH/LOW write interval, including preemption.
        # Sample DOUT AFTER falling edge to keep HIGH time short.
        begin = self.clock.perf_counter_ns()
        try:
            self.gpio.output(self.sck, self.gpio.HIGH)
            self._delay_us()
        finally:
            self.gpio.output(self.sck, self.gpio.LOW)
        elapsed = self.clock.perf_counter_ns() - begin
        if elapsed > 50_000:
            raise TimingError(
                "Clock pulse software bound %.1f us exceeds 50 us; sample rejected. "
                "Jetson/Linux cannot guarantee HX711 timing. "
                "Check GPIO performance or use a microcontroller USB bridge; "
                "do not disable the guard." % (elapsed / 1000)
            )
        self._delay_us()  # minimum LOW width; no maximum LOW width
        return elapsed / 1000

    def read(self):
        deadline = self.clock.monotonic() + self.timeout
        while self.gpio.input(self.dout):
            if self.clock.monotonic() >= deadline:
                raise TimeoutError("HX711 DOUT stayed HIGH: check power, pinmux and wiring")
            self.clock.sleep(0.001)
        ready_ns = self.clock.monotonic_ns()
        self._delay_us()  # >=0.1 us ready-to-first-clock requirement
        value, worst = 0, 0.0
        was_enabled = gc.isenabled()
        gc.disable()
        try:
            for _ in range(24):
                worst = max(worst, self._pulse())
                value = (value << 1) | int(bool(self.gpio.input(self.dout)))
            worst = max(worst, self._pulse())  # 25th pulse selects A / 128
            if not self.gpio.input(self.dout):
                raise RuntimeError("DOUT did not return HIGH after pulse 25; sample rejected")
        finally:
            if was_enabled:
                gc.enable()
        return {
            "raw_counts": signed24(value),
            "ready_observed_monotonic_ns": ready_ns,
            "read_complete_monotonic_ns": self.clock.monotonic_ns(),
            "unix_time_ns": self.clock.time_ns(),
            "max_clock_high_bound_us": worst,
        }

    def close(self):
        if self.sck in self.initialized:
            try:
                self.gpio.output(self.sck, self.gpio.LOW)
            finally:
                self.gpio.cleanup(self.initialized)
                self.initialized = []


def stats(samples):
    counts = [s["raw_counts"] for s in samples]
    if len(counts) < 2:
        raise ValueError("At least two samples required")
    if any(x in RAILS for x in counts):
        raise ValueError("ADC saturation during calibration/tare; check wiring and load")
    return {"count": len(counts), "mean_counts": statistics.mean(counts),
            "std_counts": statistics.stdev(counts),
            "min_counts": min(counts), "max_counts": max(counts)}


def make_calibration(zero, loaded, force_N, identity, hardware):
    if not math.isfinite(force_N) or force_N == 0:
        raise ValueError("Reference force must be finite and nonzero")
    zs, ls = stats(zero), stats(loaded)
    delta = ls["mean_counts"] - zs["mean_counts"]
    # Basic bad-connection/no-load-change check, not an uncertainty model.
    if abs(delta) <= max(10.0, 10 * max(zs["std_counts"], ls["std_counts"])):
        raise ValueError("Calibration load change is too small relative to noise")
    return {"schema_version": 1, "calibration_id": str(uuid.uuid4()),
            "created_unix_time_ns": time.time_ns(), "reader_version": VERSION,
            "channel": "A", "gain": 128, "identity": identity, "hardware": hardware,
            "zero_counts": zs["mean_counts"], "counts_per_N": delta / force_N,
            "reference_force_N": force_N,
            "zero_statistics": zs, "loaded_statistics": ls,
            "zero_samples": zero, "loaded_samples": loaded,
            "tare_history": [],
            "method": "two-point force calibration; linearity not yet established"}


def load_calibration(filename):
    data = json.loads(Path(filename).read_text())
    if data.get("schema_version") != 1 or data.get("channel") != "A" or data.get("gain") != 128:
        raise ValueError("Expected schema 1, channel A / gain 128 calibration")
    for key in ("zero_counts", "counts_per_N"):
        if not math.isfinite(float(data[key])):
            raise ValueError("Non-finite calibration " + key)
    if data["counts_per_N"] == 0:
        raise ValueError("Zero calibration scale")
    return data


def force_from_counts(counts, calibration):
    return (counts - calibration["zero_counts"]) / calibration["counts_per_N"]


def write_json_new(filename, data):
    with open(filename, "x", encoding="utf-8") as file:
        json.dump(data, file, indent=2, allow_nan=False)
        file.write("\n")


def collect(sensor, count):
    sensor.read()  # discard conversion held while the operator adjusted the load
    return [sensor.read() for _ in range(count)]


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def sample_count(value):
    number = int(value)
    if number < 2:
        raise argparse.ArgumentTypeError("must be at least 2")
    return number


def nonnegative_int(value):
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be nonnegative")
    return number


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dout", type=int, default=29, help="physical BOARD pin (default 29)")
    p.add_argument("--sck", type=int, default=31, help="physical BOARD pin (default 31)")
    p.add_argument("--timeout", type=positive, default=2.0)
    p.add_argument("--backend", choices=["auto", "native", "gpio"], default="auto",
                   help="auto uses compiled native helper on verified P3768 pins 29/31")
    p.add_argument("--timing-retries", type=nonnegative_int, default=3,
                   help="ADC reset retries after rejected clock timing; 0 stops immediately")
    sub = p.add_subparsers(dest="mode", required=True)
    cal = sub.add_parser("calibrate", help="interactive unloaded + known-force calibration")
    ref = cal.add_mutually_exclusive_group(required=True)
    ref.add_argument("--reference-mass-g", type=float, help="signed mass equivalent in grams")
    ref.add_argument("--reference-force-N", type=float, help="signed applied force in newtons")
    cal.add_argument("--gravity", type=positive, default=G0)
    cal.add_argument("--samples", type=sample_count, default=50)
    cal.add_argument("--output", required=True)
    cal.add_argument("--load-cell-id", required=True)
    cal.add_argument("--rig-id", required=True)
    cal.add_argument("--positive-direction", required=True)
    cal.add_argument("--notes", default="")
    tare = sub.add_parser("tare", help="save a NEW calibration with updated zero")
    tare.add_argument("--calibration", required=True)
    tare.add_argument("--output", required=True)
    tare.add_argument("--samples", type=sample_count, default=50)
    for mode in ("raw", "read"):
        s = sub.add_parser(mode, help="read raw ADC" if mode == "raw" else "read calibrated force")
        if mode == "read":
            s.add_argument("--calibration", required=True)
        s.add_argument("--output", help="optional new CSV; omit to print only")
        s.add_argument("--duration", type=positive, help="seconds; omit for Ctrl-C")
        s.add_argument("--test-id", default="")
        s.add_argument("--scope", choices=["unknown", "single_rotor", "whole_aircraft"], default="unknown")
        s.add_argument("--configuration-id", default="")
        s.add_argument("--notes", default="")
    return p


def display(sensor, args, calibration):
    """Print every accepted sample without creating any measurement files."""
    sensor.read()  # discard conversion held before starting the display
    end = None if args.duration is None else time.monotonic() + args.duration
    while end is None or time.monotonic() < end:
        sample = sensor.read()
        raw = sample["raw_counts"]
        at_rail = raw in RAILS
        suffix = ""
        if calibration is not None and not at_rail:
            suffix = "  force=%+.5f N" % force_from_counts(raw, calibration)
        print("raw=%d%s%s" % (raw, suffix, "  SATURATED" if at_rail else ""), flush=True)


def log(sensor, args, calibration, hardware):
    output = Path(args.output)
    metadata_path = Path(str(output) + ".meta.json")
    if output.exists() or metadata_path.exists():
        raise FileExistsError("Output CSV or metadata already exists; choose a new output")
    metadata = {
        "schema_version": 1, "reader_version": VERSION, "hardware": hardware,
        "arguments": vars(args), "calibration": calibration,
        "calibration_file_sha256": (
            hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest() if calibration else None),
        "timestamp_convention": "host observes ready; NOT ADC integration midpoint",
        "battery_voltage_source": None, "motor_command_source": None,
        "began_unix_time_ns": time.time_ns(),
    }
    write_json_new(metadata_path, metadata)
    fields = ["sequence", "unix_time_ns", "ready_observed_monotonic_ns",
              "read_complete_monotonic_ns", "elapsed_s", "raw_counts", "force_N",
              "mass_equivalent_g", "adc_at_rail", "max_clock_high_bound_us",
              "timing_rejections_total", "timing_rejections_before_sample",
              "calibration_id", "test_id", "test_scope", "hardware_configuration_id",
              "command", "command_units", "loaded_voltage_V"]
    count, rails = 0, 0
    start = time.monotonic_ns()
    end = None if args.duration is None else start + int(args.duration * 1e9)
    next_display = 0
    status, error = "complete", None
    try:
        with open(output, "x", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()
            sensor.read()  # discard pre-log conversion
            while end is None or time.monotonic_ns() < end:
                sample = sensor.read()
                raw = sample["raw_counts"]
                at_rail = raw in RAILS
                rails += int(at_rail)
                force = None if calibration is None or at_rail else force_from_counts(raw, calibration)
                row = dict(sample, sequence=count,
                           elapsed_s=(sample["ready_observed_monotonic_ns"] - start) / 1e9,
                           force_N="" if force is None else force,
                           mass_equivalent_g="" if force is None else force / G0 * 1000,
                           adc_at_rail=int(at_rail),
                           calibration_id="" if calibration is None else calibration["calibration_id"],
                           test_id=args.test_id, test_scope=args.scope,
                           hardware_configuration_id=args.configuration_id,
                           timing_rejections_total=sample.get("timing_rejections_total", 0),
                           timing_rejections_before_sample=sample.get("timing_rejections_before_sample", 0),
                           command="", command_units="", loaded_voltage_V="")
                writer.writerow(row)
                file.flush()
                count += 1
                if sample["ready_observed_monotonic_ns"] >= next_display:
                    suffix = "" if force is None else "  force=%+.5f N" % force
                    print("raw=%d%s%s" % (raw, suffix, "  SATURATED" if at_rail else ""), flush=True)
                    next_display = sample["ready_observed_monotonic_ns"] + 250_000_000
    except KeyboardInterrupt:
        status = "stopped_by_user"
    except BaseException as exc:
        status, error = "failed", "%s: %s" % (type(exc).__name__, exc)
        raise
    finally:
        elapsed = (time.monotonic_ns() - start) / 1e9
        # Separate new summary preserves metadata and completed CSV even after errors.
        write_json_new(str(output) + ".summary.json", {
            "status": status, "error": error, "samples_written": count,
            "rail_samples": rails, "elapsed_s": elapsed,
            "timing_rejections_total": sensor.rejected_total if isinstance(sensor, RecoveringHX711) else 0,
            "adc_recovery_resets": sensor.resets_total if isinstance(sensor, RecoveringHX711) else 0,
            "effective_samples_per_s": count / elapsed if elapsed else None,
            "ended_unix_time_ns": time.time_ns()})
    print("Saved %d samples to %s" % (count, output))


def main(argv=None):
    args = parser().parse_args(argv)
    sensor = None
    try:
        if args.output is not None:
            for filename in [args.output, args.output + ".meta.json", args.output + ".summary.json"]:
                if Path(filename).exists():
                    raise FileExistsError(filename + " already exists")
        calibration = load_calibration(args.calibration) if hasattr(args, "calibration") else None
        if args.mode == "calibrate":
            force = (args.reference_mass_g / 1000 * args.gravity
                     if args.reference_mass_g is not None else args.reference_force_N)
            if not math.isfinite(force) or force == 0:
                raise ValueError("Reference force must be finite and nonzero")
        try:
            import Jetson.GPIO as GPIO
        except ImportError as exc:
            raise RuntimeError("Install Jetson.GPIO on the Jetson; see README.md") from exc
        hardware = {"gpio_version": GPIO.VERSION, "jetson_info": GPIO.JETSON_INFO,
                    "pin_numbering": "BOARD", "dout": args.dout, "sck": args.sck,
                    "python": sys.version}
        from hx711_native import LIBRARY, NativeHX711, supported_board
        native_available = LIBRARY.is_file() and supported_board() and (args.dout, args.sck) == (29, 31)
        if args.backend == "native" and not native_available:
            raise RuntimeError("Native helper missing, unsupported board or pins; see README build command")
        if args.backend == "native" or (args.backend == "auto" and native_available):
            sensor = NativeHX711(args.timeout, timing_error=TimingError)
            hardware["backend"] = "native_gpio_cdev"
        else:
            sensor = HX711(GPIO, args.dout, args.sck, args.timeout)
            hardware["backend"] = "Jetson.GPIO"
        print("Reader backend: " + hardware["backend"], file=sys.stderr)
        sensor = RecoveringHX711(sensor, args.timing_retries)
        hardware["timing_retries"] = args.timing_retries
        if args.mode == "calibrate":
            input("Motor power OFF. Rig installed, zero applied test force; settle, then Enter: ")
            zero = collect(sensor, args.samples)
            input("Apply signed reference force %+.6f N; settle, then Enter: " % force)
            loaded = collect(sensor, args.samples)
            identity = {key: getattr(args, key) for key in
                        ("load_cell_id", "rig_id", "positive_direction", "notes")}
            calibration = make_calibration(zero, loaded, force, identity, hardware)
            calibration["reference_mass_g"] = args.reference_mass_g
            calibration["gravity_m_s2"] = args.gravity if args.reference_mass_g is not None else None
            write_json_new(args.output, calibration)
            print("Saved %s; counts/N = %.8g" % (args.output, calibration["counts_per_N"]))
        elif args.mode == "tare":
            input("Motor power OFF. Remove applied test force, keep mounted rig; settle, then Enter: ")
            zero = collect(sensor, args.samples)
            old_id = calibration["calibration_id"]
            calibration["tare_history"].append({
                "parent_calibration_id": old_id, "previous_zero_counts": calibration["zero_counts"],
                "samples": zero, "statistics": stats(zero), "hardware": hardware,
                "unix_time_ns": time.time_ns()})
            calibration["calibration_id"] = str(uuid.uuid4())
            calibration["zero_counts"] = stats(zero)["mean_counts"]
            write_json_new(args.output, calibration)
            print("Saved new tare to " + args.output)
        elif args.output is None:
            display(sensor, args, calibration)
        else:
            log(sensor, args, calibration, hardware)
        return 0
    except (KeyboardInterrupt, EOFError):
        print("Cancelled.", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as exc:
        print("ERROR: " + str(exc), file=sys.stderr)
        return 1
    finally:
        if sensor is not None:
            sensor.close()
            if isinstance(sensor, RecoveringHX711) and sensor.rejected_total:
                print("Timing summary: %d rejected conversions, %d ADC recovery resets." %
                      (sensor.rejected_total, sensor.resets_total), file=sys.stderr)


if __name__ == "__main__":
    sys.exit(main())
