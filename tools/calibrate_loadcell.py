#!/usr/bin/env python3
"""Interactive 100 g load-cell calibration; stdlib fit, JSON raw data, no motor commands."""
import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time
import uuid

from hx711_jetson import G0, RAILS, VERSION, RecoveringHX711, TimingError, positive, sample_count
from hx711_native import NativeHX711, supported_board, LIBRARY


def schedule(maximum, step, return_sweep=True):
    if maximum <= 0 or step <= 0 or maximum % step:
        raise ValueError("Maximum load must be a positive multiple of the step")
    if maximum // step < 3:
        raise ValueError("Use at least four load levels including zero")
    ascending = [("ascending", mass) for mass in range(0, maximum + 1, step)]
    descending = [("descending", mass) for mass in range(maximum - step, -1, -step)]
    return ascending + (descending if return_sweep else [])


def point_statistics(point):
    values = [sample["raw_counts"] for sample in point["samples"]]
    if len(values) < 2:
        raise ValueError("Incomplete point: %s %s g" % (point["sweep"], point["mass_g"]))
    if any(value in RAILS for value in values):
        raise ValueError("ADC saturation in calibration data; no fit produced")
    half = len(values) // 2
    return {"sample_count": len(values), "mean_counts": statistics.mean(values),
            "std_counts": statistics.stdev(values), "min_counts": min(values),
            "max_counts": max(values),
            "first_to_last_half_drift_counts": statistics.mean(values[half:]) - statistics.mean(values[:half])}


def fit_session(session):
    if session.get("collection_status") != "complete":
        raise ValueError("Incomplete sessions are preserved but cannot become a calibration")
    points = []
    for point in session["points"]:
        points.append(dict(sweep=point["sweep"], mass_g=point["mass_g"], **point_statistics(point)))
    rising = [point for point in points if point["sweep"] == "ascending"]
    x = [point["mass_g"] for point in rising]
    y = [point["mean_counts"] for point in rising]
    if len(set(x)) != len(x) or len(x) < 4 or min(x) != 0:
        raise ValueError("Need at least four distinct ascending levels, including zero")
    if any(not math.isfinite(float(value)) for value in x + y):
        raise ValueError("Non-finite reference or ADC value")
    xm, ym = statistics.mean(x), statistics.mean(y)
    sxx = sum((value - xm) ** 2 for value in x)
    gradient = sum((a - xm) * (b - ym) for a, b in zip(x, y)) / sxx
    intercept = ym - gradient * xm
    signal = abs(gradient) * (max(x) - min(x))
    noise = max(point["std_counts"] for point in rising)
    if signal <= max(10, 10 * noise):
        raise ValueError("Load response too small relative to noise; check mounting/wiring")
    residuals = [b - (intercept + gradient * a) for a, b in zip(x, y)]
    ss_error = sum(value * value for value in residuals)
    ss_total = sum((value - ym) ** 2 for value in y)
    standard_error = math.sqrt(ss_error / (len(x) - 2) / sxx)
    for point in points:
        point["fitted_counts"] = intercept + gradient * point["mass_g"]
        point["residual_counts"] = point["mean_counts"] - point["fitted_counts"]
        point["residual_equivalent_g"] = point["residual_counts"] / gradient
        point["repeatability_std_equivalent_g"] = point["std_counts"] / abs(gradient)
    rising_by_mass = {point["mass_g"]: point for point in points if point["sweep"] == "ascending"}
    hysteresis = [{"mass_g": point["mass_g"],
                   "descending_minus_ascending_counts": point["mean_counts"] - rising_by_mass[point["mass_g"]]["mean_counts"],
                   "equivalent_g": (point["mean_counts"] - rising_by_mass[point["mass_g"]]["mean_counts"]) / gradient}
                  for point in points if point["sweep"] == "descending" and point["mass_g"] in rising_by_mass]
    zero_points = [point for point in points if point["mass_g"] == 0]
    final_zero = zero_points[-1]["mean_counts"]
    zero_drift = final_zero - zero_points[0]["mean_counts"] if len(zero_points) > 1 else None
    gravity = float(session["gravity_m_s2"])
    sign = session["force_sign"]
    if not math.isfinite(gravity) or gravity <= 0 or sign not in (-1, 1):
        raise ValueError("Invalid gravity or force sign")
    counts_per_N = gradient * 1000 / (sign * gravity)
    force_range = sorted([sign * min(x) / 1000 * gravity, sign * max(x) / 1000 * gravity])
    return {"gradient_counts_per_g": gradient, "inverse_gradient_g_per_count": 1 / gradient,
            "counts_per_N": counts_per_N, "N_per_count": 1 / counts_per_N,
            "fit_intercept_counts": intercept, "final_zero_counts": final_zero,
            "gradient_regression_standard_error_counts_per_g": standard_error,
            "r_squared": 1 - ss_error / ss_total,
            "ascending_residual_RMSE_equivalent_g": math.sqrt(ss_error / len(x)) / abs(gradient),
            "ascending_max_abs_residual_equivalent_g": max(abs(v) for v in residuals) / abs(gradient),
            "zero_return_drift_counts": zero_drift,
            "zero_return_drift_equivalent_g": zero_drift / gradient if zero_drift is not None else None,
            "max_abs_hysteresis_equivalent_g": max((abs(p["equivalent_g"]) for p in hysteresis), default=None),
            "fit_method": "unweighted linear least squares of ascending load-level means; descending held out",
            "points": points, "hysteresis": hysteresis,
            "reference_mass_range_g": [min(x), max(x)], "measured_force_range_N": force_range,
            "uncertainty_note": "Regression standard error omits weight accuracy, gravity uncertainty, alignment and other systematic errors"}


def save_session(directory, session):
    temporary = directory / "readings.json.tmp"
    temporary.write_text(json.dumps(session, indent=2, allow_nan=False) + "\n")
    temporary.replace(directory / "readings.json")


def calibration_document(session, fit):
    zero = session["points"][-1] if session["points"][-1]["mass_g"] == 0 else session["points"][0]
    return {"schema_version": 1, "calibration_id": str(uuid.uuid4()),
            "created_unix_time_ns": time.time_ns(), "reader_version": VERSION,
            "channel": "A", "gain": 128, "method": fit["fit_method"],
            "identity": {"load_cell_id": session["load_cell_id"], "rig_id": session["rig_id"],
                         "positive_direction": session["positive_direction"]},
            "hardware": session["hardware"], "counts_per_N": fit["counts_per_N"],
            "zero_counts": fit["final_zero_counts"], "zero_source": "last measured zero-load point; fitted intercept retained separately",
            "zero_samples": zero["samples"], "zero_statistics": point_statistics(zero),
            "tare_history": [], "fit": fit,
            "measured_force_range_N": fit["measured_force_range_N"],
            "reference_mass_range_g": fit["reference_mass_range_g"],
            "gravity_m_s2": session["gravity_m_s2"], "force_sign": session["force_sign"],
            "raw_data_file": "readings.json", "quality_status": "candidate; review residuals/hysteresis before use",
            "limitations": ["validity outside the measured load range is not established",
                            "opposite-direction loading is not validated by a sign change",
                            "capacity and reference-weight uncertainties are not independently verified"]}


def format_report(fit):
    lines = ["Load-cell linear calibration (ascending points)",
             "ADC counts = %.9g + (%+.9g counts/g) * added mass_g" %
             (fit["fit_intercept_counts"], fit["gradient_counts_per_g"]),
             "Gradient: %+.9g counts/g; inverse: %+.9g g/count" %
             (fit["gradient_counts_per_g"], fit["inverse_gradient_g_per_count"]),
             "Force scale: %+.9g counts/N; inverse: %+.9g N/count" %
             (fit["counts_per_N"], fit["N_per_count"]),
             "R^2: %.8f; ascending residual RMS: %.5f g; max abs residual: %.5f g" %
             (fit["r_squared"], fit["ascending_residual_RMSE_equivalent_g"], fit["ascending_max_abs_residual_equivalent_g"]),
             "Final zero used by force reader: %.6f counts" % fit["final_zero_counts"],
             "Return-to-zero drift (g equivalent): %s" % fit["zero_return_drift_equivalent_g"],
             "Max ascending/descending difference (g equivalent): %s" % fit["max_abs_hysteresis_equivalent_g"],
             "", "sweep        added_g       mean_counts       std_counts      residual_g"]
    for point in fit["points"]:
        lines.append("%-12s %7.0f %17.4f %16.4f %15.5f" %
                     (point["sweep"], point["mass_g"], point["mean_counts"], point["std_counts"], point["residual_equivalent_g"]))
    lines += ["", "No outliers were removed. Residuals are not a complete measurement-uncertainty estimate.",
              "Ascending loads determine the gradient; descending loads check hysteresis/drift.",
              "Review this candidate before using it. Range: %s g added reference mass." % fit["reference_mass_range_g"]]
    return "\n".join(lines) + "\n"


def finish(directory, session):
    fit = fit_session(session)
    calibration = calibration_document(session, fit)
    with (directory / "calibration.json").open("x") as file:
        json.dump(calibration, file, indent=2, allow_nan=False)
        file.write("\n")
    report = format_report(fit)
    with (directory / "report.txt").open("x") as file:
        file.write(report)
    print("\n" + report)
    print("Calibration saved:", directory / "calibration.json")
    print("Read force with: python3 hx711_jetson.py read --calibration %s" % (directory / "calibration.json"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-mass-g", type=int, default=1000)
    parser.add_argument("--step-g", type=int, default=100)
    parser.add_argument("--samples", type=sample_count, default=40)
    parser.add_argument("--settle", type=positive, default=2.0, help="seconds after pressing Enter")
    parser.add_argument("--force-sign", type=int, choices=(-1, 1), default=1,
                        help="+1: reference weights load in positive force direction; -1: opposite")
    parser.add_argument("--positive-direction", default="applied reference-weight loading direction")
    parser.add_argument("--gravity", type=positive, default=G0)
    parser.add_argument("--ascending-only", action="store_true")
    parser.add_argument("--load-cell-id", default="unspecified")
    parser.add_argument("--rig-id", default="unspecified")
    parser.add_argument("--notes", default="")
    parser.add_argument("--output", help="new run directory; never overwrite a previous run")
    parser.add_argument("--refit", help="fit an existing complete readings.json without GPIO")
    args = parser.parse_args()
    sensor, session, directory = None, None, None
    try:
        steps = schedule(args.max_mass_g, args.step_g, not args.ascending_only)
        directory = Path(args.output or time.strftime("loadcell_calibration_%Y%m%d_%H%M%S"))
        directory.mkdir(parents=True, exist_ok=False)
        if args.refit:
            session = json.loads(Path(args.refit).read_text())
            save_session(directory, session)
            finish(directory, session)
            return 0
        if not LIBRARY.is_file() or not supported_board():
            raise RuntimeError("Run on the configured Jetson with the compiled native HX711 helper")
        import Jetson.GPIO as GPIO
        session = {"schema_version": 1, "collection_status": "collecting", "reader_version": VERSION,
                   "started_unix_time_ns": time.time_ns(), "force_sign": args.force_sign,
                   "positive_direction": args.positive_direction, "gravity_m_s2": args.gravity,
                   "load_cell_id": args.load_cell_id, "rig_id": args.rig_id, "notes": args.notes,
                   "rated_load_cell_capacity_g": None, "capacity_status": "unverified",
                   "reference_weight_source": "operator supplied; nominal grams, uncertainty unspecified",
                   "reference_mass_definition": "total ADDITIONAL reference mass; mounted fixture/preload stays in place",
                   "hardware": {"gpio_version": GPIO.VERSION, "jetson_info": GPIO.JETSON_INFO,
                                "backend": "native_gpio_cdev", "channel": "A", "gain": 128,
                                "physical_pins": {"dout": 29, "sck": 31}},
                   "settings": vars(args), "points": []}
        save_session(directory, session)
        print("Motor battery must stay disconnected. Keep the mounted fixture unchanged.")
        print("Use reference loads within the cell/fixture rating, including preload; capacity is unverified.")
        print("Sequence:", ", ".join("%d g" % mass for _, mass in steps))
        print("Each prompt specifies TOTAL added mass, not another amount to add. Ctrl-C preserves partial readings.")
        sensor = RecoveringHX711(NativeHX711(timing_error=TimingError), retries=3)
        for sweep, mass in steps:
            input("\n%s: set TOTAL added reference mass to %d g; let it settle, then press Enter: " %
                  (sweep.upper(), mass))
            time.sleep(args.settle)
            sensor.read()  # discard conversion held while the mass changed
            point = {"sweep": sweep, "mass_g": mass,
                     "reference_force_N": args.force_sign * mass / 1000 * args.gravity,
                     "samples": [], "began_unix_time_ns": time.time_ns()}
            session["points"].append(point)
            try:
                for _ in range(args.samples):
                    sample = sensor.read()
                    point["samples"].append(sample)
                    if sample["raw_counts"] in RAILS:
                        raise ValueError("ADC at rail: stop loading; calibration refused")
            finally:
                save_session(directory, session)
            point["statistics"] = point_statistics(point)
            print("  mean=%+.3f counts; std=%.3f; within-point drift=%+.3f; accepted=%d" %
                  (point["statistics"]["mean_counts"], point["statistics"]["std_counts"],
                   point["statistics"]["first_to_last_half_drift_counts"], args.samples), flush=True)
            save_session(directory, session)
        session["collection_status"] = "complete"
        session["ended_unix_time_ns"] = time.time_ns()
        session["timing_rejected_conversions"] = sensor.rejected_total
        session["ADC_recovery_resets"] = sensor.resets_total
        save_session(directory, session)
        finish(directory, session)
        return 0
    except (KeyboardInterrupt, EOFError):
        if session is not None:
            session["collection_status"] = "interrupted"
        print("\nStopped; completed and partial readings preserved. No calibration fitted.")
        return 130
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print("ERROR:", exc, file=sys.stderr)
        if session is not None and session.get("collection_status") == "collecting":
            session["collection_status"] = "failed"
            session["error"] = str(exc)
        return 1
    finally:
        if sensor is not None:
            sensor.close()
            if session is not None:
                session["timing_rejected_conversions"] = sensor.rejected_total
                session["ADC_recovery_resets"] = sensor.resets_total
        if session is not None and directory is not None:
            save_session(directory, session)


if __name__ == "__main__":
    sys.exit(main())
