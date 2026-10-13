#!/usr/bin/env python3
"""Fit local first-order-plus-dead-time models to a completed step capture.

Requires numpy, scipy, and matplotlib. Fits only the first 1.1 s around the
step so slow battery/thermal drift does not dominate the transient estimate.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def fopdt(t, y0, yfinal, delay_s, tau_s):
    elapsed = np.maximum(0.0, t - delay_s)
    return y0 + (yfinal - y0) * (1.0 - np.exp(-elapsed / tau_s))


def load_data(directory):
    rows = list(csv.DictReader(open(directory / "samples.csv", newline="", encoding="utf-8")))
    meta = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    event_row = next(r for r in rows if r["phase"] == "post_step")
    event_ns = int(event_row["command_send_monotonic_ns"])
    usable = [r for r in rows if r["phase"] in ("pre_step", "post_step")]
    t = np.asarray([(int(r["sample_monotonic_ns"]) - event_ns) / 1e9 for r in usable])
    rpm = np.asarray([np.mean([float(r[f"rpm_M{i}"]) for i in range(1, 5)]) for r in usable])
    thrust = np.asarray([float(r["collective_thrust_N"]) for r in usable])
    mask = (t >= -0.5) & (t <= 1.1)
    return t[mask], rpm[mask], thrust[mask], meta


def fit_channel(t, y, initial_guess):
    lo = float(np.min(y))
    hi = float(np.max(y))
    span = max(hi - lo, 1e-9)
    # Constrain delay to a plausible range, but note that sub-sample parameters
    # remain poorly identifiable at the observed ~0.087 s row interval.
    lower = [lo - span, lo - span, 0.0, 0.003]
    upper = [hi + span, hi + span, 0.3, 1.0]
    params, _ = curve_fit(fopdt, t, y, p0=initial_guess, bounds=(lower, upper), maxfev=100000)
    fitted = fopdt(t, *params)
    residual = y - fitted
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    r2 = float(1.0 - np.sum(residual ** 2) / np.sum((y - np.mean(y)) ** 2))
    y0, yf, delay, tau = (float(x) for x in params)
    return {"initial_output": y0, "final_output": yf, "dead_time_s": delay,
            "time_constant_s": tau, "change": yf-y0, "rmse": rmse, "R_squared": r2,
            "model_10_to_90_s": 2.197224577 * tau,
            "model_command_to_90_percent_s": delay + 2.302585093 * tau,
            "model_equation": "y(t)=y0 for t<=L; otherwise y0+(yfinal-y0)*(1-exp(-(t-L)/tau))"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    summary = json.loads((args.run_directory / "summary.json").read_text(encoding="utf-8"))
    if summary.get("status") != "completed":
        raise SystemExit("Refusing to fit a capture that did not complete its requested step interval")
    t, rpm, thrust, meta = load_data(args.run_directory)
    dt = np.diff(t)
    median_dt = float(np.median(dt))
    rpm_fit = fit_channel(t, rpm, [np.mean(rpm[t < 0]), np.mean(rpm[(t > .3) & (t < .9)]), .05, .03])
    thrust_fit = fit_channel(t, thrust, [np.mean(thrust[t < 0]), np.mean(thrust[(t > .3) & (t < .9)]), .15, .04])
    start = meta["configuration"]["start_command"]
    step = meta["configuration"]["step_command"]
    du = step - start
    rpm_fit["local_gain_per_command"] = rpm_fit["change"] / du
    rpm_fit["gain_units"] = "mechanical RPM per Betaflight external command count"
    thrust_fit["local_gain_per_command"] = thrust_fit["change"] / du
    thrust_fit["gain_units"] = "N per Betaflight external command count; collective load-cell result"
    output = {
        "model": "first-order plus dead time (FOPDT), local fit to one collective step",
        "capture_directory": args.run_directory.name,
        "start_command": start, "step_command": step, "command_change_counts": du,
        "fit_window_s_from_command": [-0.5, 1.1],
        "median_sample_interval_s": median_dt,
        "fits": {"mean_mechanical_RPM_of_four_rotors": rpm_fit,
                 "collective_thrust": thrust_fit},
        "limitations": [
            "Rows are sampled about every 0.087 s; fitted dead time and time constants below this scale are weakly identifiable and should not be interpreted as precise motor delay.",
            "RPM channels are queried over USB and thrust is read sequentially from a nominal 10 SPS HX711.",
            "Thrust response includes propeller, fixture, load-cell conversion, and filtering dynamics; it is not motor-only dynamics.",
            "This local fit characterizes the tested command range and battery condition only; it is not a general motor or sim-to-real model.",
            "FOPDT is a convenient descriptive model; no independent validation run or uncertainty interval was collected."
        ]}
    out_json = args.run_directory / "step_model_fit.json"
    out_json.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    t_dense = np.linspace(-0.5, 1.1, 500)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.scatter(t, rpm, s=18, label="measured mean RPM")
    ax1.plot(t_dense, fopdt(t_dense, rpm_fit["initial_output"], rpm_fit["final_output"],
                            rpm_fit["dead_time_s"], rpm_fit["time_constant_s"]),
             label="FOPDT fit")
    ax1.set_ylabel("Mean reported mechanical RPM")
    ax1.legend()
    ax1.grid(True, alpha=.3)
    ax2.scatter(t, thrust, s=18, label="measured collective thrust")
    ax2.plot(t_dense, fopdt(t_dense, thrust_fit["initial_output"], thrust_fit["final_output"],
                            thrust_fit["dead_time_s"], thrust_fit["time_constant_s"]),
             label="FOPDT fit")
    ax2.set_xlabel("Time from motor command packet (s)")
    ax2.set_ylabel("Collective thrust (N)")
    ax2.legend()
    ax2.grid(True, alpha=.3)
    fig.suptitle(f"Sanity collective step {start} → {step}; sample interval ≈ {median_dt:.3f} s")
    fig.tight_layout()
    fig.savefig(args.run_directory / "step_model_fit.png", dpi=160)
    print(f"Wrote {out_json} and step_model_fit.png")
    print(json.dumps(output["fits"], indent=2))


if __name__ == "__main__":
    main()
