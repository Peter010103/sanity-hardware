#!/usr/bin/env python3
"""Summarize a motor_step_response.py run and plot measured response channels."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def read_rows(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def numeric(row, field):
    try:
        value = float(row[field])
        return value if math.isfinite(value) else None
    except (KeyError, TypeError, ValueError):
        return None


def crossing(points, target, direction):
    for (t0, y0), (t1, y1) in zip(points, points[1:]):
        if direction * (y0 - target) < 0 <= direction * (y1 - target):
            if y1 == y0:
                return t1
            return t0 + (target-y0) * (t1-t0) / (y1-y0)
    return None


def crossing_interval(points, target, direction):
    """Return the sample bracket containing a threshold crossing; do not interpolate."""
    for (t0, y0), (t1, y1) in zip(points, points[1:]):
        if direction * (y0 - target) < 0 <= direction * (y1 - target):
            return [t0, t1]
    return None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run_directory", type=Path)
    args = p.parse_args()
    rows = read_rows(args.run_directory / "samples.csv")
    run_summary = json.loads((args.run_directory / "summary.json").read_text())
    capture_complete = run_summary.get("status") == "completed"
    pre = [r for r in rows if r["phase"] == "pre_step"]
    post = [r for r in rows if r["phase"] == "post_step"]
    if len(pre) < 3 or len(post) < 3:
        raise SystemExit("Need at least three pre-step and three post-step samples")
    event_ns = int(post[0]["command_send_monotonic_ns"])
    times = [(int(r["sample_monotonic_ns"])-event_ns)/1e9 for r in rows]
    pre_times = [float(r["elapsed_s"]) for r in pre]
    # Use final one second of the pre-step and post-step windows for steady levels.
    pre_end = max(pre_times)
    post_end = max(float(r["elapsed_s"]) for r in post)
    pre_idx = [i for i, r in enumerate(pre) if float(r["elapsed_s"]) >= pre_end - 1.0]
    post_idx = [i for i, r in enumerate(post) if float(r["elapsed_s"]) >= post_end - 1.0]
    channels = ["collective_thrust_N"] + [f"rpm_M{i}" for i in range(1, 5)]
    intervals = sorted((int(b["sample_monotonic_ns"])-int(a["sample_monotonic_ns"]))/1e9
                       for a, b in zip(rows, rows[1:]))
    median_interval = statistics.median(intervals) if intervals else None
    summary = {"step_event_definition": "first new-command packet host monotonic timestamp",
               "sampling_note": "Sequential nominal 10 SPS HX711 and FC queries; sub-0.3 s response is unresolved",
               "sample_count": len(rows), "median_row_interval_s": median_interval,
               "capture_status": run_summary.get("status"),
               "capture_error": run_summary.get("error"),
               "completed_requested_post_step_duration": capture_complete,
               "channels": {}}
    plotted = {}
    for channel in channels:
        before = [((int(r["sample_monotonic_ns"])-event_ns)/1e9, numeric(r, channel))
                  for r in pre]
        after = [((int(r["sample_monotonic_ns"])-event_ns)/1e9, numeric(r, channel))
                 for r in post]
        before = [(t, y) for t, y in before if y is not None]
        after = [(t, y) for t, y in after if y is not None]
        if len(before) < 3 or len(after) < 3:
            summary["channels"][channel] = {"status": "insufficient valid samples"}
            continue
        initial_values = [numeric(pre[i], channel) for i in pre_idx]
        final_values = [numeric(post[i], channel) for i in post_idx]
        initial_values = [v for v in initial_values if v is not None]
        final_values = [v for v in final_values if v is not None]
        if len(initial_values) < 2 or len(final_values) < 2:
            summary["channels"][channel] = {"status": "insufficient steady-window samples"}
            continue
        initial = statistics.mean(initial_values)
        final = statistics.mean(final_values)
        delta = final - initial
        direction = 1 if delta >= 0 else -1
        points = sorted(before + after)
        threshold = initial + direction * abs(delta) * 0.05
        onset = crossing_interval(points, threshold, direction)
        t10 = crossing_interval(points, initial + 0.1 * delta, direction)
        t90 = crossing_interval(points, initial + 0.9 * delta, direction)
        rise_bounds = None
        if t10 is not None and t90 is not None:
            rise_bounds = [max(0.0, t90[0] - t10[1]), max(0.0, t90[1] - t10[0])]
        status = "ok" if abs(delta) > 1e-12 else "no measurable step response"
        if not capture_complete:
            status = "truncated capture; endpoint-window response is provisional"
        summary["channels"][channel] = {
            "status": status,
            "initial_steady_window_mean": initial,
            "final_endpoint_window_mean_not_confirmed_steady": final,
            "change": delta, "onset_5_percent_s": onset,
            "rise_or_fall_10_to_90_bracket_s": rise_bounds,
            "t10_crossing_bracket_s": t10, "t90_crossing_bracket_s": t90,
            "threshold_definition": "10% and 90% of change from initial-window mean to final endpoint-window mean",
            "timing_resolution_warning": "Intervals are bounded by observed sample times without interpolation; channels are sampled sequentially at about 0.1 s or slower."}
        plotted[channel] = points
    out_json = args.run_directory / "response_analysis.json"
    out_json.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print(f"Wrote {out_json}; matplotlib unavailable, no plot generated")
        return
    fig, ax_rpm = plt.subplots(figsize=(9, 5))
    ax_thrust = ax_rpm.twinx()
    for channel, points in plotted.items():
        target = ax_thrust if channel == "collective_thrust_N" else ax_rpm
        target.plot([x for x, _ in points], [y for _, y in points], marker=".",
                    label=channel)
    ax_rpm.axvline(0, color="black", linewidth=1, linestyle="--")
    ax_rpm.set_xlabel("Time from command packet (s)")
    ax_rpm.set_ylabel("Reported RPM per motor")
    ax_thrust.set_ylabel("Measured collective thrust (N)")
    handles1, labels1 = ax_rpm.get_legend_handles_labels()
    handles2, labels2 = ax_thrust.get_legend_handles_labels()
    ax_rpm.legend(handles1 + handles2, labels1 + labels2, loc="best")
    ax_rpm.grid(True, alpha=.3)
    fig.tight_layout()
    plot_path = args.run_directory / "step_response.png"
    fig.savefig(plot_path, dpi=160)
    print(f"Wrote {out_json} and {plot_path}")


if __name__ == "__main__":
    main()
