#!/usr/bin/env python3
"""Summarize v2 experiments; optional matplotlib overview (no hardware access)."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics as st


def rows(directory, name):
    with open(directory / (name + ".csv"), newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def distribution(values):
    if not values:
        return None
    a = sorted(values)
    return dict(count=len(a), median=st.median(a), p95=a[min(len(a)-1, math.ceil(.95*len(a))-1)], maximum=max(a))


def mean(rows_, field):
    a = [float(r[field]) for r in rows_ if r.get(field) not in ("", None)]
    return st.mean(a) if a else None


def within(samples, a, b, start, end=None):
    return [r for r in samples if a <= int(r[start]) and int(r[end or start]) <= b]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", type=Path)
    args = p.parse_args(argv)
    d = args.directory
    meta = json.loads((d/"metadata.json").read_text())
    summary = json.loads((d/"summary.json").read_text())
    if meta["schema_version"] != 2:
        p.error("This analyzer requires a v2 bench capture")
    rpm, force, batt, commands, phases = [rows(d, n) for n in ("rpm", "force", "battery", "commands", "phases")]
    valid_force = [r for r in force if r["valid"] == "True"]
    baseline = summary["stopped_baseline_N"]
    sign = meta["settings"]["thrust_sign"]
    report = dict(capture_status=summary["status"], capture_error=summary["error"], rows=summary["rows"],
        rpm_reply_intervals_s=distribution([(int(b["reply_ns"])-int(a["reply_ns"]))/1e9 for a,b in zip(rpm,rpm[1:])]),
        rpm_round_trip_s=distribution([(int(r["reply_ns"])-int(r["request_start_ns"]))/1e9 for r in rpm]),
        force_ready_intervals_s=distribution([(int(b["ready_observed_ns"])-int(a["ready_observed_ns"]))/1e9 for a,b in zip(force,force[1:])]),
        rpm_achieved_hz=None if len(rpm)<2 else (len(rpm)-1)*1e9/(int(rpm[-1]["reply_ns"])-int(rpm[0]["reply_ns"])),
        force_achieved_hz=None if len(force)<2 else (len(force)-1)*1e9/(int(force[-1]["ready_observed_ns"])-int(force[0]["ready_observed_ns"])),
        baseline_return_shift_N=summary["baseline_return_shift_N"],
        stop_acknowledgements=summary["stop_acknowledgements"],
        limitations=["RPM reply timestamps are host timestamps, not known FC sampling times.",
                     "Repeated RPM values alone cannot distinguish quantization from stale telemetry.",
                     "The last 0.75 seconds of a phase is an endpoint window, not an automatic proof of settling.",
                     "Do not use old single-step fit scripts with this multi-edge independent-channel schema."])
    report["rotors"] = {}
    for i in range(1,5):
        key = "rpm_M%d" % i
        values = [float(r[key]) for r in rpm]
        report["rotors"][key] = dict(nonzero_count=sum(v>0 for v in values), distinct_values=len(set(values)),
            adjacent_change_count=sum(a!=b for a,b in zip(values,values[1:])))
    result = []
    by_event = {r["event"]:r for r in commands}
    for phase in phases:
        end = int(phase["end_ns"])
        start = max(int(phase["ack_ns"]), end-750_000_000)
        ff = within(valid_force, start, end, "ready_observed_ns")
        rr = within(rpm, start, end, "request_start_ns", "reply_ns")
        bb = within(batt, start, end, "request_start_ns", "reply_ns")
        load = mean(ff, "load_N")
        vector = by_event.get(phase["command_event"], {})
        record = dict(phase=phase["phase"], role=phase["role"], cycle=phase["cycle"], completed=phase["completed"],
            **{"command_M%d" % i:vector.get("M%d" % i) for i in range(1,5)},
            endpoint_force_samples=len(ff), endpoint_rpm_samples=len(rr), mean_load_N=load,
            mean_force_delta_N=None if baseline is None or load is None else sign*(load-baseline),
            mean_voltage_V=mean(bb,"voltage_V"), mean_current_A=mean(bb,"current_A"),
            **{"mean_rpm_M%d" % i:mean(rr,"rpm_M%d" % i) for i in range(1,5)})
        sums = [sum(float(r["omega_reported_rad_s_M%d" % i])**2 for i in range(1,5)) for r in rr]
        record["mean_sum_reported_omega_squared"] = st.mean(sums) if sums else None
        result.append(record)
    with open(d/"phase_summary.csv", "w", newline="", encoding="utf-8") as f:
        if result:
            writer = csv.DictWriter(f, fieldnames=list(result[0]))
            writer.writeheader()
            writer.writerows(result)
    (d/"timing_report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Summaries written; install matplotlib on the analysis computer to plot.")
        return 0
    origin = meta["started_monotonic_ns"]
    def x(data, key):
        return [(int(r[key])-origin)/1e9 for r in data]
    fig, axes = plt.subplots(4, 1, figsize=(12,10), sharex=True)
    for i in range(1,5):
        if commands:
            axes[0].step(x(commands,"request_start_ns"),[int(r["M%d" % i]) for r in commands],where="post",label="M%d" % i)
        if rpm:
            axes[1].plot(x(rpm,"reply_ns"),[float(r["rpm_M%d" % i]) for r in rpm],linewidth=.9,label="M%d" % i)
    axes[0].set_ylabel("Requested command")
    axes[1].set_ylabel("FC-reported RPM")
    if rpm:
        axes[1].legend(ncol=4,fontsize=8)
    yforce = [float(r["load_N"]) if baseline is None else sign*(float(r["load_N"])-baseline) for r in valid_force]
    axes[2].plot(x(valid_force,"ready_observed_ns"),yforce,".-",color="black",markersize=3)
    axes[2].set_ylabel("Load (N)" if baseline is None else "Axial force change (N)")
    if batt:
        axes[3].plot(x(batt,"reply_ns"),[float(r["voltage_V"]) for r in batt],color="tab:blue",label="FC voltage")
        twin = axes[3].twinx()
        twin.plot(x(batt,"reply_ns"),[float(r["current_A"]) for r in batt],color="tab:orange",label="FC current")
        twin.set_ylabel("Current (A)",color="tab:orange")
    axes[3].set_ylabel("Voltage (V)",color="tab:blue")
    axes[3].set_xlabel("Host monotonic time from recording start (s)")
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle("Sanity %s — battery %s — %s" % (meta["preset"],meta["battery_id"],summary["status"]))
    fig.tight_layout()
    fig.savefig(d/"overview.png",dpi=150)
    plt.close(fig)
    print("Wrote overview.png, phase_summary.csv and timing_report.json in " + str(d))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
