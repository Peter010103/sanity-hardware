#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def main():
    parser = argparse.ArgumentParser(description="Independent ROS/Zenoh sequence and selected-history checks")
    parser.add_argument("run",type=Path)
    parser.add_argument("--kind",choices=["nominal","slow_policy","stale_tf"],default="nominal")
    a = parser.parse_args()
    receipt = json.loads((a.run/"controller/receipt.json").read_text())
    sim = json.loads((a.run/"simulation_receipt.json").read_text())
    control = records(a.run/"controller/control.jsonl")
    history = records(a.run/"controller/history.jsonl")
    commands = records(a.run/"controller/commands.jsonl")
    plant = records(a.run/"plant.jsonl")
    checks = {"no_hardware_access":sim["hardware_access"] is False,
        "neutral_disarmed_finish":sim["final_receiver"]["enabled"] is False and sim["final_receiver"]["ctbr"]==[0.]*4,
        "ordered_tick_ids":[r["tick"] for r in control]==list(range(len(control))),
        "every_selected_tick_committed":len(control)==len(history)==receipt["history_processed"]==receipt["history_submitted"]}
    max_history_error = 0.
    for index,(row,h) in enumerate(zip(control,history)):
        checks["ordered_tick_ids"] &= h["observation"]["tick"]==h["selection"]["tick"]==row["tick"]
        checks["every_selected_tick_committed"] &= h["selection"]==row["history_event"] and h["observation"]==row["history_observation"]
        if "selected_action" in h["selection"]:
            expected = np.asarray(h["selection"]["selected_action"])
        else:
            expected = (np.asarray(row["command"])-receipt["policy"]["hover_ctrl"])/receipt["policy"]["scaling"]
        max_history_error = max(max_history_error,float(np.max(abs(np.asarray(h["committed"]["action"])-expected))))
        checks["every_selected_tick_committed"] &= h["result"]["history_age"]==min(index,receipt["policy"]["history_length_steps"])
        checks["every_selected_tick_committed"] &= h["committed"]["history_age"]==min(index+1,receipt["policy"]["history_length_steps"])
    checks["selected_history_matches_output"] = max_history_error<2e-6
    t = np.asarray([r["time_s"] for r in control])
    ref = np.asarray([r["reference"]["position"] for r in control])
    physical_times = np.asarray([r["stamp_ns"]-receipt["started_ros_ns"] for r in plant])/1e9
    position = np.column_stack([np.interp(t,physical_times,[r["state"][axis] for r in plant]) for axis in range(3)])
    errors = position-ref
    moving = np.asarray([r["reference"]["phase"] in ("circle_accelerate","circle_cruise","circle_decelerate") for r in control])
    final = t>=t[-1]-5
    clocked = sim.get("clock") == "ROS simulation clock"
    gaps = np.diff([r["control_ros_ns"] if clocked else r["control_ns"] for r in control])/1e6
    command_gaps = np.diff([r["ros_ns"] if clocked else r["host_ns"] for r in commands])/1e6
    metrics = {"control_ticks":len(control),"duration_s":float(t[-1]),
        "circle_xy_rmse_m":float(np.sqrt(np.mean(np.sum(errors[moving,:2]**2,axis=1)))) if moving.any() else None,
        "circle_3d_rmse_m":float(np.sqrt(np.mean(np.sum(errors[moving]**2,axis=1)))) if moving.any() else None,
        "max_xy_error_m":float(np.max(np.linalg.norm(errors[:,:2],axis=1))),
        "final_altitude_mae_m":float(np.mean(abs(errors[final,2]))),
        "control_gap_ms":dict(zip(["median","p95","max"],np.percentile(gaps,[50,95,100]).tolist())),
        "command_gap_ms":dict(zip(["median","p95","max"],np.percentile(command_gaps,[50,95,100]).tolist())),
        "wall_control_gap_max_ms":float(np.max(np.diff([r["control_ns"] for r in control])/1e6)),
        "wall_command_gap_max_ms":float(np.max(np.diff([r["host_ns"] for r in commands])/1e6)),
        "pd_callback_max_ms":max(r["callback_ms"] for r in control),
        "inference_ms":dict(zip(["median","p95","max"],np.percentile([r["result"]["inference_ms"] for r in history],[50,95,100]).tolist())),
        "history_action_max_error":max_history_error,"blend_start_s":receipt["blend_start_s"],
        "policy_start_s":receipt["policy_start_s"],"orbit_start_s":receipt["orbit_start_s"],"fallback":receipt["fallback"]}
    if moving.any():
        center = np.asarray(receipt["parameters"]["origin_xy"])
        angle = np.unwrap(np.arctan2(position[moving,1]-center[1],position[moving,0]-center[0]))
        metrics["measured_circle_laps"] = float(abs(angle[-1]-angle[0])/(2*np.pi))
        metrics["circle_altitude_mae_m"] = float(np.mean(abs(errors[moving,2])))
        metrics["circle_max_altitude_error_m"] = float(np.max(abs(errors[moving,2])))
    if a.kind=="nominal":
        checks.update(completed=receipt["error"] is None and t[-1]>=59.9,
            no_fallback=receipt["fallback"] is False,
            pd_reposition=all(r["policy_weight"]==0 for r in control if r["reference"]["phase"]=="reposition"),
            circle_full_policy=bool(moving.any()) and all(r["policy_weight"]==1 and r["policy_valid"] and r["policy_tick"] in (r["tick"],r["tick"]-1) for r,m in zip(control,moving) if m),
            two_laps_then_hover=control[-1]["reference"]["phase"]=="circle_hold" and 1.9<metrics.get("measured_circle_laps",0)<2.1,
            no_ground_recontact=all(r["contacts"]==0 for r,time in zip(plant,physical_times) if time>12.),
            tracking=metrics["circle_xy_rmse_m"] is not None and metrics["circle_xy_rmse_m"]<.1 and metrics["max_xy_error_m"]<.2,
            final_hover=metrics["final_altitude_mae_m"]<.07)
    elif a.kind=="slow_policy":
        delayed = next(h for h in history if h["observation"].get("test_delay_s",0)>0)
        begin,end = delayed["observation"]["prepared_host_ns"],delayed["result"]["finished_host_ns"]
        metrics["actual_inference_stall_ms"] = (end-begin)/1e6
        metrics["pd_updates_during_stall"] = sum(begin<=r["control_ns"]<=end for r in control)
        metrics["command_updates_during_stall"] = sum(begin<=r["host_ns"]<=end for r in commands)
        first_fallback = next((r for r in control if r["fallback"]),None)
        checks["fallback_caused_by_injected_stall"] = bool(first_fallback and
            first_fallback["control_ns"]>=begin and first_fallback["control_ns"]<=end and
            all(not r["fallback"] for r in control if r["control_ns"]<begin))
        checks["pd_does_not_wait_for_actor"] = metrics["pd_updates_during_stall"]>=2
        checks["commands_do_not_wait_for_actor"] = metrics["command_updates_during_stall"]>=4
        checks.update(completed=receipt["error"] is None and t[-1]>=59.9,
            latched_pd_fallback=receipt["fallback"] is True and control[-1]["phase"]=="fallback_pd",
            command_timer_continues=metrics["command_gap_ms"]["max"]<40,
            pd_timer_continues=metrics["control_gap_ms"]["max"]<40)
    else:
        checks["stale_state_stops"] = bool(receipt["error"] and "Mocap" in receipt["error"])
    summary = {"kind":a.kind,"passed":bool(all(checks.values())),"checks":{k:bool(v) for k,v in checks.items()},
               "metrics":metrics,"controller_error":receipt["error"],"scope":"Closed-loop MuJoCo / ROS timers / TF estimator / native Zenoh-Pico/MSP; no hardware.",
               "clock":sim.get("clock","live ROS clock"),"live_clock_qualified":False}
    (a.run/"assessment.json").write_text(json.dumps(summary,indent=2)+"\n")
    fig,axes = plt.subplots(2,2,figsize=(10,7),layout="constrained")
    axes[0,0].plot(ref[:,0],ref[:,1],"--",label="reference")
    axes[0,0].plot(position[:,0],position[:,1],label="simulated aircraft")
    axes[0,0].set(xlabel="x [m]",ylabel="y [m]",title="PD transfer and policy circle")
    axes[0,0].axis("equal");axes[0,0].legend()
    axes[0,1].plot(t,ref[:,2],"--",label="reference")
    axes[0,1].plot(t,position[:,2],label="altitude")
    axes[0,1].set(xlabel="time [s]",ylabel="z [m]",title="Takeoff and altitude hold")
    axes[0,1].legend()
    axes[1,0].plot(t,[r["policy_weight"] for r in control])
    axes[1,0].set(xlabel="time [s]",ylabel="policy authority",ylim=(-.05,1.05),title="Continuous history, gated handover")
    axes[1,1].plot(t[1:],gaps,label="PD timer interval",alpha=.8)
    ages=[1000*r["policy_age_s"] if r["policy_age_s"] is not None else np.nan for r in control]
    axes[1,1].plot(t,ages,label="policy source age",alpha=.6)
    axes[1,1].axhline(40,color="red",ls="--",label="40 ms freshness limit")
    axes[1,1].set(xlabel="time [s]",ylabel="ms",title="Inference runs outside both timers")
    axes[1,1].legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.suptitle(f"ROS mocap / native Zenoh circle ({'simulation clock' if clocked else 'live clock'}): {a.kind} — {'PASS' if summary['passed'] else 'FAIL'}")
    fig.savefig(a.run/"sequence.png",dpi=170)
    fig.savefig(a.run/"sequence.pdf")
    print(json.dumps(summary,indent=2))
    if not summary["passed"]:
        raise SystemExit(1)


if __name__=="__main__":
    main()
