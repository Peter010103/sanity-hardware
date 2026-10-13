#!/usr/bin/env python3
"""Identify Sanity bench models from raw, independently timestamped channels.

Reproducible offline analysis; never opens hardware. numpy, scipy, matplotlib.
Reads data/raw; writes fits and reports to build/actuator_fit.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import sys
from datetime import datetime, timezone

import numpy as np
import scipy
from scipy.optimize import least_squares
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({'lines.linewidth': .8, 'grid.linestyle': ':'})

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
WREF = 2500.0


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        if rows:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


def js(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


def export_parameters(report):
    model = report["models"]
    limits = report["validity"]
    parameters = json.loads((ROOT/"parameters/actuators.json").read_text())
    parameters["thrust"].update(
        k_N_per_rad_s_squared=model["extended_static_thrust"]["k_N_per_rad_s_squared"],
        measured_speed_range_rad_s=limits["extended_static_measured_omega_rad_s"])
    parameters["response"].update(
        delay_s=model["spinning_dynamics"]["fitted_parameters"][0],
        tau_s=model["spinning_dynamics"]["fitted_parameters"][1])
    parameters["command_to_speed"].update(
        coefficients_a_b_c_rad_s=model["steady_speed"]["coefficients_a_b_c_rad_s_M1_to_M4"],
        command_range_counts=limits["spinning_commands_counts"],
        voltage_range_V=limits["new_plateau_voltage_V"])
    return parameters


def write_parameters(path, parameters):
    text = json.dumps(parameters, indent=2, allow_nan=False)
    text = re.sub(r"\[\s*([^\[\]{}]*?)\s*\]",
                  lambda m: json.dumps(json.loads(m.group(0))), text)
    path.write_text(text + "\n")


def metrics(actual, predicted):
    y, p = np.asarray(actual), np.asarray(predicted)
    if not len(y):
        return None
    e = p-y
    return dict(n=len(y), rmse=float(np.sqrt(np.mean(e**2))), mae=float(np.mean(abs(e))),
                bias=float(np.mean(e)), maximum_absolute_error=float(np.max(abs(e))),
                R_squared=None if np.var(y)==0 else float(1-np.sum(e**2)/np.sum((y-y.mean())**2)))


def times(rows, key, origin):
    return np.array([(int(r[key])-origin)/1e9 for r in rows])


def load_runs(root):
    runs = []
    inventory = []
    for d in sorted(root.glob("bench_*")):
        if not d.is_dir() or not (d/"metadata.json").exists():
            continue
        meta = json.loads((d/"metadata.json").read_text())
        summary = json.loads((d/"summary.json").read_text())
        preset = meta["preset"]
        inventory.append(dict(directory=str(d), preset=preset, status=summary["status"],
                              use="mounted calibration diagnostic" if preset=="weights" else
                                  "timing diagnostic" if preset=="probe" else "identification/validation"))
        if preset in ("probe", "weights") or summary["status"] != "completed":
            continue
        rpm = read_csv(d/"rpm.csv")
        force = [r for r in read_csv(d/"force.csv") if r["valid"]=="True"]
        batt = read_csv(d/"battery.csv")
        phase = [r for r in read_csv(d/"phases.csv") if r["completed"]=="True"]
        cmd = read_csv(d/"commands.csv")
        origin = meta["started_monotonic_ns"]
        rt = times(rpm,"reply_ns",origin)
        rstart = times(rpm,"request_start_ns",origin)
        ft = times(force,"ready_observed_ns",origin)
        bt = times(batt,"reply_ns",origin)
        speed = np.array([[float(r["rpm_M%d"%i]) for i in range(1,5)] for r in rpm])*2*np.pi/60
        load = np.array([float(r["load_N"]) for r in force])
        baseline = summary["stopped_baseline_N"]
        thrust = meta["settings"]["thrust_sign"]*(load-baseline)
        voltage = np.array([float(r["voltage_V"]) for r in batt])
        commands = {r["event"]:np.array([int(r["M%d"%i]) for i in range(1,5)]) for r in cmd}
        runs.append(dict(name=d.name,d=d,meta=meta,summary=summary,preset=preset,origin=origin,
                         rt=rt,rstart=rstart,ft=ft,bt=bt,w=speed,load=load,thrust=thrust,V=voltage,
                         phases=phase,commands=commands))
    return runs, inventory


def partition(run, phase):
    if run["preset"] in ("multilevel", "reference") or (run["preset"]=="individual" and phase["role"]=="reference"):
        return "test"
    return "selection" if phase["role"]=="validation" else "train"


def static_points(runs):
    points = []
    for run in runs:
        for p in run["phases"]:
            u = run["commands"][p["command_event"]]
            if np.max(u)<=1000:
                continue
            end = (int(p["end_ns"])-run["origin"])/1e9
            start = max((int(p["ack_ns"])-run["origin"])/1e9,end-.75)
            rm = (run["rstart"]>=start)&(run["rt"]<=end)
            fm = (run["ft"]>=start)&(run["ft"]<=end)
            bm = (run["bt"]>=start)&(run["bt"]<=end)
            if rm.sum()<10 or fm.sum()<5 or bm.sum()<1:
                continue
            w = run["w"][rm]
            point = dict(run=run["name"],phase=p["phase"],partition=partition(run,p),
                         context="individual" if np.sum(u>1000)==1 else "collective",
                         source="independent_channels", voltage_V=float(np.mean(run["V"][bm])),
                         thrust_N=float(np.mean(run["thrust"][fm])), force_sd_N=float(np.std(run["thrust"][fm],ddof=1)),
                         force_n=int(fm.sum()),rpm_n=int(rm.sum()),battery_warning=False)
            # Average powers, not powers of average speed.
            point["omega_samples"] = w
            for i in range(4):
                point["command_M%d"%(i+1)] = int(u[i])
                point["omega_M%d_rad_s"%(i+1)] = float(w[:,i].mean())
                point["omega_squared_M%d"%(i+1)] = float(np.mean(w[:,i]**2))
            points.append(point)
    return points


def legacy_points(root):
    points, inventory = [], []
    for d in sorted(root.glob("thrust_sweep_*")):
        if not (d/"samples.csv").exists():
            continue
        raw = read_csv(d/"samples.csv")
        inventory.append(dict(directory=str(d),use="settled force-versus-speed cross-check/extended-range fit; no dynamic fit",
                              reason="sequential row timing; battery warning/critical at high commands retained as flagged static observations"))
        if not raw:
            continue
        # Same final-ten-sample windows as the earlier sweep analysis.
        for command in sorted({int(r["stage_command"]) for r in raw if r.get("stage")=="command"}):
            if command<=1000:
                continue
            group = [r for r in raw if r.get("stage")=="command" and int(r["stage_command"])==command][-10:]
            if len(group)<5:
                continue
            if any(r["out_of_calibration_range"]=="True" or r["force_limit_exceeded"]=="True" for r in group):
                raise ValueError("Legacy static window has invalid force")
            w = np.array([[float(r["rpm_reported_M%d"%i]) for i in range(1,5)] for r in group])*2*np.pi/60
            f = np.array([float(r["collective_thrust_N"]) for r in group])
            point = dict(run=d.name,phase="command_%d"%command,partition="legacy",context="collective",
                         source="legacy_sweep",voltage_V=float(np.mean([float(r["battery_voltage_V"]) for r in group])),
                         thrust_N=float(f.mean()),force_sd_N=float(f.std(ddof=1)),force_n=len(group),rpm_n=len(group),
                         battery_warning=any(int(r["battery_state_code"])!=0 for r in group),omega_samples=w)
            for i in range(4):
                point["command_M%d"%(i+1)] = command
                point["omega_M%d_rad_s"%(i+1)] = float(w[:,i].mean())
                point["omega_squared_M%d"%(i+1)] = float(np.mean(w[:,i]**2))
            points.append(point)
    return points, inventory


def static_prediction(name, params, points):
    result = []
    for p in points:
        w = p["omega_samples"]/WREF
        if name=="shared_square":
            y=params[0]*np.sum(w**2,axis=1)
        elif name=="rotor_square":
            y=np.sum(np.array(params)*w**2,axis=1)
        elif name=="shared_power":
            y=params[0]*np.sum(w**params[1],axis=1)
        elif name=="rotor_power":
            y=np.sum(np.array(params[:4])*w**params[4],axis=1)
        elif name=="shared_quadratic":
            y=np.sum(np.maximum(params[0]*w**2+params[1]*w,0),axis=1)
        elif name=="shared_threshold_square":
            y=np.sum(np.maximum(params[0]*w**2-params[1],0),axis=1)
        else:
            raise ValueError(name)
        result.append(float(y.mean()))
    return np.array(result)


def static_fit(name, points):
    specs=dict(shared_square=([.2],[0],[2]),rotor_square=([.2]*4,[0]*4,[2]*4),
               shared_power=([.2,2.],[0,1],[2,4]),rotor_power=([.2]*4+[2.],[0]*4+[1],[2]*4+[4]),
               shared_quadratic=([.2,0],[0,-2],[2,2]),
               shared_threshold_square=([.2,.005],[0,0],[2,.1]))
    x,lo,hi=specs[name]
    y=np.array([p["thrust_N"] for p in points])
    # Equal total influence per run, rather than one vote per repeated plateau.
    counts={r:sum(p["run"]==r for p in points) for r in {p["run"] for p in points}}
    weight=np.array([1/np.sqrt(counts[p["run"]]) for p in points])
    fit=least_squares(lambda x:weight*(static_prediction(name,x,points)-y),x,bounds=(lo,hi),loss="soft_l1",f_scale=.03)
    return fit.x


def extract_edges(runs):
    edges, starts, stops = [], [], []
    for run in runs:
        previous=np.array([1000]*4)
        for phase in run["phases"]:
            current=run["commands"][phase["command_event"]]
            event=(int(phase["request_start_ns"])-run["origin"])/1e9
            t=run["rt"]-event
            for i in range(4):
                if current[i]==previous[i]:
                    continue
                if previous[i]<=1000 or current[i]<=1000:
                    kind="startup" if previous[i]<=1000 else "stop"
                else:
                    kind="spinning"
                before=(t>=-.4)&(t<=-.03)
                after=(t>=.65)&(t<=1.25)
                window=(t>=-.10)&(t<=.6)
                if before.sum()<3 or after.sum()<10 or window.sum()<10:
                    continue
                y0=float(np.median(run["w"][before,i])); yf=float(np.median(run["w"][after,i]))
                amp=yf-y0
                if abs(amp)<100:
                    continue
                edge=dict(run=run["name"],phase=phase["phase"],partition=partition(run,phase),motor=i,
                          direction="up" if amp>0 else "down",kind=kind,from_command=int(previous[i]),to_command=int(current[i]),
                          t=t[window],t_mid=(run["rt"][window]+run["rstart"][window])/2-event,
                          y=run["w"][window,i],z=(run["w"][window,i]-y0)/amp,y0=y0,yf=yf,amp=amp,
                          command_ack_delay_s=(int(phase["ack_ns"])-int(phase["request_start_ns"]))/1e9,event=event)
                {"spinning":edges,"startup":starts,"stop":stops}[kind].append(edge)
            previous=current
    return edges,starts,stops


def step_prediction(name, x, edge, timestamp="reply"):
    t=edge["t"] if timestamp=="reply" else edge["t_mid"]
    if name=="first_order": L,tau=0,x[0]
    elif name=="fopdt": L,tau=x
    elif name=="direction_tau": L=x[0]; tau=x[1 if edge["direction"]=="up" else 2]
    elif name=="direction_delay_tau":
        L,tau=x[:2] if edge["direction"]=="up" else x[2:]
    elif name=="rotor_direction_tau":
        L=x[0]; tau=x[1+2*edge["motor"]+(edge["direction"]=="down")]
    elif name=="two_pole":
        L,t1,t2=x
        elapsed=np.maximum(0,t-L)
        if abs(t1-t2)<1e-7:
            return 1-(1+elapsed/t1)*np.exp(-elapsed/t1)
        return 1-(t1*np.exp(-elapsed/t1)-t2*np.exp(-elapsed/t2))/(t1-t2)
    else:raise ValueError(name)
    return 1-np.exp(-np.maximum(0,t-L)/tau)


def dynamic_fit(name, edges, timestamp="reply", x0=None):
    specs=dict(first_order=([.05],[.003],[.3]),fopdt=([.015,.04],[0,.003],[.08,.3]),
               direction_tau=([.015,.03,.05],[0,.003,.003],[.08,.3,.3]),
               direction_delay_tau=([.015,.03,.015,.05],[0,.003,0,.003],[.08,.3,.08,.3]),
               rotor_direction_tau=([.015]+[.04]*8,[0]+[.003]*8,[.08]+[.3]*8),
               two_pole=([.005,.02,.025],[0,.003,.003],[.08,.3,.3]))
    x,lo,hi=specs[name]
    fun=lambda x:np.concatenate([(step_prediction(name,x,e,timestamp)-e["z"])/np.sqrt(len(e["z"])) for e in edges])
    return least_squares(fun,x if x0 is None else x0,bounds=(lo,hi),loss="soft_l1",f_scale=.025).x


def dynamic_metrics(name, x, edges, timestamp="reply"):
    if not edges:return None
    errors=[step_prediction(name,x,e,timestamp)-e["z"] for e in edges]
    return dict(edges=len(edges),samples=sum(len(e) for e in errors),
                normalized_rmse=float(np.sqrt(np.mean([np.mean(e**2) for e in errors]))),
                rpm_rmse=float(np.sqrt(np.mean([np.mean((err*edge["amp"]*60/(2*np.pi))**2) for err,edge in zip(errors,edges)]))),
                interpretation="Conditional transient-shape test using measured pre/post speed levels; not an end-to-end command prediction")


def steady_prediction(name, x, points):
    degree=2 if name.startswith("quadratic") else 1
    exponent=0 if name.endswith("no_voltage") else 1 if name.endswith("voltage_linear") else x[-1]
    coef=np.array(x[:4*(degree+1)]).reshape(4,degree+1)
    result=[]
    for p,i in points:
        q=(p["command_M%d"%(i+1)]-1350)/100
        result.append(float(sum(coef[i,j]*q**j for j in range(degree+1))*(p["voltage_V"]/8)**exponent))
    return np.array(result)


def steady_fit(name, points):
    degree=2 if name.startswith("quadratic") else 1
    n=4*(degree+1)
    x=np.tile([2400,700]+([-30] if degree==2 else []),4).astype(float)
    lo=np.tile([500,100]+([-500] if degree==2 else []),4).tolist()
    hi=np.tile([5000,2000]+([500] if degree==2 else []),4).tolist()
    if name.endswith("voltage_fitted"):
        x=np.r_[x,1.];lo.append(0);hi.append(3)
    y=np.array([p["omega_M%d_rad_s"%(i+1)] for p,i in points])
    counts={r:sum(p["run"]==r for p,i in points) for r in {p["run"] for p,i in points}}
    weights=np.array([1/np.sqrt(counts[p["run"]]) for p,i in points])
    fit=least_squares(lambda x:weights*(steady_prediction(name,x,points)-y),x,bounds=(lo,hi),loss="soft_l1",f_scale=30)
    return fit.x


def choose(scores, score_key, tolerance=.05):
    # Prefer the least complex model within 5% of the best selection RMSE.
    best=min(s[score_key] for s in scores)
    return min([s for s in scores if s[score_key]<=best*(1+tolerance)],key=lambda s:(s["parameters"],s[score_key]))


def speed_target(name, x, commands, voltage):
    degree=2 if name.startswith("quadratic") else 1
    exponent=0 if name.endswith("no_voltage") else 1 if name.endswith("voltage_linear") else x[-1]
    coef=np.array(x[:4*(degree+1)]).reshape(4,degree+1)
    u=np.asarray(commands)
    q=(u-1350)/100
    w=sum(coef[:,j]*q**j for j in range(degree+1))*(voltage/8)**exponent
    return np.where(u<=1000,0,w)


def force_target(name,x,w):
    a=np.asarray(w)/WREF
    if name=="shared_square": return x[0]*np.sum(a**2,axis=-1)
    if name=="shared_power": return x[0]*np.sum(a**x[1],axis=-1)
    if name=="rotor_square": return np.sum(np.asarray(x)*a**2,axis=-1)
    if name=="rotor_power": return np.sum(np.asarray(x[:4])*a**x[4],axis=-1)
    if name=="shared_threshold_square": return np.sum(np.maximum(x[0]*a**2-x[1],0),axis=-1)
    return np.sum(np.maximum(x[0]*a**2+x[1]*a,0),axis=-1)


def delay_tau(name,x,motor,direction):
    if name=="first_order":return 0,x[0]
    if name=="fopdt":return x
    if name=="direction_tau":return x[0],x[1 if direction=="up" else 2]
    if name=="direction_delay_tau":return x[:2] if direction=="up" else x[2:]
    if name=="rotor_direction_tau":return x[0],x[1+2*motor+(direction=="down")]
    raise ValueError("Two-pole simulation not selected/implemented")


def simulate_run(run,speed_model,dynamic_model):
    """Free run from one measured state after warmup; no subsequent state resets.

    Recorded voltage is supplied as an input. Stop/start phases excluded. Exact
    exponential updates on a 1 ms grid. Each command event keeps its own delay.
    """
    phases=[]
    for phase in run["phases"]:
        u=run["commands"][phase["command_event"]]
        if np.max(u)>1000: phases.append(phase)
    # Collective held-out runs contain one initial warmup and uninterrupted spinning.
    first=phases[0]
    start=(int(first["end_ns"])-run["origin"])/1e9-.20
    end=(int(phases[-1]["end_ns"])-run["origin"])/1e9
    grid=np.arange(start,end,.001)
    u0=run["commands"][first["command_event"]]
    events=[(start,u0)]
    for phase in phases[1:]:
        event=(int(phase["request_start_ns"])-run["origin"])/1e9
        events.append((event,run["commands"][phase["command_event"]]))
    delayed=[]
    old=u0.copy()
    for at,u in events[1:]:
        for i in range(4):
            if old[i]!=u[i]:
                direction="up" if u[i]>old[i] else "down"
                L,tau=delay_tau(dynamic_model["candidate"],dynamic_model["fitted_parameters"],i,direction)
                delayed.append((at+L,i,u[i],tau))
        old=u.copy()
    delayed.sort()
    current=u0.copy(); state=np.array([np.interp(start,run["rt"],run["w"][:,i]) for i in range(4)])
    tau=np.array([delay_tau(dynamic_model["candidate"],dynamic_model["fitted_parameters"],i,"up")[1] for i in range(4)])
    prediction=np.empty((len(grid),4));index=0
    voltage=run["V"][np.clip(np.searchsorted(run["bt"],grid,side="right")-1,0,len(run["V"])-1)]
    for j,t in enumerate(grid):
        while index<len(delayed) and delayed[index][0]<=t:
            _,i,u,tau_i=delayed[index];current[i]=u;tau[i]=tau_i;index+=1
        target=speed_target(speed_model["candidate"],speed_model["fitted_parameters"],current,voltage[j])
        if j:state=target+(state-target)*np.exp(-(grid[j]-grid[j-1])/tau)
        prediction[j]=state
    return grid,prediction


def filtered_signal(t,y,sample_t,L,tau):
    grid=np.arange(t[0],max(sample_t[-1],t[-1])+.002,.002)
    u=np.interp(grid-L,t,y,left=y[0],right=y[-1])
    alpha=np.exp(-.002/max(tau,1e-9))
    from scipy.signal import lfilter
    filtered,_=lfilter([1-alpha],[1,-alpha],u,zi=[alpha*u[0]])
    return np.interp(sample_t,grid,filtered)


def fit_observation(runs, static_model):
    candidates=[]
    targets=[]
    for run in runs:
        if run["preset"]=="individual":continue
        use=np.zeros(len(run["ft"]),dtype=bool)
        for phase in run["phases"]:
            if partition(run,phase)!="train" or phase["role"]=="stopped":continue
            a=(int(phase["ack_ns"])-run["origin"])/1e9
            b=(int(phase["end_ns"])-run["origin"])/1e9
            use|=(run["ft"]>=a)&(run["ft"]<=b)
        if use.any():
            truth=force_target(static_model["candidate"],static_model["fitted_parameters"],run["w"])
            targets.append((run,truth,use))
    for name in ["instantaneous","delay_only","delay_first_order"]:
        def residual(x):
            L=0 if name=="instantaneous" else x[0]
            tau=x[1] if name=="delay_first_order" else 1e-7
            return np.concatenate([(filtered_signal(run["rt"],truth,run["ft"],L,tau)[use]-run["thrust"][use])/np.sqrt(use.sum()) for run,truth,use in targets])
        if name=="instantaneous":x=[]
        else:
            initial=[.10,.05] if name=="delay_first_order" else [.12]
            bounds=([0,.005],[.35,.3]) if name=="delay_first_order" else ([0],[.35])
            x=least_squares(residual,initial,bounds=bounds,loss="soft_l1",f_scale=.015).x.tolist()
        data=dict(candidate=name,fitted_parameters=x,parameters=len(x),measurement_model_only=True)
        for label in ["train","test"]:
            yy=[];pp=[]
            for run in runs:
                if run["preset"]=="individual":continue
                if (run["preset"] in ["multilevel","reference"]) != (label=="test"):continue
                # Ignore stopped windows, include all commanded spinning phases and startup.
                mask=np.zeros(len(run["ft"]),dtype=bool)
                for phase in run["phases"]:
                    if max(run["commands"][phase["command_event"]])<=1000:continue
                    a=(int(phase["ack_ns"])-run["origin"])/1e9;b=(int(phase["end_ns"])-run["origin"])/1e9
                    if label=="train" and partition(run,phase)!="train":continue
                    mask|=(run["ft"]>=a)&(run["ft"]<=b)
                f=force_target(static_model["candidate"],static_model["fitted_parameters"],run["w"])
                L=x[0] if x else 0;tau=x[1] if len(x)>1 else 1e-7
                pp.extend(filtered_signal(run["rt"],f,run["ft"],L,tau)[mask]);yy.extend(run["thrust"][mask])
            data[label]=metrics(yy,pp)
        candidates.append(data)
    return candidates


def bootstrap_parameters(name, data, fit, repeats, rng):
    groups={}
    for item in data:
        groups.setdefault((item["run"],item["phase"]),[]).append(item)
    blocks=list(groups.values()); samples=[]
    for _ in range(repeats):
        picked=[item for j in rng.integers(0,len(blocks),len(blocks)) for item in blocks[j]]
        samples.append(fit(name,picked))
    if not samples:return None
    a=np.array(samples)
    return dict(method="resample complete command/plateau blocks (all simultaneous rotors together)",
                repetitions=repeats,lower_2_5_percent=np.quantile(a,.025,axis=0).tolist(),upper_97_5_percent=np.quantile(a,.975,axis=0).tolist(),
                limitations="Conditional on recorded calibration, pole scaling, host timestamp convention and fixture; systematic uncertainty excluded")


def finish(preliminary,runs,points,legacy,edges,starts,stops,speed_points,args):
    out=args.output;selected=preliminary["selected"];rng=np.random.default_rng(260613)
    final={}
    for component,fit,data in [("static_thrust",static_fit,points),("spinning_dynamics",dynamic_fit,edges),("steady_speed",steady_fit,speed_points)]:
        name=selected[component]["candidate"]
        x=fit(name,data)
        final[component]=dict(candidate=name,fitted_parameters=x.tolist(),fit_scope="refitted after selection using all applicable v2 bench data",
                              independent_test_metrics_for_training_only_version=selected[component]["test"])
        if component!="steady_speed":
            final[component]["conditional_bootstrap_interval"]=bootstrap_parameters(name,data,fit,args.bootstrap,rng)
    # Extended thrust profile: include legacy speed/force points, regardless of
    # battery state, but never reuse them for command mapping or fast dynamics.
    extended=[]
    older_train=legacy[::2];older_selection=legacy[1::2]
    new_train=[p for p in points if p["partition"]=="train"]
    new_selection=[p for p in points if p["partition"]=="selection"]
    new_test=[p for p in points if p["partition"]=="test"]
    for name in ["shared_square","shared_power","shared_threshold_square","shared_quadratic","rotor_square","rotor_power"]:
        x=static_fit(name,new_train+older_train)
        row=dict(candidate=name,parameters=len(x),fitted_parameters=x.tolist())
        for label,data in [("new_selection",new_selection),("legacy_selection",older_selection),("new_test",new_test)]:
            row[label]=metrics([a["thrust_N"] for a in data],static_prediction(name,x,data))
        row["selection_rmse"]=math.sqrt((row["new_selection"]["rmse"]**2+row["legacy_selection"]["rmse"]**2)/2)
        extended.append(row)
    best_wide=choose(extended,"selection_rmse")
    wide_x=static_fit(best_wide["candidate"],points+legacy)
    final["extended_static_thrust"]=dict(candidate=best_wide["candidate"],fitted_parameters=wide_x.tolist(),
        status="descriptive full-range profile, including flagged high-load legacy samples",
        selection_metrics=best_wide,all_data_fit=metrics([p["thrust_N"] for p in points+legacy],static_prediction(best_wide["candidate"],wide_x,points+legacy)),
        limitations=["Only one ascending high-speed sweep, with battery warning/critical and an incomplete final stage.",
                     "Held-out legacy levels share the same run; they are not independent high-speed repetitions.",
                     "No valid command-to-speed or transient extension above 1450 counts follows from this fit."])
    final["extended_static_thrust"]["conditional_bootstrap_interval"]=bootstrap_parameters(best_wide["candidate"],points+legacy,static_fit,args.bootstrap,rng)
    js(out/"extended_static_comparison.json",extended)
    observation=fit_observation(runs,selected["static_thrust"])
    observation_best=min(observation,key=lambda r:r["train"]["rmse"])
    js(out/"loadcell_observation_comparison.json",observation)
    final["loadcell_observation"]=dict(**observation_best,
        meaning="Descriptive delay/filter from recorded RPM-derived thrust to HX711 force; includes acquisition/fixture/filter effects. Do not apply to simulated aerodynamic force.",
        fit_scope="training force/RPM records only; retained without test refit")
    # Edge-by-edge fit reveals variation and startup/stop behavior separately.
    edge_table=[]
    for e in edges+starts+stops:
        def fun(x):
            return 1-np.exp(-np.maximum(0,e["t"]-x[0])/x[1])-e["z"]
        x=least_squares(fun,[.02,.04],bounds=([0,.003],[.5,.5]),loss="soft_l1",f_scale=.05).x
        pred=fun(x)+e["z"]
        post=e["t"]>=0
        observed=[]
        for q in [.1,.9]:
            inds=np.where(post&(e["z"]>=q))[0]
            observed.append(None if not len(inds) else float(e["t"][inds[0]]))
        edge_table.append(dict(run=e["run"],phase=e["phase"],partition=e["partition"],motor=e["motor"]+1,
            kind=e["kind"],direction=e["direction"],from_command=e["from_command"],to_command=e["to_command"],
            delay_s=float(x[0]),tau_s=float(x[1]),normalized_rmse=float(np.sqrt(np.mean((pred-e["z"])**2))),
            y0_rad_s=e["y0"],yfinal_rad_s=e["yf"],observed_10_percent_time_s=observed[0],observed_90_percent_time_s=observed[1],
            command_ack_delay_s=e["command_ack_delay_s"]))
    write_csv(out/"edge_fits.csv",edge_table)
    diagnostic={}
    for kind in ["spinning","startup","stop"]:
        group=[r for r in edge_table if r["kind"]==kind]
        diagnostic[kind]=dict(n=len(group),median_fit_delay_s=float(np.median([r["delay_s"] for r in group])),
            median_fit_tau_s=float(np.median([r["tau_s"] for r in group])),
            median_observed_command_to_90_s=float(np.median([r["observed_90_percent_time_s"] for r in group if r["observed_90_percent_time_s"] is not None])))
    final["startup_stop_diagnostics"]=dict(summary=diagnostic,
        status="not incorporated into spinning model",
        caution="Startup has ESC synchronization/telemetry acquisition behavior; zero telemetry during stop does not independently prove physical rotor stop")
    midpoint=dynamic_fit(final["spinning_dynamics"]["candidate"],edges,timestamp="midpoint")
    final["spinning_dynamics"]["request_reply_midpoint_sensitivity_parameters"]=midpoint.tolist()
    final["spinning_dynamics"]["timing_meaning"]="Host command send-start to FC-RPM reply. Cannot separate USB/FC/ESC/telemetry delay from physical motor response."
    # Untouched-test end-to-end predictions use TRAINING parameters, with one
    # initial measured state after warmup and recorded causal voltage input.
    simulations=[];simulation_records=[]
    for run in runs:
        if run["preset"] not in ("multilevel","reference"):continue
        grid,w=simulate_run(run,selected["steady_speed"],selected["spinning_dynamics"])
        mask=(run["rt"]>=grid[0])&(run["rt"]<=grid[-1])
        predicted=np.column_stack([np.interp(run["rt"][mask],grid,w[:,i]) for i in range(4)])
        fm=(run["ft"]>=grid[0]+.35)&(run["ft"]<=grid[-1])
        model_force=force_target(selected["static_thrust"]["candidate"],selected["static_thrust"]["fitted_parameters"],w)
        op=observation_best["fitted_parameters"]
        filtered=filtered_signal(grid,model_force,run["ft"][fm],op[0] if op else 0,op[1] if len(op)>1 else 1e-7)
        score=dict(run=run["name"],rpm=metrics((run["w"][mask]*60/(2*np.pi)).ravel(),(predicted*60/(2*np.pi)).ravel()),
            observed_loadcell_force_N=metrics(run["thrust"][fm],filtered),
            initial_state="one measured rotor state after initial warmup; no later state reset",
            input="recorded command transitions and causally held FC voltage; voltage evolution is not predicted")
        simulation_records.append(score)
        simulations.append((run,grid,w,model_force,fm,filtered))
        data=[]
        for j,t in enumerate(run["rt"][mask]):
            row=dict(time_s=float(t),run=run["name"])
            for i in range(4):
                row["measured_RPM_M%d"%(i+1)]=float(run["w"][mask][j,i]*60/(2*np.pi))
                row["predicted_RPM_M%d"%(i+1)]=float(predicted[j,i]*60/(2*np.pi))
            data.append(row)
        write_csv(out/(run["preset"]+"_heldout_predictions.csv"),data)
    final["end_to_end_test"]=simulation_records
    # An explicit simpler conventional square-law fit is useful for simulators.
    square=static_fit("shared_square",points)[0]/WREF**2
    final["conventional_square_law_alternative"]=dict(k_N_per_rad_s_squared=float(square),
        equation="T_i=k*omega_i**2; T_collective=sum(T_i)",
        training_only_independent_test_metrics=next(s for s in preliminary["candidates"] if s["candidate"]=="shared_square")["test"],
        status="simpler approximation; selected local model predicts reserved plateaus more accurately")
    equations=dict(shared_square="T_i=alpha*(omega_i/2500)^2",rotor_square="T_i=alpha_i*(omega_i/2500)^2",
                   shared_power="T_i=alpha*(omega_i/2500)^p",rotor_power="T_i=alpha_i*(omega_i/2500)^p",
                   shared_quadratic="T_i=max(alpha*(omega_i/2500)^2+beta*(omega_i/2500),0)",
                   shared_threshold_square="T_i=max(alpha*(omega_i/2500)^2-beta,0)")
    for key in ["static_thrust","extended_static_thrust"]:
        final[key]["equation"]=equations[final[key]["candidate"]]+"; T_collective=sum_i T_i"
        final[key]["warning"]="Empirical speed/force relationship; any mathematical low-speed cutoff does not identify ESC command deadband."
    final["static_thrust"]["k_N_per_rad_s_squared"]=float(final["static_thrust"]["fitted_parameters"][0]/WREF**2)
    final["extended_static_thrust"]["k_N_per_rad_s_squared"]=float(final["extended_static_thrust"]["fitted_parameters"][0]/WREF**2)
    final["steady_speed"]["equation"]="q=(command-1350)/100; omega_ss_i=(V/8)*(a_i+b_i*q+c_i*q^2)"
    final["steady_speed"]["coefficients_a_b_c_rad_s_M1_to_M4"]=np.asarray(final["steady_speed"]["fitted_parameters"][:12]).reshape(4,3).tolist()
    final["spinning_dynamics"]["equation"]="domega_i/dt=(omega_ss_i(u_i(t-L),V(t))-omega_i)/tau"
    weights=[]
    for d in args.experiments.glob("bench_weights_*"):
        for r in read_csv(d/"weight_checks.csv"):
            weights.append(dict(run=d.name,cycle=int(r["cycle"]),error_N=float(r["error_N"]),
                                relative_error=float(r["error_N"])/float(r["expected_delta_N"])))
    max_w=max(float(np.max(p["omega_samples"])) for p in points)
    min_w=min(float(np.min(p["omega_samples"][p["omega_samples"]>0])) for p in points)
    voltage=[p["voltage_V"] for p in points]
    motor_coef=np.asarray(final["steady_speed"]["fitted_parameters"][:12]).reshape(4,3)
    checks=dict(parameters_finite=all(np.all(np.isfinite(m["fitted_parameters"])) for m in final.values() if "fitted_parameters" in m),
                steady_speed_monotonic_1250_to_1450=bool(np.all(motor_coef[:,1]-2*abs(motor_coef[:,2])>0)),
                static_thrust_nonnegative_in_range=bool(np.all(force_target(final["static_thrust"]["candidate"],final["static_thrust"]["fitted_parameters"],np.tile(np.linspace(0,max_w,101)[:,None],(1,4)))>=0)),
                zero_speed_zero_thrust=bool(force_target(final["static_thrust"]["candidate"],final["static_thrust"]["fitted_parameters"],np.zeros(4))==0),
                dynamic_time_constant_positive=bool(final["spinning_dynamics"]["fitted_parameters"][-1]>0),
                hardware_policy_deployment_validated=False)
    if not all(checks[k] for k in checks if k!="hardware_policy_deployment_validated"):
        raise ValueError("Fitted model failed consistency checks")
    old_steps=[dict(directory=str(d),use="excluded from fast-dynamics fitting",reason="old logger queried RPM before blocking HX711, with later shared row timestamp; see data/raw/steps/timing_audit.json")
               for d in (ROOT/"data/raw/steps").glob("motor_step_*") if d.is_dir()]
    sources={str(run["d"].relative_to(args.experiments.parent)): {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in run["d"].iterdir() if p.name in ("rpm.csv","force.csv","commands.csv","phases.csv","battery.csv","metadata.json","summary.json","calibration_reference.json")} for run in runs}
    for d in args.sweeps.glob("thrust_sweep_*"):
        if (d/"samples.csv").exists():
            sources[str(d.relative_to(args.sweeps.parent))]={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in d.iterdir() if p.name in ("samples.csv","metadata.json","summary.json")}
    output=dict(schema_version=1,generated_utc=datetime.now(timezone.utc).isoformat(),readiness="bench-identified actuator model; not a validated aircraft dynamics or policy-deployment model",
        rotor_order="hardware outputs M1,M2,M3,M4; physical position/spin direction not inferred",omega_units="rad/s from FC reported RPM*2*pi/60; FC-configured 12 poles unverified",
        thrust_reference_omega_rad_s=WREF,models=final,selection=preliminary,
        profiles=dict(compact_full_range=dict(thrust="extended_static_thrust",speed="steady_speed",dynamics="spinning_dynamics",
                                              note="one-coefficient square law fits all 51 settled points; high-speed static evidence remains flagged"),
                      local_accuracy=dict(thrust="static_thrust",speed="steady_speed",dynamics="spinning_dynamics",
                                          note="offset correction improves new-range force accuracy; do not extrapolate correction to high speed")),
        validity=dict(spinning_commands_counts=[1250,1450],new_measured_omega_rad_s=[min_w,max_w],new_plateau_voltage_V=[min(voltage),max(voltage)],
                      extended_static_measured_omega_rad_s=[min(float(p["omega_samples"].min()) for p in legacy),max(float(p["omega_samples"].max()) for p in legacy)],
                      notes="Ranges are envelopes of observations, not an independently sampled Cartesian product of command, speed and voltage. No extrapolation guarantee."),
        mounted_weight_check=weights,excluded_legacy_dynamic_inputs=old_steps,
        uncertainty_notes=["Statistical intervals exclude force calibration/fixture scale, physical pole count and unknown FC telemetry sample age.",
                           "Individual rotor force measurements remain conditional on the operator-declared off-axis fixture check; centre weight check does not independently prove it.",
                           "No reaction-torque coefficient, physical yaw-moment model, battery dynamics or flying inertia identified.",
                           "This MSP direct motor-test map is distinct from Sanity's flight-controller attitude/collective-thrust policy interface."],
        checks=checks,source_sha256=sources,fitting_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        tools=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,matplotlib=matplotlib.__version__,seed=260613),
        reproduce="python3 tools/fit_actuators.py --bootstrap %d"%args.bootstrap)
    js(out/"fit_report.json",output)
    write_parameters(out/"actuators.json", export_parameters(output))
    # Full-range and independent-test plots.
    fig,ax=plt.subplots(2,1,figsize=(11,8),sharex=True)
    markers={"train":"o","selection":"s","test":"^","legacy":"x"}
    for label in ["train","selection","test","legacy"]:
        group=[p for p in points+legacy if p["partition"]==label]
        xx=[np.sqrt(sum(p["omega_squared_M%d"%i] for i in range(1,5))/4) for p in group]
        yy=[p["thrust_N"] for p in group]
        # Plot individual points as equal-speed collective equivalents; distinguish clearly.
        for context in ["collective","individual"]:
            subset=[(x,y,p) for x,y,p in zip(xx,yy,group) if p["context"]==context]
            if not subset:continue
            x=[a[0]*(2 if context=="individual" else 1) for a in subset]
            y=[a[1]*(4 if context=="individual" else 1) for a in subset]
            ax[0].scatter(x,y,marker=markers[label],s=30,alpha=.75,label=label+" "+context+(" (4× visualization only)" if context=="individual" else ""))
    sweep=np.linspace(0,5400,500)
    for key,color in [("static_thrust","tab:blue"),("extended_static_thrust","tab:orange")]:
        model=final[key]; f=force_target(model["candidate"],model["fitted_parameters"],np.tile(sweep[:,None],(1,4)))
        if key=="static_thrust":
            inside=(sweep>=min_w)&(sweep<=max_w)
            ax[0].plot(sweep,np.where(inside,f,np.nan),color=color,label="local fit (new measured range)")
            ax[0].plot(sweep,np.where(~inside,f,np.nan),color=color,ls="--",alpha=.55,label="local extrapolation diagnostic only")
        else:
            ax[0].plot(sweep,f,color=color,label="full-range square law")
    ax[0].set_ylabel("Collective thrust or scaled individual display (N)")
    ax[0].legend(fontsize=7,ncol=2);ax[0].grid(alpha=.2)
    for label in ["train","selection","test","legacy"]:
        group=[p for p in points+legacy if p["partition"]==label]
        xx=[np.sqrt(sum(p["omega_squared_M%d"%i] for i in range(1,5))/max(1,sum(p["command_M%d"%i]>1000 for i in range(1,5)))) for p in group]
        residual=static_prediction(final["static_thrust"]["candidate"],final["static_thrust"]["fitted_parameters"],group)-[p["thrust_N"] for p in group]
        ax[1].scatter(xx,residual,marker=markers[label],s=25,label=label)
    ax[1].axhline(0,color="black",lw=.8);ax[1].set_ylabel("Local-model residual: predicted − measured (N)")
    ax[1].set_xlabel("RMS rotor speed (rad/s); individual display corrected to single-rotor speed")
    ax[1].grid(alpha=.2);fig.tight_layout();fig.savefig(out/"thrust_model.png",dpi=160);plt.close(fig)
    fig,ax=plt.subplots(2,2,figsize=(11,8),sharex=True,sharey=True)
    for i in range(4):
        a=ax.ravel()[i]
        for direction,color in [("up","tab:blue"),("down","tab:orange")]:
            for e in edges:
                if e["motor"]==i and e["direction"]==direction:
                    a.scatter(e["t"],e["z"],s=4,alpha=.15,color=color)
            dummy=dict(t=np.linspace(-.03,.30,300),motor=i,direction=direction)
            a.plot(dummy["t"],step_prediction(final["spinning_dynamics"]["candidate"],final["spinning_dynamics"]["fitted_parameters"],dummy),color=color,label=direction)
        a.set_title("M%d normalized spinning edges"%(i+1));a.grid(alpha=.2);a.legend()
    fig.supxlabel("Time from host command send (s)");fig.supylabel("Fraction of measured speed step");fig.tight_layout();fig.savefig(out/"spinning_response.png",dpi=160);plt.close(fig)
    for run,grid,w,model_force,fm,filtered in simulations:
        fig,ax=plt.subplots(3,1,figsize=(12,9),sharex=True)
        for i in range(4):
            ax[0].plot(run["rt"],run["w"][:,i]*60/(2*np.pi),lw=.8,alpha=.65,label="measured M%d"%(i+1))
            ax[0].plot(grid,w[:,i]*60/(2*np.pi),lw=.8,ls="--",label="predicted M%d"%(i+1))
        ax[0].set_ylabel("Reported mechanical RPM");ax[0].legend(fontsize=7,ncol=4)
        ax[1].plot(run["ft"],run["thrust"],"k.",ms=4,label="measured axial force")
        ax[1].plot(grid,model_force,label="Modelled rotor thrust (sensor lag excluded)",alpha=.7)
        ax[1].plot(run["ft"][fm],filtered,label="predicted load-cell observation")
        ax[1].set_ylabel("Force (N)");ax[1].legend(fontsize=8)
        ax[2].plot(run["bt"],run["V"],label="measured voltage supplied to model");ax[2].set_ylabel("FC voltage (V)")
        ax[2].set_xlabel("Host time from recording start (s)")
        for a in ax:a.grid(alpha=.2);a.set_xlim(grid[0],grid[-1])
        fig.suptitle(run["preset"]+": held-out prediction using training-only parameters")
        fig.tight_layout();fig.savefig(out/(run["preset"]+"_validation.png"),dpi=160);plt.close(fig)
    report=["SANITY ACTUATOR IDENTIFICATION", "", "Readiness: "+output["readiness"],
            "Model selection: "+preliminary["selection_rule"],"No hardware commands were sent during fitting.","",
            "PRIMARY LOCAL MODEL (refit after held-out evaluation)",
            "Omega is FC-reported mechanical RPM * 2*pi/60, in rad/s. Physical 12-pole count unverified.",
            "q=(MSP command-1350)/100; omega_ss=(V/8)*(a_i+b_i*q+c_i*q^2).",
            "Hardware output order M1..M4; coefficient rows:",str(np.asarray(final["steady_speed"]["fitted_parameters"][:12]).reshape(4,3)),
            "omega response: "+str(final["spinning_dynamics"]),
            "Local thrust: "+final["static_thrust"]["equation"],
            str(final["static_thrust"]),"", "FULL-RANGE STATIC PROFILE",str(final["extended_static_thrust"]),"",
            "INDEPENDENT END-TO-END TESTS",json.dumps(simulation_records,indent=2),"",
            "Measured voltage was supplied; battery evolution is not modeled. One initial state after warmup; no subsequent state reset.",
            "The validation scores use training-only parameters. Final export subsequently refits selected forms to all v2 data; those exact coefficients have no untouched test left.",
            "Normalized edge-shape scores use measured pre/post levels; distinguish them from full command prediction.","",
            "LOAD-CELL OBSERVATION (do not apply this filter to simulator aerodynamic force)",str(final["loadcell_observation"]),"",
            "VALID OPERATING ENVELOPES",json.dumps(output["validity"],indent=2),"",
            "DATA ACCOUNTING",json.dumps(preliminary["counts"],indent=2),
            "All five powered v2 runs completed. %d spinning rotor edges share %d command events, not %d independent experiments."%
                (len(edges),len({(e["run"],e["phase"]) for e in edges}),len(edges)),
            "21 startup and 21 stop rotor traces kept as diagnostics; spin-up synchronization and telemetry loss differ from spinning dynamics.",
            "Old sequential step logs excluded from fast dynamics due to row timing bias. Their data and previous fits remain unchanged.",
            "Old full sweep supplies 10 flagged static points, including incomplete maximum stage; no high-command map or motor dynamics inferred.",
            "Individual rotor thrust use is conditional on the declared off-axis fixture check. Centre weight checks alone do not prove that check.","",
            "WEIGHT CHECK",json.dumps(weights,indent=2),"", "LIMITATIONS",*output["uncertainty_notes"],"",
            "FILES", "actuators.json: compact parameters; fit_report.json: diagnostics, uncertainty and provenance.",
            "candidate_comparison.json / extended_static_comparison.json: all candidate scores.",
            "static_points.csv / edge_fits.csv: processed measurements and per-edge diagnostics.",
            "*_heldout_predictions.csv: time-matched raw speed versus held-out prediction.",
            "thrust_model.png / spinning_response.png / *_validation.png: fits, residuals and held-out overlays.","",
            "REPRODUCE",output["reproduce"],"Requirements: numpy, scipy, matplotlib. Raw recordings never modified."]
    (out/"MODEL_REPORT.txt").write_text("\n".join(report)+"\n")
    print(json.dumps(dict(final_models={k:v for k,v in final.items() if k in ["static_thrust","spinning_dynamics","extended_static_thrust","steady_speed"]},end_to_end_test=simulation_records),indent=2))
    return output


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--experiments",type=Path,default=ROOT/"data/raw/experiments")
    p.add_argument("--sweeps",type=Path,default=ROOT/"data/raw/sweeps")
    p.add_argument("--output",type=Path,default=ROOT/"build/actuator_fit")
    p.add_argument("--bootstrap",type=int,default=100)
    args=p.parse_args(argv)
    args.output.mkdir(parents=True,exist_ok=True)
    runs,inventory=load_runs(args.experiments)
    points=static_points(runs)
    legacy,legacy_inventory=legacy_points(args.sweeps)
    inventory+=legacy_inventory
    edges,starts,stops=extract_edges(runs)
    selected={}; scores=[]
    static_candidates=["shared_square","shared_power","rotor_square","shared_quadratic","rotor_power","shared_threshold_square"]
    train=[r for r in points if r["partition"]=="train"]
    selection=[r for r in points if r["partition"]=="selection"]
    test=[r for r in points if r["partition"]=="test"]
    for name in static_candidates:
        x=static_fit(name,train)
        row=dict(component="static_thrust",candidate=name,parameters=len(x),fitted_parameters=x.tolist())
        for label,data in [("train",train),("selection",selection),("test",test),("legacy",legacy)]:
            row[label]=metrics([a["thrust_N"] for a in data],static_prediction(name,x,data))
        row["selection_rmse"]=row["selection"]["rmse"]
        scores.append(row)
    selected["static_thrust"]=choose(scores,"selection_rmse")
    dynamic_names=["first_order","fopdt","direction_tau","direction_delay_tau","rotor_direction_tau","two_pole"]
    for name in dynamic_names:
        x=dynamic_fit(name,[e for e in edges if e["partition"]=="train"])
        row=dict(component="spinning_dynamics",candidate=name,parameters=len(x),fitted_parameters=x.tolist())
        for label in ["train","selection","test"]:
            row[label]=dynamic_metrics(name,x,[e for e in edges if e["partition"]==label])
        row["selection_rmse"]=row["selection"]["normalized_rmse"]
        scores.append(row)
    selected["spinning_dynamics"]=choose([s for s in scores if s["component"]=="spinning_dynamics"],"selection_rmse")
    speed_points=[(point,i) for point in points for i in range(4) if point["command_M%d"%(i+1)]>1000]
    for name in ["linear_no_voltage","linear_voltage_linear","linear_voltage_fitted","quadratic_no_voltage","quadratic_voltage_linear","quadratic_voltage_fitted"]:
        x=steady_fit(name,[(p,i) for p,i in speed_points if p["partition"]=="train"])
        row=dict(component="steady_speed",candidate=name,parameters=len(x),fitted_parameters=x.tolist())
        for label in ["train","selection","test"]:
            group=[(p,i) for p,i in speed_points if p["partition"]==label]
            row[label]=metrics([p["omega_M%d_rad_s"%(i+1)] for p,i in group],steady_prediction(name,x,group))
        row["selection_rmse"]=row["selection"]["rmse"]
        scores.append(row)
    selected["steady_speed"]=choose([s for s in scores if s["component"]=="steady_speed"],"selection_rmse")
    preliminary=dict(selection_rule="least complex within 5% of lowest selection RMSE; test runs untouched during selection",
                     selected=selected,candidates=scores,counts=dict(static=len(points),legacy=len(legacy),normal_edges=len(edges),normal_command_events=len({(e["run"],e["phase"]) for e in edges}),startup_edges=len(starts),stop_edges=len(stops)),inventory=inventory)
    js(args.output/"candidate_comparison.json",preliminary)
    write_csv(args.output/"static_points.csv",[{k:v for k,v in a.items() if k!="omega_samples"} for a in points+legacy])
    print(json.dumps(dict(counts=preliminary["counts"],selected=selected),indent=2))
    return finish(preliminary,runs,points,legacy,edges,starts,stops,speed_points,args)


if __name__=="__main__":
    main()
