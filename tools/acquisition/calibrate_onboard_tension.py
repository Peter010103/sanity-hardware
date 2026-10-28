#!/usr/bin/env python3
"""Read-only ESP/HX711 UDP capture, hanging-drone calibration, and offline fit.

Force is absolute tension, not thrust. No aircraft/receiver commands are sent.
Only Python's standard library is required except for optional plotting.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import socket
import statistics
import time

from hx711_timing import validate_timing_extension

G0 = 9.80665
SCHEMA = "sanity_hx711_v1"


class ADCRecoveryTimeout(RuntimeError):
    """No complete clean window before the recovery budget expired."""


def positive(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("Must be finite and positive")
    return value


def masses(text):
    try:
        values = [float(x) for x in text.split(",")]
    except ValueError as error:
        raise argparse.ArgumentTypeError("Comma-separated masses in grams") from error
    if not values or any(not math.isfinite(x) or x < 0 for x in values):
        raise argparse.ArgumentTypeError("Masses must be finite and nonnegative")
    return values


def decode_packet(data, device):
    obj = json.loads(data)
    if not isinstance(obj, dict):
        raise ValueError("Packet must be an object")
    if obj.get("schema") != SCHEMA or type(obj.get("device_id")) is not int or obj.get("device_id") != device:
        raise ValueError("Wrong schema/device")
    keys = ("seq", "ready_us", "read_us", "raw", "flags", "max_high_us", "send_drops", "resets")
    if any(type(obj.get(key)) is not int for key in keys):
        raise ValueError("Noninteger sample field")
    if not isinstance(obj.get("boot_id"), str) or len(obj["boot_id"]) != 16:
        raise ValueError("Missing boot identity")
    int(obj["boot_id"], 16)
    if not 0 <= obj["seq"] <= 0xffffffff or not -8388608 <= obj["raw"] <= 8388607:
        raise ValueError("ADC or sequence out of range")
    if obj["channel"] != "A" or obj["gain"] != 128 or obj["dout_gpio"] != 1 or obj["sck_gpio"] != 2:
        raise ValueError("Wrong channel/gain/pins")
    if obj["flags"] < 0 or obj["flags"] & ~15:
        raise ValueError("Unknown status flags")
    if min(obj[k] for k in ("ready_us", "read_us", "max_high_us", "send_drops", "resets")) < 0:
        raise ValueError("Negative timestamp/counter")
    if obj["flags"] == 0 and (obj["raw"] in (-8388608, 8388607) or obj["max_high_us"] > 45 or
                                not 0 < obj["ready_us"] <= obj["read_us"]):
        raise ValueError("Invalid sample presented as healthy")
    if any(obj[k]>0xffffffff for k in ("send_drops","resets")) or any(obj[k]>(1<<63)-1 for k in ("ready_us","read_us")):
        raise ValueError("Timestamp/counter overflow")
    validate_timing_extension(obj)
    return obj


class UDPStream:
    def __init__(self, args, raw_file):
        self.device = args.device_id
        self.source_ip = args.source_ip
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((args.bind, args.port))
        self.socket.settimeout(0.2)
        self.file = raw_file
        self.boot = None
        self.peer = None
        self.sequence = None
        self.last_ready = None
        self.received = self.invalid = self.flagged = self.gaps = self.duplicates = 0
        self.phase = "startup"
        self.converter_resets = None
        self.last_fault = None
        self.capture_diagnostics = {}

    def receive(self):
        data, address = self.socket.recvfrom(4096)
        host_ns = time.monotonic_ns()
        if self.source_ip and address[0] != self.source_ip:
            self.invalid += 1
            return None
        try:
            packet = decode_packet(data, self.device)
        except (ValueError, KeyError, TypeError):
            self.invalid += 1
            return None
        packet.update(host_monotonic_ns=host_ns, source_ip=address[0], phase=self.phase)
        self.file.write(json.dumps(packet, allow_nan=False) + "\n")
        self.received += 1
        if self.boot is None:
            self.boot, self.peer = packet["boot_id"], address[0]
        if self.boot != packet["boot_id"] or self.peer != address[0]:
            raise RuntimeError("ESP restarted or source changed; start a new calibration session")
        if self.sequence is not None:
            delta = (packet["seq"] - self.sequence) & 0xffffffff
            if delta == 0 or delta > 0x7fffffff:
                self.duplicates += 1
                return None
            self.gaps += delta - 1
        self.sequence = packet["seq"]
        if self.converter_resets is not None and ((packet["resets"]-self.converter_resets)&0xffffffff)>0x7fffffff:
            raise RuntimeError("ADC reset counter reversed within a boot")
        self.converter_resets = packet["resets"]
        if packet["flags"]:
            self.flagged += 1
            if packet["flags"] & 7:
                self.last_fault = {key:packet[key] for key in ("seq","ready_us","flags","max_high_us","resets")}
            return None
        if self.last_ready is not None and packet["ready_us"] <= self.last_ready:
            raise RuntimeError("ADC readiness clock reversed within a boot")
        self.last_ready = packet["ready_us"]
        return packet

    def next_valid(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                packet = self.receive()
            except socket.timeout:
                continue
            if packet is not None:
                return packet
        raise RuntimeError("No fresh healthy HX711 samples for %.1f s; check power/bridge/pins/host IP" % timeout)

    def drain(self):
        self.socket.settimeout(0.0)
        try:
            # Discard packets queued while the operator handled weights.
            for _ in range(100000):
                try:
                    self.receive()
                except BlockingIOError:
                    return
            raise RuntimeError("Cannot drain telemetry backlog")
        finally:
            self.socket.settimeout(0.2)

    def window(self, duration, phase):
        self.phase = phase
        packets = [self.next_valid()]
        start = time.monotonic()
        while time.monotonic() - start < duration:
            packets.append(self.next_valid())
        if len(packets) < 3:
            raise RuntimeError("Insufficient samples")
        return packets

    def recovering_window(self, duration, phase, settle_s=0.0, recovery_s=30.0):
        """Collect one clean source-clock window, restarting after ADC faults.

        Invalid/settling samples remain in raw.jsonl. Earlier fragments never
        enter this point's fit. A reset-counter change also catches lost fault
        packets. Source/boot/clock identity violations still propagate.
        """
        self.phase = phase
        start = time.monotonic()
        deadline = start + duration + settle_s + recovery_s
        settle_until = start + settle_s
        last_healthy = None
        packets = []
        marker = (self.flagged, self.invalid, self.converter_resets, self.gaps, self.duplicates)
        diagnostics = dict(discarded_windows=0, recovery_events=[],
                           flagged_packets=0, sequence_gaps=0, invalid_packets=0)
        self.capture_diagnostics = diagnostics
        initial = (self.flagged, self.gaps, self.invalid)
        reported_fault = None
        next_status = start + 5.0
        while time.monotonic() < deadline:
            try:
                packet = self.receive()
            except socket.timeout:
                packet = None
            now = time.monotonic()
            current = (self.flagged, self.invalid, self.converter_resets, self.gaps, self.duplicates)
            reset_changed = marker[2] is not None and current[2] != marker[2]
            interrupted = current[:2] != marker[:2] or current[3:] != marker[3:] or reset_changed
            if interrupted:
                if packets:
                    diagnostics["discarded_windows"] += 1
                packets = []
                settle_until = now + settle_s
                fault_id = None if self.last_fault is None else self.last_fault["seq"]
                if fault_id != reported_fault or reset_changed:
                    event = dict(self.last_fault or {}, elapsed_s=now-start)
                    if reset_changed:
                        event["reset_counter_changed"] = True
                    diagnostics["recovery_events"].append(event)
                    print("ADC recovery: discarding affected readings; keep the same weight. "
                          "Waiting for %.1f clean seconds." % duration, flush=True)
                    reported_fault = fault_id
            marker = current
            if last_healthy is not None and now-last_healthy > 1.0 and packets:
                diagnostics["discarded_windows"] += 1
                diagnostics["recovery_events"].append(dict(elapsed_s=now-start, reason="healthy stream gap >1s"))
                packets = []
                settle_until = now + settle_s
            if packet is not None:
                last_healthy = now
                # A source-time gap also catches buffered/late arrival bursts.
                if packets and packet["ready_us"]-packets[-1]["ready_us"] > 1000000:
                    diagnostics["discarded_windows"] += 1
                    diagnostics["recovery_events"].append(dict(elapsed_s=now-start, reason="source sample gap >1s"))
                    packets = []
                    settle_until = now + settle_s
                if now >= settle_until:
                    packets.append(packet)
                    if len(packets) >= 3 and packet["ready_us"]-packets[0]["ready_us"] >= duration*1e6:
                        diagnostics.update(elapsed_s=now-start, flagged_packets=self.flagged-initial[0],
                            sequence_gaps=self.gaps-initial[1], invalid_packets=self.invalid-initial[2])
                        return packets
            if now >= next_status:
                print("Still waiting for a clean ADC window; leave this weight unchanged.", flush=True)
                next_status = now + 5.0
        diagnostics.update(elapsed_s=time.monotonic()-start, flagged_packets=self.flagged-initial[0],
            sequence_gaps=self.gaps-initial[1], invalid_packets=self.invalid-initial[2])
        raise ADCRecoveryTimeout("No clean %.1f s ADC window after %.0f s; load is unchanged and completed points are saved" %
                                 (duration, diagnostics["elapsed_s"]))


    def summary(self):
        return dict(boot_id=self.boot, source_ip=self.peer, received=self.received,
                    invalid_packets=self.invalid, flagged_packets=self.flagged,
                    sequence_gaps=self.gaps, duplicate_or_old_packets=self.duplicates)


def statistics_for(packets):
    raw = [p["raw"] for p in packets]
    t = [p["ready_us"] / 1e6 for p in packets]
    mean_t, mean_raw = statistics.mean(t), statistics.mean(raw)
    sxx = sum((x - mean_t)**2 for x in t)
    slope = sum((x - mean_t)*(y - mean_raw) for x, y in zip(t, raw)) / sxx if sxx else 0.0
    return dict(samples=len(raw), mean_counts=mean_raw, std_counts=statistics.stdev(raw),
                min_counts=min(raw), max_counts=max(raw),
                adc_headroom_fraction=1-max(abs(v) for v in raw)/8388608,
                duration_source_s=t[-1]-t[0],
                sample_rate_hz=(len(raw)-1)/(t[-1]-t[0]), drift_counts_per_s=slope,
                max_read_span_us=max(p["read_us"]-p["ready_us"] for p in packets),
                max_clock_high_us=max(p["max_high_us"] for p in packets),
                max_interval_ms=max(b-a for a, b in zip(t, t[1:]))*1000)


def schedule(added, checks, cycles, quick=False):
    if len(added) < 3 or added[0] != 0 or any(b <= a for a, b in zip(added, added[1:])):
        raise ValueError("Need at least three strictly increasing added masses starting at 0")
    if cycles < 1:
        raise ValueError("At least one cycle required")
    result = [dict(kind="unloaded", cycle=0, added_g=0.0)]
    if quick:
        return (result + [dict(kind="ascending", cycle=1, added_g=x) for x in added]
                + [dict(kind="descending", cycle=1, added_g=x) for x in reversed(added[:-1])]
                + [dict(kind="unloaded", cycle=2, added_g=0.0)])
    for cycle in range(1, cycles+1):
        result += [dict(kind="ascending", cycle=cycle, added_g=x) for x in added]
        result += [dict(kind="descending", cycle=cycle, added_g=x) for x in reversed(added[:-1])]
    result += [dict(kind="check", cycle=cycles, added_g=x) for x in checks]
    result += [dict(kind="creep", cycle=cycles, added_g=0.0),
               dict(kind="unloaded", cycle=cycles+1, added_g=0.0)]
    return result


def fit_session(session):
    points = session["points"]
    fit_points = [p for p in points if p["kind"] == "ascending"]
    first_zero = next((p for p in points if p["kind"] == "unloaded"), None)
    if first_zero is None or len({p["force_N"] for p in fit_points}) < 3:
        raise ValueError("Need unloaded reference and at least three loaded ascending levels")
    train = [first_zero] + fit_points
    x, y = [p["force_N"] for p in train], [p["mean_counts"] for p in train]
    if any(not math.isfinite(v) for v in x+y):
        raise ValueError("Nonfinite calibration values")
    xm, ym = statistics.mean(x), statistics.mean(y)
    sxx = sum((v-xm)**2 for v in x)
    scale = sum((a-xm)*(b-ym) for a, b in zip(x, y))/sxx
    if abs(scale)*(max(x)-min(x)) <= max(10.0, 10*max(p["std_counts"] for p in train)):
        raise ValueError("Force response too small relative to noise; inspect strain transfer/bridge")
    intercept = ym-scale*xm
    diagnostics = []
    for p in points:
        diagnostics.append(dict(kind=p["kind"], cycle=p["cycle"], force_N=p["force_N"],
            added_g=p["added_g"], mean_counts=p["mean_counts"],
            residual_N=(p["mean_counts"]-intercept)/scale-p["force_N"],
            std_N=p["std_counts"]/abs(scale), drift_N_per_s=p["drift_counts_per_s"]/scale))
    hysteresis = []
    for p in points:
        if p["kind"] != "descending":
            continue
        up = next(q for q in fit_points if q["cycle"]==p["cycle"] and q["added_g"]==p["added_g"])
        hysteresis.append((p["mean_counts"]-up["mean_counts"])/scale)
    zero = [p for p in points if p["kind"]=="unloaded"]
    baseline = [p for p in points if p["kind"] in ("ascending", "descending") and p["added_g"]==0]
    checks = [p["residual_N"] for p in diagnostics if p["kind"]=="check"]
    max_tested = max(x)
    rms = math.sqrt(statistics.mean(p["residual_N"]**2 for p in diagnostics if p["kind"]=="ascending"))
    return dict(schema="sanity_tension_calibration_v1", counts_per_N=scale, zero_counts=intercept,
        equation="tension_N = (raw_counts - zero_counts) / counts_per_N",
        sign="positive tension along calibrated hanging load direction; scale may be negative",
        force_levels_tested_N=sorted(set(x)), force_range_N=[min(x), max_tested],
        loaded_force_range_N=[min(p["force_N"] for p in fit_points), max_tested],
        ascending_residual_RMS_N=rms,
        max_absolute_residual_N=max(abs(p["residual_N"]) for p in diagnostics),
        max_hysteresis_N=max(map(abs,hysteresis), default=None),
        unloaded_return_drift_N=(zero[-1]["mean_counts"]-zero[0]["mean_counts"])/scale if len(zero)>1 else None,
        baseline_repeatability_range_N=(max(p["mean_counts"] for p in baseline)-min(p["mean_counts"] for p in baseline))/abs(scale),
        max_check_error_N=max(map(abs,checks), default=None), points=diagnostics,
        quality_status="experimental candidate; inspect hysteresis, creep and independent checks before force feedback",
        limitations=["Not a mount strength test or flight-control qualification",
            "Untested intervals between zero and the lightest hanging load are not independently validated",
            "Standard deviation includes autocorrelated ADC samples; not uncertainty of the mean",
            "Tape slip, plastic creep, temperature and changed routing can invalidate calibration"],
        identity=session["identity"], gravity_m_s2=session["gravity_m_s2"],
        hanging_base_mass_g=session["base_mass_g"])


def save_json(path, obj):
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(obj, indent=2, allow_nan=False)+"\n")
    temporary.replace(path)


def response_model_review(session):
    """Diagnostic forward curves counts(force); no automatic inverse/selection."""
    import numpy as np
    points=session["points"]
    zero=[p for p in points if p["kind"]=="unloaded"]
    review=dict(status="diagnostic only; force conversion is still the linear candidate",
        interpretation="Nonlinearity, direction dependence and time drift require repeated loads to distinguish",
        unloaded_return_change_counts=zero[-1]["mean_counts"]-zero[0]["mean_counts"] if len(zero)>1 else None,
        branches={},hysteresis_counts=[])
    for kind in ("ascending","descending"):
        branch=[p for p in points if p["kind"]==kind]
        if not branch:
            continue
        # Measured zero for this direction; never use the final zero to train loading.
        if zero:
            branch=[zero[0] if kind=="ascending" else zero[-1]]+branch
        x=np.array([p["force_N"] for p in branch],dtype=float)
        y=np.array([p["mean_counts"] for p in branch],dtype=float)
        models=[]
        for degree in (1,2,3):
            if len(np.unique(x))<degree+3:
                continue
            poly=np.polynomial.Polynomial.fit(x,y,degree)
            residual=y-poly(x)
            held=[]
            for level in np.unique(x):
                keep=x!=level
                candidate=np.polynomial.Polynomial.fit(x[keep],y[keep],degree)
                held.extend((candidate(x[~keep])-y[~keep]).tolist())
            derivative=poly.deriv()(np.linspace(float(min(x)),float(max(x)),201))
            denominator=float(np.sum((y-np.mean(y))**2))
            models.append(dict(degree=degree,coefficients_counts_by_force_power=poly.convert().coef.tolist(),
                force_range_N=[float(min(x)),float(max(x))],
                training_RMS_counts=float(np.sqrt(np.mean(residual**2))),
                training_R_squared=1-float(np.sum(residual**2))/denominator if denominator else None,
                leave_one_force_level_out_RMS_counts=float(np.sqrt(np.mean(np.array(held)**2))),
                monotonic_over_tested_range=bool(np.all(derivative>0) or np.all(derivative<0))))
        review["branches"][kind]=models
    for p in points:
        if p["kind"]!="descending":
            continue
        up=next((q for q in points if q["kind"]=="ascending" and q["cycle"]==p["cycle"] and q["added_g"]==p["added_g"]),None)
        if up is not None:
            review["hysteresis_counts"].append(dict(added_g=p["added_g"],cycle=p["cycle"],
                unloading_minus_loading_counts=p["mean_counts"]-up["mean_counts"]))
    return review


def plot_calibration(directory, session, fit):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    for kind, marker in (("unloaded","x"), ("ascending","o"), ("descending","s"), ("check","D"), ("creep","^")):
        pp = [p for p in fit["points"] if p["kind"]==kind]
        if not pp:
            continue
        axes[0,0].plot([p["force_N"] for p in pp], [p["mean_counts"] for p in pp], marker, label=kind)
        axes[0,1].plot([p["force_N"] for p in pp], [p["residual_N"]*1000 for p in pp], marker, label=kind)
    x = fit["force_range_N"]
    axes[0,0].plot(x,[fit["zero_counts"]+fit["counts_per_N"]*v for v in x], color="0.3")
    for p in session["points"]:
        if p["kind"] != "creep":
            continue
        samples=p["packets"]
        t0=samples[0]["ready_us"]
        axes[1,0].plot([(s["ready_us"]-t0)/1e6 for s in samples],
                       [((s["raw"]-fit["zero_counts"])/fit["counts_per_N"]-p["force_N"])*1000 for s in samples], lw=0.7)
    axes[1,1].plot([p["std_N"]*1000 for p in fit["points"]], "o-")
    axes[0,0].set(xlabel="Known total tension (N)", ylabel="Raw ADC counts")
    axes[0,1].set(xlabel="Known total tension (N)", ylabel="Calibration error (mN)")
    axes[1,0].set(xlabel="Constant-load hold time (s)", ylabel="Baseline error / creep (mN)")
    axes[1,1].set(xlabel="Measurement order", ylabel="Within-point noise SD (mN)")
    for ax in axes.flat:
        ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=8)
    fig.suptitle("Hanging-drone quarter-bridge calibration · experimental candidate")
    fig.tight_layout()
    fig.savefig(directory/"calibration.png", dpi=180)
    fig.savefig(directory/"calibration.pdf")
    plt.close(fig)
    review=response_model_review(session)
    save_json(directory/"response_models.json",review)
    import numpy as np
    fig,axes=plt.subplots(1,2,figsize=(11,4.5))
    colours={"ascending":"#007d9c","descending":"#c46e12"}
    zeros=[p for p in session["points"] if p["kind"]=="unloaded"]
    if zeros:
        axes[0].plot([0]*len(zeros),[p["mean_counts"] for p in zeros],"x",color="black",label="unloaded references")
    positions=[];errors=[];labels=[]
    for kind,models in review["branches"].items():
        branch=[p for p in session["points"] if p["kind"]==kind]
        axes[0].plot([p["force_N"] for p in branch],[p["mean_counts"] for p in branch],"o-",
                     color=colours[kind],label="loading" if kind=="ascending" else "unloading")
        for model in models:
            grid=np.linspace(*model["force_range_N"],151)
            values=np.polynomial.polynomial.polyval(grid,model["coefficients_counts_by_force_power"])
            axes[0].plot(grid,values,linestyle={1:":",2:"--",3:"-."}[model["degree"]],
                         color=colours[kind],alpha=.65,label="%s degree%d" % (kind,model["degree"]))
            positions.append(len(positions));errors.append(model["leave_one_force_level_out_RMS_counts"])
            labels.append(("load" if kind=="ascending" else "unload")+" d%d" % model["degree"])
    axes[1].bar(positions,errors,color="#586b80");axes[1].set_xticks(positions,labels,rotation=30,ha="right")
    axes[0].set(xlabel="Known total tension (N)",ylabel="Raw ADC counts")
    axes[1].set(ylabel="Held-out force-level RMS prediction error (counts)")
    axes[0].legend(fontsize=7)
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle("Response-model comparison — diagnostic only; no automatic selection")
    fig.tight_layout();fig.savefig(directory/"response_models.png",dpi=180);fig.savefig(directory/"response_models.pdf")
    plt.close(fig)


def run(args):
    directory=args.output
    directory.mkdir(parents=True,exist_ok=False)
    if args.command=="fit":
        session=json.loads(args.session.read_text())
        fit=fit_session(session)
        save_json(directory/"calibration.json",fit)
        plot_calibration(directory,session,fit)
        print(json.dumps({k:v for k,v in fit.items() if k!="points"},indent=2))
        return
    if args.command=="calibrate":
        if args.quick:
            args.warmup_s, args.settle_s, args.cycles = 2.0, 1.0, 1
            args.check_added_g = []
        steps=schedule(args.added_g,args.check_added_g,args.cycles,quick=args.quick)
        if args.quick:
            # Keep about30s automatic time while including both load directions.
            args.sample_s=max(1.0,(30.0-args.warmup_s)/len(steps)-args.settle_s)
        if any(x <= min(args.added_g) or x >= max(args.added_g) or x in args.added_g for x in args.check_added_g):
            raise ValueError("Check masses must be distinct from fitted levels and strictly inside the added-mass range")
    calibration=json.loads(args.calibration.read_text()) if args.command=="monitor" else None
    if calibration and (calibration.get("schema")!="sanity_tension_calibration_v1" or
                        not math.isfinite(calibration["counts_per_N"]) or calibration["counts_per_N"]==0):
        raise ValueError("Invalid calibration")
    if calibration and calibration["identity"]["device_id"]!=args.device_id:
        raise ValueError("Calibration belongs to a different receiver/device")
    stream=None
    session=dict(schema="sanity_hanging_calibration_session_v1",created_unix_ns=time.time_ns(),
        identity=dict(sensor_id=getattr(args,"sensor_id",None),device_id=args.device_id,
                      mount_notes=getattr(args,"notes",None),bridge="one 350ohm gauge + three nominal 352ohm resistors"),
        base_mass_g=getattr(args,"base_mass_g",None),gravity_m_s2=getattr(args,"gravity",G0),
        arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},points=[],measurement_attempts=[])
    with (directory/"raw.jsonl").open("x") as raw:
        try:
            stream=UDPStream(args,raw)
            if args.command!="calibrate":
                packets=stream.window(args.duration,args.command)
                summary=statistics_for(packets)
                summary.update(stream.summary())
                if calibration:
                    scale,zero=calibration["counts_per_N"],calibration["zero_counts"]
                    summary["mean_tension_N"]=(summary["mean_counts"]-zero)/scale
                    summary["std_tension_N"]=summary["std_counts"]/abs(scale)
                    with (directory/"force.csv").open("x",newline="") as f:
                        writer=csv.writer(f);writer.writerow(["ready_us","raw","tension_N","outside_tested_force_range"])
                        lo,hi=calibration["force_range_N"]
                        for p in packets:
                            force=(p["raw"]-zero)/scale
                            writer.writerow([p["ready_us"],p["raw"],force,not lo<=force<=hi])
                save_json(directory/"summary.json",summary)
                print(json.dumps(summary,indent=2))
                return
            print("No control commands. Keep motors unpowered; all force must pass through the gauge mount.")
            print("Base mass must include the complete hanging assembly, battery/ESP/HX711/fixtures.")
            if args.quick:
                automatic_s=args.warmup_s+len(steps)*(args.settle_s+args.sample_s)
                print("Quick load/unload capture: approximately %.0f s automatic time, plus weight changes; %.2f s per reading." %
                      (automatic_s,args.sample_s))
            print("Warm-up: collecting for %.1f s. Keep the mount unloaded." % args.warmup_s)
            while True:
                try:
                    stream.recovering_window(args.warmup_s,"warmup")
                    break
                except ADCRecoveryTimeout as error:
                    save_json(directory/"session.json",session)
                    print(error)
                    input("Check the connection, then press Enter to retry warm-up (Ctrl-C stops): ")
                    stream.drain()
            for number,step in enumerate(steps,1):
                unloaded=step["kind"]=="unloaded"
                force=0.0 if unloaded else (args.base_mass_g+step["added_g"])/1000*args.gravity
                label={"unloaded":"ZERO-LOAD REFERENCE — support/detach the drone",
                       "ascending":"LOAD — add weight up to the requested total",
                       "descending":"UNLOAD — remove weight down to the requested total",
                       "check":"CHECK WEIGHT", "creep":"CONSTANT-LOAD HOLD"}[step["kind"]]
                print("\n[%d/%d] %s" % (number,len(steps),label))
                if unloaded:
                    print("Reference = 0 g external load: fully support/detach the drone so the mount carries no weight.")
                else:
                    print("Hang complete %.3f g base + %.3f g added = %.3f g; tension %.6f N." %
                          (args.base_mass_g,step["added_g"],args.base_mass_g+step["added_g"],force))
                input("Arrange load, stop touching it, then press Enter: " if args.quick else
                      "Arrange load, stop touching it, then press Enter (10 s countdown follows): ")
                stream.phase="discard_backlog";stream.drain()
                for left in range(0 if args.quick else 10,0,-1):
                    print("%d..." % left,flush=True)
                    stream.window(1.0,"countdown")
                duration=args.creep_s if step["kind"]=="creep" else args.sample_s
                while True:
                    try:
                        packets=stream.recovering_window(duration,step["kind"],settle_s=args.settle_s)
                    except ADCRecoveryTimeout as error:
                        session["measurement_attempts"].append(dict(step,force_N=force,accepted=False,
                            diagnostics=dict(stream.capture_diagnostics)))
                        save_json(directory/"session.json",session)
                        print(error)
                        input("Keep this weight attached; fix the connection if needed, then Enter to retry (Ctrl-C stops): ")
                        stream.phase="discard_backlog";stream.drain()
                        continue
                    break
                diagnostics=dict(stream.capture_diagnostics)
                point=dict(step,force_N=force,packets=packets,**statistics_for(packets))
                accepted_gaps=sum(((b["seq"]-a["seq"]) & 0xffffffff)-1 for a,b in zip(packets,packets[1:]))
                diagnostics["accepted_window_sequence_gaps"]=accepted_gaps
                point.update(sequence_gaps=accepted_gaps,flagged_packets=0,
                             recovery_diagnostics=diagnostics)
                if point["adc_headroom_fraction"]<0.05:
                    session["measurement_attempts"].append(dict(step,force_N=force,accepted=False,diagnostics=diagnostics,
                        reason="less than 5% ADC headroom"))
                    raise RuntimeError("Less than 5% ADC headroom; inspect bridge balance/gain before further loading")
                session["measurement_attempts"].append(dict(step,force_N=force,accepted=True,diagnostics=diagnostics))
                if point["sequence_gaps"]:
                    print("Telemetry gaps: %d; retained fresh samples and recorded this limitation." % point["sequence_gaps"])
                session["points"].append(point)
                save_json(directory/"session.json",session)
                print("Saved %d samples, %.1f Hz, mean %.2f counts, SD %.2f counts" %
                      (point["samples"],point["sample_rate_hz"],point["mean_counts"],point["std_counts"]))
            fit=fit_session(session)
            save_json(directory/"calibration.json",fit)
            plot_calibration(directory,session,fit)
            print("Candidate calibration and plots:",directory)
            print("Scale %+.9g counts/N; unloaded drift %s N" %
                  (fit["counts_per_N"],fit["unloaded_return_drift_N"]))
            if fit["max_hysteresis_N"] is not None:
                print("Max loading/unloading difference: %s N equivalent under the linear candidate" % fit["max_hysteresis_N"])
            print("See response_models.json/png for linear vs nonlinear diagnostics; no model is automatically selected.")
        finally:
            if stream:
                session["transport"]=stream.summary()
                stream.socket.close()
            save_json(directory/"session.json",session)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    subs=parser.add_subparsers(dest="command",required=True)
    for name in ("probe","calibrate","monitor","fit"):
        p=subs.add_parser(name)
        p.add_argument("--output",type=Path,required=True,help="New directory, never overwritten")
        if name=="fit":
            p.add_argument("session",type=Path)
            continue
        p.add_argument("--bind",default="0.0.0.0")
        p.add_argument("--port",type=int,default=7460)
        p.add_argument("--device-id",type=int,default=9)
        p.add_argument("--source-ip",default="10.0.0.109")
        if name in ("probe","monitor"):
            p.add_argument("--duration",type=positive,default=30.0)
        if name=="monitor":
            p.add_argument("--calibration",type=Path,required=True)
        if name=="calibrate":
            p.add_argument("--base-mass-g",type=positive,required=True)
            p.add_argument("--added-g",type=masses,required=True,help="Measured added totals, e.g. 0,5,10,15,20")
            p.add_argument("--check-added-g",type=masses,default=[],help="Optional independent masses not used to fit; omitted by default")
            p.add_argument("--quick",action="store_true",help="About30s automatic load/unload sweep: 2s warm-up, 1s settle and adaptive recording durations; no countdown/repeats/creep")
            p.add_argument("--cycles",type=int,default=2)
            p.add_argument("--warmup-s",type=positive,default=120.0)
            p.add_argument("--settle-s",type=positive,default=5.0)
            p.add_argument("--sample-s",type=positive,default=10.0)
            p.add_argument("--creep-s",type=positive,default=60.0)
            p.add_argument("--gravity",type=positive,default=G0)
            p.add_argument("--sensor-id",required=True)
            p.add_argument("--notes",default="")
    args=parser.parse_args()
    try:
        run(args)
    except (ValueError,RuntimeError,OSError,EOFError,KeyboardInterrupt) as error:
        parser.exit(1,"Stopped: %s\n" % (error or "operator interruption; partial data retained"))


if __name__=="__main__":
    main()
