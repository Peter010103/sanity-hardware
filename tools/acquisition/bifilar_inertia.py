#!/usr/bin/env python3
"""Record a bifilar pendulum with OptiTrack; estimate combined suspended inertia."""

import argparse
import csv
import datetime
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import find_peaks, savgol_filter
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
COLUMNS = ["host_monotonic_s", "host_unix_s", "stamp_sec", "stamp_nanosec",
           "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"]


def geometry(mass_g=87.38, length_cm=64.2, top_cm=15.6, bottom_cm=13.2):
    values = np.array([mass_g, length_cm, top_cm, bottom_cm], dtype=float)
    if not np.isfinite(values).all() or np.any(values <= 0):
        raise ValueError("Mass and dimensions must be finite and positive")
    m, length, a, b = mass_g / 1000, length_cm / 100, top_cm / 200, bottom_cm / 200
    if length <= abs(a-b):
        raise ValueError("Strings too short for the attachment spacing")
    height = math.sqrt(length**2 - (a-b)**2)
    stiffness = m * 9.80665 * a * b / height
    return dict(mass_total_kg=m, string_length_m=length, upper_full_spacing_m=2*a,
                lower_full_spacing_m=2*b, vertical_separation_m=height,
                gravity_m_s2=9.80665, stiffness_Nm_rad=stiffness,
                inertia_per_T_squared=stiffness/(4*math.pi**2))


def fit_oscillation(t, angle):
    if len(t) < 100 or t[-1]-t[0] < 8:
        raise ValueError("Need at least eight seconds of usable free oscillation")
    if not np.isfinite(angle).all() or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError("Angles and strictly increasing timestamps must be finite")
    if np.max(np.diff(t)) > .1:
        raise ValueError("Tracking gap exceeds 100 ms inside the fit window; select a continuous window")
    # Frequency is fitted against source timestamps, not network arrival timing.
    dt = max(float(np.median(np.diff(t))), .02)
    grid = np.arange(0., t[-1]-t[0], dt)
    y = np.interp(grid, t-t[0], angle)
    centred = grid-grid.mean()
    trend = np.polyfit(centred, y, 1)
    detrended = y-np.polyval(trend, centred)
    if np.std(detrended) < math.radians(.15):
        raise ValueError("No resolvable torsional oscillation (less than 0.15 deg RMS)")
    frequencies = np.fft.rfftfreq(len(grid), dt)
    power = abs(np.fft.rfft(detrended*np.hanning(len(grid))))**2
    indices = np.flatnonzero((frequencies >= .2) & (frequencies <= 10))
    starts = indices[np.argsort(power[indices])[-5:]]

    def solve(parameters, return_model=False):
        frequency, decay = parameters
        envelope = np.exp(-decay*grid)
        phase = 2*math.pi*frequency*grid
        matrix = np.column_stack([np.ones(len(grid)), centred,
                                  envelope*np.cos(phase), envelope*np.sin(phase)])
        coefficients = np.linalg.lstsq(matrix, y, rcond=None)[0]
        model = matrix@coefficients
        return (model, coefficients) if return_model else model-y

    solutions = [least_squares(solve, [frequencies[i], .02],
                               bounds=([.2, 0.], [10., 1.]),
                               xtol=1e-10, ftol=1e-10, gtol=1e-10)
                 for i in starts]
    result = min(solutions, key=lambda x: float(x.fun@x.fun))
    model, coefficients = solve(result.x, True)
    frequency, decay = result.x
    oscillation = model-(coefficients[0]+coefficients[1]*centred)
    residual_rms = float(np.sqrt(np.mean((model-y)**2)))
    oscillation_rms = float(np.sqrt(np.mean(oscillation**2)))
    amplitude = float(np.hypot(*coefficients[2:]))
    ratio = residual_rms / max(oscillation_rms, 1e-12)
    failures = []
    warnings = []
    if not result.success:
        failures.append("Oscillation fit did not converge")
    if frequency*grid[-1] < 6:
        failures.append("Fewer than six fitted cycles")
    if ratio > .15:
        failures.append("Fit residual exceeds 15% of oscillation RMS; inspect release, swing and tracking")
    if math.degrees(amplitude) < .5:
        failures.append("Fitted starting amplitude below 0.5 deg")
    if frequency < .205 or frequency > 9.95 or decay > .98:
        failures.append("Fit reached frequency/decay search boundary")
    if math.degrees(amplitude) > 12:
        warnings.append("Amplitude above 12 deg: small-angle approximation needs review")
    if decay < 1e-5:
        warnings.append("Decay is at zero bound; damping was not resolved")
    corrected = y-(coefficients[0]+coefficients[1]*centred)
    window = max(5, int(.12/frequency/dt) | 1)
    window = min(window, (len(y)-1)//2*2+1)
    smooth = savgol_filter(corrected, window, 2)
    peaks, _ = find_peaks(smooth, distance=max(1, int(.65/frequency/dt)),
                          prominence=max(math.radians(.1), .15*amplitude*np.exp(-decay*grid[-1])))
    periods = np.diff(grid[peaks])
    periods = periods[(periods > .7/frequency) & (periods < 1.3/frequency)]
    summary = dict(frequency_damped_hz=float(frequency), period_damped_s=float(1/frequency),
                   decay_per_s=float(decay), amplitude_at_fit_start_deg=math.degrees(amplitude),
                   residual_rms_deg=math.degrees(residual_rms), residual_to_oscillation_rms=ratio,
                   fitted_cycles=float(frequency*grid[-1]),
                   peak_period_median_s=float(np.median(periods)) if len(periods) else None,
                   peak_period_sd_s=float(np.std(periods, ddof=1)) if len(periods)>1 else None,
                   peak_period_count=len(periods), failures=failures, warnings=warnings)
    return summary, grid, y, model


def plot_decay_review(directory, cfg, stamp, t, rotations, start, end):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    tail=t >= max(float(t[-1])-5., 0.)
    reference=rotations[tail].mean()
    q=(rotations*reference.inv()).as_quat()
    angle=np.angle((q[:,3]+1j*q[:,2])**2)
    candidates=[]
    details=[]
    for candidate_start in [start, start+3, start+5, start+7, start+10]:
        candidate_end=min(candidate_start+10., end)
        row=dict(start_s=float(candidate_start),end_s=float(candidate_end))
        select=(t>=candidate_start)&(t<=candidate_end)
        if candidate_end-candidate_start<8 or select.sum()<100:
            continue
        try:
            fit,grid,y,model=fit_oscillation(stamp[select],angle[select])
            inertia=cfg['geometry']['stiffness_Nm_rad']/((2*np.pi*fit['frequency_damped_hz'])**2+fit['decay_per_s']**2)
            row.update(period_s=fit['period_damped_s'],decay_per_s=fit['decay_per_s'],
                       amplitude_deg=fit['amplitude_at_fit_start_deg'],relative_residual=fit['residual_to_oscillation_rms'],
                       candidate_combined_inertia_kg_m2=inertia,failures=fit['failures'])
            details.append((len(candidates),select,fit,grid,y,model))
        except ValueError as exc:
            row['failures']=[str(exc)]
        candidates.append(row)
    with (directory/'window_comparison.csv').open('w',newline='') as stream:
        fields=['start_s','end_s','period_s','decay_per_s','amplitude_deg','relative_residual',
                'candidate_combined_inertia_kg_m2','failures']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(candidates)
    # Fixed selection rule: earliest passing small-angle window, then a labelled fallback.
    eligible=[d for d in details if not d[2]['failures'] and d[2]['amplitude_at_fit_start_deg']<=12]
    chosen=next(iter(eligible or details),None)
    overview_start=min(4.,max(0.,start))
    visible=(t>=overview_start)&(t<=end)
    fig,axes=plt.subplots(3,1,figsize=(10,8.8),layout='constrained')
    blue='#1766a0';orange='#df7424'
    axes[0].plot(t[visible],np.degrees(angle[visible]),color=blue,lw=.9)
    axes[0].axvspan(0,overview_start,color='0.90')
    if overview_start:
        axes[0].text(overview_start/2,0,'Release /\nhandling\nexcluded',ha='center',va='center',fontsize=8,color='0.35')
    axes[0].set(xlim=(0,max(end,.1)),ylabel='Vertical twist (deg)',
                title='Recorded decay; orientation relative to final five-second mean')
    axes[0].set_xlabel('Seconds after countdown beep')
    review=dict(axis=cfg['axis'],diagnostic_only=True,frame_inertia_subtracted=False,
                overview_starts_s=overview_start,reference='Mean orientation over final five seconds',
                selection_rule='Earliest passing 10-second window with fitted amplitude <=12 deg; otherwise first attempted fit, clearly flagged.',
                primary_result_not_replaced_by_review=True,windows=candidates)
    if chosen is None:
        for ax in axes[1:]:
            ax.text(.5,.5,'No resolvable short-window oscillation; inspect raw trace',ha='center',transform=ax.transAxes)
            ax.set_axis_off()
        review['status']='no_resolvable_short_window'
    else:
        index,select,fit,grid,y,model=chosen
        row=candidates[index];frequency=fit['frequency_damped_hz'];decay=fit['decay_per_s']
        matrix=np.column_stack([np.ones(len(grid)),grid-grid.mean(),
                                np.exp(-decay*grid)*np.cos(2*np.pi*frequency*grid),
                                np.exp(-decay*grid)*np.sin(2*np.pi*frequency*grid)])
        coefficients=np.linalg.lstsq(matrix,y,rcond=None)[0]
        baseline=coefficients[0]+coefficients[1]*(grid-grid.mean())
        amplitude=float(np.hypot(*coefficients[2:]))
        envelope=amplitude*np.exp(-decay*grid)
        first=t[np.flatnonzero(select)[0]];fit_time=first+grid
        axes[0].axvspan(row['start_s'],row['end_s'],color='#3c9b74',alpha=.12)
        axes[1].plot(fit_time,np.degrees(y),color=blue,lw=1.2,label='Measured twist')
        axes[1].plot(fit_time,np.degrees(model),color=orange,lw=1.1,label='Damped fit')
        axes[1].plot(fit_time,np.degrees(baseline+envelope),color=orange,ls='--',lw=.9)
        axes[1].plot(fit_time,np.degrees(baseline-envelope),color=orange,ls='--',lw=.9)
        axes[1].set(xlim=(row['start_s'],row['end_s']),ylabel='Vertical twist (deg)',
                    xlabel='Seconds after countdown beep',
                    title=f"Closer view: period {fit['period_damped_s']:.3f} s; {fit['fitted_cycles']:.1f} cycles")
        axes[1].legend(loc='upper right',ncol=2,fontsize=9)
        visible_times=t[visible];visible_angles=angle[visible]
        dt=float(np.median(np.diff(stamp[visible])))
        window=min(21,(len(visible_angles)-1)//2*2+1)
        smooth=savgol_filter(visible_angles,window,2)
        peaks,_=find_peaks(abs(smooth),distance=max(1,int(.30/frequency/dt)),prominence=np.radians(.08))
        axes[2].scatter(visible_times[peaks],np.degrees(abs(smooth[peaks])),s=10,color=blue,label='Measured peak magnitudes')
        extended=np.linspace(overview_start,end,600)
        axes[2].plot(extended,np.degrees(amplitude*np.exp(np.clip(-decay*(extended-first),-50,50))),
                     color=orange,ls='--',label='Short-window envelope, extended')
        half_life=math.log(2)/decay if decay>1e-5 else None
        tau=1/decay if decay>1e-5 else None
        axes[2].set(xlim=(overview_start,max(end,overview_start+.1)),ylim=(0,None),
                    xlabel='Seconds after countdown beep',ylabel='Peak magnitude (deg)',
                    title=f'Amplitude time constant ≈ {tau:.1f} s; half-life ≈ {half_life:.1f} s' if tau else 'Decay not resolved')
        axes[2].legend(loc='upper right',fontsize=9)
        review.update(status='provisional_short_window' if eligible else 'review_required',
                      window_s=[row['start_s'],row['end_s']],fit=fit,
                      candidate_combined_inertia_kg_m2=row['candidate_combined_inertia_kg_m2'],
                      amplitude_time_constant_s=tau,amplitude_half_life_s=half_life)
    for ax in axes:ax.grid(alpha=.2)
    review_label="Diagnostic short-window fit" if review["status"]=="provisional_short_window" else "REVIEW REQUIRED: no passing small-angle fit"
    fig.suptitle(f"{cfg['axis'].upper()} bifilar pendulum: measured decay\n{review_label}; drone + wooden frame",fontsize=14)
    fig.savefig(directory/'decay.png',dpi=180);fig.savefig(directory/'decay.pdf');plt.close(fig)
    (directory/'decay_review.json').write_text(json.dumps(review,indent=2)+'\n')
    print(f"Decay overview/close-up/envelope: {directory/'decay.png'}",flush=True)
    return review


def analyze(directory, fit_start=None, fit_end=None, mass_g=None):
    cfg = json.loads((directory/'config.json').read_text())
    if mass_g is not None:
        previous=cfg["geometry"]
        cfg["geometry"]=geometry(mass_g,previous["string_length_m"]*100,
                                 previous["upper_full_spacing_m"]*100,previous["lower_full_spacing_m"]*100)
    if cfg["geometry"].get("mass_total_kg") is None:
        raise ValueError("Recorded mass is unconfirmed after the setup change. Reanalyze with --mass-g NEW_TOTAL_G after weighing; raw poses are unaffected.")
    capture = json.loads((directory/'capture_receipt.json').read_text())
    if capture.get('cue_monotonic_s') is None:
        raise ValueError("Countdown never finished; no measurement trial was started")
    with (directory/'poses.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("No poses recorded")
    host = np.array([float(r['host_monotonic_s']) for r in rows])
    secs = np.array([int(r['stamp_sec']) for r in rows], dtype=np.int64)
    nanos = np.array([int(r['stamp_nanosec']) for r in rows], dtype=np.int64)
    stamp = (secs-secs[0]).astype(float)+(nanos-nanos[0])*1e-9
    cue = capture['cue_monotonic_s']
    since_cue = host-cue
    start = cfg['fit_start_s_after_cue'] if fit_start is None else fit_start
    end = float(since_cue[-1]) if fit_end is None else fit_end
    selected = (since_cue >= start) & (since_cue <= end)
    if selected.sum() < 100:
        raise ValueError("Not enough samples in the requested fit window")
    quaternions = np.array([[float(r[k]) for k in ['qx','qy','qz','qw']] for r in rows])
    if not np.isfinite(quaternions).all() or np.any(np.linalg.norm(quaternions, axis=1) < .5):
        raise ValueError("Invalid tracked quaternion")
    rotations = Rotation.from_quat(quaternions)
    relative = rotations*rotations[0].inv()
    q = relative.as_quat()
    # Swing/twist decomposition about world vertical, independent of marker axes.
    twist = np.unwrap(2*np.arctan2(q[:,2], q[:,3]))
    plot_decay_review(directory, cfg, stamp, since_cue, rotations, start, end)
    selected_t = stamp[selected]
    fit, grid, y, fitted = fit_oscillation(selected_t, twist[selected])
    tilt = np.arccos(np.clip(relative.as_matrix()[:,2,2], -1, 1))
    tilt_change = float(np.degrees(np.max(tilt[selected])))
    if tilt_change > 3:
        fit['warnings'].append("More than 3 deg of tilt change: review coupled rocking/swing")
    stiffness = cfg['geometry']['stiffness_Nm_rad']
    omega = 2*math.pi*fit['frequency_damped_hz']
    inertia = stiffness/(omega**2+fit['decay_per_s']**2)
    accepted = not fit['failures']
    positions = np.array([[float(r[k]) for k in ['x_m','y_m','z_m']] for r in rows])
    summary = dict(status='provisional_combined_estimate' if accepted else 'fit_rejected',
                   axis=cfg['axis'], tracked_frame=cfg['tracked_frame'],
                   inertia_combined_about_suspension_axis_kg_m2=inertia if accepted else None,
                   candidate_inertia_kg_m2=inertia, fit=fit,
                   fit_start_s_after_cue=float(start), fit_end_s_after_cue=float(end),
                   max_tilt_change_from_initial_deg=tilt_change,
                   position_peak_to_peak_m=np.ptp(positions[selected],axis=0).tolist(),
                   geometry=cfg['geometry'], analysis_mass_override_g=mass_g, frame_inertia_subtracted=False,
                   drone_com_offset_corrected=False, geometry_uncertainties_included=False,
                   notes=["Combined drone, frame and markers about the suspension axis; not drone-only inertia.",
                          "Assumes equal taut uncrossed strings, aligned attachment midpoints, centred load and small torsion.",
                          "Fit is offset + linear drift + exp(-decay*t)*(A*cos(omega*t)+B*sin(omega*t)).",
                          "Inertia = m*g*(top/2)*(bottom/2)/(h*(omega_d^2+decay^2)).",
                          "Peak-period scatter and fit residual are diagnostics, not total measurement uncertainty.",
                          "Position excursions include marker-origin motion; they do not independently measure pendulum swing.",
                          "Frame inertia and drone parallel-axis offset require later correction."],
                   control_publishers_started=False)
    (directory/'result.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (directory/'fit.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['time_since_fit_start_s','observed_angle_rad','fitted_angle_rad'])
        writer.writerows(zip(grid,y,fitted))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True,layout='constrained')
    axes[0].plot(since_cue,np.degrees(twist),lw=.8,label='OptiTrack vertical twist')
    first_host=since_cue[np.flatnonzero(selected)[0]]
    axes[0].plot(first_host+grid,np.degrees(fitted),lw=1.5,label='Damped fit')
    axes[0].axvspan(start,end,color='green',alpha=.07,label='Analysis window')
    axes[0].axvline(0,color='black',ls=':',lw=1);axes[0].set_ylabel('Angle (deg)');axes[0].legend(loc='upper right')
    axes[1].plot(first_host+grid,np.degrees(y-fitted),lw=.7)
    axes[1].set_ylabel('Fit residual (deg)');axes[1].set_xlabel('Time after countdown beep (s)')
    title=f"{cfg['axis'].upper()} | combined inertia {inertia:.5g} kg m² | T = {fit['period_damped_s']:.4f} s"
    if not accepted:title='REJECTED FIT | '+title
    fig.suptitle(title)
    for ax in axes:ax.grid(alpha=.25)
    fig.savefig(directory/'oscillation.png',dpi=180);fig.savefig(directory/'oscillation.pdf');plt.close(fig)
    print(f"\n{summary['status']}: T = {fit['period_damped_s']:.5f} s",flush=True)
    if accepted:print(f"Drone + frame inertia = {inertia:.7g} kg m² (frame not subtracted)",flush=True)
    for line in fit['failures']+fit['warnings']:print('Review: '+line,flush=True)
    print(f"Results: {directory/'result.json'}\nPlot: {directory/'oscillation.png'}",flush=True)
    return summary


def record(args, directory, cfg):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from tf2_msgs.msg import TFMessage
    count=duplicates=0;previous=None;first=None;last=None;last_receive=-math.inf;cue=None
    error=None;interrupted=False
    rclpy.init();node=rclpy.create_node('bifilar_inertia_recorder')
    stream=(directory/'poses.csv').open('x',newline='');writer=csv.writer(stream);writer.writerow(COLUMNS)

    def receive(message):
        nonlocal count,duplicates,previous,first,last,last_receive
        for transform in message.transforms:
            if transform.header.frame_id!=args.parent or transform.child_frame_id!=args.body:continue
            stamp=transform.header.stamp.sec*1_000_000_000+transform.header.stamp.nanosec
            if previous is not None and stamp<=previous:duplicates+=1;continue
            previous=stamp;now=time.monotonic();q=transform.transform.rotation;p=transform.transform.translation
            writer.writerow([now,time.time(),transform.header.stamp.sec,transform.header.stamp.nanosec,
                             p.x,p.y,p.z,q.x,q.y,q.z,q.w])
            count+=1;first=first if first is not None else stamp;last=stamp;last_receive=now

    subscription=node.create_subscription(TFMessage,'/tf',receive,QoSProfile(depth=1000,
        reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE))
    try:
        print(f"Waiting for fresh {args.parent} -> {args.body} tracking...",flush=True)
        deadline=time.monotonic()+15
        while count<20 or time.monotonic()-last_receive>.2:
            if time.monotonic()>deadline:raise RuntimeError('No fresh tracked poses within 15 seconds; check OptiTrack/DDS')
            rclpy.spin_once(node,timeout_sec=.05)
        print(f"Tracking ready. Walk to the setup during the {args.countdown:g}-second countdown.",flush=True)
        print("At the beep: gently twist 5–10 degrees about vertical and RELEASE. Then leave it untouched.",flush=True)
        until=time.monotonic()+args.countdown;previous_count=None
        while time.monotonic()<until:
            remaining=math.ceil(until-time.monotonic())
            if remaining!=previous_count:print(f"  {remaining}...",flush=True);previous_count=remaining
            if time.monotonic()-last_receive>.5:raise RuntimeError('Tracking lost during countdown; trial not started')
            rclpy.spin_once(node,timeout_sec=.02)
        cue=time.monotonic();print('\aRECORDING — twist gently and release now.',flush=True)
        notice=cue+5
        while time.monotonic()-cue<args.duration:
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()-last_receive>2:raise RuntimeError('Tracking absent for two seconds; saved data retained')
            if time.monotonic()>=notice:
                stream.flush();print(f"  {time.monotonic()-cue:.0f}/{args.duration:g} seconds; {count} poses",flush=True);notice+=5
        print('\aRecording complete.',flush=True)
    except (KeyboardInterrupt,ExternalShutdownException):
        interrupted=True;print('\nStopped; saving available data.',flush=True)
    except Exception as exc:
        error=f'{type(exc).__name__}: {exc}'
    finally:
        stream.flush();stream.close();node.destroy_node();rclpy.try_shutdown()
        receipt=dict(timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),samples=count,
                     cue_monotonic_s=cue,first_stamp_ns=first,last_stamp_ns=last,
                     rate_hz=(count-1)/((last-first)*1e-9) if count>1 else None,
                     duplicate_or_reversed_stamps=duplicates,interrupted=interrupted,error=error,
                     pose_only=True,control_publishers_started=False)
        (directory/'capture_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    if error:raise RuntimeError(error)
    return receipt


def self_test():
    rng=np.random.default_rng(17)
    t=np.arange(0,35,.005);frequency=1.27;decay=.045
    angle=.06+.0002*t+np.exp(-decay*t)*(.1*np.cos(2*np.pi*frequency*t)+.03*np.sin(2*np.pi*frequency*t))
    angle+=rng.normal(0,math.radians(.01),len(t))
    # Marker axes may be offset; global vertical rotation must still be recovered.
    base=Rotation.from_euler('xyz',[.2,-.15,.4])
    rotations=Rotation.from_rotvec(np.column_stack([np.zeros(len(t)),np.zeros(len(t)),angle]))*base
    q=(rotations*rotations[0].inv()).as_quat();q[::2]*=-1
    recovered=np.unwrap(2*np.arctan2(q[:,2],q[:,3]))
    fit,*_=fit_oscillation(t,recovered)
    assert not fit['failures'],fit
    assert abs(fit['frequency_damped_hz']-frequency)<.0002,fit
    assert abs(fit['decay_per_s']-decay)<.0005,fit
    g=geometry(86.38,64.5,15.6,13.);expected=g['stiffness_Nm_rad']/((2*np.pi*frequency)**2+decay**2)
    estimated=g['stiffness_Nm_rad']/((2*np.pi*fit['frequency_damped_hz'])**2+fit['decay_per_s']**2)
    assert abs(estimated/expected-1)<.0005
    assert abs(g['inertia_per_T_squared']-.00016869826340956722)<1e-14
    for times,angles in [(t,np.zeros(len(t))),(np.r_[t[:1000],t[1100:]],np.r_[angle[:1000],angle[1100:]])]:
        try:fit_oscillation(times,angles)
        except ValueError:pass
        else:raise AssertionError('Invalid trial should be rejected')
    bad=angle+.07*np.sin(2*np.pi*1.83*t)
    assert fit_oscillation(t,bad)[0]['failures'],'Mixed oscillation should fail residual gate'
    print('PASS: synthetic period/damping/inertia, marker rotation/quaternion signs, stationary/gap/mixed-mode rejection.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--countdown',type=float,default=10.,help='Seconds to walk to setup before beep (default10)')
    p.add_argument('--duration',type=float,default=45.,help='Recording seconds after beep (default45)')
    p.add_argument('--fit-start',type=float,default=None,help='Analyze from this many seconds after beep (default5)')
    p.add_argument('--fit-end',type=float,help='Optional end of fit window, seconds after beep')
    p.add_argument('--axis',choices=['yaw','roll','pitch'],default='yaw',help='Drone axis mounted vertically')
    p.add_argument('--body',default='RigidBody');p.add_argument('--parent',default='map')
    p.add_argument('--mass-g',type=float,default=None,help='Total suspended mass; collection-only leaves this unconfirmed unless supplied')
    p.add_argument('--collect-only',action='store_true',help='Save raw poses and metadata; skip all analysis and plots')
    p.add_argument('--setup-label',default='',help='Operator label for marker/frame configuration, saved with each recording')
    p.add_argument('--length-cm',type=float,default=64.2)
    p.add_argument('--top-cm',type=float,default=15.6);p.add_argument('--bottom-cm',type=float,default=None,help='Full spacing: defaults13.2cm yaw,15.5cm pitch/roll')
    p.add_argument('--output',type=Path,help='New recording directory (never overwrite)')
    p.add_argument('--analyze',type=Path,help='Reanalyze an existing recording without ROS or live capture')
    p.add_argument('--self-test',action='store_true',help='Offline synthetic analysis tests; no live subscription')
    args=p.parse_args()
    if args.self_test:self_test();return
    if not np.isfinite([args.countdown,args.duration]).all() or args.countdown<0 or args.duration<10:
        p.error('Require nonnegative finite countdown and at least10 seconds recording')
    if args.fit_start is not None and (not math.isfinite(args.fit_start) or args.fit_start<0):p.error('Invalid fit start')
    if args.fit_end is not None and (not math.isfinite(args.fit_end) or args.fit_end<=0):p.error('Invalid fit end')
    if args.collect_only and args.analyze:p.error("--collect-only cannot be combined with --analyze")
    if args.analyze:
        analyze(args.analyze.resolve(),args.fit_start,args.fit_end,mass_g=args.mass_g);return
    bottom_cm=args.bottom_cm if args.bottom_cm is not None else (13.2 if args.axis=='yaw' else 15.5)
    g=geometry(87.38 if args.mass_g is None else args.mass_g,args.length_cm,args.top_cm,bottom_cm)
    if args.collect_only and args.mass_g is None:
        for key in ["mass_total_kg","stiffness_Nm_rad","inertia_per_T_squared"]:g[key]=None
    label=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    directory=args.output or ROOT/'build/bifilar'/('sanity_bifilar_'+label+'_'+args.axis)
    directory=directory.absolute();directory.mkdir(parents=True,exist_ok=False)
    cfg=dict(axis=args.axis,tracked_frame=args.body,parent_frame=args.parent,geometry=g,
             collection_only=args.collect_only,setup_label=args.setup_label,
             mass_status='unconfirmed' if g['mass_total_kg'] is None else ('operator_supplied' if args.mass_g is not None else 'previous_setup_default'),
             marker_to_drone_orientation_confirmed=False,
             countdown_s=args.countdown,record_duration_s=args.duration,
             fit_start_s_after_cue=5. if args.fit_start is None else args.fit_start,
             frame_inertia_subtracted=False,
             geometry_source='https://gdcplnr.edu.in/admin/uploads/1683i%20sem%20%20PRACTICALS.pdf (bifilar experiment)',
             assumptions=['Equal actual string lengths; uncrossed strings; attachment midpoints aligned.',
                          'Drone labelled axis vertical; rigid fixture; total suspended mass includes markers.',
                          'Suspension axis approximately through combined centre of mass; small-angle torsion.'])
    (directory/'config.json').write_text(json.dumps(cfg,indent=2)+'\n')
    print('Output: '+str(directory),flush=True)
    try:
        receipt=record(args,directory,cfg)
        if args.collect_only:
            print(f'Raw collection saved: {directory}\nAnalysis and plots deferred; no inertia calculated.',flush=True)
        elif receipt['cue_monotonic_s'] is not None:analyze(directory,args.fit_start,args.fit_end)
    except Exception as exc:
        message=f'{type(exc).__name__}: {exc}'
        error_file='capture_error.json' if args.collect_only else 'analysis_error.json'
        (directory/error_file).write_text(json.dumps({'error':message,'raw_data_retained':True},indent=2)+'\n')
        raise SystemExit(f'{message}\nData retained in {directory}')


if __name__=='__main__':
    main()
