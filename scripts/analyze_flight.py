#!/usr/bin/env python3
"""Analyze host MPC tracking at recorded timestamps; no fitted alignment or device I/O."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def read_controls(run):
    candidates=[run/'controller/control.jsonl',run/'control.jsonl',run/'flight/events.jsonl',run/'events.jsonl',run/'flight/controller/control.jsonl']
    path=next((p for p in candidates if p.is_file()),None)
    if path is None:
        nested=[run/'capture',run/'flight']
        for child in nested:
            if child.is_dir():
                try:return read_controls(child)
                except FileNotFoundError:pass
        raise FileNotFoundError('No supported host control log under '+str(run))
    rows=[]
    for line in path.open():
        item=json.loads(line)
        if path.name=='events.jsonl' and item.get('kind') not in ('control','hover_control'):continue
        item=item.get('record',item)
        state=item.get('state');ref=item.get('reference')
        if state is None or ref is None:continue
        state=np.asarray(state,dtype=float)
        if state.shape!=(13,) or not np.isfinite(state).all():continue
        position=np.asarray(ref['position'],dtype=float);velocity=np.asarray(ref['velocity'],dtype=float)
        modern=item.get('outer_controller',{}).get('mpc',{})
        weight=item.get('policy_weight',modern.get('weight',0.))
        solver=item.get('mpc') or modern.get('solver') or (item.get('result') or {}).get('mpc') or {}
        command=item.get('command')
        if isinstance(command,dict):command=[command['thrust_N'],*command['rates_rad_s']]
        time_s=item.get('time_s',item.get('elapsed_s',item.get('control_monotonic_s')))
        if time_s is None:time_s=item.get('host_monotonic_ns',item.get('host_ns',0))*1e-9
        rows.append(dict(time_s=float(time_s),state=state.tolist(),position_reference=position.tolist(),velocity_reference=velocity.tolist(),
            phase=item.get('phase',(item.get('command') or {}).get('phase') if isinstance(item.get('command'),dict) else ''),
            weight=float(weight),command=command,solver=solver,state_age_s=item.get('state_age_s',item.get('mocap_age_s')),
            moving_full_mpc=bool(weight>=.999 and np.linalg.norm(velocity)>1e-3)))
    return path,rows


def metrics(rows):
    selected=[r for r in rows if r['moving_full_mpc']]
    result=dict(samples=len(rows),moving_full_mpc_samples=len(selected),
        method='Same-tick causal mocap root state minus recorded reference; full MPC weight>=0.999 and reference speed>0.001m/s. No post-hoc phase/time/bias fitting.',
        limitation='Filtered host state and endpoint receipts are observations, not independent truth or proof of flight qualification.')
    if not selected:return result
    x=np.array([r['state'] for r in selected]);p=np.array([r['position_reference'] for r in selected]);v=np.array([r['velocity_reference'] for r in selected])
    error=x[:,:3]-p;velocity_error=x[:,7:10]-v
    q=x[:,3:7];tilt=np.degrees(np.arccos(np.clip(1-2*(q[:,1]**2+q[:,2]**2),-1,1)))
    result.update(xy_position_rmse_m=float(np.sqrt(np.mean(np.sum(error[:,:2]**2,axis=1)))),
        xyz_position_rmse_m=float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),
        velocity_3d_rmse_m_s=float(np.sqrt(np.mean(np.sum(velocity_error**2,axis=1)))),
        peak_measured_speed_m_s=float(np.linalg.norm(x[:,7:10],axis=1).max()),peak_reference_speed_m_s=float(np.linalg.norm(v,axis=1).max()),
        peak_measured_tilt_deg=float(tilt.max()))
    latency=[r['solver'].get('total_ms') for r in selected if r['solver'].get('total_ms') is not None]
    if latency:result['mpc_total_ms_p50_p99_max']=np.percentile(latency,[50,99,100]).tolist()
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--no-plots',action='store_true');args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    source,rows=read_controls(args.run);result=metrics(rows)
    result.update(source=str(source.resolve()),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    (args.output/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (args.output/'tracking.csv').open('w',newline='') as f:
        fields=['time_s','phase','mpc_weight','moving_full_mpc']+[f'{kind}_{axis}' for kind in ('position','position_reference','velocity','velocity_reference') for axis in 'xyz']
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for r in rows:
            out=dict(time_s=r['time_s'],phase=r['phase'],mpc_weight=r['weight'],moving_full_mpc=r['moving_full_mpc'])
            for kind,value in [('position',r['state'][:3]),('position_reference',r['position_reference']),('velocity',r['state'][7:10]),('velocity_reference',r['velocity_reference'])]:
                out.update({kind+'_'+axis:v for axis,v in zip('xyz',value)})
            writer.writerow(out)
    if rows and not args.no_plots:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        t=np.array([r['time_s'] for r in rows]);t-=t[0]
        state=np.array([r['state'] for r in rows]);ref=np.array([r['position_reference'] for r in rows]);vel=np.array([r['velocity_reference'] for r in rows])
        fig,axes=plt.subplots(2,3,figsize=(12,6),sharex=True)
        for j,axis in enumerate('xyz'):
            axes[0,j].plot(t,ref[:,j],'--',label='Reference',lw=.9);axes[0,j].plot(t,state[:,j],label='Measured',lw=.9)
            axes[1,j].plot(t,vel[:,j],'--',lw=.9);axes[1,j].plot(t,state[:,7+j],lw=.9)
            axes[0,j].set_ylabel(axis+' [m]');axes[1,j].set_ylabel('v'+axis+' [m/s]');axes[1,j].set_xlabel('Time [s]')
            for a in axes[:,j]:a.grid(ls=':',alpha=.5)
        axes[0,0].legend();fig.tight_layout();fig.savefig(args.output/'tracking.png',dpi=180);fig.savefig(args.output/'tracking.pdf');plt.close(fig)
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
