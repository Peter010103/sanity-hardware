#!/usr/bin/env python3
"""Compact native-MJX fleet flip overview: full paths and body pitch-rate response."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path);parser.add_argument('--seed',type=int);parser.add_argument('--output',type=Path)
    args=parser.parse_args();run=args.run.resolve();report=json.loads((run/'summary.json').read_text());plan=report['plan']
    episode=next((e for e in report['episodes'] if e['seed']==args.seed),None) if args.seed is not None else report['episodes'][0]
    if episode is None:parser.error('Seed not found')
    root=Path(plan['workspace']);source=run/'source/loco_mjx/experiments/sanity_pitch_mpc/run_in_reference.py'
    if not source.exists():source=root/'loco_mjx/experiments/sanity_pitch_mpc/run_in_reference.py'
    spec=importlib.util.spec_from_file_location('frozen_flip_reference',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    profile=run/'source/loco_mjx/experiments/sanity_pitch_mpc/run_in_profile.json'
    if not profile.exists():profile=root/'loco_mjx/experiments/sanity_pitch_mpc/run_in_profile.json'
    reference=module.RunInPitchReference(plan['mpc_configuration'],plan['laps'],plan['bottom_m'],plan['direction'],profile=json.loads(profile.read_text()))
    plt.rcParams.update({'font.family':'Avenir','font.size':13,'axes.labelsize':14,'axes.titlesize':15,'lines.linewidth':.8,
        'grid.linestyle':':','grid.linewidth':.5,'legend.handlelength':1.1,'pdf.fonttype':3})
    fig=plt.figure(figsize=(10.2,8.),layout='constrained');grid=fig.add_gridspec(2,1,height_ratios=[1.45,1.])
    path=fig.add_subplot(grid[0],projection='3d');rate=fig.add_subplot(grid[1]);colors=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00','#56B4E9']
    common=episode['group_epoch'];body_rates=[];commands=[];times=None;all_positions=[]
    for index,drone in enumerate(plan['drones']):
        records=[]
        with (run/f"{drone['id']:02d}"/f"episode_{episode['seed']}.jsonl").open() as file:
            for line in file:
                r=json.loads(line)
                if common-.5<=r['time_s']<=common+reference.moving_s+1.:
                    records.append((r['time_s']-common,r['state'],r['command'],r['reference']['position']))
        t=np.asarray([r[0] for r in records]);values=np.asarray([r[1] for r in records]);cmd=np.asarray([r[2] for r in records]);desired=np.asarray([r[3] for r in records])
        path.plot(values[:,0],values[:,1],values[:,2],color=colors[index%len(colors)],label=drone['tracked_frame'].replace('sanity','Sanity'))
        path.plot(desired[:,0],desired[:,1],desired[:,2],color='#555555',ls='--',alpha=.5,label='Reference' if index==0 else None)
        path.scatter(*values[0,:3],s=14,color=colors[index%len(colors)])
        if times is None:times=t
        body_rates.append(np.interp(times,t,values[:,11]));commands.append(np.interp(times,t,cmd[:,2]));all_positions.append(values[:,:3])
    actual=np.asarray(body_rates);outgoing=np.asarray(commands);goal=reference.sample(reference.hold+times)[:,11]
    rate.plot(times,goal,color='#222222',ls='--',label='Reference')
    rate.fill_between(times,actual.min(axis=0),actual.max(axis=0),color='#0072B2',alpha=.22,label='Range across aircraft')
    rate.plot(times,np.median(actual,axis=0),color='#0072B2',label='Actual, median')
    rate.plot(times,np.median(outgoing,axis=0),color='#D55E00',alpha=.8,label='Command, median')
    rate.set_xlabel('Time from common start [s]');rate.set_ylabel('Pitch rate [rad/s]');rate.grid(True);rate.spines[['top','right']].set_visible(False)
    rate.legend(ncols=2,loc='lower right',fontsize=11)
    path.set_xlabel('X [m]',labelpad=6);path.set_ylabel('Y [m]',labelpad=6);path.set_zlabel('Z [m]',labelpad=5)
    positions=np.vstack(all_positions);extent=np.ptp(positions,axis=0);path.set_box_aspect(np.maximum(extent,[1.,1.,1.]));path.view_init(elev=23,azim=-58)
    path.set_zlim(0.,2.);path.legend(ncols=4,loc='upper center',bbox_to_anchor=(.5,1.02),fontsize=10,frameon=False)
    for axis in (path.xaxis,path.yaxis,path.zaxis):axis._axinfo['grid'].update(linestyle=':',linewidth=.5,color=(.6,.6,.6,.5))
    fig.suptitle(f"{len(plan['drones'])} coordinated MPC pitch flips — native MJX",fontsize=17)
    output=(args.output or run/'flip_overview').resolve();output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output.with_suffix('.pdf'));fig.savefig(output.with_suffix('.png'),dpi=180)
    print(output.with_suffix('.pdf'))


if __name__=='__main__':main()
