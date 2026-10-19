#!/usr/bin/env python3
"""Plot full and zoomed bifilar collections without period or inertia fitting."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
DATA=ROOT/'data/processed/inertia/20261019'


def extract(run):
    directory=ROOT/run['run_directory']
    source=directory/'poses.csv'
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    if digest!=run['pose_csv_sha256']:
        raise ValueError('Raw recording differs from manifest: '+str(source))
    with source.open() as stream:
        rows=list(csv.DictReader(stream))
    cue=json.loads((directory/'capture_receipt.json').read_text())['cue_monotonic_s']
    time=np.array([float(row['host_monotonic_s']) for row in rows])-cue
    quaternions=np.array([[float(row[key]) for key in ['qx','qy','qz','qw']] for row in rows])
    if not np.isfinite(quaternions).all() or np.any(np.linalg.norm(quaternions,axis=1)<.5):
        raise ValueError('Invalid quaternion in '+str(source))
    rotations=Rotation.from_quat(quaternions)
    late=time>=time[-1]-10
    reference=rotations[late].mean()
    relative=(rotations*reference.inv()).as_quat()
    # Swing/twist decomposition about recorded map +Z, as in the acquisition tool.
    if np.any(np.hypot(relative[:,2],relative[:,3])<1e-8):
        raise ValueError('Vertical twist decomposition singular')
    angle=np.degrees(np.unwrap(2*np.arctan2(relative[:,2],relative[:,3])))
    angle-=np.median(angle[late])
    return time,angle,digest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start',type=float,default=0,help='Zoom start in seconds after the countdown cue')
    parser.add_argument('--end',type=float,default=10,help='Zoom end in seconds after the countdown cue')
    parser.add_argument('--output',type=Path,default=ROOT/'build/inertia/timeseries')
    args=parser.parse_args()
    if args.end<=args.start:parser.error('--end must be greater than --start')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,
                         'pdf.fonttype':42,'savefig.facecolor':'white'})
    inputs=json.loads((DATA/'measurement_inputs.json').read_text())
    manifest={'runs':[{'axis':r['axis_label'],'run_directory':r['raw_run_directory'],'pose_csv_sha256':r['pose_csv_sha256']} for r in inputs['trials']]}
    fig,axes=plt.subplots(3,2,figsize=(13,8),gridspec_kw={'width_ratios':[1.1,1]},sharex='col')
    colors=['#2166ac','#238b45','#c45721'];records=[]
    args.output.mkdir(parents=True,exist_ok=True)
    for row,(run,color) in enumerate(zip(manifest['runs'],colors)):
        time,angle,digest=extract(run)
        selected=(time>=args.start)&(time<=args.end)
        if selected.sum()<2:raise ValueError('Requested zoom outside '+run['axis']+' recording')
        post=time>=0
        overview,zoom=axes[row]
        for ax in [overview,zoom]:
            ax.plot(time[post],angle[post],color=color,lw=.9)
            ax.axhline(0,color='#888888',lw=.7,alpha=.6,zorder=0)
            ax.grid(alpha=.18)
            ax.set_ylabel('Vertical twist (°)')
            ax.yaxis.set_major_locator(MaxNLocator(5))
        overview.axvspan(args.start,args.end,color=color,alpha=.11,zorder=0)
        overview.set_xlim(0,45)
        zoom.set_xlim(args.start,args.end)
        low,high=np.percentile(angle[selected],[0,100]);margin=max((high-low)*.09,.3)
        zoom.set_ylim(low-margin,high+margin)
        overview.text(.98,.88,run['axis'].capitalize()+' mounting',transform=overview.transAxes,
                      ha='right',fontweight='bold',color=color)
        with (args.output/(run['axis']+'_vertical_twist.csv')).open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['time_after_cue_s','vertical_twist_deg'])
            writer.writerows(zip(time,angle))
        records.append({'axis':run['axis'],'source_directory':run['run_directory'],
                        'source_csv_sha256':digest,'zoom_samples':int(selected.sum()),
                        'zoom_angle_range_deg':[float(low),float(high)]})
    axes[0,0].set_title('Full collection · shaded region is shown at right',fontsize=11,pad=13)
    axes[0,1].set_title(f'Zoom · {args.start:g}–{args.end:g} s',fontsize=11,pad=13)
    for ax in axes[-1]:ax.set_xlabel('Time after countdown cue (s)')
    fig.suptitle('Bifilar pendulum: measured angular time series',fontsize=17,y=.97)
    fig.text(.5,.925,'Yaw, pitch and roll identify the mounting; every trace measures twist about the fixed vertical axis.',ha='center',fontsize=10)
    fig.text(.5,.025,'Raw quaternion-derived angles; no smoothing or oscillation fit. Zero angle: late-recording reference. Actual release time was not separately marked.',ha='center',fontsize=9,color='#555555')
    fig.subplots_adjust(left=.075,right=.975,top=.85,bottom=.1,hspace=.27,wspace=.24)
    stem=f'collection_zoom_{args.start:g}_{args.end:g}s'
    for extension in ['png','pdf']:
        output=args.output/(stem+'.'+extension)
        fig.savefig(output,dpi=190)
        print('Saved: '+str(output),flush=True)
    plt.close(fig)
    metadata={'zoom_window_s_after_cue':[args.start,args.end],
              'time_basis':'host_monotonic_s minus capture countdown cue',
              'angular_definition':'q_relative = q_sample * inverse(q_reference); angle=unwrap(2 atan2(q_relative.z,q_relative.w)); late median removed',
              'reference':'Quaternion mean over final 10 seconds',
              'vertical_axis':'Recorded map +Z, consistent with acquisition plotting convention',
              'release_time_separately_recorded':False,'smoothing_applied':False,
              'period_or_inertia_fit_performed':False,'raw_recordings_modified':False,'recordings':records}
    (args.output/(stem+'.json')).write_text(json.dumps(metadata,indent=2)+'\n')


if __name__=='__main__':main()
