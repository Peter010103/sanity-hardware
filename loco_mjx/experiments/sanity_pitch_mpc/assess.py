"""Assess retained pitch-loop simulations, including failures and failed tuning variants."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
import numpy as np
from scipy.spatial.transform import Rotation

BLUE='#0072B2';ORANGE='#D55E00';GREEN='#009E73';RED='#B2182B';GRAY='#777777'


def load(study,seed):
    summary=json.loads((study/'summary.json').read_text())
    rows=[json.loads(line) for line in (study/f'episode_{seed:04d}.jsonl').read_text().splitlines()]
    if not rows:raise ValueError('No recorded control samples')
    state=np.array([r['state'] for r in rows]);reference=np.array([r['reference'] for r in rows]);t=np.array([r['time_s'] for r in rows])
    rotation=Rotation.from_quat(state[:,[4,5,6,3]]).as_matrix();refrotation=Rotation.from_quat(reference[:,[4,5,6,3]]).as_matrix()
    direction=summary['reference']['direction']
    pitch=-np.unwrap(np.arctan2(rotation[:,0,2],rotation[:,2,2]))/(2*np.pi)*direction
    rpitch=-np.unwrap(np.arctan2(refrotation[:,0,2],refrotation[:,2,2]))/(2*np.pi)*direction
    metric=next(e for e in summary['episodes'] if e['seed']==seed)
    return dict(summary=summary,rows=rows,state=state,ref=reference,t=t,pitch=pitch,rpitch=rpitch,metric=metric)


def trajectory(ax,d):
    p=d['summary']['reference'];theta=np.linspace(0,2*np.pi,501)
    ax.plot(p['horizontal_radius_m']*np.sin(theta),p['bottom_m']+p['vertical_radius_m']*(1-np.cos(theta)),color=ORANGE,ls='--')
    ax.plot(d['state'][:,0],d['state'][:,2],color=BLUE)
    ax.axhline(2.,color=RED,ls=':',label='2m ceiling')
    ax.set(xlabel='X [m]',ylabel='Z [m]',ylim=(.25,2.08),aspect='equal');ax.grid(True)


def shade(ax,d):
    r=d['summary']['reference'];ax.axvspan(r['hold_s'],min(d['t'][-1],r['hold_s']+r['moving_s']),color=GREEN,alpha=.09,linewidth=0)
    if d['metric']['failure']:ax.axvline(d['t'][-1],color=RED,ls='--')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path);args=parser.parse_args();out=args.root
    font=Path.home()/'.local/share/fonts/Avenir Regular.ttf'
    if font.is_file():font_manager.fontManager.addfont(str(font))
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=str(font)).get_name() if font.is_file() else 'DejaVu Sans','font.size':14,'axes.labelsize':15,
        'axes.titlesize':15,'legend.fontsize':12,'lines.linewidth':.8,'axes.linewidth':.8,'grid.linestyle':':','grid.alpha':.4,
        'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':3,'svg.fonttype':'path'})
    studies=[];allmetrics=[];verification=[]
    for path in sorted(out.glob('*/summary.json')):
        s=json.loads(path.read_text());name=path.parent.name;episodes=s['episodes']
        expected=s.get('requested_episodes',8 if name=='randomized_7p1' else len(episodes))
        success=[e for e in episodes if e['success']]
        study={'study':name,'records':len(episodes),'requested_episodes':expected,'finished':bool(s.get('finished_utc')),
            'successes':len(success),'solver':s['controller']['solver'],'observations':s['observations'],
            'observer_compensation':s.get('observer_compensation','pose'),'randomized_parameters':s['plant']['randomized'],
            'training_random_action_delay_enabled':s['plant'].get('training_random_action_delay_enabled',s['plant']['randomized']),
            'collective_lag_randomization_enabled':s['plant'].get('collective_lag_randomization_enabled',s['plant'].get('training_random_action_delay_enabled',s['plant']['randomized'])),
            'known_extra_command_delay_ms':1000*s['extra_command_delay_s'],'voltage_V':s['plant']['voltage_V'],
            'laps':s['reference']['laps'],'direction':s['reference']['direction'],
            'horizontal_radius_m':s['reference']['horizontal_radius_m'],'vertical_radius_m':s['reference']['vertical_radius_m'],
            'reference_top_m':s['reference']['top_m'],'phase_rate_rad_s':s['reference']['phase_rate_rad_s'],
            'thrust_control_weight':s['controller']['configuration'].get('control_cost',[.01])[0],
            'success_position_rmse_range_m':[min(e['moving_position_rmse_m'] for e in success),max(e['moving_position_rmse_m'] for e in success)] if success else None}
        studies.append(study)
        for e in episodes:
            record={k:v for k,v in e.items() if k not in ('physical_parameters','rejected_solver_result')};record['study']=name
            record.update({k:study[k] for k in ['observations','randomized_parameters','training_random_action_delay_enabled','collective_lag_randomization_enabled','known_extra_command_delay_ms','voltage_V','laps','direction']})
            p=path.parent/e.get('trajectory',f"episode_{e['seed']:04d}.jsonl")
            if p.exists() and e['ticks']:
                d=load(path.parent,e['seed']);state,ref,t=d['state'],d['ref'],d['t'];r=s['reference']
                mask=(t>=r['hold_s'])&(t<=r['hold_s']+r['moving_s'])
                rmse=float(np.sqrt(np.mean(np.sum((state[mask,:3]-ref[mask,:3])**2,axis=1)))) if mask.any() else None
                if rmse is not None:np.testing.assert_allclose(rmse,e['moving_position_rmse_m'],atol=1e-12)
                assert np.all(np.diff(t)>0)
                np.testing.assert_allclose(d['pitch'][-1]-d['pitch'][0],e['body_pitch_turns'],atol=1e-12)
                if e['success']:
                    assert e['failure'] is None and e['inversions']>=r['laps'] and e['hover_recovered'] and state[:,2].max()<=2.
                    assert e['body_pitch_turns']>r['laps']-.15 and e['measured_path_turns']>r['laps']-.15
                record['peak_pitch_command_rad_s']=float(max(abs(row['command'][2]) for row in d['rows']))
                record['deadline_overruns_in_applied_commands']=sum(row['mpc']['total_ms']>25 for row in d['rows'])
                verification.append({'study':name,'seed':e['seed'],'rmse_verified':True,'success_criterion_verified':True})
            allmetrics.append(record)
    result={'scope':'Simulation only; studies retain failed controls, iteration variants, current-rate counterexample and incomplete initial harness batch.',
        'no_hardware_access':True,'height':'Initial hover.5m;reference.5--1.8m;absolute ceiling2m;simulation ground-clearance stop.15m.',
        'rate_scope':'All successful pitch-loop studies use assumed simulation-only pitch authority9.5rad/s; existing deployed2.3rad/s unchanged.',
        'physics_scope':'Pinned loco_mjx MJX calibrated FC-voltage/firmware/collective lag/rate servo/headroom with implicit servo and air drag. High-rate response is not identified.',
        'selection':'Different stages reuse matched seeds; do not pool them into independent reliability statistics.',
        'delay_scope':'Early randomized suites used the training component that jointly varies unknown command delay/jitter AND collective lag. Later parameter-only suites hold collective lag nominal and separately apply known delays. Isolated delay and lag cohorts vary one component each. Repeated tuning seeds are not independent qualification samples.',
        'studies':studies,'episodes':allmetrics}
    (out/'assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    fields=['study','seed','success','failure','ticks','body_pitch_turns','measured_path_turns','inversions','hover_recovered','moving_position_rmse_m','moving_velocity_rmse_m_s',
        'final_position_error_m','final_speed_m_s','peak_pitch_command_rad_s','deadline_overruns_in_applied_commands','observations','randomized_parameters',
        'training_random_action_delay_enabled','collective_lag_randomization_enabled','known_extra_command_delay_ms','voltage_V','laps','direction']
    with (out/'metrics.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(allmetrics)
    with (out/'study_summary.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(studies[0]),extrasaction='ignore');writer.writeheader();writer.writerows(studies)
    selected=[('initial_causal',0,'Causal mocap: pose extrapolation'),('causal_compensated',20,'Tuned causal-state prediction'),('causal_delay20',30,'Tuned prediction +20ms known delay')]
    fig,axes=plt.subplots(3,3,figsize=(17,13));fig.subplots_adjust(left=.07,right=.98,top=.86,bottom=.08,hspace=.36,wspace=.31)
    for j,(study,seed,title) in enumerate(selected):
        d=load(out/study,seed);trajectory(axes[0,j],d)
        axes[0,j].set_title(title+f"\n{d['summary']['reference']['laps']} loop(s) | RMSE {100*d['metric']['moving_position_rmse_m']:.2f} cm",pad=15)
        for ax,ref,actual,label in [(axes[1,j],d['ref'][:,2],d['state'][:,2],'Z [m]'),(axes[2,j],d['rpitch'],d['pitch'],'Pitch rotations')]:
            ax.plot(d['t'],ref,color=ORANGE,ls='--');ax.plot(d['t'],actual,color=BLUE);shade(ax,d);ax.grid(True)
            ax.set(xlabel='Time [s]',ylabel=label)
        axes[1,j].axhline(2.,color=RED,ls=':')
    fig.suptitle('Sanity CTBR | MPC vertical pitch-loop simulation',fontsize=22,y=.98)
    fig.legend(handles=[Line2D([],[],color=ORANGE,ls='--',label='Reference'),Line2D([],[],color=BLUE,label='MJX plant'),Line2D([],[],color=RED,ls=':',label='2 m ceiling'),Patch(facecolor=GREEN,alpha=.12,label='Loop window')],
        loc='upper center',bbox_to_anchor=(.5,.95),ncol=4,frameon=False,handlelength=1.1)
    for ext in ('pdf','png','svg'):fig.savefig(out/('comparison.'+ext),dpi=170)
    observer_figure=fig
    final_cases=[('margin_nominal',902,'Final reference | causal mocap'),('margin_reverse40',200,'Reverse pitch | known40ms delay'),('margin_joint',102,'Joint lag/delay uncertainty')]
    final_fig,axes=plt.subplots(3,3,figsize=(17,13));final_fig.subplots_adjust(left=.07,right=.98,top=.84,bottom=.08,hspace=.37,wspace=.31)
    for j,(study,seed,title) in enumerate(final_cases):
        d=load(out/study,seed);trajectory(axes[0,j],d);m=d['metric']
        outcome='Five loops +hover recovered' if m['success'] else f"STOPPED after {m['body_pitch_turns']:.2f} pitch turns"
        error_label='RMSE' if m['success'] else 'Partial RMSE'
        axes[0,j].set_title(title+'\n'+outcome+f"\n{error_label}: {100*m['moving_position_rmse_m']:.2f} cm",pad=14)
        for ax,ref,actual,label in [(axes[1,j],d['ref'][:,2],d['state'][:,2],'Z [m]'),(axes[2,j],d['rpitch'],d['pitch'],'Pitch [turns]')]:
            ax.plot(d['t'],ref,color=ORANGE,ls='--');ax.plot(d['t'],actual,color=BLUE);shade(ax,d);ax.grid(True);ax.set(xlabel='Time [s]',ylabel=label)
        axes[1,j].axhline(2.,color=RED,ls=':')
    final_fig.suptitle('Sanity CTBR | Final pitch-loop reference and robustness boundary',fontsize=21,y=.985)
    final_fig.legend(handles=[Line2D([],[],color=ORANGE,ls='--',label='Reference'),Line2D([],[],color=BLUE,label='MJX plant'),Line2D([],[],color=RED,ls='--',label='Rejected output /stop'),Patch(facecolor=GREEN,alpha=.12,label='Loop window')],loc='upper center',bbox_to_anchor=(.5,.95),ncol=4,frameon=False,handlelength=1.1)
    for ext in ('pdf','png','svg'):final_fig.savefig(out/('final_comparison.'+ext),dpi=170)
    with PdfPages(out/'simulation_report.pdf') as report:
        report.savefig(final_fig);plt.close(final_fig)
        report.savefig(observer_figure);plt.close(observer_figure)
        focus=['five_loops_truth','causal_compensated','causal_delay20','parameter_uncertainty_7p1','parameter_uncertainty_reverse_delay40','isolated_unknown_delay','isolated_collective_lag','final_tuned_lag','margin_reverse40','margin_joint','randomized_7p1','current_rate_limit_baseline']
        ss=[next(s for s in studies if s['study']==name) for name in focus]
        fig,ax=plt.subplots(figsize=(18,11));y=np.arange(len(ss))
        ax.barh(y,[s['successes'] for s in ss],color=GREEN,label='Completed loops +hover')
        ax.barh(y,[s['records']-s['successes'] for s in ss],left=[s['successes'] for s in ss],color=RED,alpha=.7,label='Failed criteria /rejected output')
        ax.barh(y,[s['requested_episodes']-s['records'] for s in ss],left=[s['records'] for s in ss],color=GRAY,alpha=.4,label='Initial harness interruption')
        ax.set_yticks(y,labels=['True state, perturbed starts','Causal state, tuned','Causal +known20ms','Mass/inertia/servo gains,7.1V','Gains/inertia,reverse +known40ms','Gains/inertia +unknown delay only','Gains/inertia +collective lag only','Tuned objective +lag variation','Final reference,reverse +known40ms','Final reference,joint lag/delay','Initial joint uncertainty (partial batch)','Current2.3rad/s pitch limit'])
        for i,s in enumerate(ss):ax.text(s['requested_episodes']+.10,i,f"{s['successes']}/{s['requested_episodes']}",va='center')
        ax.invert_yaxis();ax.set(xlabel='Episodes',title='Separate simulation cohorts; repeated tuning seeds are not independent reliability trials',xlim=(0,9));ax.grid(axis='x');ax.legend(frameon=False,handlelength=1.1,loc='lower right')
        fig.tight_layout();fig.savefig(out/'robustness.pdf');report.savefig(fig);plt.close(fig)
        details=[('margin_nominal',902),('margin_reverse40',200),('margin_joint',102),('final_tuned_lag',101),('randomized_7p1',102),('current_rate_limit_baseline',0)]
        for study,seed in details:
            d=load(out/study,seed);m=d['metric'];fig,axes=plt.subplots(2,3,figsize=(17,10),layout='constrained')
            fig.suptitle(f"{study} | seed{seed} | "+('completed loops and hover' if m['success'] else 'FAILED: '+str(m['failure'] or 'maneuver/recovery criterion')),fontsize=19)
            trajectory(axes[0,0],d)
            times=d['t'];commands=np.array([r['command'] for r in d['rows']])
            series=[(axes[0,1],d['ref'][:,2],d['state'][:,2],'Z [m]'),(axes[0,2],d['rpitch'],d['pitch'],'Pitch rotations'),
                (axes[1,0],commands[:,2],d['state'][:,11],'Body pitch rate [rad/s]'),(axes[1,1],commands[:,0],d['state'][:,13],'Collective thrust [N]'),
                (axes[1,2],None,np.linalg.norm(d['state'][:,:3]-d['ref'][:,:3],axis=1),'Position error [m]')]
            for ax,ref,actual,label in series:
                if ref is not None:ax.plot(times,ref,color=ORANGE,ls='--',label='Command' if 'rate' in label or 'thrust' in label else 'Reference')
                ax.plot(times,actual,color=BLUE,label='MJX plant');shade(ax,d);ax.set(xlabel='Time [s]',ylabel=label);ax.grid(True)
            axes[0,1].axhline(2.,color=RED,ls=':');axes[1,0].legend(frameon=False,handlelength=1.1)
            report.savefig(fig);fig.savefig(out/(study+'_diagnostics.pdf'));plt.close(fig)
    (out/'verification.json').write_text(json.dumps({'passed':True,'episode_checks':verification,'completed_trajectory_records':len(verification),
        'all_available_failed_trajectories_included':True,'initial_harness_missing_episode_traces':1,'physical_rate_baseline_does_not_flip':not next(s for s in studies if s['study']=='current_rate_limit_baseline')['successes']},indent=2)+'\n')
    print(json.dumps(studies,indent=2))


if __name__=='__main__':main()
