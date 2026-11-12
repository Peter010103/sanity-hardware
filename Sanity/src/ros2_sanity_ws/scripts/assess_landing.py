#!/usr/bin/env python3
"""Summarise recorded flight/shadow runs and check simulated landing evidence."""
import argparse
import json
from pathlib import Path
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    a=parser.parse_args()
    simulated=(a.run/'simulation').is_dir()
    run=a.run/'simulation' if simulated else a.run
    receipt=json.loads((run/'controller/receipt.json').read_text())
    control=rows(run/'controller/control.jsonl')
    history=rows(run/'controller/history.jsonl')
    events=rows(run/'controller/events.jsonl')
    wire=[e for e in events if e['kind']=='zenoh_tx_command']
    status=[e for e in events if e['kind']=='zenoh_rx_status']
    checks={'controller_completed_without_error':receipt['error'] is None,
        'no_recording_error':receipt['recording_error'] is None,
        'ticks_ordered':[r['tick'] for r in control]==list(range(len(control))),
        'all_selected_history_committed':len(control)==len(history)==receipt['history_processed']==receipt['history_submitted'],
        'raw_actor_inputs_recorded':bool(history) and all('actor_inputs' in h['result'] for h in history),
        'zenoh_wire_and_status_recorded':bool(wire and status),
        'mcap_recorded':bool(list((a.run/'mocap').glob('*.mcap'))),
        'rosbag_index_finalized':(a.run/'mocap/metadata.yaml').is_file()}
    is_mpc=receipt.get('controller')=='mpc'
    if is_mpc:
        checks.pop('raw_actor_inputs_recorded')
        checks['mpc_inputs_recorded']=bool(history) and all('controller_inputs' in h['result'] and 'reference_preview' in h['observation'] for h in history)
    max_history_error=0.
    for r,h in zip(control,history):
        checks['all_selected_history_committed'] &= r['history_event']==h['selection'] and r['history_observation']==h['observation']
        if is_mpc:
            max_history_error=max(max_history_error,float(np.max(abs(np.asarray(h['committed']['ctbr'])-r['command']))))
            checks['all_selected_history_committed'] &= h['committed']['ticks']==r['tick']+1
            continue
        expected=h['selection'].get('selected_action')
        if expected is None:
            expected=(np.asarray(r['command'])-receipt['policy']['hover_ctrl'])/receipt['policy']['scaling']
        max_history_error=max(max_history_error,float(np.max(abs(np.asarray(h['committed']['action'])-expected))))
        checks['all_selected_history_committed'] &= h['result']['history_age']==min(r['tick'],8) and h['committed']['history_age']==min(r['tick']+1,8)
    checks['history_matches_selected_command']=max_history_error<2e-6
    if receipt['policy'].get('residual'):
        checks['outer_history_matches_selected_commands']=bool(history) and all(
            h['committed'].get('outer',{}).get('selected_velocity_world')==h['selection'].get('selected_velocity_world') and
            h['committed'].get('outer',{}).get('selected_ctbr')==r['command'] and
            h['committed'].get('outer',{}).get('ticks')==r['tick']+1 for r,h in zip(control,history))
        checks['outer_causal_history_continuous']=all(
            h['result']['outer']['previous_velocity_world']==previous['committed']['outer']['selected_velocity_world'] and
            h['result']['outer']['previous_selected_ctbr']==previous['committed']['outer']['selected_ctbr'] and
            h['result']['outer']['hidden_before']==previous['result']['outer']['hidden_after'] for previous,h in zip(history,history[1:]))
        checks['outer_raw_inputs_recorded']=bool(history) and all('preview' in h['observation'] and 'state_features' in h['result']['outer'] for h in history)
    metrics={'controller':receipt.get('controller','policy'),'selected_ticks':len(control),'history_action_max_error':max_history_error,'wire_commands':len(wire),'receiver_statuses':len(status)}
    if is_mpc:
        solves=[h['result']['mpc'] for h in history]
        metrics['mpc_solve_ms_p50_p95_p99_max']=np.percentile([r['solve_ms'] for r in solves],[50,95,99,100]).tolist() if solves else None
        metrics['mpc_invalid_results']=sum(not r['success'] for r in solves)
        metrics['mpc_total_ms_p50_p95_p99_max']=np.percentile([r['total_ms'] for r in solves],[50,95,99,100]).tolist() if solves else None
        metrics['mpc_core_ms_p50_p95_p99_max']=np.percentile([r.get('core_solve_ms',r['solve_ms']) for r in solves],[50,95,99,100]).tolist() if solves else None
        checks['mpc_selected_results_valid']=all(r.get('mpc',{}).get('success') for r in control if r['policy_weight']>0)
    metrics['mocap_rejected_samples']=sum(e['kind']=='mocap_rejected_sample' for e in events)
    metrics['mocap_estimator_errors']=list(dict.fromkeys(e['record']['packet']['error'] for e in events
        if e['kind']=='tf_estimate' and e['record']['packet'].get('error')))
    metrics['mission_plan']=receipt.get('mission_plan')
    goals=[r.get('goal_diagnostics') for r in control if r.get('goal_diagnostics')]
    metrics['velocity_goal_limited_ticks']=sum(g['limited'] for g in goals)
    metrics['peak_raw_velocity_goal_m_s']=max((float(np.linalg.norm(g['raw_world_velocity_m_s'])) for g in goals),default=None)
    metrics['peak_selected_velocity_goal_m_s']=max((float(np.linalg.norm(g['selected_world_velocity_m_s'])) for g in goals),default=None)
    pattern=receipt['parameters'].get('mission_pattern','circle')
    if control:
        t=np.asarray([r['time_s'] for r in control]);state=np.asarray([r['state'] for r in control]);ref=np.asarray([r['reference']['position'] for r in control])
        moving=np.asarray([r['reference']['phase'] in tuple(pattern+'_'+phase for phase in ('accelerate','cruise','decelerate')) for r in control])
        metrics.update(duration_s=float(t[-1]),landing_start_s=receipt['landing_start_s'],touchdown_confirmed_s=receipt['landing_touchdown_s'],landed=receipt['landed'],landing_reason=receipt['landing_reason'],
            final_altitude_m=float(state[-1,2]),ground_altitude_m=receipt['ground_root'],command_limited_ticks=sum(r['command_limited'] for r in control),
            circle_xy_rmse_m=float(np.sqrt(np.mean(np.sum((state[moving,:2]-ref[moving,:2])**2,axis=1)))) if moving.any() else None,
            policy_start_s=receipt.get('policy_start_s'),orbit_start_s=receipt.get('orbit_start_s'),
            maximum_policy_weight=max(r['policy_weight'] for r in control),
            final_height_above_initial_ground_m=float(state[-1,2]-receipt['ground_root']))
        metrics['trajectory_pattern']=pattern
        metrics['trajectory_xy_rmse_m']=metrics['circle_xy_rmse_m']
        if pattern!='circle':metrics['circle_xy_rmse_m']=None
        metrics['reference_laps_completed']=max((r['reference'].get('reference_laps',0.) for r in control),default=0.)
        if receipt['parameters']['enable_output']:
            if receipt['error'] and 'mocap' in receipt['error'].lower():
                checks['mocap_fault_stopped']=True
            else:
                checks['landing_completed']=receipt['landed'] and receipt['error'] is None
                exit_condition=receipt.get('landing_exit_condition',{})
                height_limit=exit_condition.get('height_tolerance_m',.015)
                checks['near_ground_before_disarm']=abs(state[-1,2]-receipt['ground_root'])<=height_limit
                if exit_condition:
                    metrics['landing_exit_condition']=exit_condition
                    start=receipt.get('landing_touchdown_s')
                    exit_rows=[r for r in control if start is not None and start-exit_condition['dwell_s']-1e-6<=r['time_s']<=start+1e-6]
                    checks['landing_low_height_dwell']=bool(exit_rows) and start-exit_rows[0]['time_s']>=exit_condition['dwell_s']-.021 and all(abs(r['state'][2]-receipt['ground_root'])<=height_limit for r in exit_rows)
                checks['collective_zero_before_disarm']=abs(control[-1]['command'][0])<1e-6
                landed_rows=[r for r in control if r['reference']['phase'].startswith('landing_')]
                checks['landing_uses_pd']=bool(landed_rows) and all(r['policy_weight']==0 for r in landed_rows if r['time_s']>=receipt['landing_start_s']+2.01)
                if receipt['landing_reason']=='mission_complete':
                    complete=moving.any() and all(r['policy_weight']==1 for r,m in zip(control,moving) if m) and not receipt['fallback']
                    if pattern=='circle':
                        checks['complete_circle_before_landing']=complete
                        angle=np.unwrap(np.arctan2(state[moving,1]-receipt['parameters']['origin_xy'][1],state[moving,0]-receipt['parameters']['origin_xy'][0]))
                        metrics['measured_laps']=float(abs(angle[-1]-angle[0])/(2*math.pi)) if angle.size>=2 else None
                        checks['two_circle_laps']=metrics['measured_laps'] is not None and abs(metrics['measured_laps']-receipt['parameters'].get('laps',2))<.1
                    else:
                        checks['complete_trajectory_before_landing']=complete
                        checks['requested_reference_laps_completed']=metrics['reference_laps_completed']>=receipt['parameters'].get('laps',2)-1e-6
        fig,ax=plt.subplots(2,2,figsize=(10,7),layout='constrained')
        ax[0,0].plot(ref[:,0],ref[:,1],'--',label='reference');ax[0,0].plot(state[:,0],state[:,1],label='mocap estimate');ax[0,0].axis('equal');ax[0,0].set(xlabel='x [m]',ylabel='y [m]');ax[0,0].legend()
        ax[0,1].plot(t,ref[:,2],'--',label='reference');ax[0,1].plot(t,state[:,2],label='altitude');ax[0,1].set(xlabel='time [s]',ylabel='z [m]');ax[0,1].legend()
        ax[1,0].plot(t,[r['policy_weight'] for r in control],label='MPC authority' if is_mpc else 'policy authority');ax[1,0].set(xlabel='time [s]',ylabel='authority',ylim=(-.05,1.05));ax[1,0].legend()
        ax[1,1].plot(t,[r['command'][0] for r in control],label='selected collective');ax[1,1].set(xlabel='time [s]',ylabel='thrust [N]');ax[1,1].legend()
        for axis in ax.flat:axis.grid(alpha=.2)
        fig.suptitle('Sanity takeoff / '+pattern+' / PID landing — '+('ROS simulation clock' if simulated else 'live clock'))
        fig.savefig(a.run/'experiment.png',dpi=150);fig.savefig(a.run/'experiment.pdf')
    delayed = [h for h in history if h['observation'].get('test_delay_s',0)>0]
    if delayed:
        h = delayed[0]
        begin,end = h['observation']['prepared_host_ns'],h['result']['finished_host_ns']
        metrics['injected_stall_wall_ms'] = (end-begin)/1e6
        metrics['pd_updates_during_stall'] = sum(begin<=r['control_ns']<=end for r in control)
        commands = rows(run/'controller/commands.jsonl')
        metrics['command_updates_during_stall'] = sum(begin<=r['host_ns']<=end for r in commands)
        first = next((r for r in control if r['fallback']),None)
        checks['fallback_caused_by_stall'] = bool(first and begin<=first['control_ns']<=end)
        checks['pd_independent_of_inference'] = metrics['pd_updates_during_stall']>=2
        checks['commands_independent_of_inference'] = metrics['command_updates_during_stall']>=4
        checks['fault_landed'] = receipt['landed'] and receipt['landing_reason']=='policy_fallback'
    if (a.run/'mocap/metadata.yaml').is_file():
        import yaml
        metadata = yaml.safe_load((a.run/'mocap/metadata.yaml').read_text())['rosbag2_bagfile_information']
        metrics['rosbag_topic_counts'] = {r['topic_metadata']['name']:r['message_count'] for r in metadata['topics_with_message_count']}
        checks['raw_mocap_messages_in_bag'] = metrics['rosbag_topic_counts'].get('/tf',0)>0
    if wire:
        checks['final_wire_disarmed']=wire[-1]['enabled'] is False and wire[-1]['values']==[1500,1500,1500,1000]
    metrics['shutdown_disabled_acknowledged']=receipt.get('sender_shutdown_acknowledged')
    if 'sender_shutdown_acknowledged' in receipt:
        checks['shutdown_disabled_acknowledged']=receipt['sender_shutdown_acknowledged'] is True
    if simulated:
        sim=json.loads((run/'simulation_receipt.json').read_text())
        checks['final_receiver_disarmed']=sim['final_receiver']['enabled'] is False
        plant=rows(run/'plant.jsonl')
        metrics['final_physical_altitude_m']=plant[-1]['state'][2]
        metrics['final_ground_contacts']=plant[-1]['contacts']
        if receipt['landed']:
            checks['physical_ground_contact']=plant[-1]['contacts']>0
    experiment=json.loads((a.run/'experiment_receipt.json').read_text())
    checks['rosbag_closed_cleanly']=experiment['rosbag_returncode']==0
    summary={'passed':bool(all(checks.values())),'checks':{k:bool(v) for k,v in checks.items()},'metrics':metrics,'controller_error':receipt['error'],
        'scope':'Functional simulation with native Zenoh and supplied motor data; not flight qualification' if simulated else 'Recorded live deployment; tracking alone does not validate motor feedforward',
        'motor_mapping':receipt['mapping'],'live_clock_qualified':False}
    (a.run/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({'passed':summary['passed'],'checks':summary['checks'],'metrics':metrics,'controller_error':receipt['error']},indent=2))
    if not summary['passed']:raise SystemExit(1)


if __name__=='__main__':main()
