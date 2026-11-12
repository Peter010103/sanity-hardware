"""Native batched MJX validation of the same coordinated pitch missions; no network."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import sys
import time
os.environ.setdefault('JAX_PLATFORMS','cuda');os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import jax
import jax.numpy as jnp
import numpy as np
from scipy.spatial.transform import Rotation

from sanity_zenoh.fleet_pitch_core import PitchFleetClock,create_missions,check_paths


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--plan',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();plan=json.loads(args.plan.read_text());root=Path(plan['workspace']);output=args.output
    sys.path.insert(0,str(root/'loco_mjx/experiments/sanity_pitch_mpc'))
    from plant import PitchLoopPlant
    from run import SolverWorker
    output.mkdir(parents=True,exist_ok=False)
    configuration=plan['mpc_configuration'];drones=plan['drones'];count=len(drones);devices=[d['id'] for d in drones]
    plant=PitchLoopPlant(root/'outputs/sanity_pitch_flips_hover_20261021/source',worlds=count,voltage=plan['voltage_V'],
        randomized=plan['randomized'],mass_kg=configuration['mass_kg'],ground_height=.03,
        random_actuator_lag=plan['random_actuator_lag'],random_action_delay=plan['random_action_delay'],rate_response='flight',
        angular_rate_filter_tau_s=configuration.get('angular_rate_filter_tau_s',.05),position_filter_tau_s=configuration.get('position_filter_tau_s',.01),attitude_filter_tau_s=configuration.get('attitude_filter_tau_s',.01),wire_rate_scale=1000)
    configuration={**configuration,**plant.parameters}
    missions=create_missions(root,configuration,drones,plan['laps'],plan['bottom_m'],plan['direction'],skip_horizontal_limit=plan.get('skip_horizontal_limit',False))
    homes={d['id']:d['home_xy'] for d in drones};separation,trees=check_paths(missions,homes,plan['body_radius_m'],plan['tracking_margin_m'],minimum_path_separation_m=plan.get('minimum_path_separation_m'),skip_swept_path_check=plan.get('skip_swept_path_check',False))
    cfg=output/'mpc_config.json';cfg.write_text(json.dumps(configuration,indent=2)+'\n')
    workers={};pool=ThreadPoolExecutor(max_workers=count)
    report={'scope':'Native independent batched MJX worlds with shared-world placement and coordinated mission adapter; no inter-drone aero/contact; no ROS/Zenoh/ESP',
        'physical_commands_sent':False,'plan':plan,'plant':plant.info,'separation':separation,'episodes':[]}
    sources=[*Path(__file__).parent.glob('fleet_pitch*.py'),Path(__file__).with_name('pitch_mission.py'),*Path(root/'loco_mjx/experiments/sanity_pitch_mpc').glob('*.py')]
    sources.append(root/'loco_mjx/experiments/sanity_pitch_mpc/run_in_profile.json')
    if plant.flight_rate is not None:sources.append(Path(plant.flight_rate['profile']))
    report['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    for p in sources:
        target=output/'source'/p.relative_to(root);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(p.read_bytes())
    try:
        for d in devices:
            folder=output/f'{d:02d}';folder.mkdir();workers[d]=SolverWorker(cfg,folder)
        for episode in range(plan['episodes']):
            seed=plan['seed']+episode;started=time.monotonic()
            state=plant.reset_compiled(jax.random.split(jax.random.PRNGKey(seed),count),.2)
            missions=create_missions(root,configuration,drones,plan['laps'],plan['bottom_m'],plan['direction'],skip_horizontal_limit=plan.get('skip_horizontal_limit',False))
            clock=PitchFleetClock(missions,skip_prestart_readiness_check=plan.get('skip_prestart_readiness_check',False),skip_path_corridor_check=plan.get('skip_path_corridor_check',False));pending={d:None for d in devices};queues={d:[np.zeros(4) for _ in range(plan['output_delay_ticks'])] for d in devices}
            records={d:[] for d in devices};errors={};outputs={d:(output/f'{d:02d}'/f'episode_{seed}.jsonl').open('w') for d in devices}
            obs,ages=plant.observe_compiled(state);obs=np.asarray(obs).copy()
            for i,d in enumerate(devices):obs[i,:2]+=homes[d];workers[d].call(op='reset')
            clock.begin(0.,dict(zip(devices,obs)))
            minimum_distance=float('inf');last_applied={d:np.zeros(4) for d in devices};stall_samples=0
            for tick in range(6001):
                now=tick*.02;raw,ages=plant.observe_compiled(state);observations=np.asarray(raw).copy();ages=np.asarray(ages)
                qpos=np.asarray(state['physical'].data.qpos).copy();qvel=np.asarray(state['physical'].data.qvel)
                for i,d in enumerate(devices):observations[i,:2]+=homes[d];qpos[i,:2]+=homes[d]
                commands={};applied=[];references={};selected={d:False for d in devices};candidates=dict(pending)
                for i,d in enumerate(devices):
                    m=missions[d]
                    if d in errors or m.done:commands[d]=np.zeros(4);references[d]=m.sample(now);continue
                    try:
                        command,reference=m.choose(now,observations[i],pending[d],configuration['maximum_thrust_N'])
                        m.commit(now,command);selected[d]=True;commands[d]=command;references[d]=reference
                        clock.check_path_corridor(now,d,qpos[i,:3],trees[d],plan['tracking_margin_m'])
                    except Exception as error:
                        errors[d]=str(error);commands[d]=np.zeros(4);references[d]=m.sample(now);clock.request_landing('participant_failure')
                clock.update(now)
                for i,a in enumerate(devices):
                    for j in range(i+1,count):minimum_distance=min(minimum_distance,float(np.linalg.norm(qpos[i,:3]-qpos[j,:3])))
                futures={}
                for i,d in enumerate(devices):
                    m=missions[d]
                    if d not in errors and not m.done and m.landing is None:
                        delay=np.asarray(state['physical'].internal_state['mocap_state']['pose_delay'])[i]
                        request=m.inference_request(now,observations[i],float(ages[i]),{'position':float(delay[0]),'attitude':float(delay[1])})
                        futures[d]=(pool.submit(workers[d].call,**request),request)
                    else:pending[d]=None
                for d,(future,request) in futures.items():
                    result=future.result();result.update(request_ns=request['request_ns'],reference_epoch=request['reference_epoch'],motion_epoch=request['motion_epoch'])
                    pending[d]=result
                for i,d in enumerate(devices):
                    command=commands[d];q=queues[d]
                    if q:q.append(command.copy());actual=q.pop(0)
                    else:actual=command
                    stall=plan.get('arrival_stall')
                    stalled=bool(stall and d==stall['device'] and clock.epoch is not None
                        and clock.epoch+stall['start_after_common_s']<=now<clock.epoch+stall['start_after_common_s']+stall['duration_s'])
                    if stalled:actual=last_applied[d].copy();stall_samples+=1
                    else:last_applied[d]=actual.copy()
                    applied.append(actual)
                    row={'tick':tick,'time_s':now,'state':np.r_[qpos[i],qvel[i]].tolist(),'observation':observations[i].tolist(),
                        'phase':missions[d].phase,'reference':references[d],'command':command.tolist(),'applied':actual.tolist(),
                        'motion_epoch':missions[d].motion_epoch,'group_epoch':clock.epoch,'state_age_s':float(ages[i]),
                        'mpc':candidates[d]['mpc'] if candidates[d] else None,'latest_mpc_candidate':candidates[d],
                        'submitted_request':futures[d][1] if d in futures else None,'inference_result':pending[d],
                        'horizontal_distance_m':missions[d].horizontal_distance_m,'horizontal_limit_bypassed':missions[d].skip_horizontal_limit,
                        'control_source':missions[d].control_source,'committed_tick':missions[d].tick,'selected_history_committed':selected[d],'arrival_stalled':stalled}
                    records[d].append(row);outputs[d].write(json.dumps(row)+'\n')
                if all(m.done or d in errors for d,m in missions.items()):break
                state=plant.step_compiled(state,jnp.asarray(applied))
            for file in outputs.values():file.close()
            assessments={}
            for d in devices:
                rows=records[d];physical=np.asarray([r['state'] for r in rows]);rot=Rotation.from_quat(physical[:,[4,5,6,3]]).as_matrix()
                pitch=np.unwrap(np.arctan2(rot[:,0,2],rot[:,2,2]));turns=-(pitch[-1]-pitch[0])/(2*np.pi)*plan['direction']
                inverted=rot[:,2,2]<-.7;inversions=int(np.count_nonzero(inverted&~np.r_[False,inverted[:-1]]))
                moving=np.asarray([r['phase'] in ('run_in','pitch_entry','pitch_loops','pitch_exit','run_out') for r in rows])
                ref=np.asarray([r['reference']['position'] for r in rows]);error=physical[:,:3]-ref
                success=bool(d not in errors and missions[d].done and turns>plan['laps']-.15 and inversions==plan['laps'])
                assessments[str(d)]={'success':success,'failure':errors.get(d),'landed':missions[d].done,'pitch_turns':float(turns),'inversions':inversions,
                    'moving_rmse_m':float(np.sqrt(np.mean(np.sum(error[moving]**2,axis=1)))) if moving.any() else None,
                    'height_range_m':[float(physical[:,2].min()),float(physical[:,2].max())],'selected_ticks':missions[d].tick,'motion_epoch':missions[d].motion_epoch}
            episode_report={'seed':seed,'passed':all(a['success'] for a in assessments.values()) and clock.abort_reason is None,'drones':assessments,
                'group_epoch':clock.epoch,'abort_reason':clock.abort_reason,'minimum_measured_separation_m':minimum_distance,'events':clock.events,'arrival_stall_samples':stall_samples,'arrival_stall_effective_hold_s':stall_samples*.02,'arrival_stall_new_command_gap_s':(stall_samples+1)*.02 if stall_samples else 0.,'wall_s':time.monotonic()-started}
            report['episodes'].append(episode_report);report['passed']=all(e['passed'] for e in report['episodes'])
            (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(episode_report),flush=True)
    finally:
        pool.shutdown(wait=True)
        for worker in workers.values():worker.close()
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
