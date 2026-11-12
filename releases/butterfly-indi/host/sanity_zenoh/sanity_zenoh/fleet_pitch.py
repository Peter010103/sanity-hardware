"""Discover Sanity bodies and prepare/test coordinated full-attitude MPC flips."""
import argparse
from copy import deepcopy
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import uuid
import numpy as np
import yaml
from sanity_zenoh.fleet_discovery import discover_frames,choose_frames,print_inventory,selected_configuration
from sanity_zenoh.fleet_config import FleetConfig
from sanity_zenoh.fleet_home_capture import capture_samples,place_at_captured_homes
from sanity_zenoh.fleet_pitch_core import create_missions,check_paths,simulation_homes,BODY_RADIUS_M,TRACKING_MARGIN_M,PATH_SAMPLING_ALLOWANCE_M


def build_plan(root,registry,drones,configuration,laps=1,bottom=.5,direction=1,minimum_path_separation_m=None,skip_swept_path_check=False,skip_prestart_readiness_check=False,skip_path_corridor_check=False,skip_horizontal_limit=False):
    if type(skip_prestart_readiness_check) is not bool:raise ValueError('skip_prestart_readiness_check must be a boolean')
    for key,value in (('skip_path_corridor_check',skip_path_corridor_check),('skip_horizontal_limit',skip_horizontal_limit)):
        if type(value) is not bool:raise ValueError(key+' must be a boolean')
    cfg=deepcopy(configuration)
    lease=float(cfg.get('command_lease_s',1.))
    if not np.isfinite(lease) or not 0<lease<=1.:raise ValueError('Pitch command lease must be in(0,1]seconds')
    if cfg.get('reference_mode')!='run_in':raise ValueError('Initial fleet flips require the tested run-in reference')
    if cfg.get('rate_prediction')!='flight':raise ValueError('Use the current flight-rate predictor for coordinated flips')
    if cfg.get('flight_rate_profile'):
        path=Path(cfg['flight_rate_profile']);cfg['flight_rate_profile']=str((path if path.is_absolute() else root/path).resolve())
    drones=deepcopy(drones)
    for drone in drones:
        calibration=Path(drone['calibration']);drone['calibration']=str((calibration if calibration.is_absolute() else root/calibration).resolve())
    missions=create_missions(root,cfg,drones,laps,bottom,direction,skip_horizontal_limit=skip_horizontal_limit)
    separation,_=check_paths(missions,{d['id']:d['home_xy'] for d in drones},minimum_path_separation_m=minimum_path_separation_m,skip_swept_path_check=skip_swept_path_check)
    return {'workspace':str(root),'drones':deepcopy(drones),'mpc_configuration':cfg,'laps':laps,'bottom_m':bottom,'direction':direction,
        'transport':deepcopy(registry['transport']),'mocap':deepcopy(registry['mocap']),
        'body_radius_m':BODY_RADIUS_M,'tracking_margin_m':TRACKING_MARGIN_M,'separation':separation,
        'minimum_path_separation_m':separation['minimum_path_separation_m'],'skip_swept_path_check':skip_swept_path_check,
        'skip_prestart_readiness_check':skip_prestart_readiness_check,
        'skip_path_corridor_check':skip_path_corridor_check,'skip_horizontal_limit':skip_horizontal_limit,
        'reference':{str(d):m.reference.info for d,m in missions.items()},
        'controller':'Full-attitude pitch acados MPC; not the upright hover MPC',
        'sequence':'Concurrent PD takeoff -> own run-in hover -> all-ready/common clock -> run-in/flip/exit -> recovered hover -> PD landing',
        'start_lead_s':.5,'frames':'Common mocap world; parallel X-Z pitch planes; no old single-aircraft tilt calibration copied',
        'failure':'A healthy airborne member finishes its planned maneuver/recovery before PD landing; no inverted PD fallback. Emergency disarm is immediate.',
        'limits_scope':'Existing25ms MPC/40ms state-result-command freshness/1s lease;9.5rad/s pitch requires SZC3 receiver capability',
        'high_rate_scope':'Small-rate flight fit extrapolated to flip rates; physical high-rate authority and group radio/power/visibility remain unqualified'}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[6])
    mode=parser.add_mutually_exclusive_group();mode.add_argument('--sim',action='store_true');mode.add_argument('--fly',action='store_true');mode.add_argument('--list',action='store_true')
    parser.add_argument('--frames-confirmed',action='store_true');parser.add_argument('--plan-only',action='store_true')
    parser.add_argument('--registry',type=Path);parser.add_argument('--pitch-config',type=Path)
    parser.add_argument('--drones',nargs='+');parser.add_argument('--discovery-s',type=float,default=3.)
    parser.add_argument('--laps',type=int,default=1);parser.add_argument('--bottom',type=float,default=.5);parser.add_argument('--direction',type=int,choices=(-1,1),default=1)
    parser.add_argument('--episodes',type=int,default=1);parser.add_argument('--seed',type=int,default=91);parser.add_argument('--voltage',type=float,default=7.7)
    parser.add_argument('--output-delay-ticks',type=int,default=1);parser.add_argument('--randomized',action='store_true');parser.add_argument('--random-actuator-lag',action='store_true');parser.add_argument('--random-action-delay',action='store_true')
    parser.add_argument('--stall-device',help='Simulation only:body whose command arrivals pause')
    parser.add_argument('--stall-ms',type=float,default=128.);parser.add_argument('--stall-at-s',type=float,default=1.55,help='After common motion epoch')
    parser.add_argument('--lane-spacing',type=float,default=1.2,help='Synthetic simulation lanes only; physical homes come from /tf')
    parser.add_argument('--minimum-path-separation',type=float,help='Metres: clearance between full swept paths (default 0.72, minimum 0.32). Lower values allow closer starting layouts with less tracking-error reserve; nominal runtime corridor is 0.20m')
    parser.add_argument('--skip-swept-path-check',action='store_true',help='Skip pairwise flip swept-path overlap rejection; record distances only. Other configured checks remain active')
    parser.add_argument('--skip-prestart-readiness-check',action='store_true',help='Bypass readiness-loss abort during the scheduled countdown; initial all-ready barrier and other configured checks remain active')
    parser.add_argument('--skip-path-corridor-check',action='store_true',help='Bypass group landing when an aircraft exceeds its 0.20m path corridor; record crossings')
    parser.add_argument('--skip-horizontal-limit',action='store_true',help='Bypass the 2m horizontal distance stop in PD, MPC and landing; altitude and other checks remain active')
    parser.add_argument('--output',type=Path);parser.add_argument('--control',type=Path);parser.add_argument('--action',choices=['land','emergency-stop']);parser.add_argument('--device',type=int)
    args=parser.parse_args(argv);root=args.workspace.resolve();pkg=root/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh'
    if args.control:
        from sanity_zenoh.fleet import main as operator
        if not args.action:parser.error('--control requires --action')
        command=['--workspace',str(root),'--control',str(args.control),'--action',args.action]
        if args.device is not None:command+=['--device',str(args.device)]
        return operator(command)
    if not 1<=args.laps<=10 or not 1<=args.episodes<=20 or not 0<=args.output_delay_ticks<=2 or not 1<=args.discovery_s<=30 or not np.isfinite([args.bottom,args.voltage,args.lane_spacing,args.discovery_s]).all() or not .2<=args.bottom<1. or not 6.7<=args.voltage<=8.2 or args.lane_spacing<=0:
        parser.error('Invalid maneuver/simulation parameters')
    if args.minimum_path_separation is not None and (not np.isfinite(args.minimum_path_separation) or args.minimum_path_separation<2*BODY_RADIUS_M+PATH_SAMPLING_ALLOWANCE_M):
        parser.error('--minimum-path-separation must be finite and at least 0.32m')
    if args.stall_device and not args.sim:parser.error('--stall-device is simulation only')
    if not np.isfinite([args.stall_ms,args.stall_at_s]).all() or not 0<args.stall_ms<=1000 or args.stall_at_s<0:parser.error('Invalid simulated stall')
    if args.fly and not args.frames_confirmed:parser.error('--fly requires aligned body frames and --frames-confirmed')
    path=(args.registry or pkg/'config/fleet_hover_registry.yaml').resolve();registry=yaml.safe_load(path.read_text())
    registered=[d['tracked_frame'] for d in registry['drones']]
    with tempfile.TemporaryDirectory(prefix='sanity_pitch_registry_') as temporary:
        file=Path(temporary)/'registry.yaml';file.write_text(yaml.safe_dump(selected_configuration(registry,registered)))
        FleetConfig(file,root)
    if args.sim:
        # Explicit membership is reproducible; otherwise reuse visible TF bodies when available.
        detected=discover_frames(args.discovery_s,registry['mocap']['topic']) if args.drones is None else {}
        membership_source='Live TF membership; synthetic separated lanes, not captured arena positions'
        if args.drones is not None or not detected:
            detected={name:{'source':'registered simulation membership'} for name in registered}
            membership_source='Requested/registered aircraft (no live TF required); synthetic separated lanes, not current arena layout'
        chosen=choose_frames(registry,detected,args.drones)
    else:
        detected=discover_frames(args.discovery_s,registry['mocap']['topic']);chosen=choose_frames(registry,detected,args.drones)
        membership_source='Frozen live /tf discovery'
    print_inventory(registry,detected,chosen)
    if not args.sim and not args.fly and not args.plan_only:return
    if args.stall_device and args.stall_device not in chosen:parser.error('Stall body must be selected')
    if not chosen:raise ValueError('No registered commissioned Sanity bodies selected')
    data=selected_configuration(registry,chosen);capture=None
    if args.sim:
        homes=simulation_homes(chosen,args.lane_spacing)
        for drone in data['drones']:drone['home_xy']=homes[drone['tracked_frame']]
    else:
        samples,now=capture_samples(data);data,details=place_at_captured_homes(data,samples,now)
        capture={'captured_unix_ns':now,'drones':details,'samples':samples}
    configuration_path=(args.pitch_config or pkg/'config/sanity09_pitch_loop.json').resolve()
    plan=build_plan(root,registry,data['drones'],json.loads(configuration_path.read_text()),args.laps,args.bottom,args.direction,minimum_path_separation_m=args.minimum_path_separation,skip_swept_path_check=args.skip_swept_path_check,skip_prestart_readiness_check=args.skip_prestart_readiness_check,skip_path_corridor_check=args.skip_path_corridor_check,skip_horizontal_limit=args.skip_horizontal_limit)
    plan.update(membership_source=membership_source,detected=detected,selected_frames=chosen,voltage_V=args.voltage,episodes=args.episodes,seed=args.seed,
        output_delay_ticks=args.output_delay_ticks,randomized=args.randomized,random_actuator_lag=args.random_actuator_lag,random_action_delay=args.random_action_delay,
        physical_output=args.fly,registry_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    if args.stall_device:
        plan['arrival_stall']={'device':next(d['id'] for d in data['drones'] if d['tracked_frame']==args.stall_device),'duration_s':args.stall_ms/1000.,'start_after_common_s':args.stall_at_s}
    if args.plan_only:print(json.dumps(plan,indent=2));return
    suffix=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    output=(args.output or root/'outputs'/('sanity_flips_'+suffix+('_sim' if args.sim else '_flight'))).resolve()
    print('Recording:',output,flush=True)
    print('Concurrent takeoff -> shared run-in/flip clock -> MPC recovery -> individual PD landing',flush=True)
    if plan['skip_swept_path_check']:
        print('Swept-path overlap gate SKIPPED; pairwise distances recorded only. Other configured checks remain active.',flush=True)
    else:
        print(f"Full swept-path clearance: >{plan['minimum_path_separation_m']:.2f}m; nominal path corridor: {plan['tracking_margin_m']:.2f}m per drone",flush=True)
        if not plan['separation']['full_tracking_tubes_disjoint']:
            print('Reduced clearance selected: reserved tracking margins are smaller; runtime tracking bands can overlap.',flush=True)
    if plan['skip_prestart_readiness_check']:
        print('Countdown readiness-loss abort BYPASSED; initial all-ready barrier and other configured checks remain active.',flush=True)
    if plan['skip_path_corridor_check']:print('Path-corridor landing trigger BYPASSED; crossings will be logged.',flush=True)
    if plan['skip_horizontal_limit']:print('2m horizontal distance stop BYPASSED in PD/MPC/landing; altitude and freshness checks remain active.',flush=True)
    if args.sim:
        setup=root/'outputs'/('sanity_flips_plan_'+suffix);setup.mkdir(parents=True,exist_ok=False)
        file=setup/'plan.json';file.write_text(json.dumps(plan,indent=2)+'\n')
        environment={**os.environ,'JAX_PLATFORMS':'cuda','XLA_PYTHON_CLIENT_PREALLOCATE':'false'}
        result=subprocess.run([str(root/'.venv-sanity-train/bin/python'),'-u','-m','sanity_zenoh.fleet_pitch_sim','--plan',str(file),'--output',str(output)],env=environment,cwd=root)
        raise SystemExit(result.returncode)
    if len(str(output/'control.sock').encode())>=104:parser.error('Output path too long for local operator socket')
    output.mkdir(parents=True,exist_ok=False)
    from sanity_zenoh.mission_clock import record_clock_preflight
    clock=record_clock_preflight(output,required=True)
    (output/'mission_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    (output/'home_capture.json').write_text(json.dumps(capture,indent=2)+'\n');shutil.copyfile(path,output/'registry.yaml')
    sources=[*Path(__file__).parent.glob('*.py'),*Path(root/'loco_mjx/experiments/sanity_pitch_mpc').glob('*.py'),
        root/'loco_mjx/experiments/sanity_pitch_mpc/run_in_profile.json',configuration_path,root/'Sanity/src/ros2_sanity_ws/scripts/run_sanity_flips.sh']
    if plan['mpc_configuration'].get('flight_rate_profile'):sources.append(Path(plan['mpc_configuration']['flight_rate_profile']))
    sources.extend(Path(d['calibration']) if Path(d['calibration']).is_absolute() else root/d['calibration'] for d in data['drones'])
    hashes={}
    for source in sources:
        target=output/'source'/(source.relative_to(root) if source.is_relative_to(root) else Path('external')/source.name);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(source.read_bytes());hashes[str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
    (output/'manifest.json').write_text(json.dumps({'clock_sync':clock,'source_sha256':hashes,'unix_ns':time.time_ns(),'physical_output':True},indent=2)+'\n')
    from sanity_zenoh.fleet_pitch_runtime import run
    result=run(root,plan,output);print(json.dumps(result,indent=2),flush=True)
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
