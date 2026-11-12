"""Collect one PID/INDI diagnostic or matched MPC benchmark; refresh ground before flight."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import os
import re
import signal
import socket
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'loco_mjx').is_dir())
WS=ROOT/'Sanity/src/ros2_sanity_ws'
PACKAGE=Path(__file__).resolve().parent/'host/sanity_zenoh'
sys.path.insert(0,str(PACKAGE))
sys.path.insert(0,str(WS/'src/udp_broadcaster'))
from sanity_zenoh.standalone_paths import load_json
RUNNER=Path(__file__).resolve().parent/'run_high_speed_hover.sh'


def write_json(path,value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def lab_process_inventory():
    """Read-only room inventory; ownership is never inferred from a process name."""
    pattern=re.compile(r'ros2|rosbag|zenoh|natnet|mocap|pose.*bridge|sanity.*(?:hover|mpc|controller)|(?:sensor|preview).*server',re.I)
    rows=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name)==os.getpid():continue
        try:
            command=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace').strip()
            if command and pattern.search(command):
                rows.append(dict(pid=int(entry.name),pgid=os.getpgid(int(entry.name)),uid=entry.stat().st_uid,
                    command=command,ownership='pre-existing; preserved'))
        except (FileNotFoundError,ProcessLookupError,PermissionError):pass
    return dict(host=socket.gethostname(),created_unix=time.time(),processes=rows,
        scope='Read-only pre-run inventory. Existing processes are preserved; only children started by this invocation are cleaned up automatically.')


def snapshot_sources(output,config):
    """Keep the actual controller source and common model for later tuning comparisons."""
    names=['indi_hover.py','indi_hover_mission.py','full_indi_mission.py','indi_lemniscate.py',
        'indi_mpc_benchmark.py','mpc_benchmark_config.py','async_policy.py','local_policy.py',
        'emergency_landing.py','emergency_braking_mission.py',
        'mpc_process.py','command_prediction.py','mocap_diagnostics.py','firmware_collective.py','client.py','protocol.py','high_speed_lemniscate.py','mocap.py','receiver_health.py','ideal_command_prediction.py','emergency_landing.py','emergency_braking_mission.py']
    paths=[PACKAGE/'sanity_zenoh'/name for name in names]
    diagnostic_source=PACKAGE/'sanity_zenoh/indi_diagnostics.py'
    if diagnostic_source.exists():paths.append(diagnostic_source)
    # The named recipe runner selects this isolated launcher. Clock
    # helpers are shared by the standalone checkout.
    paths += [Path(__file__),RUNNER,WS/'scripts/sync_sanity_clock.py',WS/'scripts/sanity_clock_preflight.sh']
    paths += [Path(__file__).resolve().parent/name for name in ('mpc_controller.py','indi_rate_model.py','mpc_disturbance.py')]
    trajectory=config.get('trajectory') or {}
    loco=ROOT/'loco_mjx/experiments/sanity_ctbr_sim2sim'
    paths += [loco/name for name in ['takeoff_controller.py','missions.py','mpc_controller.py','mpc_disturbance.py','indi_rate_model.py']]
    settings=config.get('mpc') or {}
    paths += [Path(settings.get('config',PACKAGE/'config/sanity09_mpc.json')),
        Path(settings.get('model',ROOT/'loco_mjx/loco_mjx/environments/robots/sanity_ctbr/data/plane.xml')),
        Path(config['collective_profile'])]
    paths += [p for p in [ROOT/'firmware/fc/source.json',ROOT/'firmware/fc/recommended.cli',ROOT/'scripts/run_mpc.py'] if p.is_file()]
    manifest={}
    for path in dict.fromkeys(paths):
        relative=path.resolve().relative_to(ROOT)
        data=path.read_bytes();target=output/'source'/relative
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
        manifest[str(relative)]=hashlib.sha256(data).hexdigest()
    write_json(output/'source_manifest.json',dict(sha256=manifest,
        deployment_record_scope='Copied historical deployment receipt; actual per-run controller status is logged separately.'))


def refreshed_ground(events,expected_frame,now_ns=None):
    """Use only the final two seconds of ready, stationary, disarmed-bench pose."""
    now_ns=time.monotonic_ns() if now_ns is None else now_ns
    samples=[]
    with Path(events).open() as stream:
        for line in stream:
            item=json.loads(line)
            if item.get('kind')=='mocap_original_source':samples.append(item)
    if not samples:raise ValueError('No mocap poses recorded during the disabled bench check')
    last=samples[-1]['host_monotonic_ns']
    if not 0<=(now_ns-last)/1e9<=5.:
        raise ValueError('The last bench pose is stale; keep mocap running and retry')
    samples=[x for x in samples if x['host_monotonic_ns']>=last-2_000_000_000]
    if len(samples)<20 or any(not x['source_packet'].get('ready') or not x['source_packet'].get('frames_confirmed')
        or x['source_packet'].get('tracked_frame')!=expected_frame for x in samples):
        raise ValueError('Fresh, confirmed poses from the selected aircraft are required')
    stamps=[x['host_monotonic_ns'] for x in samples]
    gaps=np.diff(stamps)*1e-9
    if stamps[-1]-stamps[0]<1_500_000_000 or np.any(gaps<=0) or max(gaps)>.15:
        raise ValueError('A continuous, ordered recent mocap window is required')
    state=np.asarray([x['source_packet']['state'] for x in samples],dtype=float)
    if state.shape!=(len(samples),13) or not np.isfinite(state).all():raise ValueError('Invalid mocap state')
    norms=np.linalg.norm(state[:,3:7],axis=1)
    if np.any(abs(norms-1.)>.001):raise ValueError('Invalid mocap orientation')
    tilt=np.degrees(np.arccos(np.clip(Rotation.from_quat(state[:,[4,5,6,3]]).as_matrix()[:,2,2],-1.,1.)))
    speed=np.linalg.norm(state[:,7:10],axis=1)
    span=np.ptp(state[:,:3],axis=0)
    if max(tilt)>10. or np.median(speed)>.1 or max(span)>.01:
        raise ValueError('Place Sanity15 level and stationary on its takeoff surface, then retry')
    return dict(ground_m=float(np.median(state[:,2])),samples=len(samples),
        maximum_tilt_deg=float(max(tilt)),median_speed_m_s=float(np.median(speed)),
        position_span_m=span.tolist(),median_xyz_m=np.median(state[:,:3],axis=0).tolist(),
        last_pose_age_s=(now_ns-last)/1e9,tracked_frame=expected_frame)


class OwnedCommands:
    def __init__(self,output):
        self.output=output;self.rows=[];self.active=None;self.cancelled=False
        self.previous={number:signal.signal(number,self.interrupt) for number in (signal.SIGINT,signal.SIGTERM)}

    def interrupt(self,number,_frame):
        self.cancelled=True
        if self.active is not None and self.active.poll() is None:
            os.killpg(self.active.pid,number)
        else:raise KeyboardInterrupt

    def save(self):write_json(self.output/'process_ledger.json',self.rows)

    @staticmethod
    def members(group):
        result=[]
        for path in Path('/proc').iterdir():
            if path.name.isdigit():
                try:
                    if os.getpgid(int(path.name))==group:result.append(int(path.name))
                except ProcessLookupError:pass
        return result

    def run(self,name,command,timeout):
        if self.cancelled:raise KeyboardInterrupt
        with (self.output/(name+'.log')).open('x') as log:
            child=subprocess.Popen([str(x) for x in command],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            self.active=child
            row=dict(name=name,host=socket.gethostname(),pid=child.pid,pgid=child.pid,
                command=[str(x) for x in command],started_unix=time.time());self.rows.append(row)
            timed_out=False
            try:
                self.save()
                try:code=child.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    timed_out=True;os.killpg(child.pid,signal.SIGINT)
                    try:code=child.wait(timeout=45)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGTERM)
                        try:code=child.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(child.pid,signal.SIGKILL);code=child.wait()
            finally:
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGINT)
                    try:child.wait(timeout=45)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGTERM)
                        try:child.wait(timeout=5)
                        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                remaining=self.members(child.pid)
                if remaining:
                    os.killpg(child.pid,signal.SIGTERM)
                    end=time.monotonic()+3.
                    while remaining and time.monotonic()<end:
                        time.sleep(.05);remaining=self.members(child.pid)
                    if remaining:
                        os.killpg(child.pid,signal.SIGKILL);time.sleep(.1);remaining=self.members(child.pid)
                row.update(returncode=child.returncode,timed_out=timed_out,remaining_group_pids=remaining,
                    ended_unix=time.time())
                try:self.save()
                finally:self.active=None
            if self.cancelled:raise KeyboardInterrupt
            if timed_out or code or remaining:
                raise RuntimeError(f'{name} failed; inspect {self.output/(name+".log")}')

    def close(self):
        for number,handler in self.previous.items():signal.signal(number,handler)


def make_config(args,ground):
    base=load_json(getattr(args,'base_config',None) or PACKAGE/'config/sanity15_indi_mpc_constant_arclength.json')
    if ground is None:ground=base['limits']['ground_m']
    if getattr(args,'test','trajectory')!='trajectory':
        from sanity_zenoh.indi_diagnostics import build_diagnostic_config
        profile=getattr(args,'sine_profile',None) or ('extended' if args.test.endswith('-sine') else 'baseline')
        return build_diagnostic_config(base,inner=args.inner,test=args.test,ground_m=ground,sine_profile=profile)
    from sanity_zenoh.mpc_benchmark_config import build_config
    config=build_config(base,pattern=args.pattern,inner=args.inner,speed=args.speed,schedule=args.schedule,radius_m=args.radius_m,ground_m=ground,width_m=getattr(args,"width_m",None),butterfly_height_m=getattr(args,"butterfly_height_m",0.),butterfly_cycles=getattr(args,"butterfly_cycles",3))
    from sanity_zenoh.indi_hover import make_mission
    mission=make_mission(config,ROOT/'loco_mjx')
    config['emergency_braking_configuration']=dict(rate_limit_rad_s=mission.emergency_braking_rate_rad_s,ordinary_PD_rate_limit_rad_s=mission.limits.maximum_rate_rad_s,
        scope='Explicit matched native PID butterfly branch: emergency2rad/s after admission; ordinaryPD0.6rad/s')
    return config


def create_run_directory(args,config):
    if args.output is not None:
        output=args.output.resolve()
        output.mkdir(parents=True,exist_ok=False)
        return output
    if args.test=='trajectory':
        schedule=config['trajectory']['schedule'].replace('_','-')
        case=f'{args.pattern}_{args.inner.upper()}_{args.speed:g}mps_{schedule}'
        if args.radius_m is not None:case+=f'_R{config["trajectory"]["radius_m"]:g}m'
        if getattr(args,"width_m",None) is not None:case+=f'_W{config["trajectory"]["width_m"]:g}m'
        if getattr(args,"butterfly_height_m",0.):case+=f'_butterflyH{args.butterfly_height_m:g}m_C{args.butterfly_cycles}'
    else:
        case=f'{args.test}_{args.inner.upper()}'
    if args.bench_check:case+='_bench-check'
    stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%d_%H-%M-%SZ')
    stem=f'sanity15_{case}_{stamp}'
    number=1
    while True:
        suffix='' if number==1 else f'_{number:02d}'
        output=(ROOT/'outputs'/f'{stem}{suffix}').resolve()
        try:
            output.mkdir(parents=True,exist_ok=False)
            return output
        except FileExistsError:
            number+=1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test',choices=['trajectory','pd-hover','roll-pulse','pitch-pulse','yaw-pulse','roll-sine','pitch-sine','mpc-hover'],default='trajectory')
    parser.add_argument('--pattern',choices=['lemniscate','circle'])
    parser.add_argument('--inner',choices=['pid','indi'],required=True)
    parser.add_argument('--sine-profile',choices=['baseline','extended'],help='Sines default to extended:0.30rad/s at0.5,1,2,3,4Hz; baseline retains0.15rad/s at0.5,1,2Hz')
    parser.add_argument('--speed',type=float,choices=[.8,1.,1.5,3.,4.,5.,6.])
    parser.add_argument('--schedule',choices=['constant_arclength','variable_angular','indi_shaped_angular'])
    parser.add_argument('--butterfly-height-m',type=float,default=0.,help='Inclined plane Z amplitude; zero keeps planar reference')
    parser.add_argument('--butterfly-cycles',type=int,default=3,help='Each cycle is A then B: two complete eight laps')
    parser.add_argument('--width-m',type=float,help='Full Gerono Y extent; default equals radius; angular lemniscates only')
    parser.add_argument('--radius-m',type=float,help='Lemniscate: 2 (4 x 2 m) or 3.2 (historical 6.4 x 3.2 m); circle: 1.5')
    parser.add_argument('--base-config',type=Path)
    parser.add_argument('--frames-confirmed',action='store_true')
    parser.add_argument('--output',type=Path)
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--plan-only',action='store_true')
    modes.add_argument('--bench-check',action='store_true')
    modes.add_argument('--fly',action='store_true')
    args=parser.parse_args()
    if args.sine_profile is not None and not args.test.endswith('-sine'):
        parser.error('--sine-profile applies only to roll-sine or pitch-sine')
    if args.test!='trajectory' and args.butterfly_height_m:parser.error('--butterfly-height-m applies only to trajectory')
    if args.test!='trajectory' and any(value is not None for value in (args.pattern,args.speed,args.schedule,args.radius_m,args.width_m)):
        parser.error('--pattern, --speed, --schedule, --radius-m and --width-m apply only to --test trajectory')
    if args.test=='trajectory':
        args.pattern=args.pattern or 'lemniscate'
        args.speed=.8 if args.speed is None else args.speed
    live=args.fly or args.bench_check
    if live and not args.frames_confirmed:parser.error('Live checks require --frames-confirmed')
    try:config=make_config(args,None)
    except ValueError as exc:parser.error(str(exc))
    plan=dict(test=args.test,pattern=args.pattern,inner_controller=args.inner,speed_m_s=args.speed,
        trajectory=config.get('trajectory') if args.test=='trajectory' else None,diagnostic=config.get('diagnostic'),
        outer_controller='MPC' if args.test in ('trajectory','mpc-hover') else 'PD hover with logged additive excitation',
        limits=config['limits'],benchmark_envelope=config.get('benchmark_envelope') if args.test in ('trajectory','mpc-hover') else None,
        mpc=config.get('mpc') if args.test in ('trajectory','mpc-hover') else None,
        emergency_braking=config.get('emergency_braking_configuration'),
        high_speed_preparation=config.get('high_speed_preparation'),
        ground_scope='Refreshed from disabled bench immediately before live flight',
        network_or_hardware_access=live)
    if not live:
        print(json.dumps(plan,indent=2))
        if args.output:
            output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
            write_json(output/'plan.json',plan);write_json(output/'configuration.json',config)
        return 0
    preparation=config.get('high_speed_preparation')
    if preparation and not preparation.get('selected_release_supported',False):
        violations='; '.join(preparation['reference_screen']['ordinary_envelope_violations'])
        parser.error('Requested R2 reference exceeds installed ordinary command/envelope limits: '+violations+
            '. Use the prepared isolated high-speed release after matched receiver/envelope commissioning; --plan-only preserves the actual requested peak.')
    output=create_run_directory(args,config)
    write_json(output/'requested_plan.json',plan)
    snapshot_sources(output,config)
    provisional=output/'bench_configuration.json';write_json(provisional,config)
    summary=dict(success=False,output=str(output),test=args.test,pattern=args.pattern,inner_controller=args.inner,
        speed_m_s=args.speed,flight_requested=args.fly,flight_started=False,host=socket.gethostname(),
        launcher_pid=os.getpid(),started_unix=time.time())
    commands=OwnedCommands(output)
    print('Output:',output,flush=True)
    try:
        inventory=lab_process_inventory();write_json(output/'lab_process_inventory.json',inventory)
        if inventory['processes']:
            print('Existing lab processes recorded in',output/'lab_process_inventory.json',flush=True)
        print('Checking the mocap clock (three fresh samples, normally ~0.2 s).',flush=True)
        commands.run('clock_preflight',['/usr/bin/python3',WS/'scripts/sync_sanity_clock.py','--wait','45','--receipt',output/'clock_preflight.json'],65)
        print('Checking disarmed FC/receiver, recording stream and stationary mocap (normally ~3–4 s).',flush=True)
        commands.run('disabled_bench',['bash',RUNNER,'--config',provisional,'--bench-check','--frames-confirmed',
            '--minimum-bench-seconds','3','--output',output/'bench'],240)
        bench=json.loads((output/'bench/result.json').read_text())
        if not bench.get('success') or not bench.get('disabled_only') or not bench.get('shutdown_acknowledged'):
            raise RuntimeError('Disabled bench and shutdown acknowledgement did not pass')
        ground=refreshed_ground(output/'bench/events.jsonl',config['mocap']['tracked_frame'])
        write_json(output/'ground_reference.json',ground)
        config=make_config(args,ground['ground_m'])
        actual=output/'configuration.json';write_json(actual,config)
        summary.update(ground_reference=ground,configuration_sha256=hashlib.sha256(actual.read_bytes()).hexdigest())
        if args.bench_check:
            summary.update(success=True,reason='disabled_bench_complete')
            print('PASS: disarmed bench complete; fresh-ground configuration saved.',flush=True)
        else:
            if args.test=='trajectory':
                print(f'Starting {args.inner.upper()} inner / MPC outer, {args.pattern}, R={config["trajectory"]["radius_m"]:g} m, width={config["trajectory"].get("width_m",config["trajectory"]["radius_m"]):g} m, {args.speed:g} m/s, {config["trajectory"]["laps"]} eight laps.',flush=True)
            else:print(f'Starting {args.test}: {args.inner.upper()} inner / {plan["outer_controller"]}.',flush=True)
            print('Fresh-stream checks before arming; recording-only profiles normally need ~3–4 s, then the ground spool-up.',flush=True)
            summary['flight_started']=True;write_json(output/'run_summary.json',summary)
            commands.run('flight',['bash',RUNNER,'--config',actual,'--fly','--frames-confirmed','--output',output/'flight'],
                float(config['limits']['maximum_run_s'])+240.)
            result=json.loads((output/'flight/result.json').read_text())
            summary.update(success=bool(result.get('success')),reason=result.get('reason'),flight_result=result)
    except (Exception,KeyboardInterrupt) as exc:
        summary['error']=str(exc) or type(exc).__name__
        result=output/'flight/result.json'
        if result.exists():summary['flight_result']=json.loads(result.read_text())
        print('Stopped:',summary['error'],file=sys.stderr,flush=True)
    finally:
        commands.close()
        summary.update(completed_unix=time.time(),all_owned_child_groups_exited=all(not x.get('remaining_group_pids') for x in commands.rows))
        write_json(output/'run_summary.json',summary)
    print('Result:',output/'run_summary.json',flush=True)
    return 0 if summary['success'] and summary['all_owned_child_groups_exited'] else 1


if __name__=='__main__':raise SystemExit(main())
