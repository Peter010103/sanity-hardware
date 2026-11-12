import argparse
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import ipaddress
import gc
import fcntl
import json
import os
from pathlib import Path
from sanity_zenoh.mission_clock import record_clock_preflight
import queue
import signal
import socket
import subprocess
import sys
import time
import uuid

import numpy as np
import yaml

from sanity_zenoh.fleet_config import FleetConfig
from sanity_zenoh.fleet_reference import FleetMission
from sanity_zenoh.fleet_supervisor import FleetSupervisor
from sanity_zenoh.recording import EventRecorder


class FleetCoordinator:
    def __init__(self, config, nodes, recorder, shadow=False):
        self.config, self.nodes, self.recorder = config, nodes, recorder
        self.shadow = shadow
        self.started = time.monotonic()
        self.pending = queue.SimpleQueue()
        self.acks = {}
        self.preflight_since = {}
        self.supervisor = FleetSupervisor(nodes,max(p.duration for p in config.plans.values()),config.safety,recorder)
        self.started_shadow = None
        self.error = None
        self.timing = None
        self.diagnostic_health = {}


    def reports(self):
        now = time.monotonic()
        reports = {}
        for device,node in self.nodes.items():
            status = node.sender.receiver_status()
            preflight, ready, fault = False,False,node.error
            try:
                state,age,packet = node.measured_state()
                initial = next(d for d in self.config.drones if d['id']==device)
                if node.stage == 'waiting' and np.linalg.norm(state[:2]-initial['home_xy'])>.10:
                    raise ValueError('Measured takeoff position differs from configured home by >0.10m')
                preflight = bool(status and status.session==node.sender.session_id and not status.failsafe
                    and status.fc_ready and status.fc_age_ms<150 and status.calibration_id==node.mapping.calibration_id
                    and node.sender.telemetry_time is not None and now-node.sender.telemetry_time<2.
                    and now-node.sender.diagnostic_times.get('trace',-float('inf'))<2.)
                if preflight:
                    self.preflight_since.setdefault(device,now)
                    preflight = now-self.preflight_since[device]>=1.
                else:self.preflight_since.pop(device,None)
                diagnostic_ok = bool(node.sender.telemetry_time and now-node.sender.telemetry_time<2.
                    and now-node.sender.diagnostic_times.get('trace',-float('inf'))<2.)
                if self.diagnostic_health.get(device) != diagnostic_ok:
                    self.diagnostic_health[device] = diagnostic_ok
                    self.recorder({'kind':'diagnostic_health','device':device,'fresh':diagnostic_ok})
                row = node.latest_control
                if row:
                    tilt = np.degrees(np.arccos(np.clip(1.-2.*(state[4]**2+state[5]**2),-1.,1.)))
                    ready = bool(preflight and row['policy_valid'] and row['policy_weight']==1.
                        and time.monotonic_ns()-row['control_ns']<40_000_000
                        and np.linalg.norm(state[:3]-np.asarray(row['reference']['position']))<node.values['handover_position_tolerance_m']
                        and np.linalg.norm(state[7:10])<.1 and tilt<5.
                        and node.supervisor.policy_start is not None and node.elapsed_s()>=node.supervisor.policy_start+2.)
                if not self.shadow and node.stage == 'running':
                    region = np.asarray(initial['region'])
                    if np.any(state[:3]<region[0]) or np.any(state[:3]>region[1]):
                        fault = fault or 'fleet_operating_region_exceeded'
                    if np.linalg.norm(state[7:10])>self.config.safety['max_speed_m_s']:
                        fault = fault or 'fleet_recovery_speed_exceeded'
                if node.landing_request and node.landing_request!='mission_complete':
                    fault = fault or node.landing_request
            except (ValueError,TypeError) as error:
                if node.stage == 'running':fault = fault or str(error)
            reports[device] = {'preflight':preflight,'ready':ready,'fault':fault,
                'done':node.finished.is_set(),'epoch_ack':self.acks.get(device)}
        return reports


    def apply(self, actions):
        for action in actions:
            if action['action']=='schedule':
                anchor_mono=time.monotonic_ns()
                anchor_unix=time.time_ns()
                self.timing={'epoch_monotonic_ns':int(action['epoch_monotonic_s']*1e9),
                    'estimated_epoch_unix_ns':anchor_unix+int(action['epoch_monotonic_s']*1e9)-anchor_mono,
                    'anchor_monotonic_ns':anchor_mono,'anchor_unix_ns':anchor_unix,
                    'status':'scheduled','scope':'Common reference epoch; receiver arrival and FC actuation are not synchronized'}
                self.recorder({'kind':'fleet_clock_anchor',**self.timing})
            if action['action']=='mission_started' and self.timing:self.timing['status']='started'
            if action['action']=='cancel_schedule' and self.timing:self.timing['status']='cancelled'
            if action['action'] in ('land','stop') and self.timing and self.timing['status']=='scheduled':self.timing['status']='cancelled'
            targets = list(self.nodes) if action['device'] is None else [action['device']]
            for device in targets:
                node = self.nodes[device]
                if action['action']=='start':
                    node.wait_started = time.monotonic()
                    node.fleet_admitted = True
                elif action['action']=='schedule':
                    node.reference.orbit_start = action['epoch_monotonic_s']-node.started_monotonic_ns/1e9
                    self.acks[device] = action['epoch_monotonic_s']
                    node.events({'kind':'fleet_clock_ack','epoch_monotonic_s':action['epoch_monotonic_s'],
                        'local_orbit_start_s':node.reference.orbit_start,'tick':node.tick,'clock':self.timing})
                elif action['action']=='cancel_schedule':
                    node.reference.orbit_start = None
                    self.acks.pop(device,None)
                elif action['action']=='land':
                    if node.stage=='running':node.request_landing(action['reason'])
                    else:node.stop(action['reason'])
                elif action['action']=='stop':
                    node.sender.invalidate(action['reason'])
                    node.stop(action['reason'])


    def tick(self):
        while not self.pending.empty():
            request = self.pending.get()
            action, device = request.get('action'),request.get('device')
            if device is not None and device not in self.nodes:
                self.recorder({'kind':'operator_request_rejected','request':request,'reason':'unknown device'})
                continue
            self.recorder({'kind':'operator_request','request':request})
            if action=='land':
                if device is None:self.supervisor.group_land('operator_group_landing')
                else:self.supervisor.abort(device)
            elif action=='emergency-stop':self.supervisor.emergency(device)
        reports = self.reports()
        if self.shadow:
            if self.started_shadow is None and all(v['preflight'] for v in reports.values()):
                self.started_shadow = time.monotonic()
                self.apply([{'action':'start','device':d} for d in self.nodes])
            if self.started_shadow and time.monotonic()-self.started_shadow>=self.config.mission['hover_s']:
                for node in self.nodes.values():node.stop()
                self.supervisor.state='finished'
            elif time.monotonic()-self.started>self.config.safety['startup_timeout_s']:
                self.error='shadow_startup_timeout'
                self.supervisor.emergency()
            if any(v['fault'] for v in reports.values()):self.supervisor.emergency()
            self.apply(self.supervisor.drain())
            return
        if self.recorder.error:self.supervisor.group_land('fleet_recording_failed')
        # A stop request is a state transition; it never edits a selected CTBR command.
        positions = []
        for device,node in self.nodes.items():
            # A waiting/landed neighbour also occupies its hover location.
            if node.stage=='running' or self.config.separation_mode=='stationary_hover':
                try:
                    state,_,_ = node.measured_state()
                    positions.append((device,state))
                except ValueError:pass
        for index,(a,sa) in enumerate(positions):
            for b,sb in positions[index+1:]:
                margin = 2*(self.config.safety['body_radius_m']+self.config.safety['tracking_margin_m'])
                margin += (np.linalg.norm(sa[7:10])+np.linalg.norm(sb[7:10]))*self.config.safety['timing_uncertainty_s']
                if np.linalg.norm(sa[:3]-sb[:3])<margin:self.supervisor.group_land('measured_separation_margin')
        if time.monotonic()-self.started>self.config.safety['maximum_run_s']:
            self.error='fleet_total_timeout'
            self.supervisor.emergency()
        self.apply(self.supervisor.update(time.monotonic(),reports))


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        while block:=stream.read(1024*1024):digest.update(block)
    return digest.hexdigest()


def assess(output, nodes, mode, group):
    summaries = {}
    first_ticks = {}
    for device,node in nodes.items():
        records = []
        history = []
        path = output/'drones'/f'{device:02d}'
        for line in (path/'events.jsonl').open():
            event=json.loads(line)
            if event['kind']=='control':records.append(event['record'])
        for line in (path/'history.jsonl').open():history.append(json.loads(line))
        selected = [r['history_event'] for r in records]
        committed = [r['selection'] for r in history]
        ticks = [r['tick'] for r in records]
        moving = [r for r in records if r['reference']['phase'] in ('fleet_hover','fleet_line','fleet_endpoint_hold')]
        if moving:first_ticks[device]=moving[0]['control_ns']
        errors=[]
        for row in moving:
            elapsed=row['time_s']-node.reference.orbit_start
            expected=node.circle.sample_world(elapsed)
            errors.append(np.linalg.norm(np.asarray(row['reference']['position'])-expected['position']))
        reference_error=float(max(errors,default=0.))
        inputs_recorded=bool(history) and all(('controller_inputs' if node.values['controller']=='mpc' else 'actor_inputs') in r['result'] for r in history)
        checks = {'control_ticks_recorded':bool(records),'controller_inputs_recorded':inputs_recorded,
            'common_world_reference_consistent':bool(reference_error<1e-8),
            'controller_no_error':node.error is None,'shutdown_disabled_acknowledged':node.sender.shutdown_acknowledged,
            'ticks_ordered':ticks==list(range(len(ticks))), 'selected_history_committed':selected==committed,
            'recording_complete':not node.events.error,'landed':node.landed if mode!='shadow' else True}
        data = {'passed':all(checks.values()),'checks':checks,'error':node.error,
            'ticks':len(records),'tracking_samples':len(moving),'maximum_reference_error_m':reference_error,'common_epoch_monotonic_s':group.supervisor.epoch,
            'position_rmse_3d_m':float(np.sqrt(np.mean([np.sum((np.asarray(r['state'][:3])-r['reference']['position'])**2) for r in moving]))) if moving else None,
            'velocity_rmse_3d_m_s':float(np.sqrt(np.mean([np.sum((np.asarray(r['state'][7:10])-r['reference']['velocity'])**2) for r in moving]))) if moving else None}
        (path/'summary.json').write_text(json.dumps(data,indent=2)+'\n')
        summaries[str(device)] = data
    completed = mode=='shadow' or (group.supervisor.reason=='mission_complete' and group.supervisor.epoch is not None)
    summary = {'passed':bool(summaries) and all(x['passed'] for x in summaries.values()) and group.error is None and completed,
        'nominal_mission_completed':completed,
        'mode':mode,'fleet_state':group.supervisor.state,'reason':group.supervisor.reason,'error':group.error,
        'common_epoch_monotonic_s':group.supervisor.epoch,'drones':summaries,
        'takeoff_mode':group.supervisor.takeoff_mode,
        'takeoff_control_start_skew_ms':(max(n.started_monotonic_ns for n in nodes.values())-min(n.started_monotonic_ns for n in nodes.values()))/1e6 if all(n.started_monotonic_ns for n in nodes.values()) else None,
        'takeoff_timing_scope':'Simultaneous mode releases controllers in one host supervision tick; individual arming acknowledgements and radio arrivals can differ.',
        'first_reference_tick_skew_ms':(max(first_ticks.values())-min(first_ticks.values()))/1e6 if len(first_ticks)==len(nodes) else None,
        'timing_scope':'Reference selection on one host, not receiver arrival or motor actuation synchronization',
        'scope':'Software emulation is a functional host/transport check; it does not qualify airframe physics or simultaneous radios.' if mode=='sim' else 'Recorded fleet experiment'}
    (output/'fleet_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def run(config, output, mode, frames_confirmed, recorder_ready, sim_fault=None):
    if mode not in ('sim','shadow','flight'):
        raise ValueError('Unknown fleet mode')
    if mode=='flight':
        config.require_hardware()
        if not frames_confirmed:raise ValueError('Physical fleet output requires confirmed frames')
    if mode=='sim' and any(not ipaddress.ip_address(value.split('/',1)[1].rsplit(':',1)[0]).is_loopback for value in config.transport.values()):
        raise ValueError('Software validation requires loopback listeners before opening any session')
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    from sanity_zenoh.ros_sequence import SequenceNode
    from sanity_zenoh.transport import SanityTransport
    rclpy.init(args=[],signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    nodes = {}
    control_executors, control_threads = [],[]
    transport = executor = driver = group = mocap_process = simulator = None
    recorder = EventRecorder(output/'fleet_events.jsonl',isolated=True)
    operator = socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    operator.bind(str(output/'control.sock'))
    os.chmod(output/'control.sock',0o600)
    operator.setblocking(False)
    try:
        transport = SanityTransport(**config.transport)
        for drone in config.drones:
            device = drone['id']
            parameters = deepcopy(config.parameters[device])
            parameters.update(output=str(output/'drones'/f'{device:02d}'),enable_output=mode!='shadow',
                frames_confirmed=frames_confirmed or mode=='sim',recorder_ready_file=str(recorder_ready),
                listen=config.transport['listen'],telemetry_listen=config.transport['telemetry_listen'])
            # Unknown ROS defaults are deliberately excluded from programmatic overrides.
            overrides = [Parameter(name,value=value) for name,value in parameters.items()]
            node = SequenceNode(node_name=f'sanity_fleet_{device:02d}',parameter_overrides=overrides,
                transport=transport,mission_factory=lambda v,d=drone:FleetMission(config.mission,d,v),subscribe_tf=False,isolated_io=True)
            nodes[device] = node
            print(f"Drone {device:02d}: {parameters['controller']} ready; output gated",flush=True)
        from sanity_zenoh.fleet_mocap import FleetMocapProcess
        mocap_process=FleetMocapProcess(config,nodes,output)
        for device,node in nodes.items():node.external_mocap=mocap_process.readers[device]
        recorder({'kind':'mocap_worker_ready',**mocap_process.info})
        group = FleetCoordinator(config,nodes,recorder,shadow=mode=='shadow')
        driver = rclpy.create_node('sanity_fleet')
        tf_group, supervision_group = MutuallyExclusiveCallbackGroup(),MutuallyExclusiveCallbackGroup()
        fault_fired=False
        def supervise():
            nonlocal fault_fired
            if sim_fault and not fault_fired and group.supervisor.state=='running' and time.monotonic()>=group.supervisor.epoch+sim_fault['at_s']:
                simulator.inject_fault(sim_fault['kind'],sim_fault['device'],sim_fault['stall_s'])
                fault_fired=True
                recorder({'kind':'simulated_fault',**sim_fault})
            while True:
                try:
                    request = json.loads(operator.recv(4096))
                    if not isinstance(request,dict) or set(request)-{'action','device'} or request.get('action') not in ('land','emergency-stop'):
                        raise ValueError('Invalid local operator request')
                    group.pending.put(request)
                except BlockingIOError:break
                except (ValueError,TypeError) as error:recorder({'kind':'invalid_operator_request','error':str(error)})
            if simulator and simulator.error:
                group.error=group.error or simulator.error
                group.supervisor.group_land('software_simulator_failed')
            if mocap_process.error:
                group.error=group.error or mocap_process.error
                group.supervisor.group_land('isolated_mocap_or_recording_failed')
            if not recorder_ready.is_file():group.supervisor.group_land('shared_mocap_recorder_lost')
            group.tick()
        driver.create_timer(.02,supervise,callback_group=supervision_group)
        if mode=='sim':
            from sanity_zenoh.fleet_simulator import FleetSimulator
            simulator=FleetSimulator(config,nodes,output)
            recorder({'kind':'software_simulator_ready',**simulator.info})
        gc.collect()
        gc.disable()
        executor = SingleThreadedExecutor()
        executor.add_node(driver)
        import threading
        for node in nodes.values():
            control_executor=SingleThreadedExecutor()
            control_executor.add_node(node)
            control_thread=threading.Thread(target=control_executor.spin,daemon=True,name=f"fleet_control_{node.values['drone_id']}")
            control_executors.append(control_executor)
            control_threads.append(control_thread)
            control_thread.start()
        interruptions=0
        def interrupt(signum,frame):
            nonlocal interruptions
            interruptions+=1
            group.pending.put({'action':'land' if signum==signal.SIGINT and interruptions==1 else 'emergency-stop'})
        signal.signal(signal.SIGINT,interrupt)
        signal.signal(signal.SIGTERM,interrupt)
        for node in nodes.values():
            recorder({'kind':'drone_worker_ready','device':node.values['drone_id'],'worker':node.worker.info})
        deadline=time.monotonic()+3.
        while rclpy.ok():
            executor.spin_once(timeout_sec=.005)
            if group.supervisor.state in ('finished','emergency') and (mode=='shadow' or all(n.finished.is_set() for n in nodes.values())):break
            if all(n.finished.is_set() for n in nodes.values()):
                group.supervisor.state='finished'
                break
            if time.monotonic()>deadline:
                print(' | '.join(f'{d:02d} {n.stage}/{(n.latest_control or {}).get("phase","waiting")}' for d,n in nodes.items())+' | fleet '+group.supervisor.state,flush=True)
                deadline=time.monotonic()+3.
    finally:
        if executor:executor.shutdown(timeout_sec=3.)
        for control_executor in control_executors:control_executor.shutdown(timeout_sec=3.)
        for control_thread in control_threads:control_thread.join(timeout=3.)
        for node in nodes.values():
            if not node.finished.is_set():node.stop('fleet_runner_shutdown')
            node.sender.invalidate('fleet_shutdown')
        with ThreadPoolExecutor(max_workers=max(1,len(nodes))) as pool:
            futures=[pool.submit(n.close) for n in nodes.values()]
            for future in futures:
                try:future.result()
                except Exception as error:recorder({'kind':'shutdown_error','error':str(error)})
        if mocap_process:mocap_process.close()
        if simulator:simulator.close()
        for node in nodes.values():node.destroy_node()
        if driver:driver.destroy_node()
        if transport:transport.close()
        operator.close()
        (output/'control.sock').unlink(missing_ok=True)
        recorder.close()
        if rclpy.ok():rclpy.shutdown()
        gc.enable()
    if group is None:raise RuntimeError('Fleet startup failed before coordinator creation')
    (output/'mission_timing.json').write_text(json.dumps(group.timing,indent=2)+'\n')
    summary=assess(output,nodes,mode,group)
    summary['mocap_process']=mocap_process.info if mocap_process else None
    summary['mocap_recording_complete']=bool(mocap_process and not mocap_process.error)
    summary['fleet_recording_complete']=not recorder.error
    summary['passed']=summary['passed'] and not recorder.error and summary['mocap_recording_complete']
    return summary


def main(argv=None):
    parser=argparse.ArgumentParser(description='Coordinated Sanity CTBR hover and line experiments')
    parser.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[6])
    parser.add_argument('--config',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--frames-confirmed',action='store_true')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--plan-only',action='store_true')
    modes.add_argument('--sim',action='store_true')
    modes.add_argument('--shadow',action='store_true')
    modes.add_argument('--fly',action='store_true')
    parser.add_argument('--control',type=Path)
    parser.add_argument('--action',choices=['land','emergency-stop'])
    parser.add_argument('--device',type=int)
    parser.add_argument('--sim-fault',choices=['receiver-restart','delivery-stall'])
    parser.add_argument('--fault-device',type=int,default=1)
    parser.add_argument('--fault-at',type=float,default=3.)
    parser.add_argument('--stall-s',type=float,default=.128)
    args=parser.parse_args(argv)
    if args.control:
        if not args.action:parser.error('--control requires --action')
        message={'action':args.action}
        if args.device is not None:message['device']=args.device
        with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as sock:
            sock.sendto(json.dumps(message).encode(),str(args.control.resolve()/'control.sock'))
        return
    if not args.config:parser.error('--config required')
    if args.sim_fault and not args.sim:parser.error('Fault injection is strictly --sim only')
    if not np.isfinite([args.fault_at,args.stall_s]).all() or args.fault_at<0 or not 0<args.stall_s<=2.:parser.error('Invalid simulation fault timing')
    config=FleetConfig(args.config,args.workspace)
    if args.sim_fault and args.fault_device not in config.plans:parser.error('Unknown fault device')
    if args.fly:
        config.require_hardware()
        if not args.frames_confirmed:parser.error('--fly requires --frames-confirmed for every configured aircraft')
    plan={'mission':config.mission,'transport':config.transport,'separation':config.separation,'takeoff_mode':config.takeoff_mode,
        'drones':{str(d):p.metadata() for d,p in config.plans.items()},
        'hardware_pending':[d['id'] for d in config.drones if not d['hardware_confirmed']]}
    if args.plan_only:
        print(json.dumps(plan,indent=2));return
    mode='sim' if args.sim else 'flight' if args.fly else 'shadow'
    output=(args.output or config.root/'outputs'/('sanity_fleet_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')+'_'+mode+'_'+uuid.uuid4().hex[:6])).resolve()
    if len(str(output/'control.sock').encode())>=104:parser.error('Output path too long for local Unix control socket')
    output.mkdir(parents=True,exist_ok=False)
    clock_report=None if mode=='sim' else record_clock_preflight(output,required=True)
    (output/'fleet_config.yaml').write_text(yaml.safe_dump(config.data,sort_keys=False))
    (output/'mission_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    sources=list(Path(__file__).parent.glob('*.py'))+[config.path,config.root/'Sanity/src/ros2_sanity_ws/scripts/run_sanity_fleet.sh']
    sources+=list((config.root/'loco_mjx/experiments/sanity_ctbr_sim2sim').glob('*.py'))
    sources.append(config.root/'loco_mjx/loco_mjx/environments/robots/sanity_ctbr/data/plane.xml')
    sources.append(config.root/'Sanity/src/ros2_sanity_ws/scripts/run_sanity_five_hover.sh')
    sources.append(config.root/'Sanity/src/ros2_sanity_ws/scripts/run_sanity_six_hover.sh')
    sources.append(config.root/'Sanity/src/ros2_sanity_ws/scripts/run_sanity_hover.sh')
    for drone in config.drones:
        sources.append(Path(drone['calibration']))
        if drone['sequence_config']:sources.append(Path(drone['sequence_config']))
        sources.extend(Path(v) for k,v in drone['controller'].items() if k!='type' and Path(v).is_file())
    hashes={}
    for path in set(sources):
        if not path.is_file():continue
        hashes[str(path)]=sha256(path)
        relative=path.relative_to(config.root) if path.is_relative_to(config.root) else Path('external')/path.name
        if path.suffix!='.model':
            target=output/'source'/relative
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(path.read_bytes())
    manifest={'mode':mode,'argv':sys.argv,'source_checkpoint_sha256':hashes,
        'clock_anchor':{'unix_ns':time.time_ns(),'monotonic_ns':time.monotonic_ns()},
        'dds_environment':{key:os.environ.get(key) for key in ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE')},
        'physical_output':args.fly,'firmware_flash':False,'clock_sync':clock_report}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    env=os.environ.copy()
    domain_lock=None
    if mode=='sim':
        # No wildcard/external listeners, discovery or physical endpoints in software validation.
        for key,kind in [('listen',socket.SOCK_DGRAM),('telemetry_listen',socket.SOCK_STREAM)]:
            with socket.socket(socket.AF_INET,kind) as sock:
                sock.bind(('127.0.0.1',0))
                config.transport[key]=('udp/' if kind==socket.SOCK_DGRAM else 'tcp/')+'127.0.0.1:'+str(sock.getsockname()[1])
        os.environ['ROS_LOCALHOST_ONLY']='1'
        for domain in range(94,180):
            candidate=open(f'/tmp/sanity_ros_sim_domain_{domain}.lock','a')
            try:
                fcntl.flock(candidate,fcntl.LOCK_EX|fcntl.LOCK_NB)
                domain_lock=candidate
                break
            except BlockingIOError:candidate.close()
        if domain_lock is None:raise RuntimeError('No isolated software-test DDS domain available')
        os.environ['ROS_DOMAIN_ID']=str(domain)
        os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
        for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):os.environ.pop(key,None)
        env=os.environ.copy()
    manifest['effective_dds_environment']={key:os.environ.get(key) for key in manifest['dds_environment']}
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'effective_transport.json').write_text(json.dumps(config.transport,indent=2)+'\n')
    baglog=(output/'rosbag.log').open('w')
    bag=None
    ready=output/'recorder.ready'
    print('Recording:',output,flush=True)
    try:
        bag=subprocess.Popen(['ros2','bag','record','--storage','mcap','--output',str(output/'mocap'),'/tf','/tf_static','/rosout'],
            stdout=baglog,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        deadline=time.monotonic()+15.
        while not list((output/'mocap').glob('*.mcap')):
            if bag.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Mocap recorder failed; see rosbag.log')
            time.sleep(.05)
        ready.touch()
        # Recorder liveness is continuously reflected in the readiness file.
        import threading
        watch_stop=threading.Event()
        def watch_recorder():
            while not watch_stop.wait(.1):
                if bag.poll() is not None:ready.unlink(missing_ok=True);return
        watcher=threading.Thread(target=watch_recorder,daemon=True)
        watcher.start()
        fault={'kind':args.sim_fault,'device':args.fault_device,'at_s':args.fault_at,'stall_s':args.stall_s} if args.sim_fault else None
        summary=run(config,output,mode,args.frames_confirmed,ready,fault)
    except BaseException as error:
        (output/'startup_error.json').write_text(json.dumps({'error':str(error),'unix_ns':time.time_ns()},indent=2)+'\n')
        raise
    finally:
        if 'watch_stop' in locals():watch_stop.set();watcher.join(timeout=1.)
        ready.unlink(missing_ok=True)
        if bag and bag.poll() is None:bag.send_signal(signal.SIGINT)
        if bag:bag.wait(timeout=20.)
        baglog.close()
        if domain_lock is not None:domain_lock.close()
    summary['rosbag_returncode']=bag.returncode
    summary['shared_mcap_finalized']=(output/'mocap/metadata.yaml').is_file() and bag.returncode==0
    summary['passed']=summary['passed'] and summary['shared_mcap_finalized']
    (output/'fleet_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2),flush=True)
    print('Results:',output/'fleet_summary.json',flush=True)
    if not summary['passed']:raise SystemExit(1)


if __name__=='__main__':main()
