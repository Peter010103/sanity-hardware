"""Experimental ROS-mocap/shared-Zenoh runtime for coordinated full-attitude MPC."""
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from copy import deepcopy
import gc
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
from types import SimpleNamespace
import numpy as np
from sanity_zenoh.fleet_pitch_core import PitchFleetClock,create_missions,check_paths
from sanity_zenoh.pitch_mission import PitchWorker
from sanity_zenoh.recording import EventRecorder


def run(root,plan,output):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sanity_zenoh.client import SanitySender
    from sanity_zenoh.transport import SanityTransport
    from sanity_zenoh.firmware_collective import FirmwareCollective
    from sanity_zenoh.mocap import state_age
    from sanity_zenoh.fleet_mocap import FleetMocapProcess
    import yaml
    drones=plan['drones'];configuration=plan['mpc_configuration'];lease_s=float(configuration.get('command_lease_s',1.))
    missions=create_missions(root,configuration,drones,plan['laps'],plan['bottom_m'],plan['direction'],skip_horizontal_limit=plan.get('skip_horizontal_limit',False))
    homes={d['id']:d['home_xy'] for d in drones}
    separation,trees=check_paths(missions,homes,plan['body_radius_m'],plan['tracking_margin_m'],minimum_path_separation_m=plan.get('minimum_path_separation_m'),skip_swept_path_check=plan.get('skip_swept_path_check',False))
    clock=PitchFleetClock(missions,skip_prestart_readiness_check=plan.get('skip_prestart_readiness_check',False),skip_path_corridor_check=plan.get('skip_path_corridor_check',False));nodes={};executors=[];threads=[];mocap=None;transport=None;bag=None
    events=EventRecorder(output/'fleet_events.jsonl',isolated=True)
    operator=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);operator.bind(str(output/'control.sock'));operator.setblocking(False)
    os.chmod(output/'control.sock',0o600)
    rclpy.init(args=[],signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    driver=rclpy.create_node('sanity_fleet_pitch');driver_executor=SingleThreadedExecutor();driver_executor.add_node(driver)
    baglog=(output/'rosbag.log').open('w');state={'phase':'startup','since':time.monotonic(),'preflight_since':None,'error':None,'arming_since':None}
    base=yaml.safe_load((root/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/circle_sequence.yaml').read_text())['sanity_sequence']['ros__parameters']

    class PitchNode(Node):
        def __init__(self,drone):
            super().__init__(f"sanity_pitch_{drone['id']:02d}")
            self.drone=drone;self.device=drone['id'];self.mission=missions[self.device];self.lock=threading.RLock()
            self.output=output/'drones'/f'{self.device:02d}';self.output.mkdir(parents=True)
            self.events=EventRecorder(self.output/'events.jsonl',isolated=True)
            self.history=EventRecorder(self.output/'history.jsonl',isolated=True,stamp=False)
            self.values={**base,'tracked_frame':drone['tracked_frame'],'parent_frame':plan['mocap']['parent_frame'],
                'world_from_map_xyzw':plan['mocap']['world_from_map_xyzw'],'marker_to_base_xyz':drone['marker_to_base_xyz'],
                'marker_to_base_xyzw':drone['marker_to_base_xyzw'],'pitch_pose_filter_enabled':True,'frames_confirmed':True,'velocity_filter_tau_s':.02,
                'angular_rate_filter_tau_s':configuration.get('angular_rate_filter_tau_s',.05),
                'position_filter_tau_s':configuration.get('position_filter_tau_s',.01),'attitude_filter_tau_s':configuration.get('attitude_filter_tau_s',.01)}
            self.worker=None;self.external_mocap=None;self.stage='waiting';self.finished=False;self.error=None;self.slot=None;self.last_control=None
            try:
                self.mapping=FirmwareCollective(drone['calibration'],high_rate=True)
                self.sender=SanitySender(device=self.device,listen=plan['transport']['listen'],telemetry_listen=plan['transport']['telemetry_listen'],
                    diagnostics=True,ctbr_mapping=self.mapping,physical_units=True,on_event=self.events,transport=transport)
            except BaseException:
                self.events.close();self.history.close();self.destroy_node();raise
            self.create_timer(.01,self.transmit);self.create_timer(.02,self.control)

        def measured_state(self):
            if self.external_mocap is None:raise ValueError('Waiting for isolated mocap')
            packet=self.external_mocap.snapshot()
            if packet.get('ready') and packet.get('tracked_frame')!=self.drone['tracked_frame']:raise ValueError('Mocap body routing mismatch')
            value,age=state_age(packet,time.monotonic_ns())
            return value,age,packet

        def feedback_ready(self,disabled=False):
            status=self.sender.receiver_status();now=time.monotonic()
            return bool(status and status.pitch_loop_supported and status.fc_ready and status.fc_age_ms<150
                and status.calibration_id==self.mapping.calibration_id and status.session==self.sender.session_id
                and not status.failsafe and (not disabled or not status.enabled)
                and self.sender.telemetry_time and now-self.sender.telemetry_time<2.
                and now-self.sender.diagnostic_times.get('trace',-float('inf'))<2.)

        def prepare(self):
            status=self.sender.receiver_status();self.mapping.set_voltage(status.voltage_mv/1000.)
            _,forces=self.mapping.curve();cfg=deepcopy(configuration)
            cfg.update(physical_force_range_N=[float(forces[0]),float(forces[-1])],maximum_thrust_N=min(cfg['maximum_thrust_N'],float(forces[-1])))
            self.mission.config.update(cfg);self.mission.bounds[0,1]=cfg['maximum_thrust_N'];self.mission.reference.validate(cfg)
            path=self.output/'mpc_config.json';path.write_text(json.dumps(cfg,indent=2)+'\n')
            self.worker=PitchWorker(root,path,self.output)
            self.events({'kind':'worker_ready','info':self.worker.info,'mapping':self.mapping.provenance,'voltage_mv':status.voltage_mv})

        def stop(self,reason=None):
            with self.lock:
                if self.finished:return
                self.error=reason;self.finished=True;self.stage='stopped';self.slot=None
                self.sender.invalidate(reason or 'Pitch mission completed')
                self.events({'kind':'stopped','reason':reason,'landed':self.mission.done,'tick':self.mission.tick})

        def transmit(self):
            try:
                with self.lock:
                    if self.finished:return
                    now=time.monotonic();status=self.sender.receiver_status()
                    if self.stage=='waiting':self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=lease_s);return
                    if not (status and status.pitch_loop_supported and status.fc_ready and status.fc_age_ms<150):raise ValueError('Receiver/FC freshness or SZC3 capability lost')
                    if self.stage=='enabling':
                        if not self.mission.pd.liftoff.preflight(self.measured_state()[0],now):
                            raise ValueError('Aircraft lost level/stationary liftoff readiness before arming')
                        self.sender.ctbr(True,0.,(0.,0.,0.),valid_for_s=lease_s);return
                    self.measured_state()
                    if status.session!=self.sender.session_id or status.failsafe or not status.enabled or self.sender.rearm_required:raise ValueError('Receiver ownership/interlock/lease lost')
                    if self.slot is None:
                        if now-self.mission.started>.04:raise ValueError('No fresh controller command')
                        return
                    if now-self.slot[0]>=.04:raise ValueError('Pitch controller output expired')
                    self.mapping.set_voltage(status.voltage_mv/1000.)
                    ctbr=self.slot[1];self.sender.ctbr(True,float(ctbr[0]),tuple(ctbr[1:]),valid_for_s=lease_s)
            except Exception as error:self.stop('Command: '+str(error))

        def control(self):
            try:
                with self.lock:
                    if self.stage!='running' or self.finished:return
                    if self.events.error or self.history.error:raise ValueError('Controller recording failed')
                    now=time.monotonic();value,age,packet=self.measured_state()
                    if self.worker.error and self.mission.epoch is not None and self.mission.landing is None:raise ValueError(self.worker.error)
                    status=self.sender.receiver_status()
                    if status is None:raise ValueError('Receiver status missing')
                    self.mapping.set_voltage(status.voltage_mv/1000.)
                    result=self.worker.latest();ctbr,reference=self.mission.choose(now,value,result,self.mapping.limit(3.2))
                    self.mission.commit(now,ctbr);self.slot=(now,ctbr.copy())
                    request=self.mission.inference_request(now,value,age,packet.get('pose_filter_delay_s')) if self.mission.landing is None else None
                    row={'tick':self.mission.tick,'host_ns':int(now*1e9),'unix_ns':time.time_ns(),'state_stamp_ns':packet['stamp_ns'],
                        'state':value.tolist(),'state_age_s':age,'phase':self.mission.phase,'reference':reference,'command':ctbr.tolist(),
                        'motion_epoch':self.mission.motion_epoch,'control_source':self.mission.control_source,'result':result,'request':request,
                        'handover_gate':self.mission.gate,'hover_gate':self.mission.hover_gate,'force_estimate_N':self.mission.force,
                        'horizontal_distance_m':self.mission.horizontal_distance_m,'horizontal_limit_bypassed':self.mission.skip_horizontal_limit}
                    self.history({'tick':self.mission.tick,'selected_stamp_ns':int(now*1e9),'selected_ctbr':ctbr.tolist(),'committed_tick':self.mission.tick})
                    self.events({'kind':'control','record':row});self.last_control=row
                    if self.mission.done:self.stop();return
                    if request is not None and not self.worker.error:self.worker.submit(request)
            except Exception as error:self.stop('Controller: '+str(error))

        def close(self):
            self.stop(self.error);cleanup=[]
            actions=[self.sender.close,*([self.worker.close] if self.worker else []),self.events.close,self.history.close]
            for action in actions:
                try:action()
                except Exception as error:cleanup.append(str(error))
            receipt={'device':self.device,'frame':self.drone['tracked_frame'],'error':self.error,'landed':self.mission.done,
                'ticks':self.mission.tick,'phase':self.mission.phase,'motion_epoch':self.mission.motion_epoch,
                'shutdown_acknowledged':self.sender.shutdown_acknowledged,'shutdown_error':self.sender.shutdown_error,
                'recording_error':self.events.error or self.history.error or ('; '.join(cleanup) if cleanup else None),'finished_unix_ns':time.time_ns()}
            (self.output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')

    def emergency(reason):
        state['error']=state['error'] or reason
        for node in nodes.values():node.stop(reason)

    def supervise():
        now=time.monotonic()
        while True:
            try:
                request=json.loads(operator.recv(4096))
                if not isinstance(request,dict) or set(request)-{'action','device'} or request.get('action') not in ('land','emergency-stop'):
                    raise ValueError('Invalid local operator request')
                action=request['action'];device=request.get('device')
                if device is not None and (type(device) is not int or device not in nodes):raise ValueError('Unknown operator device')
                if action=='land':clock.request_landing('operator_group_landing')
                elif action=='emergency-stop':
                    if device is None:emergency('operator_emergency_stop')
                    elif device in nodes:nodes[device].stop('operator_individual_emergency');clock.request_landing('participant_failure')
            except BlockingIOError:break
            except (ValueError,TypeError) as error:events({'kind':'invalid_operator_request','error':str(error)})
        if clock.abort_reason and state['phase'] in ('startup','enabling'):
            emergency('Startup cancelled before takeoff');return
        if state['phase'] in ('startup','enabling') and any(n.error for n in nodes.values()):
            emergency('Participant startup failure');return
        with ExitStack() as stack:
            for node in nodes.values():stack.enter_context(node.lock)
            if state['phase']=='startup':
                if now-state['since']>150.:emergency('Startup timeout waiting for all mocap/receiver/FC/capabilities');return
                prepared=False
                for node in nodes.values():
                    if node.worker is None and node.feedback_ready(disabled=True):node.prepare();prepared=True;break
                if prepared:return
                try:
                    ready=all(n.worker is not None and n.feedback_ready(disabled=True) and np.linalg.norm(n.measured_state()[0][:2]-n.drone['home_xy'])<=.1
                        and n.mission.pd.liftoff.preflight(n.measured_state()[0],now) for n in nodes.values())
                except ValueError:ready=False
                state['preflight_since']=(state['preflight_since'] or now) if ready else None
                if state['preflight_since'] and now-state['preflight_since']>=1.:
                    for node in nodes.values():node.stage='enabling'
                    state['phase']='enabling';state['arming_since']=now
                    events({'kind':'group_arm_request','host_ns':int(now*1e9),'devices':list(nodes)})
            elif state['phase']=='enabling':
                if now-state['arming_since']>2.:emergency('Group arming acknowledgement timeout');return
                if all(n.feedback_ready() and n.sender.receiver_status().enabled for n in nodes.values()):
                    values={d:n.measured_state()[0] for d,n in nodes.items()};clock.begin(now,values)
                    for node in nodes.values():node.stage='running'
                    state['phase']='running'
            elif state['phase']=='running':
                if events.error or (mocap and mocap.error):clock.request_landing('fleet_recording_or_mocap_worker_failed')
                if any(n.error for n in nodes.values()):clock.request_landing('participant_failure')
                values={}
                for d,node in nodes.items():
                    if node.finished:continue
                    try:values[d]=node.measured_state()[0]
                    except ValueError:continue  # Individual40ms timer handles stale output immediately.
                    clock.check_path_corridor(now,d,values[d][:3],trees[d],plan['tracking_margin_m'])
                for i,a in enumerate(values):
                    for b in list(values)[i+1:]:
                        if np.linalg.norm(values[a][:3]-values[b][:3])<2*plan['body_radius_m']:emergency('minimum_airframe_separation');return
                clock.update(now)
            while clock.events:events(clock.events.pop(0))

    interrupts=0
    def interrupt(signum,_):
        nonlocal interrupts
        interrupts+=1
        if signum==signal.SIGINT and interrupts==1 and state['phase']=='running':clock.request_landing('operator_interrupt')
        else:emergency('Emergency termination')
    previous={s:signal.signal(s,interrupt) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        bag=subprocess.Popen(['ros2','bag','record','--storage','mcap','--output',str(output/'mocap'),'/tf','/tf_static','/rosout'],stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+10.
        while not list((output/'mocap').glob('*.mcap')):
            if bag.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Shared MCAP recorder startup failed')
            time.sleep(.02)
        transport=SanityTransport(**plan['transport'])
        for drone in drones:
            node=PitchNode(drone);nodes[drone['id']]=node
        mocap=FleetMocapProcess(SimpleNamespace(mocap=plan['mocap']),nodes,output)
        for device,node in nodes.items():
            node.external_mocap=mocap.readers[device]
            executor=SingleThreadedExecutor();executor.add_node(node);executors.append(executor)
            thread=threading.Thread(target=executor.spin,daemon=True,name=f'pitch_control_{device}');threads.append(thread);thread.start()
        driver.create_timer(.02,supervise)
        gc.collect();gc.disable();last_print=0.
        while rclpy.ok() and not all(n.finished for n in nodes.values()):
            driver_executor.spin_once(timeout_sec=.005)
            if bag.poll() is not None:clock.request_landing('Shared MCAP recorder stopped')
            if time.monotonic()-last_print>=3.:
                print(' | '.join(f'{d:02d}:{n.mission.phase}' for d,n in nodes.items()),flush=True);last_print=time.monotonic()
    except BaseException as error:emergency(str(error));raise
    finally:
        driver_executor.shutdown(timeout_sec=3.)
        for executor in executors:executor.shutdown(timeout_sec=3.)
        for thread in threads:thread.join(timeout=3.)
        with ThreadPoolExecutor(max_workers=max(1,len(nodes))) as pool:
            futures=[pool.submit(n.close) for n in nodes.values()]
            for future in futures:
                try:future.result()
                except Exception as error:events({'kind':'shutdown_error','error':str(error)});state['error']=state['error'] or str(error)
        if mocap:mocap.close()
        if transport:transport.close()
        for node in nodes.values():node.destroy_node()
        driver.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if bag and bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            try:bag.wait(timeout=10.)
            except subprocess.TimeoutExpired:bag.terminate();bag.wait(timeout=3.)
        baglog.close();operator.close();(output/'control.sock').unlink(missing_ok=True);events.close();gc.enable()
        for s,handler in previous.items():signal.signal(s,handler)
    receipts={str(d):json.loads((n.output/'receipt.json').read_text()) for d,n in nodes.items()}
    from scipy.spatial.transform import Rotation
    for device,node in nodes.items():
        rows=[e['record'] for line in (node.output/'events.jsonl').open() if (e:=json.loads(line)).get('kind')=='control']
        history=[json.loads(line) for line in (node.output/'history.jsonl').open()]
        assessment={'history_complete':len(rows)==len(history) and all(r['tick']==h['tick']==h['committed_tick'] and r['command']==h['selected_ctbr'] for r,h in zip(rows,history)),
            'pitch_turns':0.,'inversions':0,'moving_position_rmse_m':None}
        if rows:
            values=np.asarray([r['state'] for r in rows]);rotations=Rotation.from_quat(values[:,[4,5,6,3]]).as_matrix()
            pitch=np.unwrap(np.arctan2(rotations[:,0,2],rotations[:,2,2]));inverted=rotations[:,2,2]<-.7
            assessment.update(pitch_turns=float(-(pitch[-1]-pitch[0])/(2*np.pi)*plan['direction']),
                inversions=int(np.count_nonzero(inverted&~np.r_[False,inverted[:-1]])))
            moving=np.asarray([r['phase'] in ('run_in','pitch_entry','pitch_loops','pitch_exit','run_out') for r in rows])
            error=values[:,:3]-np.asarray([r['reference']['position'] for r in rows])
            if moving.any():assessment['moving_position_rmse_m']=float(np.sqrt(np.mean(np.sum(error[moving]**2,axis=1))))
        assessment['flip_completed']=assessment['pitch_turns']>plan['laps']-.15 and assessment['inversions']==plan['laps']
        receipts[str(device)]['assessment']=assessment
        (node.output/'assessment.json').write_text(json.dumps(assessment,indent=2)+'\n')
    summary={'passed':clock.started and not clock.abort_reason and all(r['landed'] and r['error'] is None and r['shutdown_acknowledged'] and not r['recording_error'] and r['assessment']['flip_completed'] and r['assessment']['history_complete'] for r in receipts.values()),
        'group_epoch':clock.epoch,'abort_reason':clock.abort_reason,'error':state['error'],'drones':receipts,
        'mcap_finalized':(output/'mocap/metadata.yaml').exists() and bag.returncode==0,'recording_complete':not events.error,'separation':separation}
    summary['passed']=summary['passed'] and summary['mcap_finalized'] and summary['recording_complete']
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary
