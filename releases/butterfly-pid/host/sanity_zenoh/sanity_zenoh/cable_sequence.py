import argparse
import gc
from concurrent.futures import TimeoutError as FutureTimeout
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sanity_zenoh.cable_state import CableMocap, CommandObserver
from sanity_zenoh.cable_mission import CableMission
from sanity_zenoh.cable_health import VoltageGuard
from sanity_zenoh.cable_configuration import controller_configuration, load_physical_configuration
from sanity_zenoh.cable_process import CableWorker, pin_cpu_affinity, move_background_threads, next_control_slot
from sanity_zenoh.client import SanitySender
from sanity_zenoh.firmware_collective import FirmwareCollective
from sanity_zenoh.recording import EventRecorder
from sanity_zenoh.cable_engagement import configuration as engagement_configuration, raw_geometry


class CableSequenceNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('sanity_cable',parameter_overrides=parameter_overrides or [])
        defaults={'loco_root':'','profile':'','physical_config':'','esp_collective_profile':'','output':'','worker_python':'','cable_engagement_config':'',
                  'drone_id':9,'payload_mass_g':-1.,'enable_output':False,'simulation':False,'mission':'','controller':'','listen':'udp/10.0.0.200:7447','telemetry_listen':'tcp/10.0.0.200:7448',
                  'test_solver_delay_at_s':-1.,'test_solver_delay_s':.12,'test_control_delay_at_s':-1.,'test_control_delay_s':.045,'takeoff_land_only':False,'mpc_hover_only':False,'hover_duration_s':5.,'skip_ground_height_check':False,'landing_speed_profile':'','landing_cutoff_height_m':-1.,'landing_cutoff_dwell_s':-1.}
        for name,value in defaults.items():self.declare_parameter(name,value)
        self.values={key:self.get_parameter(key).value for key in defaults}
        v=self.values
        self.sim_clock=bool(self.get_parameter('use_sim_time').value)
        if self.sim_clock and not v['simulation']:raise ValueError('Physical flight requires the live ROS clock')
        if v['simulation'] and (not v['listen'].startswith('tcp/127.0.0.1:') or v['telemetry_listen']):
            raise ValueError('Simulation requires an isolated loopback listener with no external telemetry endpoint')
        if (v['test_solver_delay_at_s']>=0 or v['test_control_delay_at_s']>=0) and not v['simulation']:raise ValueError('Fault injection is simulation-only')
        self.experiment=Path(v['loco_root']).resolve()/'experiments/sanity_cable_mpc'
        self.profile=json.loads(Path(v['profile']).read_text())
        if v['cable_engagement_config']:self.profile['cable_engagement']=json.loads(Path(v['cable_engagement_config']).read_text())
        self.engagement_settings=engagement_configuration(self.profile.get('cable_engagement'))
        self.cable_shadow=None;self.shadow_service=None;self.shadow_start_error=None
        for key in ('mission','controller'):
            if v[key]:self.profile[key]=v[key]
        if v['mpc_hover_only']:self.profile['mpc_hover_only']=True
        if v['takeoff_land_only']:self.profile.update(takeoff_land_only=True,hover_duration_s=v['hover_duration_s'])
        if v['skip_ground_height_check']:self.profile['skip_ground_height_check']=True
        if v['landing_speed_profile']:self.profile['landing_speed_profile']=v['landing_speed_profile']
        for key in ('landing_cutoff_height_m','landing_cutoff_dwell_s'):
            if v[key]!=-1.:self.profile[key]=v[key]
        if self.profile['mission'] not in ('circle_slow','circle_fast','payload_orbit') or self.profile['controller'] not in ('aware','unaware'):
            raise ValueError('Choose a supported cable mission and aware/unaware controller')
        if self.profile['mission']=='payload_orbit' and self.profile['controller']!='aware':raise ValueError('Payload orbit requires the aware controller')
        if not v['simulation'] and self.profile.get('start_airborne',False):raise ValueError('Airborne initial fixture is simulation-only')
        if not all(math.isfinite(float(self.profile[key])) for key in ('floor_height_m','hover_height_m','maximum_height_m','climb_s','output_delay_s','maximum_run_s')) or not 0<=self.profile['output_delay_s']<=.04:
            raise ValueError('Finite mission parameters and output delay0..40ms required')
        self.cpu_affinity=pin_cpu_affinity(self.profile.get('controller_cpu_affinity',[]))
        calibration=dict(self.profile['mocap'])
        if self.profile.get('dedicated_control_thread',False) or (self.profile.get('pid_control_thread',False) and self.profile.get('takeoff_land_only',False)):calibration['reset_cable_mode_on_gap']=True
        self.profile['mocap']=calibration
        if v['enable_output'] and not v['simulation'] and not (calibration.get('frames_confirmed') and self.profile.get('geometry_confirmed')):
            raise ValueError('Aircraft/payload frame transforms and attachment geometry must be confirmed for hardware output')
        self.physical=load_physical_configuration(self.experiment,self.profile,v['profile'],v['physical_config'])
        if v['payload_mass_g']!=-1.:
            mass=float(v['payload_mass_g'])/1000.
            if not math.isfinite(mass) or not .005<=mass<=.1:raise ValueError('Payload mass must be 5..100 g, including markers and excluding cable')
            self.physical['payload_mass_kg']=mass
            self.physical['payload_mass_source']='Explicit launch parameter payload_mass_g, includes markers and excludes cable'
            size=np.asarray(self.physical['payload_size_m'])
            self.physical['payload_transverse_inertia_kg_m2']=mass*(size[0]**2+size[2]**2)/12
            self.physical['payload_inertia_source']='Uniform cube proxy scaled with explicit payload_mass_g; not independently measured'
        self.voltage_guard=VoltageGuard(self.profile.get('flight_voltage'))
        self.cfg=controller_configuration(self.physical,self.profile,bool(v['simulation']),bool(v['enable_output']))
        if self.profile.get('hardware_ablation'):self.get_logger().warning('ABLATION: rigid total known mass; no cable feedback or payload dynamic feedforward in MPC. Failure thresholds request PD landing.')
        sys.path.insert(0,str(self.experiment.parent))
        from sanity_cable_mpc.model import make_model
        self.model=make_model(self.cfg)
        self.mocap_lock=threading.RLock()
        self.mocap=CableMocap(self.physical,calibration)
        stationary=bool(self.profile.get('takeoff_land_only',False) or self.profile.get('mpc_hover_only',False))
        reference_kind='stationary hover reference' if stationary else 'rigid-mass analytic circle reference' if self.cfg.get('prediction_mode')=='rigid_total_mass' else 'analytic payload spiral reference' if self.profile['mission']=='payload_orbit' else 'cached length-dependent cable references'
        self.get_logger().info('Preparing '+reference_kind+'; aircraft output remains disabled')
        reference_started=time.perf_counter()
        self.flight=CableMission(self.cfg,self.profile,self.experiment)
        reference_info=self.flight.reference.info
        cache_info=reference_info.get('length_reference_sources',reference_info.get('pendulum_reference_cache','analytic'))
        self.get_logger().info(f'References ready in {time.perf_counter()-reference_started:.2f}s; source={cache_info}')
        self.output=Path(v['output']).resolve();self.output.mkdir(parents=True,exist_ok=False)
        (self.output/'config.json').write_text(json.dumps(self.cfg,indent=2)+'\n')
        (self.output/'profile.json').write_text(json.dumps(self.profile,indent=2)+'\n')
        self.log=EventRecorder(self.output/'control.jsonl')
        self.command_log=EventRecorder(self.output/'commands.jsonl')
        self.tf_log=EventRecorder(self.output/'tf.jsonl')
        self.event_log=EventRecorder(self.output/'events.jsonl')
        self.mapping=FirmwareCollective(v['esp_collective_profile'])
        if not self.profile.get('takeoff_land_only',False):self.get_logger().info('Preparing cable MPC solver and cache; aircraft output remains disabled')
        self.worker=None if self.profile.get('takeoff_land_only',False) else CableWorker(v['worker_python'] or sys.executable,self.experiment,self.output/'config.json',self.output,self.profile.get('solver_cpu_affinity',[]),self.profile.get('response_reader_cpu_affinity',[]))
        self.transport_events=EventRecorder(self.output/'transport.jsonl')
        self.gc_was_enabled=gc.isenabled();gc.collect();gc.disable()
        try:
            self.sender=SanitySender(device=v['drone_id'],listen=v['listen'],telemetry_listen=v['telemetry_listen'] or None,ctbr_mapping=self.mapping,
                                     physical_units=True,disabled_only=not v['enable_output'],on_event=self.transport_events,diagnostics=True)
        except BaseException:
            self.transport_events.close()
            if self.worker is not None:self.worker.close()
            if self.gc_was_enabled:gc.enable()
            for log in (self.log,self.command_log,self.tf_log,self.event_log):log.close()
            raise
        self.phase='waiting';self.slot=None;self.pending=None;self.observer=None;self.previous_applied=None
        self.fatal=None;self.finished=threading.Event();self.requested_landing=None;self.last_now=None;self.last_control_wall=None
        self.ticks=0;self.solver_results=0;self.solver_rejections=0;self.full_times=[];self.estimation_times=[]
        self.wall_start=time.monotonic();self.last_measurement_error=None;self.countdown_start_ns=None;self.startup_wait_log_ns=None
        self.control_thread_enabled=bool(self.profile.get('dedicated_control_thread',False) or (self.profile.get('pid_control_thread',False) and self.profile.get('takeoff_land_only',False)))
        self.ros_thread_id=threading.get_native_id()
        self.control_thread=None;self.next_selection_ns=None;self.control_busy=False;self.control_deadline_misses=0;self.recent_control_misses=deque(maxlen=8);self.control_delay_injected=False
        self.reported_flight_events=0
        self.progress_timer=self.create_timer(.2,self.report_flight_events)
        self.subscription=self.create_subscription(TFMessage,self.profile.get('tf_topic','/tf'),self.receive_tf,QoSProfile(depth=30,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.control_timer=self.create_timer(.02,self.control_tick)
        self.command_timer=self.create_timer(.01,self.command_tick)
        if self.control_thread_enabled:self.control_timer.cancel();self.command_timer.cancel()
        if self.engagement_settings['mode']=='shadow':self.start_cable_shadow()
        move_background_threads(self.profile.get('background_cpu_affinity',[]))
        self.get_logger().info('Cable mission ready; aircraft output '+('permitted after readiness checks' if v['enable_output'] else 'DISABLED'))


    def shadow_context(self):
        # Copy only; compute geometry outside the shared mocap lock. previous
        # contains calibrated raw poses before the position/pendulum projection.
        poses={}
        if not self.mocap_lock.acquire(blocking=False):
            return {'valid':False,'reasons':['raw_geometry_snapshot_busy']},self.flight.phase,False
        try:
            for key,estimator in self.mocap.estimators.items():
                if estimator.previous is None or estimator.count<5:continue
                stamp,point,rotation=estimator.previous
                latest=self.mocap.buffers[key][-1] if self.mocap.buffers[key] else {}
                poses[key]={'stamp_ns':int(stamp),'position':point.tolist(),'quaternion_xyzw':rotation.as_quat().tolist(),'orientation_held':latest.get('orientation_held',True)}
        finally:self.mocap_lock.release()
        physical=self.shadow_physical
        calibration=None
        if self.engagement_settings['cable_length_source']=='hover_calibration':
            calibration=dict(self.flight.length_calibration.diagnostic)
            if self.flight.length_applied is not None:physical={**physical,'cable_length_m':float(self.flight.length_applied)}
        geometry=raw_geometry(physical,poses,self.get_clock().now().nanoseconds,self.engagement_settings,self.flight.payload_floor_height_m)
        if self.engagement_settings['cable_length_source']=='hover_calibration':
            geometry['length_calibration']=calibration
            geometry['length_source']='Stable-hover effective geometric length; not independent measured-tension proof'
            if self.flight.length_applied is None:
                geometry['valid']=False;geometry['reasons'].append('hover_cable_length_pending')
        if not self.profile.get('geometry_confirmed') or not self.profile['mocap'].get('frames_confirmed'):
            geometry['valid']=False;geometry['reasons'].append('attachment_or_mocap_calibration_unconfirmed')
        return geometry,self.flight.phase,self.engagement_settings['path_clear_confirmed']


    def start_cable_shadow(self):
        try:
            from std_srvs.srv import Trigger
            from sanity_zenoh.cable_strain import CableStrainShadow
            # Freeze independently configured geometry, before hover-length fit.
            self.shadow_physical=json.loads(json.dumps(self.physical))
            helpers=Path(self.values['loco_root']).resolve().parent/'Sanity/sanity-hardware/tools/acquisition'
            self.cable_shadow=CableStrainShadow(self.engagement_settings,helpers,self.output,self.shadow_context)
            self.shadow_service=self.create_service(Trigger,'~/confirm_sensor_unloaded',self.confirm_sensor_unloaded)
            self.cable_shadow.start()
            self.get_logger().warning('Cable engagement SHADOW ONLY: logs proposed decisions; no flight gating. Baseline, polarity, thresholds and timing bounds require validation.')
        except Exception as error:
            self.shadow_start_error=type(error).__name__+': '+str(error)
            if self.cable_shadow is not None:self.cable_shadow.close();self.cable_shadow=None
            self.record_event({'kind':'cable_shadow_start_failed','reason':self.shadow_start_error,'control_affected':False})
            self.get_logger().error('Shadow monitoring unavailable: '+self.shadow_start_error)


    def confirm_sensor_unloaded(self, request, response):
        if self.cable_shadow is None:response.success=False;response.message='Shadow task unavailable'
        else:response.success,response.message=self.cable_shadow.confirm_unloaded()
        self.record_event({'kind':'operator_unloaded_confirmation_request','accepted':response.success,'message':response.message})
        return response


    def receive_tf(self, message):
        now=self.get_clock().now().nanoseconds
        records=[];rejections=[]
        with self.mocap_lock:
            for tf in message.transforms:
                if tf.child_frame_id not in (self.profile['mocap']['aircraft_frame'],self.profile['mocap']['payload_frame']):continue
                p,q=tf.transform.translation,tf.transform.rotation
                stamp=tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
                record={'stamp_ns':stamp,'arrival_ns':now,'parent':tf.header.frame_id,'child':tf.child_frame_id,'position':[v if math.isfinite(v) else None for v in (p.x,p.y,p.z)],'quaternion_xyzw':[v if math.isfinite(v) else None for v in (q.x,q.y,q.z,q.w)],'estimator_accepted':False}
                records.append(record)
                try:
                    accepted=self.mocap.update(tf.header.frame_id,tf.child_frame_id,stamp,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],now)
                    record['estimator_accepted']=accepted is not None
                    if accepted is not None:record.update(orientation_held=accepted.get('orientation_held',False),orientation_stamp_ns=accepted.get('orientation_stamp_ns',stamp),pose_filter_tau_s=accepted.get('pose_filter_tau_s'),pose_filter_delay_s=accepted.get('pose_filter_delay_s'))
                except ValueError as error:
                    record['rejection']=str(error)
                    self.last_measurement_error=str(error)
                    rejections.append({'kind':'mocap_rejection','stamp_ns':stamp,'reason':str(error),'child':tf.child_frame_id,'diagnostics':getattr(error,'diagnostics',{})})
        for record in records:self.tf_log(record)
        for record in rejections:self.record_event(record)


    def report_flight_events(self):
        events=self.flight.events[self.reported_flight_events:]
        for event in events:
            self.record_event({'kind':'mission_event',**event})
            message=('MPC hover readiness: '+event['reason']) if event['phase']=='mpc_hover_status' else 'Phase: '+event['phase']+(' — '+event['reason'] if event.get('reason') else '')
            self.get_logger().info(message)
        self.reported_flight_events+=len(events)


    def record_event(self, record):
        self.event_log(record)


    def sample_mocap(self, now, require_payload=True):
        with self.mocap_lock:return self.mocap.sample(max(now,self.get_clock().now().nanoseconds),require_payload=require_payload)


    def start_control_thread(self):
        if not self.control_thread_enabled or self.control_thread is not None:return
        self.control_thread=threading.Thread(target=self.run_control_loop,name='sanity_flight_control',daemon=True)
        self.control_thread.start()
        background=self.profile.get('background_cpu_affinity',[])
        ros_cpus=self.profile.get('ros_cpu_affinity',background)
        if ros_cpus:os.sched_setaffinity(0,set(ros_cpus))
        self.record_event({'kind':'control_runtime_started','runtime':'dedicated_control_thread','controller_cpus':self.cpu_affinity,'background_cpus':background,'ros_cpus':ros_cpus})


    def run_control_loop(self):
        try:
            if self.cpu_affinity:os.sched_setaffinity(0,set(self.cpu_affinity))
            next_control=next_command=None
            while not self.finished.is_set():
                now=self.get_clock().now().nanoseconds if self.sim_clock else time.monotonic_ns()
                if now<=0:
                    self.finished.wait(.001);continue
                if next_command is None:next_control=next_command=now
                if now>=next_command:
                    if now>=next_control:
                        next_deadline=next_control_slot(now,next_control)
                        if self.sim_clock:self.next_selection_ns=next_deadline
                        else:
                            selection_clock=self.get_clock().now().nanoseconds
                            self.next_selection_ns=selection_clock+next_deadline-time.monotonic_ns()
                        self.control_tick()
                        current=self.get_clock().now().nanoseconds if self.sim_clock else time.monotonic_ns()
                        next_control=next_control_slot(current,next_control)
                    self.command_tick()
                    current=self.get_clock().now().nanoseconds if self.sim_clock else time.monotonic_ns()
                    next_command+=max(1,(current-next_command)//10_000_000+1)*10_000_000
                else:
                    self.finished.wait(.0005 if self.sim_clock else min(.005,(next_command-now)/1e9))
        except Exception as error:self.stop('Control runtime: '+type(error).__name__+': '+str(error))


    def measurement(self, now):
        try:return self.sample_mocap(now)
        except ValueError as error:
            self.last_measurement_error=str(error)
            if self.phase=='active':
                packet=self.sample_mocap(now,require_payload=False)
                packet['payload_observation_error']=str(error)
                return packet
            raise


    def check_control_clock(self, now_ns, wall_s):
        if self.last_now is not None:
            ros_ms=(now_ns-self.last_now)/1e6
            wall_ms=(wall_s-self.last_control_wall)*1000
            if not 0<ros_ms<=40.:
                diagnostic={'stamp_ns':int(now_ns),'ros_interval_ms':ros_ms,'monotonic_interval_ms':wall_ms}
                if ros_ms>40. and self.control_thread_enabled:
                    self.control_deadline_misses+=1;self.recent_control_misses.append(now_ns)
                    repeated=sum(now_ns-stamp<=1_000_000_000 for stamp in self.recent_control_misses)>=3
                    if ros_ms>80. or repeated:self.requested_landing='Repeated or long control scheduling delay'
                    self.record_event({'kind':'control_deadline_miss',**diagnostic,'response':'controlled_landing' if self.requested_landing else 'fresh_state_pid_recompute'})
                    self.get_logger().warning(f'Control scheduling interval {ros_ms:.3f}ms; '+('requesting landing' if self.requested_landing else 'recomputing from fresh mocap'))
                else:
                    self.record_event({'kind':'control_clock_fault',**diagnostic})
                    raise ValueError(f'Control clock gap or reversal: ROS interval={ros_ms:.3f}ms, monotonic interval={wall_ms:.3f}ms, allowed0..40ms')
        self.last_now=now_ns;self.last_control_wall=wall_s


    def control_tick(self):
        if self.phase!='active' or self.finished.is_set():return
        delay_at=self.values['test_control_delay_at_s']
        if delay_at>=0 and self.flight.start_ns is not None and (self.get_clock().now().nanoseconds-self.flight.start_ns)/1e9>=delay_at:
            self.values['test_control_delay_at_s']=-1.
            time.sleep(self.values['test_control_delay_s']);self.control_delay_injected=True
        if self.ticks==0:move_background_threads(self.profile.get('background_cpu_affinity',[]),preserve=[self.ros_thread_id] if self.control_thread_enabled else [])
        now=self.get_clock().now().nanoseconds
        started=time.perf_counter();cpu_started=time.thread_time();timing={}
        self.control_busy=True
        try:
            packet=self.measurement(now)
            for recorder in (self.log,self.command_log,self.tf_log,self.event_log,self.transport_events):
                if recorder.error:self.requested_landing='Flight recording failed: '+recorder.error
            self.check_control_clock(now,started)
            if self.profile.get('flight_voltage') and self.values['enable_output']:
                status=self.sender.receiver_status()
                reason=self.voltage_guard.update(None if status is None else status.voltage_mv/1000.,now)
                if reason and self.flight.phase in ('acquire','blend','mpc_hover','mission','settle'):self.requested_landing=reason
            timing['measurement_ms']=(time.perf_counter()-started)*1000
            if self.observer is None:self.observer=CommandObserver(self.model,now-1_000_000_000)
            if self.requested_landing is not None and self.flight.start_ns is not None:
                self.flight.request_landing(packet,now,self.requested_landing);self.requested_landing=None
            plan=self.flight.update(packet,now)
            if self.flight.length_pending is not None:
                if self.pending is not None:raise ValueError('MPC request outstanding during PID length calibration')
                length=self.flight.length_pending
                self.model.set_cable_length(length)
                with self.mocap_lock:self.mocap.set_cable_length(length)
                self.flight.apply_cable_length(length,now)
                self.cfg['cable_length_m']=length
                packet=self.measurement(now)
            timing['mission_ms']=(time.perf_counter()-started)*1000
            candidate=None;diagnostic=None;result_age=None
            if self.pending is not None:
                future,target,prep_ms,submitted_wall=self.pending
                if not future.done() and not self.control_thread_enabled:
                    remaining=.025-prep_ms/1000-(time.perf_counter()-submitted_wall)
                    if remaining>0:
                        try:future.result(timeout=remaining)
                        except FutureTimeout:pass
                        except Exception:pass
                if future.done():
                    try:result=future.result()
                    except Exception as error:
                        result={'error':'Solver IPC failure: '+str(error),'roundtrip_ms':2000.}
                        self.flight.request_landing(packet,now,'Solver process failed')
                    self.pending=None
                    result_age=(now-target)/1e9
                    full_ms=(result.get('completed_wall_s',time.perf_counter())-submitted_wall)*1000+prep_ms
                    self.full_times.append(full_ms)
                    diagnostic=result.get('mpc',{})
                    if 'error' not in result and -.002<=result_age<=.02 and full_ms<=25. and diagnostic.get('valid') and (not self.cfg.get('runtime_cable_length',False) or (diagnostic.get('cable_length_m')==self.cfg['cable_length_m'] and diagnostic.get('payload_mass_kg')==self.cfg['payload_mass_kg'])):
                        candidate=np.asarray(result['command']);self.solver_results+=1
                    else:
                        self.solver_rejections+=1
                        self.record_event({'kind':'mpc_rejection','stamp_ns':now,'result_age_s':result_age,'result':result})
                        if self.flight.phase in ('blend','mpc_hover','mission','settle'):
                            if not diagnostic.get('prediction_feasible',diagnostic.get('valid',False)):
                                reason=f"MPC prediction rejected: {diagnostic.get('worst_constraint','unknown')} violation={diagnostic.get('constraint_violation')}, status={diagnostic.get('status')}"
                                self.flight.request_landing(packet,now,reason)
                elif now>=target+20_000_000:
                    diagnostic={'valid':False,'pending':True,'reason':'Solver response not yet available'}
            timing['result_ms']=(time.perf_counter()-started)*1000
            if not packet.get('taut',False):candidate=None
            selected,weight=self.flight.select(plan['pd'],candidate,packet,now)
            if self.flight.finished:
                self.phase='done';self.finished.set();selected[:]=0.
            self.slot={'stamp_ns':now,'command':selected.tolist()}
            self.command_tick()
            timing['emission_ms']=(time.perf_counter()-started)*1000
            if self.fatal:return
            inference_input=None
            if plan['need_mpc'] and self.pending is None and self.flight.phase in ('acquire','blend','mpc_hover','mission','settle'):
                preparation=time.perf_counter()
                selection_ns=getattr(self,'next_selection_ns',None) or now+20_000_000
                selection_ns=next_control_slot(self.get_clock().now().nanoseconds,selection_ns)
                apply_ns=selection_ns+int(self.profile['output_delay_s']*1e9)
                prepared=self.observer.forecast(packet,apply_ns)
                timing['forecast_ms']=(time.perf_counter()-started)*1000
                preview=self.flight.preview(apply_ns+np.rint(np.arange(self.cfg['nodes']+1)*self.cfg['prediction_dt_s']*1e9).astype(np.int64))
                prep_ms=(time.perf_counter()-started)*1000;self.estimation_times.append(prep_ms)
                delay=0.
                if self.values['test_solver_delay_at_s']>=0 and self.flight.mission_start_ns is not None and (now-self.flight.mission_start_ns)/1e9>=self.values['test_solver_delay_at_s']:
                    delay=self.values['test_solver_delay_s'];self.values['test_solver_delay_at_s']=-1.
                timing['preview_ms']=(time.perf_counter()-started)*1000
                inference_input={'history_capture_ns':int(self.get_clock().now().nanoseconds),'preparation_ms':prep_ms,'apply_ns':int(apply_ns),'state':prepared.tolist(),'preview_first':preview[0].tolist(),'history_last_apply_ns':int(self.observer.records[-1][0]),'history_last_command':self.observer.records[-1][2].tolist()}
                submitted_wall=time.perf_counter()
                future=self.worker.submit(prepared,preview,delay,previous_command=self.observer.records[-1][2],cable_length_m=self.cfg['cable_length_m'] if self.cfg.get('runtime_cable_length',False) else None,payload_mass_kg=self.cfg['payload_mass_kg'] if self.cfg.get('runtime_cable_length',False) else None)
                self.pending=(future,selection_ns,prep_ms,submitted_wall)
            timing['dispatch_ms']=(time.perf_counter()-started)*1000
            state=None
            if 'direction' in packet:
                state=self.observer.state(packet).tolist()
            record={'control_started_wall_s':started,'timing_cumulative_wall_ms':timing,'callback_thread_cpu_ms':(time.thread_time()-cpu_started)*1000,'mpc_input':inference_input,'mission_time_s':(now-self.flight.mission_start_ns)/1e9 if self.flight.mission_start_ns is not None else None,'tick':self.ticks,'stamp_ns':now,'source_stamp_ns':packet['stamp_ns'],'phase':self.flight.phase,'state':state,'vehicle':packet['vehicle'].tolist(),
                    'payload':packet.get('payload',np.array([])).tolist(),'hook_distance_m':packet.get('hook_distance_m'),'taut':packet.get('taut',False),
                    'payload_observation_error':packet.get('payload_observation_error'),'cable_length_residual_m':packet.get('cable_length_residual_m'),'cable_length_tolerance_m':packet.get('cable_length_tolerance_m'),
                    'payload_position_filter_delay_s':packet.get('payload_position_filter_delay_s'),'effective_cable_length_m':self.cfg['cable_length_m'],'payload_mass_kg':self.cfg['payload_mass_kg'],
                    'cable_length_calibration':self.flight.length_calibration.diagnostic if self.flight.length_calibration.enabled else None,
                    'failure_guard':self.flight.failure_guard.diagnostic if self.profile.get('failure_guard') else None,'payload_health':self.flight.payload_health.diagnostic if self.profile.get('payload_health') else None,'payload_spiral':self.flight.orbit_diagnostic if self.profile.get('payload_spiral') else None,
                    'voltage_guard':self.voltage_guard.diagnostic if self.profile.get('flight_voltage') else None,
                    'hover_qualification':self.flight.hover_qualification if self.flight.phase=='mpc_hover' else None,
                    'swing_deg':packet.get('swing_deg'),'payload_internal_tilt_deg':packet.get('payload_internal_tilt_deg'),'payload_orientation_held':packet.get('payload_orientation_held',False),'cube_lowest_height_m':packet.get('cube_lowest_height_m'),'reference':plan['reference'],
                    'selected':selected.tolist(),'pd':plan['pd'].tolist(),'mpc_weight':weight,'mpc':diagnostic,'result_age_s':result_age,
                    'load_fraction':plan['load_fraction'],'control_callback_ms':(time.perf_counter()-started)*1000,'pair_skew_s':packet.get('pair_skew_s')}
            if self.cable_shadow is not None:record['cable_engagement_shadow']=self.cable_shadow.latest
            self.log(record);self.ticks+=1
        except Exception as error:self.stop(type(error).__name__+': '+str(error))
        finally:self.control_busy=False


    def command_tick(self):
        if self.finished.is_set():
            self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=self.profile.get('command_lease_s',1.));return
        now=self.get_clock().now().nanoseconds
        try:
            if self.phase=='waiting':
                self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=self.profile.get('command_lease_s',1.))
                try:packet=self.sample_mocap(now)
                except ValueError as error:
                    self.reset_countdown('Mocap: '+str(error),now)
                    if time.monotonic()-self.wall_start>20.:raise ValueError('Startup timeout waiting for synchronized aircraft/payload mocap')
                    return
                ground=self.flight.check_ground_start(packet) if not self.profile.get('start_airborne',False) else None
                status=self.sender.receiver_status()
                if not self.values['enable_output']:
                    self.phase='active';self.control_tick();return
                if status and status.session==self.sender.session_id and not status.enabled and not status.failsafe:
                    if self.profile.get('flight_voltage') and not self.voltage_guard.start_ready(status.voltage_mv/1000.):
                        reason=f'Battery readiness: {status.voltage_mv/1000.:.2f}V; required {self.voltage_guard.minimum_start:.2f}..8.50V'
                        self.report_startup_wait(reason,now);self.reset_countdown(reason,now);return
                    countdown=float(self.profile.get('countdown_s',10. if self.profile.get('takeoff_land_only',False) else 0.))
                    if countdown>0.:
                        if self.countdown_start_ns is None:
                            self.countdown_start_ns=now
                            self.record_event({'kind':'prearm_ground_check','stamp_ns':now,'ground':ground})
                            self.get_logger().warning(f'Synchronized mocap and receiver ready; output enables in{countdown:g}seconds')
                        if (now-self.countdown_start_ns)/1e9<countdown:return
                    self.phase='enabling'
                else:
                    reason=('Receiver status stale/missing; sender='+str(self.sender.last_error)) if status is None else f'Receiver not ready: session_match={status.session==self.sender.session_id}, enabled={status.enabled}, failsafe={status.failsafe}, fc_ready={status.fc_ready}, fc_age_ms={status.fc_age_ms}, voltage_V={status.voltage_mv/1000:.2f}, rejected={status.rejected}'
                    self.report_startup_wait(reason,now)
                    self.reset_countdown(reason,now)
                return
            if self.phase=='enabling':
                packet=self.sample_mocap(now)
                if not self.profile.get('start_airborne',False):self.flight.check_ground_start(packet)
                status=self.sender.receiver_status()
                if self.profile.get('flight_voltage') and (status is None or not self.voltage_guard.start_ready(status.voltage_mv/1000.)):raise ValueError('Battery readiness lost before output enable')
                self.sender.ctbr(True,0.,(0.,0.,0.),valid_for_s=self.profile.get('command_lease_s',1.))
                status=self.sender.receiver_status()
                if status and status.session==self.sender.session_id and status.enabled and not status.failsafe:
                    self.phase='active';self.control_tick()
                return
            self.sample_mocap(now,require_payload=False)
            if self.control_thread_enabled and self.slot is not None and (now-self.slot['stamp_ns'])/1e9>=.04:
                if self.control_busy:raise ValueError('Control calculation exceeded40ms command freshness')
                self.control_tick()
                if self.finished.is_set():return
                now=self.get_clock().now().nanoseconds
            if self.slot is None or not 0<=(now-self.slot['stamp_ns'])/1e9<.04:raise ValueError('Cable control command expired')
            command=np.asarray(self.slot['command'])
            self.sender.ctbr(bool(self.values['enable_output']),float(command[0]),tuple(command[1:]),valid_for_s=self.profile.get('command_lease_s',1.))
            if self.values['enable_output']:
                status=self.sender.receiver_status()
                if status is None or not status.enabled or status.failsafe or self.sender.rearm_required:raise ValueError('Receiver lease/interlock lost')
            values=self.sender.values
            applied=np.r_[np.interp(values[3],*self.mapping.curve()) if values[3]>1000 else 0.,(np.array(values[:3])-1500)/500*np.radians(600.)*[1.,1.,-1.]]
            if not self.values['enable_output']:applied[:]=0.
            apply_ns=now+int(self.profile['output_delay_s']*1e9)
            if self.observer is not None and (self.previous_applied is None or np.any(applied!=self.previous_applied)):
                self.observer.commit(apply_ns,applied);self.previous_applied=applied.copy()
            self.command_log({'stamp_ns':now,'estimated_apply_ns':apply_ns,'enabled':self.values['enable_output'],'selected':command.tolist(),'mapped':applied.tolist(),'rc':list(values),'mapping':dict(self.mapping.last)})
        except Exception as error:self.stop('Command: '+type(error).__name__+': '+str(error))


    def report_startup_wait(self, reason, now_ns):
        if self.startup_wait_log_ns is None or now_ns-self.startup_wait_log_ns>=2_000_000_000:
            self.startup_wait_log_ns=now_ns
            self.record_event({'kind':'waiting_receiver','stamp_ns':int(now_ns),'reason':reason})
            self.get_logger().warning('Waiting for receiver: '+reason)


    def reset_countdown(self, reason, now_ns):
        if self.countdown_start_ns is not None:
            self.record_event({'kind':'countdown_reset','stamp_ns':int(now_ns),'reason':reason})
            self.get_logger().warning('Takeoff countdown reset: '+reason)
        self.countdown_start_ns=None


    def stop(self, reason=None):
        if reason and self.fatal is None:
            self.fatal=reason;self.get_logger().error(reason)
        self.sender.invalidate(reason or 'Cable mission ended');self.phase='done';self.finished.set()


    def close(self):
        self.finished.set()
        if self.control_thread is not None:self.control_thread.join(timeout=2.)
        self.sender.invalidate(self.fatal or 'Cable mission complete')
        self.sender.close();self.transport_events.close()
        if self.worker is not None:self.worker.close()
        self.report_flight_events()
        shadow_receipt=self.cable_shadow.close() if self.cable_shadow is not None else {'mode':self.engagement_settings['mode'],'start_error':self.shadow_start_error,'control_affected':False}
        for log in (self.log,self.command_log,self.tf_log,self.event_log):log.close()
        receipt={'trajectory':self.flight.reference.info,'scope':'cable deployment software; simulation-only evidence when simulation=true','parameters':self.values,'profile':self.profile,'controller':self.worker.info if self.worker is not None else {'scope':'PID takeoff/land','solver_initialized':False},
                 'recording_errors':{name:getattr(self,name).error for name in ('log','command_log','tf_log','event_log','transport_events')},'transport_recording_error':self.transport_events.error,'control_runtime':'dedicated_control_thread' if self.control_thread_enabled else 'ROS_timers','control_deadline_misses':self.control_deadline_misses,'control_delay_injected':self.control_delay_injected,
                 'cpu_affinity':self.cpu_affinity,'fatal':self.fatal,'effective_cable_length_m':self.cfg['cable_length_m'],'payload_mass_kg':self.cfg['payload_mass_kg'],'cable_length_calibration':self.flight.length_calibration.diagnostic if self.flight.length_calibration.enabled else None,'failure_guard':self.flight.failure_guard.diagnostic if self.profile.get('failure_guard') else None,'abort_reason':self.flight.abort_reason,'mission_completed':self.flight.completed_mission,'landed':self.flight.finished,
                 'events':self.flight.events,'ticks':self.ticks,'solver_results':self.solver_results,'solver_rejections':self.solver_rejections,
                 'full_calculation_ms_p50_p99_max':np.percentile(self.full_times,[50,99,100]).tolist() if self.full_times else [],
                 'preparation_ms_p50_p99_max':np.percentile(self.estimation_times,[50,99,100]).tolist() if self.estimation_times else [],
                 'feedback':'Synchronized calibrated aircraft and payload TF plus command-derived actuator observer; no MuJoCo state/force input',
                 'prediction_delay_s':self.profile['output_delay_s'],'delay_scope':'configured external output delay, not a measured transport latency',
                 'cable_engagement_shadow':shadow_receipt,'shutdown_acknowledged':self.sender.shutdown_acknowledged,'mocap_rejections':self.mocap.rejected,'last_measurement_error':self.last_measurement_error}
        (self.output/'receipt.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
        if self.gc_was_enabled:gc.enable()


def main(args=None):
    rclpy.init(args=args,signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node=None;executor=SingleThreadedExecutor();interrupts=0
    try:
        node=CableSequenceNode();executor.add_node(node);node.start_control_thread()
        def interrupt(signum,frame):
            nonlocal interrupts
            interrupts+=1
            if signum==signal.SIGINT and interrupts==1 and node.phase=='active':node.requested_landing='Operator requested landing'
            else:node.stop('Emergency operator termination')
        signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
        while not node.finished.is_set():executor.spin_once(timeout_sec=.005)
    finally:
        if node is not None:node.close();node.destroy_node()
        executor.shutdown(timeout_sec=2.);rclpy.try_shutdown()
    if node is not None and node.fatal:raise SystemExit(node.fatal)


if __name__=='__main__':main()
