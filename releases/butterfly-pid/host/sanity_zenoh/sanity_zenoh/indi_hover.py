"""Bounded first-hover workflow. No hardware action without --bench-check/--fly."""
from sanity_zenoh.standalone_paths import load_json
import argparse
from dataclasses import asdict
import importlib.util
import json
import math
from pathlib import Path
import queue
import signal
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
from sanity_zenoh.indi_hover_mission import HoverLimits, IndiHoverMission
from sanity_zenoh.receiver_health import receiver_readiness

def load_control(loco_root):
    path=Path(loco_root)/'experiments/sanity_ctbr_sim2sim/takeoff_controller.py'
    spec=importlib.util.spec_from_file_location('sanity_indi_hover_takeoff',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

def validate_logging_profile(config,limits):
    """INDI plus observation traffic requires its own explicit flight profile."""
    timing=config.get('command_timing',{})
    if not isinstance(timing,dict) or type(timing.get('enabled',False)) is not bool:
        raise ValueError('command_timing.enabled must be boolean')
    backend=config.get('controller_backend','reference_command_force')
    if backend not in ('reference_command_force','paper_measured_rpm_ctbr_compatibility'):
        raise ValueError('Unknown explicitly selected controller backend')
    full=config.get('flight_profile') in ('indi_full_flight','indi_mpc_benchmark')
    if config.get('ground_support_transition',False):
        raise ValueError('Experimental ground trim support transition was rejected for flight; use archived research source only')
    if config.get('inner_controller')=='pid' and config.get('ground_indi',{}).get('start_climb_on_ack',False):
        raise ValueError('Immediate climb on acknowledgement is an explicit INDI-only experiment')
    if config.get('flight_profile') not in (None,'indi_full_flight','indi_mpc_benchmark'):
        raise ValueError('Unknown flight_profile')
    if full and limits.pid_only:
        raise ValueError('Full INDI flight cannot use pid_only limits')
    logging=config.get('estimator_logging')
    if not logging:return
    if type(logging.get('require_initialized',True)) is not bool:
        raise ValueError('estimator_logging.require_initialized must be boolean')
    profile=logging.get('flight_profile')
    if full:
        if profile!=config.get('flight_profile'):
            raise ValueError('Full INDI logging requires its explicit flight profile')
    elif limits.pid_only:
        if profile not in (None,'pid_hover'):
            raise ValueError('PID estimator logging requires the PID hover profile')
    elif profile!='indi_hover':
        raise ValueError('INDI estimator logging requires explicit flight_profile=indi_hover')

def controller_permission_matches(limits,control):
    """Use identical permission admission for disabled bench and flight startup."""
    return control is not None and control.get('permission') is (not limits.pid_only)

def controller_feedback_from_diagnostics(health):
    """Bind control and its age to the same locked diagnostics snapshot.

    The receiver may advance its mutable receipt timestamp after tick time was
    captured. Reading that timestamp again would give this snapshot a negative
    age despite healthy feedback.
    """
    age_ms=health.get('host_receipt_age_ms')
    age_s=age_ms*.001 if isinstance(age_ms,(int,float)) and not isinstance(age_ms,bool) else math.inf
    return (health.get('snapshot') if health.get('fresh') else None),age_s

def observation_logging_ready(snapshot,require_initialized=True,expected_epoch=None):
    """Recording availability; estimator quality is a separate logged result."""
    if require_initialized:return bool(snapshot.get('ready'))
    age=snapshot.get('state_record_age_s');latest=snapshot.get('latest')
    return (latest is not None and (expected_epoch is None or latest.get('epoch')==expected_epoch)
            and age is not None and math.isfinite(age) and 0<=age<.25
            and snapshot.get('records',0)>0 and not snapshot.get('dropped') and not snapshot.get('error'))

def observation_admitted(now,start,ready_since,*,require_initialized=True):
    # An estimator used for admission retains its original initialization dwell.
    # Recording-only diagnostics need a fresh continuous stream, not ESKF convergence.
    minimum_start,ready_dwell=(8.,3.) if require_initialized else (2.,1.)
    return now-start>=minimum_start and ready_since is not None and now-ready_since>=ready_dwell

def disabled_bench_admitted(now,start,good_since,minimum_seconds,observation_admission):
    # Receiver/FC health and logging warm up concurrently. Each must pass its
    # own continuous dwell; do not wait for logging before starting the FC dwell.
    return (good_since is not None and now-good_since>=2.
            and now-start>=minimum_seconds and observation_admission)

def make_mission(config,loco_root):
    limits_class=HoverLimits
    if config.get('flight_profile')=='indi_mpc_benchmark':
        from sanity_zenoh.indi_mpc_benchmark import BenchmarkHoverLimits
        limits_class=BenchmarkHoverLimits
    limits=limits_class(**config['limits']);validate_logging_profile(config,limits)
    full=config.get('flight_profile') in ('indi_full_flight','indi_mpc_benchmark')
    if any(key in config for key in ('mpc','benchmark_envelope','inner_controller','diagnostic')) and config.get('flight_profile')!='indi_mpc_benchmark':
        raise ValueError('MPC benchmark settings require the explicit benchmark flight profile')
    if 'trajectory' in config and not full:
        raise ValueError('Explicit trajectory requires the full INDI flight profile')
    if 'liftoff_options' in config and not full:
        raise ValueError('Explicit liftoff options require the full INDI flight profile')
    if 'spool_leveling' in config:
        if not full:
            raise ValueError('spool_leveling requires the full INDI flight profile')
        if type(config['spool_leveling']) is not bool:
            raise ValueError('spool_leveling must be boolean')
    module=load_control(loco_root)
    pd=module.PositionPD(limits.mass_kg,position_gains=(1.5,1.5,3.),velocity_gains=(1.5,1.5,2.5),
        attitude_gain=3.,integral_gains=(.8,.8,1.2),integral_acceleration_limits=(.25,.25,1.5),liftoff_options=config.get('liftoff_options'))
    if full:
        from sanity_zenoh.full_indi_mission import FullIndiMission,GroundIndiLimits
        if config.get('flight_profile')=='indi_mpc_benchmark':
            from sanity_zenoh.indi_mpc_benchmark import MpcBenchmarkMission
            if 'diagnostic' in config:
                if 'trajectory' in config:raise ValueError('Diagnostics cannot also select a moving trajectory')
                from sanity_zenoh.indi_diagnostics import DiagnosticMission
                return DiagnosticMission(limits,pd,module.TakeoffReference,GroundIndiLimits(**config.get('ground_indi',{})),
                    spool_leveling=config.get('spool_leveling',True),loco_root=loco_root,mpc_settings=config.get('mpc'),
                    benchmark_envelope=config.get('benchmark_envelope'),inner_controller=config.get('inner_controller','indi'),diagnostic_options=config['diagnostic'])
            trajectory=dict(config.get('trajectory',{}))
            kind=trajectory.pop('kind',None)
            if kind!={'lemniscate':'paired_mpc_gerono','circle':'paired_mpc_circle'}.get(trajectory.get('pattern','lemniscate')):raise ValueError('Explicit paired MPC pattern required')
            return MpcBenchmarkMission(limits,pd,module.TakeoffReference,GroundIndiLimits(**config.get('ground_indi',{})),spool_leveling=config.get('spool_leveling',True),
                trajectory_options=trajectory,loco_root=loco_root,mpc_settings=config.get('mpc'),benchmark_envelope=config.get('benchmark_envelope'),inner_controller=config.get('inner_controller','indi'))
        if 'trajectory' in config:
            from sanity_zenoh.indi_lemniscate import LemniscateIndiMission
            if not isinstance(config['trajectory'],dict) or config['trajectory'].get('kind')!='angular_lemniscate':
                raise ValueError('Explicit angular_lemniscate trajectory required')
            options=dict(config['trajectory']);options.pop('kind')
            try:
                return LemniscateIndiMission(limits,pd,module.TakeoffReference,GroundIndiLimits(**config.get('ground_indi',{})),spool_leveling=config.get('spool_leveling',True),trajectory_options=options)
            except TypeError as exc:
                raise ValueError('Unknown or invalid lemniscate setting') from exc
        return FullIndiMission(limits,pd,module.TakeoffReference,GroundIndiLimits(**config.get('ground_indi',{})),spool_leveling=config.get('spool_leveling',True))
    return IndiHoverMission(limits,pd,module.TakeoffReference)

def mission_plan(mission):
    if hasattr(mission,'diagnostic_options'):
        return [f'guarded ground native {mission.inner_controller.upper()} acknowledgement','unchanged PD1m takeoff',
                'measured stable PD hover',*(['fresh stationary MPC admission and2s blend'] if mission.worker_required else []),
                f'{mission.diagnostic_options.test} nominal{mission.diagnostic_options.duration_s:g}s',
                'at least2s final hold and measured settling',f'bounded PD landing with {mission.inner_controller.upper()} inner','disable/disarm']
    if getattr(mission,'mpc_benchmark',False):
        return [f'guarded ground native {mission.inner_controller.upper()} acknowledgement','unchanged PD1m takeoff','stable hover','PD transfer to path start',
                'fresh MPC admission and2s PD-to-MPC blend',f'{mission.trajectory_options.laps} {mission.trajectory_options.pattern} {mission.trajectory_options.schedule} MPC laps',
                'MPC return and settled hold',f'bounded PD landing with {mission.inner_controller.upper()} inner','disable/disarm']
    if getattr(mission,'trajectory_kind',None)=='angular_lemniscate':
        return ['arm in PID with request low','ground INDI request and full authority acknowledgement','INDI takeoff to1m',
                'stable hover entry','smooth transfer to configured center',
                'one speed-ramped lemniscate lap' if mission.trajectory_options.laps==1 else f'{mission.trajectory_options.laps} continuous speed-ramped lemniscate laps',
                'smooth return home','hold2s','INDI landing','motor disable']
    if getattr(mission,'full_indi',False):
        return ['arm in PID with request low','bounded ground spinup/RPM warmup','ground INDI request and actual mode/weight acknowledgement',
                'INDI takeoff','INDI hover','INDI landing','ground motor disable']
    return ['PID takeoff','PID hover with estimator logging','PD landing'] if mission.limits.pid_only else ['PID takeoff','settled PID hover','acknowledged INDI hover','acknowledged PID return','PD landing']

def simulate(config,loco_root, *,mass_scale=1.,fault=None,initial_xy=(0.,0.)):
    """Approximate host/rigid-body test; no ROS, network, device or motor access."""
    mission=make_mission(config,loco_root);limits=mission.limits;dt=.02
    initial_xy=np.asarray(initial_xy,dtype=float)
    if initial_xy.shape!=(2,) or not np.isfinite(initial_xy).all():raise ValueError('Finite launch x/y required')
    state=np.zeros(13);state[:2]=initial_xy;state[2]=limits.ground_m;state[3]=1.
    weight=0.;actual_thrust=0.;armed=False;enabled_since=None;records=[];requested=False
    full=getattr(mission,'full_indi',False);collective=None;last_rc=1000
    if full:
        from sanity_zenoh.firmware_collective import FirmwareCollective
        profile=Path(config['collective_profile'])
        if not profile.is_absolute():profile=Path(__file__).resolve().parents[1]/'config'/profile
        collective=FirmwareCollective(profile);collective.set_voltage(8.)
    control={'fc_uid_msp':limits.expected_uid,'permission':not limits.pid_only,'supported':True,'request_aux':3,
        'mode':'pid','indi_weight':0.,'eligible':True,'armed':False,'fault_latched':False,'fault':'ok','control_period_us':float(limits.expected_control_period_us),'rpm_mask':15}
    for number in range(int((limits.maximum_run_s+1)/dt)):
        now=number*dt;control['armed']=armed;control['indi_weight']=weight
        native_idle=full and armed and last_rc<1100
        if full and fault=='idle_after_touchdown' and mission.phase=='landing' and mission.landing.touch_time is not None:last_rc=1000;native_idle=True
        if full and fault=='idle_before_touchdown' and mission.phase=='indi_hover':last_rc=1000;native_idle=True
        control['mode']='pid' if weight==0 else 'indi' if weight==1 else 'to_indi' if requested and not native_idle else 'to_pid'
        if full:
            control.update(requested=requested,reason='idle' if native_idle else 'ok',eligible=not native_idle,sim_rc_throttle=last_rc)
        if fault=='entry_denied':control['eligible']=False
        if fault=='permission_missing':control['permission']=False
        mocap_age=.1 if fault=='mocap_loss' and mission.phase==('pid_hover' if limits.pid_only else 'indi_hover') else 0.
        if fault=='rpm_loss' and mission.phase=='indi_hover':
            control.update(fault_latched=True,fault='rpm');weight=max(0,weight-dt/.2)
        if fault=='indi_authority_loss' and mission.phase=='indi_hover':
            control.update(mode='pid',indi_weight=0.);weight=0.
        output=mission.tick(now,state,control,mocap_age_s=mocap_age)
        row={'time_s':now,'state':state.tolist(),'controller':dict(control),'output':output,
             'outer_controller':mission.diagnostics(output)}
        if full:row['mission_diagnostics']={'touchdown_latched':mission.landing is not None and mission.landing.touch_time is not None,'indi_confirmed_at_s':mission.indi_confirmed_at}
        records.append(row)
        if output['done']:break
        if output['enabled']:
            if enabled_since is None:enabled_since=now
            if now-enabled_since>=.1:armed=True
        else:enabled_since=None;armed=False
        requested=output['indi_requested'] and not control['fault_latched']
        if fault=='engagement_unacknowledged':requested=False
        if full:last_rc=collective.convert(config['device'],output['enabled'],output['thrust_N'],output['rates_rad_s']).throttle
        target=requested and (not full or last_rc>=1100)
        weight=float(np.clip(weight+(dt/.2 if target else -dt/.2),0,1))
        command=np.asarray(output['rates_rad_s'])
        state[10:13]+=(dt/(dt+.03+.07*weight))*(command-state[10:13])
        rotation=Rotation.from_quat(state[[4,5,6,3]])*Rotation.from_rotvec(state[10:13]*dt)
        state[3:7]=rotation.as_quat()[[3,0,1,2]]
        actual_thrust+=dt/(dt+.05)*(output['thrust_N']-actual_thrust)
        acceleration=rotation.apply([0,0,actual_thrust/(limits.mass_kg*mass_scale)])-[0,0,9.81]
        state[7:10]+=acceleration*dt;state[:3]+=state[7:10]*dt
        if state[2]<limits.ground_m:state[2]=limits.ground_m;state[9]=max(0,state[9])
    return {'completed':mission.done,'reason':mission.reason,'phases':list(dict.fromkeys(r['output']['phase'] for r in records)),
        'peak_height_m':max(r['state'][2] for r in records),'records':records,
        'scope':'Approximate host mission/rigid-body simulation; not F405 or flight qualification'}

class Recorder:
    def __init__(self,path):
        self.queue=queue.Queue(maxsize=10000);self.dropped=0;self.error=None;self.stopping=False
        self.file=Path(path).open('x');self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def record(self,event):
        try:self.queue.put_nowait({'host_monotonic_ns':time.monotonic_ns(),**event})
        except queue.Full:self.dropped+=1
    def run(self):
        try:
            while not self.stopping or not self.queue.empty():
                try:event=self.queue.get(timeout=.1)
                except queue.Empty:continue
                self.file.write(json.dumps(event,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x),allow_nan=False)+'\n')
        except Exception as exc:self.error=str(exc)
    def close(self):
        self.stopping=True;self.thread.join(timeout=3)
        if self.thread.is_alive():self.error=self.error or 'Recorder flush timed out'
        else:self.file.close()

def start_mpc_runtime(mission,output,event=None,*,disabled_only=False):
    """Only explicitly selected MPC tests own a worker; pure factories never do."""
    if disabled_only or not getattr(mission,'worker_required',False):return None
    from sanity_zenoh.indi_mpc_benchmark import MpcWorkerRuntime
    return MpcWorkerRuntime(mission,output,event)

def close_hover_io(node, recorder):
    """Disable before cleanup; release child entities before their shared session."""
    sender=getattr(node,'sender',None)
    actions=[]
    if sender is not None:actions.append(('disable',lambda:sender.ctbr(False,0,(0,0,0))))
    for name,method in [('mpc_runtime','close'),('mocap_stream','close'),('observation_sub','undeclare'),('sender','close'),('observations','close')]:
        entity=getattr(node,name,None)
        if entity is not None:actions.append((name,getattr(entity,method)))
    errors=[]
    for name,action in actions:
        try:action()
        except Exception as exc:
            errors.append(name+': '+str(exc));recorder.record({'kind':'cleanup_error','component':name,'error':str(exc)})
    return errors

def run_ros(args,config):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    from sanity_zenoh.client import SanitySender
    from sanity_zenoh.firmware_collective import FirmwareCollective
    from sanity_zenoh.mocap import MocapEstimator,state_age
    from sanity_zenoh.mocap_diagnostics import MocapDiagnostics,update_with_diagnostics,retain_fresh_packet_after_outlier
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    (output/'configuration.json').write_text(json.dumps(config,indent=2)+'\n')
    recorder=Recorder(output/'events.jsonl')
    class HoverNode(Node):
        def __init__(self):
            super().__init__('sanity_indi_hover')
            self.finished=False;self.packet=None;self.last_error=None;self.mission=make_mission(config,args.loco_root)
            self.mocap_diagnostics=MocapDiagnostics()
            self.start=time.monotonic();self.good_bench_since=None;self.bench_result=None
            self.sender=None;self.observations=None;self.observation_sub=None;self.mocap_stream=None;self.mpc_runtime=None
            self.logging_lost_since=None;self.logging_ready_since=None;self.paper_compatibility_admitted=False
            m=config['mocap']
            self.mocap=MocapEstimator(m['parent_frame'],m['tracked_frame'],
                world_quat=m['world_from_map_xyzw'],marker_to_base_xyz=m['marker_to_base_xyz'],
                marker_to_base_quat=m['marker_to_base_xyzw'],frames_confirmed=args.frames_confirmed)
            try:
                self.mpc_runtime=start_mpc_runtime(self.mission,output/'mpc',recorder.record,disabled_only=args.bench_check)
                profile=Path(config['collective_profile'])
                if not profile.is_absolute():profile=args.config.parent/profile
                self.sender=SanitySender(config['device'],listen=config['listen'],
                    ctbr_mapping=FirmwareCollective(profile,high_rate=bool(config.get('high_speed_wire',False)),selected_high_rate=bool(config.get('high_speed_wire',False))),physical_units=True,controller_selection=True,
                    diagnostics=bool(config.get('command_timing',{}).get('enabled',False)),
                    disabled_only=args.bench_check,
                    telemetry_listen=config.get('telemetry_listen') or None,on_event=recorder.record)
                self.sender.ctbr(False,0,(0,0,0))
                if config.get('estimator_logging'):
                    from sanity_zenoh.hover_observations import HoverObservations
                    from sanity_zenoh.mocap_stream import MocapStreamPublisher
                    logging=config['estimator_logging']
                    self.observations=HoverObservations(output/'observations.slog',dict(
                        device=config['device'],scope=(f'{self.mission.diagnostic_options.metadata["outer"].upper()} control diagnostic native {self.mission.inner_controller.upper()} inner' if hasattr(self.mission,'diagnostic_options') else f'paired MPC benchmark native {self.mission.inner_controller.upper()} inner' if getattr(self.mission,'mpc_benchmark',False) else 'ground-acknowledged INDI takeoff/hover/landing' if getattr(self.mission,'full_indi',False) else 'PID hover' if self.mission.limits.pid_only else 'PID/INDI hover')+'; onboard ESKF logs only; desktop TF controls aircraft',
                        created_unix_ns=time.time_ns(),configuration=config))
                    self.observation_sub=self.sender.session.declare_subscriber(
                        f"sanity/{config['device']:02d}/observation/records",self.observations.receive)
                    self.mocap_stream=MocapStreamPublisher(self.sender.session,config['device'],
                        maximum_rate_hz=logging['maximum_rate_hz'],clock_rate_hz=logging['clock_rate_hz'],
                        clock_endpoint=logging['clock_endpoint'],on_event=recorder.record,on_record=recorder.record)
                self.subscription=self.create_subscription(TFMessage,m.get('topic','/tf'),self.on_tf,
                    QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
                self.timer=self.create_timer(.02,self.tick)
            except BaseException:
                close_hover_io(self,recorder)
                super().destroy_node()
                raise

        def on_tf(self,message):
            now_ns=self.get_clock().now().nanoseconds
            for item in message.transforms:
                if (item.header.frame_id,item.child_frame_id)!=(self.mocap.parent,self.mocap.child):continue
                t=item.transform;p=t.translation;q=t.rotation
                packet,error=update_with_diagnostics(self.mocap,self.mocap_diagnostics,recorder.record,
                    parent=item.header.frame_id,child=item.child_frame_id,
                    stamp_ns=item.header.stamp.sec*1000000000+item.header.stamp.nanosec,
                    position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],
                    ros_now_ns=now_ns,monotonic_ns=time.monotonic_ns())
                if error is not None:
                    # An outlier is not an accepted measurement. Keep only
                    # an already-valid packet within its original age limit;
                    # tick() still expires it normally at 40 ms.
                    self.packet=retain_fresh_packet_after_outlier(self.packet,error,now_ns)
                    self.last_error=str(error)
                    if self.packet is not None:
                        recorder.record(dict(kind='mocap_outlier_retained_previous',
                            source_stamp_ns=self.packet['stamp_ns'],
                            source_age_s=state_age(self.packet,now_ns)[1],
                            rejected_source_stamp_ns=item.header.stamp.sec*1000000000+item.header.stamp.nanosec,
                            reason=str(error)))
                    continue
                try:
                    if packet is not None:
                        self.packet=packet
                        self.last_error=None
                        if self.mocap_stream is not None:self.mocap_stream.send_estimate(packet,self.get_clock().now().nanoseconds)
                except ValueError as exc:
                    self.packet=None;self.last_error=str(exc)
                    recorder.record({'kind':'mocap_forward_error','reason':str(exc),
                        'source_stamp_ns':packet['stamp_ns']})
        def tick(self):
            now=time.monotonic();health=self.sender.controller_diagnostics(self.mission.limits.controller_status_max_age_s);control,controller_age_s=controller_feedback_from_diagnostics(health)
            # Bind status and receipt age atomically; a receive callback can
            # otherwise replace the timestamp after this tick's clock snapshot.
            with self.sender.lock:
                receiver=self.sender.receiver_status()
                receiver_age_s=time.monotonic()-self.sender.status_time
            claim=receiver is not None and receiver_age_s<.15 and receiver.session==self.sender.session_id and not receiver.failsafe
            paper=self.sender.paper_status(self.mission.limits.controller_status_max_age_s) if config.get('controller_backend')=='paper_measured_rpm_ctbr_compatibility' else None
            if paper is not None:self.paper_compatibility_admitted=paper['native_capable'] and paper['backend']==1 and paper['control_period_us']==1000
            paper_admitted=config.get('controller_backend')!='paper_measured_rpm_ctbr_compatibility' or (self.paper_compatibility_admitted and (paper is not None or self.mission.phase!='waiting') and (not args.bench_check or paper is not None))
            observation=self.observations.snapshot(expected_epoch=receiver.epoch if receiver else 0) if self.observations else None
            observation_admission=True;logging_stream_ready=True
            if observation is not None:
                logging_stream_ready=observation_logging_ready(observation,config['estimator_logging'].get('require_initialized',True),receiver.epoch if receiver else None)
                self.logging_ready_since=(now if self.logging_ready_since is None else self.logging_ready_since) if logging_stream_ready else None
                observation_admission=observation_admitted(now,self.start,self.logging_ready_since,
                    require_initialized=config['estimator_logging'].get('require_initialized',True))
            if args.bench_check:
                if self.mission.done:
                    self.bench_result={'success':False,'reason':self.mission.reason,'disabled_only':True};self.finished=True;return
                self.sender.ctbr(False,0,(0,0,0))
                period=control.get('control_period_us') if control else None
                good=claim and receiver.fc_ready and not receiver.enabled and control is not None and control['fc_uid_msp']==config['limits']['expected_uid'] and not control['armed'] and control['mode']=='pid' and control['indi_weight']==0 and not control['requested'] and not control['fault_latched'] and control['supported'] and self.mission.limits.control_period_matches(period)
                good=good and controller_permission_matches(self.mission.limits,control) and paper_admitted
                if config.get('high_speed_wire'):good=good and receiver.indi_high_rate_supported
                self.good_bench_since=(now if self.good_bench_since is None else self.good_bench_since) if good else None
                minimum_bench_s=getattr(args,'minimum_bench_seconds',0.)
                if disabled_bench_admitted(now,self.start,self.good_bench_since,minimum_bench_s,observation_admission):
                    self.bench_result={'success':True,'controller':control,'controller_diagnostics':health,'receiver_device':receiver.device,'disabled_only':True,'estimator_logging':observation};self.finished=True
                elif now-self.start>max(30.,minimum_bench_s+10.):
                    self.bench_result={'success':False,'reason':'Disarmed controller/receiver readiness did not remain valid for2s','controller':control,
                        'controller_diagnostics':health,'receiver_status':asdict(receiver) if receiver else None,'disabled_only':True,'estimator_logging':observation};self.finished=True
                return
            state_error=None if self.packet is not None else 'no_current_packet'
            state_now_ns=self.get_clock().now().nanoseconds
            try:state,age=state_age(self.packet,state_now_ns) if self.packet else (None,math.inf)
            except ValueError as exc:state,age=None,math.inf;state_error=str(exc)
            ground_phases=('waiting','arming',*getattr(self.mission,'ground_phases',()))
            airborne=self.mission.ever_armed and self.mission.phase not in ground_phases
            pose_fresh=state is not None and math.isfinite(age) and 0<=age<.04
            receiver_health=receiver_readiness(receiver,age_s=receiver_age_s,
                session=self.sender.session_id,mode=self.sender.mode,controller=control,
                airborne=airborne,controller_fresh=health['fresh'],pose_fresh=pose_fresh)
            ready=receiver_health['ready'] and paper_admitted
            if receiver_health['landing_requested']:
                self.mission.request_landing('receiver_status_delayed')
            if not paper_admitted and self.mission.phase!='waiting':self.mission.request_landing('paper_backend_capability_unavailable')
            if observation is not None:
                if self.mission.phase=='waiting':ready=ready and observation_admission
                else:
                    self.logging_lost_since=(now if self.logging_lost_since is None else self.logging_lost_since) if not logging_stream_ready else None
                    if observation['error'] or observation['dropped'] or (self.logging_lost_since is not None and now-self.logging_lost_since>1.):
                        self.mission.request_landing('estimator_logging_unavailable')
            if recorder.error or recorder.dropped:self.mission.request_landing('recording_failure')
            if self.mpc_runtime is not None:self.mpc_runtime.prepare(self.mission,now,state,age)
            result=self.mission.tick(now,state,control,mocap_age_s=age,
                controller_age_s=controller_age_s,receiver_ready=ready)
            try:self.sender.ctbr(result['enabled'],result['thrust_N'],result['rates_rad_s'],valid_for_s=.25,indi_requested=result['indi_requested'])
            except Exception as exc:
                self.mission.stop('sender_interlock: '+str(exc));self.sender.ctbr(False,0,(0,0,0));result=self.mission.output()
            if self.mpc_runtime is not None:self.mpc_runtime.commit(result)
            recorder.record({'kind':'hover_control','control_monotonic_s':now,'state':state,'mocap_age_s':age if math.isfinite(age) else None,
                'controller':control,'paper_backend':paper,'command':result,'reference':self.mission.last_reference,'estimator_logging':observation,
                'outer_controller':self.mission.diagnostics(result),
                'controller_status_fresh':health['fresh'],'controller_status_reason':health['reason'],
                'controller_age_s':controller_age_s if math.isfinite(controller_age_s) else None,
                'controller_host_age_ms':health['host_receipt_age_ms'],'controller_sample_age_ms':health['esp_sample_age_ms'],
                'receiver_status_age_ms':receiver_health['status_age_ms'],
                'receiver_health':receiver_health,
                'receiver_fc_age_ms':receiver.fc_age_ms if receiver is not None else None,'receiver_ready':ready,
                'mocap_diagnostics':self.mocap_diagnostics.snapshot(ros_now_ns=state_now_ns,
                    monotonic_ns=time.monotonic_ns(),state_error=state_error,callback_error=self.last_error),
                'estimator_record_stream_ready':logging_stream_ready})
            if result['done']:self.finished=True
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=None;interrupts=0
    def interrupt(_number,_frame):
        nonlocal interrupts
        interrupts+=1
        if node is not None:
            if interrupts==1:node.mission.request_landing('operator_landing')
            else:node.mission.stop('operator_emergency_stop')
    previous={number:signal.signal(number,interrupt) for number in (signal.SIGINT,signal.SIGTERM)}
    try:
        node=HoverNode()
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.05)
    finally:
        for number,handler in previous.items():signal.signal(number,handler)
        if node is not None:
            cleanup_errors=close_hover_io(node,recorder)
            recorder.close()
            result=(node.bench_result or {'success':False,'reason':'runner_exception'}) if args.bench_check else {'success':node.mission.done and node.mission.reason=='test_complete','reason':node.mission.reason or 'runner_exception','phase':node.mission.phase}
            result.update(recorder_dropped=recorder.dropped,recorder_error=recorder.error,
                shutdown_acknowledged=node.sender.shutdown_acknowledged,cleanup_errors=cleanup_errors,qualification='No automatic declaration of flight qualification')
            if node.observations is not None:
                result['estimator_logging']=node.observations.snapshot()
                try:
                    from sanity_zenoh.estimator_tools import read_log
                    _,observed=read_log(output/'observations.slog')
                    result['observation_records_validated']=len(observed)
                    result['observation_crc_validation']=True
                except Exception as exc:
                    result['observation_crc_validation']=False
                    result['observation_validation_error']=str(exc)
                if not result['observation_crc_validation'] or result['estimator_logging']['error'] or result['estimator_logging']['dropped']:
                    result['success']=False
            if recorder.error or recorder.dropped or cleanup_errors or not node.sender.shutdown_acknowledged:result['success']=False
            (output/'result.json').write_text(json.dumps(result,indent=2)+'\n');node.destroy_node()
        recorder.close();rclpy.shutdown()
    print(output/'result.json')
    if not result['success']:raise SystemExit(1)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    actions=p.add_mutually_exclusive_group(required=True)
    actions.add_argument('--plan-only',action='store_true');actions.add_argument('--simulate',action='store_true')
    actions.add_argument('--bench-check',action='store_true');actions.add_argument('--fly',action='store_true')
    p.add_argument('--config',type=Path,required=True);p.add_argument('--loco-root',type=Path,required=True)
    p.add_argument('--output',type=Path);p.add_argument('--frames-confirmed',action='store_true')
    p.add_argument('--minimum-bench-seconds',type=float,default=0.,help='Keep disabled-only combined capture running before accepting readiness (0..120 seconds)')
    args=p.parse_args();config=load_json(args.config);mission=make_mission(config,args.loco_root)
    if not math.isfinite(args.minimum_bench_seconds) or not 0<=args.minimum_bench_seconds<=120:p.error('Minimum bench seconds must be finite and in 0..120')
    if args.minimum_bench_seconds and not args.bench_check:p.error('--minimum-bench-seconds requires --bench-check')
    if config.get('estimator_logging') and (args.fly or args.bench_check) and not args.frames_confirmed:p.error('Estimator forwarding requires --frames-confirmed')
    if type(config.get('device')) is not int or not 1<=config['device']<=255: p.error('Explicit receiver device1..255 required')
    if args.plan_only:
        plan={'limits':asdict(mission.limits),'device':config['device'],'mocap':config['mocap'],
            'phases':mission_plan(mission),
            'estimator_logging':config.get('estimator_logging'),'onboard_estimator_controls_aircraft':False,
            'command_timing':config.get('command_timing',{'enabled':False}),
            'outer_controller_diagnostics':{'version':1,'logged_each_control_tick':True},
            'authority_request':'SZC4 -> ESP AUX3 -> guarded FC handover','network_or_hardware_access':False}
        if getattr(mission,'full_indi',False):plan['ground_indi']=asdict(mission.ground)
        if hasattr(mission,'diagnostic_options'):
            plan.update(inner_controller=mission.inner_controller,diagnostic=mission.diagnostic_options.metadata,worker_required=mission.worker_required,outer_controller=mission.diagnostic_options.metadata['outer'],benchmark_envelope=asdict(mission.benchmark_envelope) if mission.worker_required else None)
        elif getattr(mission,'mpc_benchmark',False):
            plan.update(inner_controller=mission.inner_controller,inner_selection='configured before arming; native FC status must acknowledge; no midflight mode selection',trajectory=asdict(mission.trajectory_options),benchmark_envelope=asdict(mission.benchmark_envelope),outer_controller='isolated acados MPC; worker starts only in live/explicit validation runtime')
        print(json.dumps(plan,indent=2));return
    if args.simulate:
        if getattr(mission,'mpc_benchmark',False):p.error('MPC benchmark simulation requires the isolated production-worker validation harness')
        result=simulate(config,args.loco_root)
        if args.output:args.output.mkdir(parents=True,exist_ok=False);(args.output/'simulation.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='records'},indent=2));return
    if not args.output:p.error('A new --output directory is required')
    if args.fly and not args.frames_confirmed:p.error('--fly requires explicit --frames-confirmed')
    run_ros(args,config)

if __name__=='__main__':main()
