import gc
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from .plate_configuration import PlateDeploymentConfig
from .plate_state import PlateMocap
from .plate_process import PlateWorker,admit_solution
from .plate_transport import PlateCommandGroup
from .recording import EventRecorder
from .cable_health import VoltageGuard
from .cable_process import pin_cpu_affinity,move_background_threads


class PlateNode(Node):
    def __init__(self):
        super().__init__('sanity_plate')
        defaults=dict(workspace='',profile='',mission_profile='',output='',worker_python='',enable_output=False,no_transport=False,takeoff_land_only=False)
        for key,value in defaults.items():self.declare_parameter(key,value)
        self.values={key:self.get_parameter(key).value for key in defaults};v=self.values
        root=Path(v['workspace']).resolve();sys.path.insert(0,str(root/'loco_mjx'))
        from experiments.sanity_plate_transport.flight_model import PlateFlightModel
        from experiments.sanity_plate_transport.flight_reference import FlightReference
        from experiments.sanity_plate_transport.deployment import PlateSupervisor
        self.config=PlateDeploymentConfig(v['profile'],root,v['mission_profile'] or None)
        pin_cpu_affinity(self.config.affinity.get('control',[]))
        if v['takeoff_land_only']:self.config.settings['takeoff_land_only']=True
        if v['enable_output']:
            self.config.require_hardware()
            if v['no_transport']:raise ValueError('Enabled aircraft require command transport')
        self.output=Path(v['output']).resolve();self.output.mkdir(parents=True,exist_ok=False)
        (self.output/'profile.json').write_text(json.dumps(self.config.data,indent=2))
        (self.output/'mission.json').write_text(json.dumps(self.config.physical.dictionary(),indent=2))
        self.events=EventRecorder(self.output/'events.jsonl');self.log=EventRecorder(self.output/'control.jsonl')
        self.raw=EventRecorder(self.output/'tf.jsonl');self.commands=EventRecorder(self.output/'commands.jsonl')
        self.get_logger().info('Preparing pose observer and cable references; aircraft output disabled')
        self.model=PlateFlightModel(self.config.physical,self.config.models)
        from experiments.sanity_plate_transport.codegen import compile_model
        compile_model(self.model,root/'.cache/sanity_plate_observer',names=('dynamics','continuous_jacobian','geometry','length'))
        self.reference=FlightReference(self.model);self.reference.prepare()
        self.flight=PlateSupervisor(self.model,self.reference,self.config.settings)
        self.events({'kind':'pd_command_policy','attitude_gain':self.flight.pd[0].attitude_gain,'body_rate_slew_limited':False,
            'collective_slew_N_s':{'takeoff_pickup_lift':1.,'other':2.},'input_lower':self.model.input_lower.tolist(),'input_upper':self.model.input_upper.tolist()})
        self.get_logger().info(f'PD attitude gain={self.flight.pd[0].attitude_gain:g}; bounded body rates pass through; collective ramp retained')
        self.mocap=PlateMocap(self.config,self.model);self.mocap_lock=threading.Lock()
        self.get_logger().info('PD pickup/landing: MPC skipped' if self.config.settings.get('takeoff_land_only') else 'Loading and warming plate acados MPC; aircraft output disabled')
        self.worker=None if self.config.settings.get('takeoff_land_only') else PlateWorker(self.config,self.output,v['worker_python'] or root/'.venv-sanity-passage/bin/python')
        self.events({'kind':'solver_ready','solver':self.worker.info if self.worker else None,'takeoff_land_only':bool(self.config.settings.get('takeoff_land_only'))})
        self.group=None
        try:
            if self.config.data.get('require_clock_preflight'):
                from .mission_clock import record_clock_preflight
                record_clock_preflight(self.output,required=True)
            if not v['no_transport']:self.group=PlateCommandGroup(self.config,v['enable_output'],self.events)
        except BaseException:
            if self.worker:self.worker.close()
            for recorder in [self.events,self.log,self.raw,self.commands]:recorder.close()
            self.destroy_node()
            raise
        self.stopping=threading.Event();self.phase='waiting';self.fatal=None;self.landing_requested=False
        self.pending=None;self.active=None;self.pending_slot=None;self.active_until=0.;self.next_request=0.;self.countdown=None
        self.voltage_guards=[VoltageGuard({}) for _ in self.config.drones]
        self.started=time.monotonic();self.clock_pair=None;self.last_event=0;self.last_wait=0.
        self.gc_enabled=gc.isenabled();gc.collect();gc.disable()
        self.subscription=self.create_subscription(TFMessage,self.config.mocap['topic'],self.receive,QoSProfile(depth=30,reliability=ReliabilityPolicy.BEST_EFFORT))
        move_background_threads(self.config.affinity.get('background',[]))
        if self.config.affinity.get('ros'):os.sched_setaffinity(0,set(self.config.affinity['ros']))
        self.thread=threading.Thread(target=self.control_loop,name='plate_control_50Hz',daemon=True);self.thread.start()
        self.get_logger().info('Plate deployment ready in '+('ENABLED opt-in mode' if v['enable_output'] else 'disabled shadow mode'))
        if self.config.settings.get('skip_flight_separation_check') is True:self.get_logger().warning('In-flight separation emergency stop BYPASSED; initial spacing, active repulsion, attitude and volume guards remain')


    def receive(self, message):
        now=self.get_clock().now().nanoseconds
        with self.mocap_lock:
            for tf in message.transforms:
                t,q=tf.transform.translation,tf.transform.rotation;stamp=tf.header.stamp.sec*1000000000+tf.header.stamp.nanosec
                try:
                    packet=self.mocap.update(tf.header.frame_id,tf.child_frame_id,stamp,[t.x,t.y,t.z],[q.x,q.y,q.z,q.w],now)
                    if packet is not None:self.raw({'stamp_ns':stamp,'parent':tf.header.frame_id,'child':tf.child_frame_id,'position':[t.x,t.y,t.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w],'accepted':True})
                except ValueError as error:self.raw({'stamp_ns':stamp,'child':tf.child_frame_id,'accepted':False,'error':str(error)})


    def stop(self, reason):
        self.fatal=reason;self.stopping.set()
        if self.group:self.group.stop(reason)
        self.events({'kind':'fatal','reason':reason});self.get_logger().error(reason)


    def control_loop(self):
        if self.config.affinity.get('control'):os.sched_setaffinity(0,set(self.config.affinity['control']))
        slot=time.monotonic()
        while not self.stopping.is_set():
            delay=slot-time.monotonic()
            if delay>0 and self.stopping.wait(delay):break
            now=time.monotonic();ros_now=self.get_clock().now().nanoseconds
            try:
                if self.clock_pair:
                    old_ros,old_wall=self.clock_pair
                    if ros_now<=old_ros or abs((ros_now-old_ros)/1e9-(now-old_wall))>.02:raise ValueError('ROS/monotonic clock discontinuity')
                self.clock_pair=(ros_now,now)
                self.tick(now,ros_now)
                for recorder in [self.events,self.log,self.raw,self.commands]:
                    if recorder.error:raise RuntimeError('Plate recording failed: '+recorder.error)
            except Exception as error:
                if self.phase=='waiting' and now-self.started<self.config.timing['startup_timeout_s']:
                    self.countdown=None
                    if self.group:self.group.send(np.zeros(self.model.nu),False)
                    with self.mocap_lock:self.mocap.commit(ros_now,np.zeros(self.model.nu))
                    if now-self.last_wait>2:self.get_logger().warning(str(error));self.last_wait=now
                else:self.stop(type(error).__name__+': '+str(error))
            slot+=max(1,int((time.monotonic()-slot)/.02)+1)*.02


    def tick(self, now, ros_now):
        with self.mocap_lock:packet=self.mocap.sample(ros_now,reset_on_gap=self.phase=='waiting')
        state=packet['state'];enabled=self.values['enable_output']
        if self.phase=='waiting':
            with self.mocap_lock:self.mocap.commit(ros_now,np.zeros(self.model.nu))
            if self.group:self.group.send(np.zeros(self.model.nu),False)
            if enabled:self.group.ready(False)
            if self.countdown is None:self.countdown=now;self.get_logger().warning(f'All plate participants ready; starts in {self.config.timing["countdown_s"]:g} seconds')
            if now-self.countdown<self.config.timing['countdown_s']:return
            self.phase='enabling' if enabled else 'active'
        if self.phase=='enabling':
            self.group.send(np.zeros(self.model.nu),True)
            try:self.group.ready(True)
            except ValueError:return
            self.phase='active'
        if self.phase!='active':return
        if enabled:
            self.group.ready(True)
            for guard,sender in zip(self.voltage_guards,self.group.senders):
                reason=guard.update(sender.receiver_status().voltage_mv/1000.,ros_now)
                if reason and self.flight.started is not None:self.flight.request_landing(state,now,f'Receiver {sender.device}: '+reason)
        if self.landing_requested and self.flight.started is not None:self.flight.request_landing(state,now,'operator')
        candidate=None;solver_error=None
        if self.pending and self.pending.done():
            try:result=self.pending.result()
            except Exception as error:result={'error':str(error),'sequence':self.sequence,'host_received_s':time.monotonic()}
            if now>=self.pending_slot-.003:
                latency=result['host_received_s']-self.dispatched
                admitted,solver_error,reason=admit_solution(result,self.sequence,latency,self.pending_slot,now,self.model,self.config.timing['solver_budget_s'])
                self.events({'kind':'solver_result','sequence':result.get('sequence'),'admission':reason,'roundtrip_s':latency,'mpc':result.get('mpc'),'error':result.get('error')})
                if admitted is not None:
                    self.active=admitted;self.active_until=now+self.model.config.control_dt+.005
                self.pending=None
        if self.active is not None and now<=self.active_until:candidate=self.active
        try:selected,info=self.flight.update(state,now,candidate,solver_error)
        except ValueError as error:
            self.events({'kind':'controller_rejected_state','stamp_ns':ros_now,'source_stamp_ns':packet['stamp_ns'],
                'phase':self.flight.phase,'state':state.tolist(),'reason':str(error),'mount_frames':[d['frame'] for d in self.config.drones]})
            raise
        if self.flight.phase in ('brake','level_plate','place_plate','descend','pickup_abort','landed'):self.active=None
        if self.get_clock().now().nanoseconds-packet['stamp_ns']>=40_000_000:raise ValueError('Plate state expired before command selection')
        sent=self.group.send(selected,enabled and not self.flight.finished) if self.group else np.zeros(self.model.nu)
        apply_ns=ros_now+int(self.config.timing['output_delay_s']*1e9)
        with self.mocap_lock:self.mocap.commit(apply_ns,sent)
        self.commands({'stamp_ns':ros_now,'scheduled_apply_ns':apply_ns,'selected':selected.tolist(),'sent':sent.tolist(),'enabled':enabled and not self.flight.finished})
        self.log({'stamp_ns':ros_now,'source_stamp_ns':packet['stamp_ns'],'state':state.tolist(),'source_age_s':packet['source_age_s'],**info})
        if info['need_mpc'] and self.pending is None and now+.003>=self.next_request:
            selection=now+.04;target=ros_now+int((.04+self.config.timing['output_delay_s'])*1e9)
            with self.mocap_lock:segments=self.mocap.forecast_segments(packet['stamp_ns'],target)
            request=dict(state=state.tolist(),segments=segments,previous=sent.tolist(),offset=self.reference.offset.tolist(),yaw=self.reference.yaw,
                reference_time_s=self.flight.reference_time(selection+self.config.timing['output_delay_s']))
            if candidate is not None and np.asarray(candidate).size==3*self.model.config.drones and info['mpc_weight']>.99:request['held_force']=np.asarray(candidate).tolist()
            self.dispatched=time.monotonic();self.pending,self.sequence=self.worker.submit(request);self.pending_slot=selection;self.next_request=now+self.model.config.control_dt
            self.events({'kind':'solver_request','sequence':self.sequence,'source_stamp_ns':packet['stamp_ns'],'selection_monotonic_s':selection,'scheduled_apply_ns':target,'reference_time_s':request['reference_time_s']})
        for event in self.flight.events[self.last_event:]:self.events(event);self.get_logger().info('Phase: '+event['phase'])
        self.last_event=len(self.flight.events)
        if self.flight.finished:self.stopping.set()


    def close(self):
        self.stopping.set();self.thread.join(timeout=2)
        if self.group:self.group.close()
        if self.worker:self.worker.close()
        if self.gc_enabled:gc.enable()
        (self.output/'receipt.json').write_text(json.dumps({'fatal':self.fatal,'phase':self.flight.phase,'landed':self.flight.finished,
            'abort_reason':self.flight.reason,'enable_output':self.values['enable_output'],'hardware_calibration_deferred':not self.config.data['geometry_confirmed'],
            'solver':self.worker.info if self.worker else None},indent=2))
        for recorder in [self.events,self.log,self.raw,self.commands]:recorder.close()
        self.destroy_node()


def main():
    rclpy.init();node=None
    try:
        node=PlateNode()
        def interrupt(signum,frame):
            if node.landing_requested or node.phase!='active':node.stop('Operator emergency termination')
            else:node.landing_requested=True
        signal.signal(signal.SIGINT,interrupt)
        while rclpy.ok() and not node.stopping.is_set():rclpy.spin_once(node,timeout_sec=.02)
    finally:
        if node is not None:node.close()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
