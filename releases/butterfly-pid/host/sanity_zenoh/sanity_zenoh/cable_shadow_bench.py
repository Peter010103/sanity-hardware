"""Receive-only ROS mocap + HX711 commissioning; never creates a flight sender."""
from datetime import datetime
import json
import math
import os
import select
import sys
import time
from pathlib import Path
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from sanity_zenoh.cable_configuration import load_physical_configuration
from sanity_zenoh.cable_engagement import configuration, raw_geometry
from sanity_zenoh.cable_state import CableMocap
from sanity_zenoh.cable_strain import CableStrainShadow
from sanity_zenoh.recording import EventRecorder
from sanity_zenoh.cable_shadow_protocol import ShadowBenchProtocol


class CableShadowBench(Node):
    def __init__(self, parameter_overrides=None, context=None):
        super().__init__('sanity_cable_shadow',parameter_overrides=parameter_overrides or [],context=context)
        workspace=Path(__file__).resolve().parents[6]
        self.declare_parameter('profile',str(Path(__file__).resolve().parents[1]/'config/sanity09_hx711_hover_20261028.json'))
        self.declare_parameter('output',str(workspace/'outputs/strain_gauge'/('sanity09_shadow_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))))
        self.declare_parameter('guided_test',True)
        self.declare_parameter('cycles',3)
        self.declare_parameter('hold_s',10.)
        self.declare_parameter('release_s',10.)
        self.protocol_done=False;self.input_buffer=b'';self.stdin_fd=None;self.protocol=None
        self.started_ns=time.monotonic_ns()
        profile_path=Path(self.get_parameter('profile').value)
        self.profile=json.loads(profile_path.read_text())
        self.settings=configuration(self.profile.get('cable_engagement'))
        if self.settings['mode']!='shadow':raise ValueError('Receive-only commissioning requires shadow settings')
        self.physical=load_physical_configuration(workspace/'loco_mjx/experiments/sanity_cable_mpc',self.profile,profile_path)
        self.mocap=CableMocap(self.physical,self.profile['mocap']);self.lock=threading.RLock()
        self.output=Path(self.get_parameter('output').value);self.output.mkdir(parents=True,exist_ok=False)
        (self.output/'profile.json').write_text(json.dumps(self.profile,indent=2)+'\n')
        (self.output/'config.json').write_text(json.dumps(self.physical,indent=2)+'\n')
        self.tf_log=EventRecorder(self.output/'tf.jsonl',capacity=4096)
        self.protocol_log=EventRecorder(self.output/'bench_test_events.jsonl',capacity=4096)
        self.shadow=CableStrainShadow(self.settings,workspace/'Sanity/sanity-hardware/tools/acquisition',self.output,self.shadow_context)
        self.subscription=self.create_subscription(TFMessage,self.profile.get('tf_topic','/tf'),self.receive_tf,QoSProfile(depth=30,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.service=self.create_service(Trigger,'~/confirm_sensor_unloaded',self.confirm_unloaded)
        self.timer=self.create_timer(2.,self.report)
        if self.get_parameter('guided_test').value:
            self.stdin_fd=sys.stdin.fileno()
            self.protocol=ShadowBenchProtocol(self.get_parameter('cycles').value,self.get_parameter('hold_s').value,self.get_parameter('release_s').value,self.settings['baseline_capture_s'])
            self.protocol_timer=self.create_timer(.05,self.protocol_tick)
            print('Guided test: waiting for healthy HX711 data. Keep motors unpowered.\nThen follow Enter prompts for the load/unload trials. Classifier UNKNOWN is expected until calibrated.',flush=True)
        self.shadow.start()
        self.get_logger().info('Receive-only supervisor test: no aircraft command transport. Output: '+str(self.output))

    def receive_tf(self,message):
        now=self.get_clock().now().nanoseconds
        with self.lock:
            for tf in message.transforms:
                p,q=tf.transform.translation,tf.transform.rotation
                stamp=tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
                if tf.child_frame_id not in (self.profile['mocap']['aircraft_frame'],self.profile['mocap']['payload_frame']):continue
                record=dict(stamp_ns=stamp,arrival_ns=now,parent=tf.header.frame_id,child=tf.child_frame_id,position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w])
                try:record['accepted']=self.mocap.update(record['parent'],record['child'],stamp,record['position'],record['quaternion_xyzw'],now) is not None
                except ValueError as error:record['accepted']=False;record['reason']=str(error)
                if any(not math.isfinite(v) for key in ('position','quaternion_xyzw') for v in record[key]):
                    record['original_payload_json']=json.dumps(record)
                    for key in ('position','quaternion_xyzw'):record[key]=[v if math.isfinite(v) else None for v in record[key]]
                self.tf_log(record)

    def shadow_context(self):
        poses={}
        if not self.lock.acquire(blocking=False):return dict(valid=False,reasons=['raw_geometry_snapshot_busy']),'receive_only',False
        try:
            for name,e in self.mocap.estimators.items():
                if e.previous is None or e.count<5:continue
                stamp,position,rotation=e.previous
                latest=self.mocap.buffers[name][-1] if self.mocap.buffers[name] else {}
                poses[name]=dict(stamp_ns=int(stamp),position=position.tolist(),quaternion_xyzw=rotation.as_quat().tolist(),orientation_held=latest.get('orientation_held',True))
        finally:self.lock.release()
        g=raw_geometry(self.physical,poses,self.get_clock().now().nanoseconds,self.settings,self.profile['floor_height_m'])
        if self.settings['cable_length_source']=='hover_calibration':
            g['valid']=False;g['reasons'].append('hover_cable_length_pending_receive_only_test')
        if not self.profile.get('geometry_confirmed') or not self.profile['mocap'].get('frames_confirmed'):
            g['valid']=False;g['reasons'].append('attachment_or_frames_unconfirmed')
        return g,self.protocol.phase if self.protocol is not None else 'receive_only',False

    def confirm_unloaded(self,request,response):
        response.success,response.message=self.shadow.confirm_unloaded()
        return response

    def protocol_tick(self):
        if self.shadow.error:
            print('Sensor task stopped: '+self.shadow.error+'; saving partial recording.',flush=True);self.protocol_done=True;return
        pressed=False
        if select.select([self.stdin_fd],[],[],0)[0]:
            chunk=os.read(self.stdin_fd,4096)
            if not chunk:
                print('Input closed; saving partial recording.',flush=True);self.protocol_done=True;return
            if self.protocol.needs_enter:
                self.input_buffer+=chunk
                pressed=b'\n' in self.input_buffer
            # Discard extra/stale Enter input; never confirm a future stage.
            if pressed or not self.protocol.needs_enter:self.input_buffer=b''
        old_phase=self.protocol.phase
        events=self.protocol.step(time.monotonic_ns(),self.shadow.latest,pressed,self.shadow.confirm_unloaded)
        for event in events:
            event['expected_payload_mass_g']=self.physical['payload_mass_kg']*1000
            self.protocol_log(event)
            print('\n[%d/%d] %s' % (self.protocol.trial,self.protocol.cycles,event['message']),flush=True)
        if self.protocol.phase!=old_phase:
            self.input_buffer=b''
            # Drain input typed during the preceding timed hold.
            while select.select([self.stdin_fd],[],[],0)[0]:
                if not os.read(self.stdin_fd,4096):self.protocol_done=True;break
        self.protocol_done=self.protocol_done or self.protocol.done

    def report(self):
        if self.protocol is not None:return
        d=self.shadow.latest
        self.get_logger().info('health='+str(d.get('health',{}).get('health_ok',False))+' state='+d['state']+' hook_distance_m='+str(d.get('geometry',{}).get('hook_distance_m'))+' reasons='+str(d.get('reasons')))

    def close(self):
        receipt=self.shadow.close();self.tf_log.close();self.protocol_log.close()
        receipt['guided_test']=None if self.protocol is None else dict(completed=self.protocol.completed,final_phase=self.protocol.phase,trial=self.protocol.trial,cycles=self.protocol.cycles,hold_s=self.protocol.hold_s,release_s=self.protocol.release_s)
        receipt.update(scope='Receive-only bench: no flight sender, no controller, no aircraft commands',tf_recording_error=self.tf_log.error)
        try:
            from sanity_zenoh.cable_shadow_plot import plot_recording
            plot_recording(self.output,self.started_ns)
            print('Saved recordings and plots: '+str(self.output),flush=True)
        except Exception as error:
            receipt['plot_error']=str(error);print('Recording saved; plotting failed: '+str(error),flush=True)
        (self.output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=CableShadowBench()
        while rclpy.ok() and not node.protocol_done:rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.close();node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
