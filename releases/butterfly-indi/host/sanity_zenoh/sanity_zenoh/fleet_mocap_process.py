"""ROS /tf estimation process; publishes only to private per-drone latest-state slots."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

if __package__ in (None,''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from sanity_zenoh.mocap import MocapEstimator,MocapOutlier
from sanity_zenoh.mocap_ipc import LatestStateSlot
from sanity_zenoh.recording import EventRecorder


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    args=parser.parse_args();config=json.loads(args.config.read_text())
    rclpy.init(args=[],signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node=rclpy.create_node('sanity_fleet_mocap')
    executor=SingleThreadedExecutor();executor.add_node(node)
    estimators={};slots={};logs={};reported=set();stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        for drone in config['drones']:
            device=drone['id'];v=drone['parameters']
            estimators[device]=MocapEstimator(v['parent_frame'],v['tracked_frame'],v['world_from_map_xyzw'],
                v['marker_to_base_xyz'],v['marker_to_base_xyzw'],v['velocity_filter_tau_s'],
                frames_confirmed=v['frames_confirmed'],angular_rate_filter_tau_s=v['angular_rate_filter_tau_s'],
                position_filter_tau_s=v.get('position_filter_tau_s',0.) if v.get('pitch_pose_filter_enabled',False) else 0.,
                attitude_filter_tau_s=v.get('attitude_filter_tau_s',0.) if v.get('pitch_pose_filter_enabled',False) else 0.)
            slots[device]=LatestStateSlot(drone['slot'],writable=True)
            logs[device]=EventRecorder(Path(drone['log']),isolated=True,stamp=False)
        by_frame={(e.parent,e.child):device for device,e in estimators.items()}
        def receive(message):
            for tf in message.transforms:
                device=by_frame.get((tf.header.frame_id,tf.child_frame_id))
                if device is None:continue
                t,q=tf.transform.translation,tf.transform.rotation
                stamp=tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
                arrival=node.get_clock().now().nanoseconds
                receipt=time.monotonic_ns()
                try:
                    packet=estimators[device].update(tf.header.frame_id,tf.child_frame_id,stamp,
                        [t.x,t.y,t.z],[q.x,q.y,q.z,q.w],arrival,receipt)
                    if packet is None:continue
                except MocapOutlier as error:
                    logs[device]({'kind':'mocap_rejected_sample','input_stamp_ns':stamp,'arrival_ros_ns':arrival,
                        'packet':{'ready':False,'error':str(error)},'diagnostics':error.diagnostics})
                    continue
                except (ValueError,TypeError) as error:
                    packet={'version':1,'ready':False,'error':str(error)}
                slots[device].write(packet)
                logs[device]({'input_stamp_ns':stamp,'arrival_ros_ns':arrival,'packet':packet})
        node.create_subscription(TFMessage,config['topic'],receive,
            QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
        print(json.dumps({'ready':True,'pid':__import__('os').getpid(),'frames':list(by_frame)}),flush=True)
        while rclpy.ok() and not stopping and os.getppid()==config['parent_pid']:
            executor.spin_once(timeout_sec=.005)
            for device,log in logs.items():
                if log.error and device not in reported:
                    print(json.dumps({'error':'mocap_recording_failed','device':device,'detail':log.error}),flush=True)
                    reported.add(device)
    finally:
        executor.shutdown(timeout_sec=1.)
        errors=[]
        for device,log in logs.items():
            log.close()
            if log.error:
                errors.append(log.error)
                print(json.dumps({'error':'mocap_recording_close_failed','device':device,'detail':log.error}),flush=True)
        for slot in slots.values():slot.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if errors:raise SystemExit(1)


if __name__=='__main__':
    main()
