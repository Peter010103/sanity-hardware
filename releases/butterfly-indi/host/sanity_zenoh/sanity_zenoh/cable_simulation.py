import argparse
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import select
import shutil
import socket
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation

from sanity_zenoh.cable_sequence import CableSequenceNode
from sanity_zenoh.cable_configuration import load_physical_configuration
from sanity_zenoh.cable_assessment import assess_tracking
from sanity_zenoh.firmware_collective import FirmwareCollective
from sanity_zenoh.simulation_wire import decode_msp


class NativePlant:
    def __init__(self, root, config, ground, log, cpu_affinity=None):
        command=[str(root/'.venv-sanity-train/bin/python'),'-u',str(root/'loco_mjx/experiments/sanity_cable_mpc/deployment_plant.py'),'--config',str(config)]
        if ground:command.append('--ground-start')
        self.log=log.open('w')
        self.process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True)
        if cpu_affinity:os.sched_setaffinity(self.process.pid,set(cpu_affinity))


    def call(self, op, **values):
        self.process.stdin.write(json.dumps({'op':op,**values})+'\n');self.process.stdin.flush()
        if not select.select([self.process.stdout],[],[],5.)[0]:raise TimeoutError('Native cable plant timeout')
        line=self.process.stdout.readline()
        if not line:raise RuntimeError('Native cable plant exited; see plant_worker.log')
        return json.loads(line)


    def close(self):
        if self.process.poll() is None:
            self.process.terminate();self.process.wait(timeout=3)
        self.process.stdin.close();self.process.stdout.close();self.log.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='Actual ROS2 dual-TF -> cable mission -> native Zenoh/Pico/MSP -> independent MuJoCo cable plant.')
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--emulator',type=Path,required=True)
    parser.add_argument('--mission',choices=('circle_slow','circle_fast','payload_orbit'),default='circle_slow')
    parser.add_argument('--profile',type=Path)
    parser.add_argument('--mpc-hover-only',action='store_true')
    parser.add_argument('--controller',choices=('aware','unaware'),default='aware')
    parser.add_argument('--real-time-factor',type=float,default=1.)
    parser.add_argument('--airborne',action='store_true')
    parser.add_argument('--wall-clock',action='store_true')
    parser.add_argument('--takeoff-land-only',action='store_true')
    parser.add_argument('--skip-ground-height-check',action='store_true')
    parser.add_argument('--hover-duration-s',type=float,default=5.)
    parser.add_argument('--fault',choices=('none','payload_stale','vehicle_stale','solver_delay','pair_skew','control_delay','payload_rotation_jump','payload_position_jump'),default='none')
    parser.add_argument('--fault-at',type=float,default=2.)
    parser.add_argument('--physics-substeps',type=int,choices=(4,8,16),default=8)
    parser.add_argument('--mocap-delay-s',type=float,default=.01)
    args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID','0') in ('','0'):
        parser.error('Use localhost-only DDS and an isolated nonzero ROS_DOMAIN_ID')
    if not .1<=args.real_time_factor<=1. or not 0<=args.mocap_delay_s<=.02:parser.error('Invalid real-time factor or mocap delay')
    root=args.workspace.resolve();output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    package=root/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh'
    profile=json.loads((args.profile or package/'config/sanity_cable.json').read_text());profile.update(mission=args.mission,controller=args.controller,start_airborne=args.airborne)
    if args.mpc_hover_only:profile['mpc_hover_only']=True
    if args.takeoff_land_only:profile.update(takeoff_land_only=True,hover_duration_s=args.hover_duration_s)
    if args.skip_ground_height_check:profile['skip_ground_height_check']=True
    physical=load_physical_configuration(root/'loco_mjx/experiments/sanity_cable_mpc',profile,args.profile or package/'config/sanity_cable.json')
    controller_physical_path=output/'controller_physical_config.json'
    controller_physical_path.write_text(json.dumps(physical,indent=2)+'\n')
    profile['physical_config']=controller_physical_path.name
    profile_path=output/'simulation_profile.json';profile_path.write_text(json.dumps(profile,indent=2)+'\n')
    physical['voltage_V']=7.8;physical['integration_substeps']=args.physics_substeps;physical['native_integrator']='implicit';physical['cable_joint_armature_kg_m2']=1e-8;physical['cable_collision_model']='battery_box';physical['cable_topology']='unilateral_particles'
    physical_path=output/'physical_config.json';physical_path.write_text(json.dumps(physical,indent=2)+'\n')
    calibration=package/'config/sanity09_esp_collective.json'
    inverse=FirmwareCollective(calibration);inverse.set_voltage(7.8)
    class Inverse:
        def thrust_from_rc(self, rc):return float(np.interp(rc,*inverse.curve())) if rc>1000 else 0.
    with socket.socket() as port:
        port.bind(('127.0.0.1',0));endpoint=f'tcp/127.0.0.1:{port.getsockname()[1]}'
    snapshot=output/'source'
    paths=list((package/'sanity_zenoh').glob('cable_*.py'))+[package/'sanity_zenoh'/name for name in ('mocap.py','landing.py','client.py','transport.py','firmware_collective.py','protocol.py','simulation_wire.py','recording.py')]
    paths+=list((root/'loco_mjx/experiments/sanity_cable_mpc').glob('*.py'))+[root/'loco_mjx/experiments/sanity_cable_mpc/supplied_cable.xml']
    profiles=root/'loco_mjx/loco_mjx/environments/robots/sanity_ctbr/data'
    paths+=[calibration,package/'config/sanity_cable.json']
    paths+=[profiles/name for name in ('acro100_profile.json','fc_voltage_profile.json','yaw_reaction_20261021.json','flight_rate_20261022.json')]
    paths+=[root/'loco_mjx/loco_mjx/environments/locomotion/urma2'/name for name in ('ctbr_plant.py','flight_rate.py')]
    paths+=[root/'Sanity/src/sanity_receiver/zenoh_receiver'/name for name in ('tests/receiver_emulator.c','tests/CMakeLists.txt','main/control.c','main/control.h','main/actuator_profile.h')]
    cable_xml=ET.parse(root/'loco_mjx/experiments/sanity_cable_mpc/supplied_cable.xml')
    paths+=[root/'Sanity/sanity-hardware/models'/mesh.get('file') for mesh in cable_xml.findall('./asset/mesh')]
    source_hashes={}
    for source in paths:
        destination=snapshot/source.relative_to(root);destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,destination)
        source_hashes[str(source.relative_to(root))]=hashlib.sha256(source.read_bytes()).hexdigest()
    (output/'source_manifest.json').write_text(json.dumps({'files':source_hashes,'native_emulator_sha256':hashlib.sha256(args.emulator.read_bytes()).hexdigest()},indent=2)+'\n')
    params={'drone_id':1,'use_sim_time':not args.wall_clock,'simulation':True,'enable_output':True,'loco_root':str(root/'loco_mjx'),'profile':str(profile_path),
            'physical_config':str(controller_physical_path),'esp_collective_profile':str(calibration),'output':str(output/'controller'),'listen':endpoint,'telemetry_listen':''}
    if args.fault=='solver_delay':params.update(test_solver_delay_at_s=args.fault_at,test_solver_delay_s=.12)
    if args.fault=='control_delay':params.update(test_control_delay_at_s=args.fault_at,test_control_delay_s=.045)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    executor=SingleThreadedExecutor();node=driver=plant=emulator=None
    latest={'command':np.zeros(4),'enabled':False,'received_ns':0}
    lock=threading.Lock();clock_ns=time.time_ns() if args.wall_clock else 1_000_000_000;initial_clock_ns=clock_ns
    receiver_log=(output/'receiver.jsonl').open('w',buffering=1)
    plant_log=(output/'plant.jsonl').open('w',buffering=1)
    failed=None;native_count=0;fault_triggered=False;wall_started=time.monotonic();loop_started=loop_finished=None;post_stop_ns=None
    try:
        plant=NativePlant(root,physical_path,not args.airborne,output/'plant_worker.log',profile.get('simulation_plant_cpu_affinity',[]))
        pose=plant.call('sample')
        node=CableSequenceNode([Parameter(k,value=v) for k,v in params.items()])
        driver=rclpy.create_node('sanity_cable_simulated_mocap')
        tf_pub=driver.create_publisher(TFMessage,'/tf',20);clock_pub=driver.create_publisher(Clock,'/clock',20)
        executor.add_node(node);executor.add_node(driver);node.start_control_thread()
        emulator=subprocess.Popen([str(args.emulator),'3',endpoint,'600'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        def receive():
            global native_count
            for line in emulator.stdout:
                receiver_log.write(line)
                try:
                    decoded=decode_msp(json.loads(line),Inverse())
                    with lock:latest.update(command=np.array(decoded['ctbr']),enabled=decoded['enabled'],received_ns=time.monotonic_ns())
                    native_count+=1
                except (ValueError,KeyError,TypeError):pass
        reader=threading.Thread(target=receive,daemon=True);reader.start()
        if args.wall_clock:clock_ns=time.time_ns();initial_clock_ns=clock_ns
        history=deque([(clock_ns,pose)],maxlen=30)
        loop_started=time.monotonic()
        for step in range(20000):
            start_wall=time.monotonic()
            with lock:command=latest['command'].copy();enabled=latest['enabled']
            pose=plant.call('advance',command=command.tolist(),duration_s=.01)
            clock_ns=time.time_ns() if args.wall_clock else clock_ns+10_000_000
            history.append((clock_ns,pose))
            clock=Clock();clock.clock.sec,clock.clock.nanosec=divmod(clock_ns,10**9);clock_pub.publish(clock)
            deadline=time.monotonic()+2.
            while node.get_clock().now().nanoseconds<clock_ns:
                executor.spin_once(timeout_sec=.0005)
                if time.monotonic()>deadline:raise TimeoutError('ROS clock propagation failed')
            origin=node.flight.mission_start_ns if node.flight.mission_start_ns is not None else node.flight.hover_start_ns
            mission_t=(clock_ns-origin)/1e9 if origin is not None else -1.
            inject=mission_t>=args.fault_at and args.fault!='none'
            if args.fault=='control_delay':inject=node.control_delay_injected
            fault_triggered|=inject
            sample_stamp,sample=next((r for r in reversed(history) if r[0]<=clock_ns-int(args.mocap_delay_s*1e9)),history[0])
            transforms=[]
            for key in ('aircraft','payload'):
                if inject and args.fault==key.replace('aircraft','vehicle')+'_stale':continue
                stamp=sample_stamp
                if inject and args.fault=='pair_skew' and key=='payload':
                    stamp,sample_payload=next((r for r in reversed(history) if r[0]<=sample_stamp-20_000_000),history[0])
                else:sample_payload=sample
                state=sample_payload if key=='payload' else sample
                world=Rotation.from_quat(profile['mocap']['world_from_map_xyzw'])
                body=Rotation.from_quat(state[key+'_xyzw'])
                marker=world.inv()*body*Rotation.from_quat(profile['mocap'][key+'_marker_to_body_xyzw']).inv()
                point=world.inv().apply(state[key+'_position'])-marker.apply(profile['mocap'][key+'_marker_to_body_xyz'])
                if key=='payload' and inject and args.fault=='payload_rotation_jump' and mission_t<args.fault_at+.02:marker=marker*Rotation.from_euler('x',180.,degrees=True)
                if key=='payload' and inject and args.fault=='payload_position_jump' and mission_t<args.fault_at+.02:point=point+np.array([.03,0.,0.])
                tf=TransformStamped();tf.header.frame_id=profile['mocap']['parent_frame'];tf.child_frame_id=profile['mocap'][key+'_frame']
                tf.header.stamp.sec,tf.header.stamp.nanosec=divmod(stamp,10**9)
                tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,point)
                tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,marker.as_quat())
                transforms.append(tf)
            tf_pub.publish(TFMessage(transforms=transforms))
            for _ in range(16):executor.spin_once(timeout_sec=0.)
            plant_log.write(json.dumps({'stamp_ns':clock_ns,'applied':command.tolist(),'enabled':enabled,'phase':node.flight.phase,'mission_t':mission_t,'aircraft_position':pose['aircraft_position'],'payload_position':pose['payload_position'],'payload_xyzw':pose['payload_xyzw'],'truth':pose['truth']},allow_nan=False)+'\n')
            if node.finished.is_set():
                if profile.get('landing_cutoff_height_m',0.)>0. and node.flight.finished and not node.fatal:
                    if post_stop_ns is None:post_stop_ns=clock_ns
                    if clock_ns-post_stop_ns>=1_000_000_000:break
                else:break
            deadline=start_wall+.01/args.real_time_factor
            while time.monotonic()<deadline and (not node.finished.is_set() or post_stop_ns is not None):executor.spin_once(timeout_sec=min(.001,max(0.,deadline-time.monotonic())))
        else:raise TimeoutError('Cable sequence exceeded200simulation seconds')
    except Exception as error:
        failed=type(error).__name__+': '+str(error)
    finally:
        loop_finished=time.monotonic()
        if node is not None:node.close();node.destroy_node()
        if emulator is not None:emulator.terminate();emulator.wait(timeout=5)
        if plant is not None:plant.close()
        receiver_log.close();plant_log.close()
        if driver is not None:driver.destroy_node()
        executor.shutdown(timeout_sec=2.);rclpy.try_shutdown()
    receipt=json.loads((output/'controller/receipt.json').read_text()) if (output/'controller/receipt.json').exists() else {}
    tracking=assess_tracking(output,args.mission) if receipt and (output/'controller/commands.jsonl').exists() else {'passed':False,'reason':'No completed controller logs'}
    expected=(receipt.get('mission_completed') and receipt.get('landed') and not receipt.get('fatal') and not receipt.get('abort_reason') and tracking['passed']) if args.fault in ('none','control_delay','payload_rotation_jump','payload_position_jump') else (fault_triggered and ((receipt.get('landed') and receipt.get('abort_reason') and not receipt.get('fatal')) if args.fault!='vehicle_stale' else bool(receipt.get('fatal'))))
    if args.fault in ('payload_rotation_jump','payload_position_jump'):
        control_records=[json.loads(line) for line in (output/'controller/control.jsonl').read_text().splitlines()]
        raw_records=[json.loads(line) for line in (output/'controller/tf.jsonl').read_text().splitlines()]
        fault_observed=(any(row.get('payload_orientation_held',False) for row in control_records) or any(row.get('orientation_held',False) for row in raw_records)) if args.fault=='payload_rotation_jump' else receipt.get('mocap_rejections',0)>0
        expected=expected and fault_triggered and fault_observed
    if args.fault=='control_delay':expected=expected and fault_triggered and receipt.get('control_deadline_misses',0)>=1
    assessment={'passed':bool(not failed and expected and native_count>10 and receipt.get('shutdown_acknowledged')),'tracking':tracking,'driver_error':failed,'mission':args.mission,'controller':args.controller,'fault':args.fault,'fault_triggered':fault_triggered,'native_MSP_packets':native_count,'wall_elapsed_s':time.monotonic()-wall_started,'simulated_elapsed_s':(clock_ns-initial_clock_ns)/1e9,'simulation_loop_wall_s':loop_finished-loop_started if loop_started is not None else None,'startup_wall_s':loop_started-wall_started if loop_started is not None else None,'achieved_real_time_factor':(clock_ns-initial_clock_ns)/1e9/max(loop_finished-loop_started,1e-9) if loop_started is not None else None,'requested_real_time_factor':args.real_time_factor,'mocap_delay_s':args.mocap_delay_s,'controller_receipt':receipt,'feedback_scope':'Only timestamped aircraft/payload TF enter control; qpos/qvel/force truth logged separately by plant driver', 'native_emulator_sha256':hashlib.sha256(args.emulator.read_bytes()).hexdigest()}
    if post_stop_ns is not None:
        settled=pose['aircraft_position'][2]-node.flight.home[2]
        body=Rotation.from_quat(np.asarray(pose['truth']['qpos'])[3:7][[1,2,3,0]])
        tilt=float(np.degrees(np.arccos(np.clip(body.as_matrix()[2,2],-1.,1.))))
        assessment['cutoff_followthrough']={'post_stop_s':(clock_ns-post_stop_ns)/1e9,'aircraft_height_above_start_m':float(settled),'aircraft_tilt_deg':tilt,'last_native_output_enabled':enabled,'passed':bool(abs(settled)<.025 and tilt<15. and not enabled)}
        assessment['passed']&=assessment['cutoff_followthrough']['passed']
    (output/'assessment.json').write_text(json.dumps(assessment,indent=2)+'\n')
    print(json.dumps({'passed':assessment['passed'],'driver_error':failed,'fatal':receipt.get('fatal'),'abort_reason':receipt.get('abort_reason'),'landed':receipt.get('landed'),'mission_completed':receipt.get('mission_completed'),'output':str(output)}),flush=True)
    raise SystemExit(0 if assessment['passed'] else 1)
