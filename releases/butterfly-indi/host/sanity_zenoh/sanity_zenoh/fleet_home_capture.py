"""Capture stationary hover homes from /tf, then use the existing fleet runner."""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time
import uuid

import numpy as np
import yaml


def place_at_captured_homes(data, samples, now_ns):
    """No command interface. Preserve transforms, regions' relative extent and limits."""
    if data['mission']['kind']!='hover' or data['safety'].get('separation_mode')!='stationary_hover':
        raise ValueError('Home capture is only for stationary hover at each aircraft home')
    result=deepcopy(data);details={}
    for drone in result['drones']:
        frame=drone['tracked_frame'];rows=samples.get(frame,[])
        unique={row['stamp_ns']:row for row in rows}
        rows=sorted(unique.values(),key=lambda row:row['stamp_ns'])
        if len(rows)<20 or (rows[-1]['stamp_ns']-rows[0]['stamp_ns'])<500_000_000:
            raise ValueError('Need fresh stationary /tf samples for '+frame)
        if any(row['parent_frame']!=result['mocap']['parent_frame'] for row in rows):
            raise ValueError('Unexpected mocap parent for '+frame)
        position=np.asarray([row['position'] for row in rows],dtype=float)
        quaternion=np.asarray([row['quaternion_xyzw'] for row in rows],dtype=float)
        if (position.shape!=(len(rows),3) or quaternion.shape!=(len(rows),4)
                or not np.isfinite(position).all() or not np.isfinite(quaternion).all()
                or np.any(abs(np.linalg.norm(quaternion,axis=1)-1.)>.01)):
            raise ValueError('Invalid mocap pose for '+frame)
        age=(now_ns-rows[-1]['stamp_ns'])/1e9
        if not -.002<=age<=.04:raise ValueError('Captured mocap is stale/future for '+frame)
        if any(not -.002<=(row['receipt_unix_ns']-row['stamp_ns'])/1e9<=.04 for row in rows):
            raise ValueError('Mocap source/receipt timing invalid for '+frame)
        scatter=float(np.linalg.norm(np.std(position,axis=0)))
        if scatter>.01:raise ValueError('Keep aircraft stationary during home capture: '+frame)
        from scipy.spatial.transform import Rotation
        world=Rotation.from_quat(result['mocap']['world_from_map_xyzw'])
        rotations=Rotation.from_quat(quaternion)
        transformed=world.apply(position+rotations.apply(drone['marker_to_base_xyz']))
        home=np.median(transformed[:,:2],axis=0)
        previous=np.asarray(drone['home_xy'],dtype=float)
        region=np.asarray(drone['region'],dtype=float)
        region[:,:2]+=home-previous
        drone['home_xy']=home.tolist()
        drone['placement']['center_xy']=home.tolist()
        drone['region']=region.tolist()
        details[str(drone['id'])]={'frame':frame,'samples':len(rows),'home_xy_m':home.tolist(),
            'position_scatter_m':scatter,'latest_source_age_s':age,
            'first_source_stamp_ns':rows[0]['stamp_ns'],'last_source_stamp_ns':rows[-1]['stamp_ns'],
            'marker_to_base_xyz':drone['marker_to_base_xyz'],'marker_to_base_xyzw':drone['marker_to_base_xyzw']}
    return result,details


def capture_samples(data, duration_s=2., timeout_s=15.):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    frames={drone['tracked_frame'] for drone in data['drones']}
    samples={frame:[] for frame in frames}
    rclpy.init();node=Node('sanity_fleet_home_capture')
    def receive(message):
        receipt=time.time_ns()
        for tf in message.transforms:
            if tf.child_frame_id not in samples:continue
            p=tf.transform.translation;q=tf.transform.rotation
            samples[tf.child_frame_id].append({'parent_frame':tf.header.frame_id,
                'stamp_ns':tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec,
                'receipt_unix_ns':receipt,'position':[p.x,p.y,p.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w]})
            samples[tf.child_frame_id]=[r for r in samples[tf.child_frame_id] if r['receipt_unix_ns']>=receipt-int(duration_s*1e9)]
    node.create_subscription(TFMessage,data['mocap']['topic'],receive,qos_profile_sensor_data)
    deadline=time.monotonic()+timeout_s;last_error='No /tf received'
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            now=time.time_ns()
            recent={frame:[r for r in rows if r['receipt_unix_ns']>=now-int(duration_s*1e9)] for frame,rows in samples.items()}
            try:
                if any(not rows or rows[-1]['stamp_ns']-rows[0]['stamp_ns']<int((duration_s-.1)*1e9) for rows in recent.values()):
                    missing=[frame for frame,rows in recent.items() if len(rows)<20]
                    last_error='Waiting for /tf bodies: '+', '.join(missing or sorted(frames));continue
                place_at_captured_homes(data,recent,now)
                return recent,now
            except ValueError as error:last_error=str(error)
        raise TimeoutError(last_error+'; enable/align all Motive bodies and start the OptiTrack /tf node')
    finally:
        node.destroy_node();rclpy.shutdown()


def main(argv=None):
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--template',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--capture-duration-s',type=float,default=2.)
    parser.add_argument('--capture-timeout-s',type=float,default=15.)
    args,forward=parser.parse_known_args(argv)
    root=args.workspace.resolve();template=args.template.resolve()
    if any(a=='--config' or a.startswith('--config=') for a in forward):
        parser.error('Use --template for the hover configuration')
    if not (math.isfinite(args.capture_duration_s) and .75<=args.capture_duration_s<=5.
            and math.isfinite(args.capture_timeout_s) and args.capture_duration_s+1<=args.capture_timeout_s<=60.):
        parser.error('Capture duration0.75..5s; timeout duration+1..60s')
    from sanity_zenoh import fleet
    from sanity_zenoh.fleet_config import FleetConfig
    if any(a in ('--sim','--plan-only','--help','-h','--control') or a.startswith('--control=') for a in forward):
        command=['--workspace',str(root),'--config',str(template),*forward]
        if args.output:command+=['--output',str(args.output)]
        return fleet.main(command)
    if '--fly' in forward and '--frames-confirmed' not in forward:parser.error('--fly requires --frames-confirmed after aligning every configured body frame')
    FleetConfig(template,root)  # Validate IDs, resources, transforms and template before ROS input.
    data=yaml.safe_load(template.read_text())
    mode='flight' if '--fly' in forward else 'shadow'
    suffix=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    setup=root/'outputs'/('sanity_fleet_homes_'+suffix)
    output=(args.output or root/'outputs'/('sanity_fleet_'+suffix+'_'+mode)).resolve()
    if output.exists():raise FileExistsError(output)
    setup.mkdir(parents=True,exist_ok=False)
    print('Capturing stationary /tf homes for: '+', '.join(d['tracked_frame'] for d in data['drones']),flush=True)
    try:
        samples,now=capture_samples(data,args.capture_duration_s,args.capture_timeout_s)
        placed,details=place_at_captured_homes(data,samples,now)
        path=setup/'fleet_config.yaml';path.write_text(yaml.safe_dump(placed,sort_keys=False))
        configured=FleetConfig(path,root)  # Existing separation gates; no relaxation.
        proof={'captured_unix_ns':now,'template':str(template),'template_sha256':hashlib.sha256(template.read_bytes()).hexdigest(),
            'configuration_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'drones':details,
            'separation':configured.separation,'samples':samples,
            'scope':'Read-only mocap capture, no command connection; body alignment confirmed separately by --frames-confirmed'}
        (setup/'home_capture.json').write_text(json.dumps(proof,indent=2)+'\n')
        pairs=configured.separation['pairs']
        print(('Captured homes validated; minimum centre separation %.3fm'%min(p['center_distance_m'] for p in pairs))
            if pairs else 'Captured home validated; single-aircraft hover',flush=True)
        return fleet.main(['--workspace',str(root),'--config',str(path),'--output',str(output),*forward])
    except BaseException as error:
        (setup/'capture_or_run_error.json').write_text(json.dumps({'error':str(error),'unix_ns':time.time_ns()},indent=2)+'\n')
        raise
    finally:
        if output.is_dir() and (setup/'home_capture.json').exists():shutil.copyfile(setup/'home_capture.json',output/'home_capture.json')


if __name__=='__main__':main()
