"""List Sanity bodies on /tf and launch a configurable, fixed-membership hover fleet."""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
import time
import uuid

import numpy as np
import yaml


SANITY_FRAME=re.compile(r'sanity_?\d+',re.IGNORECASE)


def frame_key(frame):
    digits=re.search(r'\d+',frame)
    return (int(digits.group()) if digits else 0,frame)


def discover_frames(seconds=3.,topic='/tf'):
    """Read only. Does not open Zenoh, infer addresses or change DDS interfaces."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    rclpy.init();node=rclpy.create_node('sanity_fleet_discovery');frames={}
    def receive(message):
        now=time.time_ns()
        for tf in message.transforms:
            frame=tf.child_frame_id
            if not SANITY_FRAME.fullmatch(frame):continue
            row=frames.setdefault(frame,{'samples':0,'parent_frames':[]})
            row['samples']+=1
            row['last_receipt_unix_ns']=now
            row['last_source_stamp_ns']=tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec
            if tf.header.frame_id not in row['parent_frames']:row['parent_frames'].append(tf.header.frame_id)
    node.create_subscription(TFMessage,topic,receive,qos_profile_sensor_data)
    try:
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=min(.05,max(0.,end-time.monotonic())))
        now=time.time_ns()
        return {frame:{**row,'source_age_ms':(now-row['last_source_stamp_ns'])/1e6}
            for frame,row in frames.items() if now-row['last_receipt_unix_ns']<=1_000_000_000}
    finally:
        node.destroy_node();rclpy.shutdown()


def validate_frame_ids(registry):
    for drone in registry['drones']:
        frame=drone['tracked_frame']
        if not SANITY_FRAME.fullmatch(frame) or int(re.search(r'\d+',frame).group())!=drone['id']:
            raise ValueError('Receiver ID must match the numeric mocap body suffix: '+frame)


def choose_frames(registry,detected,requested=None):
    validate_frame_ids(registry)
    by_frame={d['tracked_frame']:d for d in registry['drones']}
    if len(by_frame)!=len(registry['drones']):raise ValueError('Duplicate registered mocap frame')
    if requested is not None:
        chosen=[frame.strip() for value in requested for frame in value.split(',') if frame.strip()]
        if not chosen or len(set(chosen))!=len(chosen):raise ValueError('Specify unique mocap body names in --drones')
        for frame in chosen:
            if frame not in detected:raise ValueError('Requested body not detected on /tf: '+frame)
            if frame not in by_frame:raise ValueError('No saved receiver configuration for '+frame)
            if not by_frame[frame]['hardware_confirmed'] or not by_frame[frame]['ip']:raise ValueError('Aircraft is not commissioned: '+frame)
    else:
        chosen=[frame for frame in detected if frame in by_frame and by_frame[frame]['hardware_confirmed'] and by_frame[frame]['ip']]
    return sorted(chosen,key=frame_key)


def selected_configuration(registry,frames,height=None,hover_seconds=None):
    """Use separated validation homes; the capture runner replaces them before flight."""
    validate_frame_ids(registry)
    result=deepcopy(registry);by_frame={d['tracked_frame']:d for d in result['drones']}
    if len(by_frame)!=len(result['drones']):raise ValueError('Duplicate registered mocap frame')
    if not frames or len(set(frames))!=len(frames):raise ValueError('At least one unique configured body required')
    result['drones']=[by_frame[frame] for frame in frames]
    if result['mission']['kind']!='hover' or result['safety'].get('separation_mode')!='stationary_hover':
        raise ValueError('Automatic discovery is for stationary hover at captured homes')
    if height is not None:result['mission']['height_m']=height
    if hover_seconds is not None:result['mission']['hover_s']=hover_seconds
    if any(not math.isfinite(result['mission'][key]) or result['mission'][key]<=0 for key in ('height_m','hover_s')):
        raise ValueError('Height and hover duration must be positive finite numbers')
    safety=result['safety']
    margin=safety['body_radius_m']+safety['tracking_margin_m']+safety['max_speed_m_s']*safety['timing_uncertainty_s']
    spacing=max(1.2,2*margin+.2)
    for index,drone in enumerate(result['drones']):
        previous=np.asarray(drone['home_xy'],dtype=float);home=np.array([index*spacing,0.])
        region=np.asarray(drone['region'],dtype=float);region[:,:2]+=home-previous
        drone['home_xy']=home.tolist();drone['placement']['center_xy']=home.tolist();drone['region']=region.tolist()
    # Retain the existing conservative startup/landing reserve as the count changes.
    reserve=40*len(frames)+result['mission']['hover_s']+50
    safety['maximum_run_s']=max(safety['maximum_run_s'],float(30*math.ceil(reserve/30)))
    return result


def print_inventory(registry,detected,chosen):
    by_frame={d['tracked_frame']:d for d in registry['drones']}
    print('Sanity bodies on /tf:',flush=True)
    if not detected:print('  None received.',flush=True)
    for frame in sorted(detected,key=frame_key):
        drone=by_frame.get(frame)
        if drone is None:description='unregistered; excluded'
        elif not drone['hardware_confirmed'] or not drone['ip']:description='not commissioned; excluded'
        else:description=f"ID {drone['id']:02d}  {drone['ip']}"+('  selected' if frame in chosen else '  excluded')
        print(f'  {frame:<16} {description}',flush=True)
    print('Selected: '+(', '.join(chosen) if chosen else 'none'),flush=True)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[6])
    parser.add_argument('--registry',type=Path,help='Saved fleet YAML mapping body frames to commissioned receivers')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--list',action='store_true',help='List detected bodies only (also the default)')
    mode.add_argument('--fly',action='store_true',help='Fly the selected discovered bodies')
    mode.add_argument('--shadow',action='store_true',help='Run selected controllers with output disabled')
    parser.add_argument('--drones',nargs='+',help='Optional subset of exact /tf body names; space or comma separated')
    parser.add_argument('--frames-confirmed',action='store_true')
    parser.add_argument('--discovery-s',type=float,default=3.)
    parser.add_argument('--height',type=float,help='Hover height in world metres (default from registry:1m)')
    parser.add_argument('--hover-seconds',type=float,help='Common hover duration (default from registry:20s)')
    parser.add_argument('--capture-duration-s',type=float,default=2.)
    parser.add_argument('--capture-timeout-s',type=float,default=15.)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--control',type=Path,help='Send an operator command to a running fleet directory; no discovery')
    parser.add_argument('--action',choices=['land','emergency-stop'])
    parser.add_argument('--device',type=int)
    args=parser.parse_args(argv);root=args.workspace.resolve()
    if args.control:
        if not args.action:parser.error('--control requires --action')
        from sanity_zenoh.fleet import main as fleet_main
        command=['--workspace',str(root),'--control',str(args.control),'--action',args.action]
        if args.device is not None:command+=['--device',str(args.device)]
        return fleet_main(command)
    if args.action or args.device is not None:parser.error('--action/--device require --control')
    if args.fly and not args.frames_confirmed:parser.error('--fly requires --frames-confirmed for every selected body')
    if not math.isfinite(args.discovery_s) or not 1<=args.discovery_s<=30:parser.error('--discovery-s must be1..30')
    registry_path=(args.registry or root/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/fleet_hover_registry.yaml').resolve()
    registry=yaml.safe_load(registry_path.read_text())
    from sanity_zenoh.fleet_config import FleetConfig
    # Validate all saved identities/resources independently of their obsolete template positions.
    all_frames=[d['tracked_frame'] for d in registry['drones']]
    with tempfile.TemporaryDirectory(prefix='sanity_registry_') as folder:
        path=Path(folder)/'registry.yaml'
        path.write_text(yaml.safe_dump(selected_configuration(registry,all_frames),sort_keys=False))
        FleetConfig(path,root)
    detected=discover_frames(args.discovery_s,registry['mocap']['topic'])
    chosen=choose_frames(registry,detected,args.drones)
    print_inventory(registry,detected,chosen)
    proof={'discovered_unix_ns':time.time_ns(),'discovery_s':args.discovery_s,'frames':detected,'selected_frames':chosen,
        'registry_path':str(registry_path),'registry_sha256':hashlib.sha256(registry_path.read_bytes()).hexdigest(),
        'membership':'Frozen before home capture/arming; no mid-flight joins',
        'dds_environment':{key:os.environ.get(key) for key in ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE')}}
    if not args.fly and not args.shadow:
        if args.output:
            args.output.mkdir(parents=True,exist_ok=False)
            (args.output/'fleet_discovery.json').write_text(json.dumps(proof,indent=2)+'\n')
        return
    if not chosen:raise ValueError('No commissioned Sanity bodies detected on /tf; check Motive names and registry')
    configuration=selected_configuration(registry,chosen,args.height,args.hover_seconds)
    suffix=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    setup=root/'outputs'/('sanity_fleet_discovery_'+suffix);setup.mkdir(parents=True,exist_ok=False)
    path=setup/'fleet_config.yaml';path.write_text(yaml.safe_dump(configuration,sort_keys=False))
    FleetConfig(path,root)
    proof['selected_configuration_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    (setup/'fleet_discovery.json').write_text(json.dumps(proof,indent=2)+'\n')
    shutil.copyfile(registry_path,setup/'registry.yaml')
    output=(args.output or root/'outputs'/('sanity_fleet_'+suffix+('_flight' if args.fly else '_shadow'))).resolve()
    from sanity_zenoh.fleet_home_capture import main as capture_main
    command=['--workspace',str(root),'--template',str(path),'--output',str(output),
        '--capture-duration-s',str(args.capture_duration_s),'--capture-timeout-s',str(args.capture_timeout_s),
        '--fly' if args.fly else '--shadow']
    if args.frames_confirmed:command+=['--frames-confirmed']
    try:return capture_main(command)
    finally:
        if output.is_dir():
            for name in ('fleet_discovery.json','registry.yaml'):shutil.copyfile(setup/name,output/name)


if __name__=='__main__':main()
