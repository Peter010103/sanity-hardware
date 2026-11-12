import argparse
from copy import deepcopy
from datetime import datetime,timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from .plate_configuration import PlateDeploymentConfig


def commissioning_profile(base, registry, frames, plate_frame, controller, height, hover_s, frames_confirmed=False, geometry_confirmed=False):
    if registry.get('version')!=1:raise ValueError('Hover registry version1 required')
    if len(frames)!=3 or len(set(frames))!=3:raise ValueError('Choose exactly three distinct registered drones, in cable mount order')
    if not plate_frame or plate_frame in [*frames,registry['mocap']['parent_frame']]:raise ValueError('A distinct plate rigid-body frame is required')
    if not math.isfinite(height) or not .35<=height<=.8 or not math.isfinite(hover_s) or not 2<=hover_s<=20:
        raise ValueError('Plate centre height must be0.35..0.8m above the floor; hover duration2..20s')
    if controller not in ('pd','mpc'):raise ValueError('Controller must be pd or mpc')
    known={d['tracked_frame']:d for d in registry['drones']};result=deepcopy(base)
    if len(base['drones'])!=3:raise ValueError('The commissioning base profile must have three cable mounts')
    result['transport']=deepcopy(registry['transport']);result['drones']=[]
    result['mocap'].update(deepcopy(registry['mocap']))
    result['mocap'].update(plate_frame=plate_frame,frames_confirmed=bool(frames_confirmed))
    result['geometry_confirmed']=bool(geometry_confirmed)
    result['require_clock_preflight']=True
    for index,frame in enumerate(frames):
        if frame not in known or known[frame].get('hardware_confirmed') is not True:raise ValueError('Drone not commissioned in hover registry: '+frame)
        source=known[frame]
        if type(source['id']) is not int or frame!=f"sanity{source['id']:02d}":raise ValueError('Hover registry frame/receiver identity mismatch: '+frame)
        template=next((d for d in base['drones'] if d['frame']==frame),None)
        if template is None and any('aircraft_and_cable_mass_g' in d for d in base['drones']):
            raise ValueError('Measured plate defaults missing for '+frame+'; supply a base profile with its aircraft parameters')
        drone=deepcopy(template if template is not None else base['drones'][index])
        drone.update(id=source['id'],frame=frame,hardware_confirmed=True,calibration=source['calibration'],
            marker_to_body_xyz=deepcopy(source['marker_to_base_xyz']),marker_to_body_xyzw=deepcopy(source['marker_to_base_xyzw']))
        result['drones'].append(drone)
    result['supervision'].update(start_airborne=False,hover_only=True,takeoff_land_only=controller=='pd',
        hover_dwell_s=hover_s if controller=='pd' else 1.,mpc_hover_s=hover_s,
        acquire_timeout_s=max(25.,hover_s+10.))
    result['calibration_note']=base.get('calibration_note','')+' Aircraft identities, marker transforms and collective maps inherited from the commissioned hover registry. No new actuator or inertia identification performed.'
    return result


def captured_setup(config, samples, now_ns):
    c=config.physical;world=Rotation.from_quat(config.mocap['world_from_map_xyzw']);poses=[];details={}
    frames=[config.mocap['plate_frame'],*[d['frame'] for d in config.drones]]
    for frame in frames:
        rows=sorted({r['stamp_ns']:r for r in samples.get(frame,[])}.values(),key=lambda r:r['stamp_ns'])
        if len(rows)<20 or rows[-1]['stamp_ns']-rows[0]['stamp_ns']<500_000_000:raise ValueError('Waiting for stationary /tf body: '+frame)
        if any(r['parent_frame']!=config.mocap['parent_frame'] for r in rows):raise ValueError('Unexpected mocap parent for '+frame)
        if not 0<=(now_ns-rows[-1]['stamp_ns'])/1e9<config.mocap['maximum_age_s']:raise ValueError('Stale/future mocap for '+frame)
        if any(not -.002<=(r['receipt_unix_ns']-r['stamp_ns'])/1e9<=.04 for r in rows):raise ValueError('Source/receipt timing invalid for '+frame)
        positions=np.asarray([r['position'] for r in rows]);quaternions=np.asarray([r['quaternion_xyzw'] for r in rows])
        if positions.shape!=(len(rows),3) or quaternions.shape!=(len(rows),4) or not np.isfinite(np.r_[positions.ravel(),quaternions.ravel()]).all() or np.max(abs(np.linalg.norm(quaternions,axis=1)-1))>.01:
            raise ValueError('Invalid mocap pose for '+frame)
        scatter=float(np.linalg.norm(np.std(positions,axis=0)))
        rotations=Rotation.from_quat(quaternions)
        if scatter>.01 or np.max((rotations.mean().inv()*rotations).magnitude())>np.deg2rad(3):raise ValueError('Keep all aircraft and plate stationary during capture: '+frame)
        if frame==frames[0]:
            converted=world.apply(positions-rotations.apply(c.mocap_offset));body=world*rotations.mean()
        else:
            drone=next(d for d in config.drones if d['frame']==frame)
            converted=world.apply(positions+rotations.apply(drone['marker_to_body_xyz']))
            body=world*rotations.mean()*Rotation.from_quat(drone['marker_to_body_xyzw'])
        poses.append((np.median(converted,axis=0),body))
        details[frame]=dict(samples=len(rows),scatter_m=scatter,latest_source_ns=rows[-1]['stamp_ns'],position_m=poses[-1][0].tolist(),body_quaternion_xyzw=body.as_quat().tolist())
    if (max(d['latest_source_ns'] for d in details.values())-min(d['latest_source_ns'] for d in details.values()))/1e9>config.mocap['maximum_skew_s']:
        raise ValueError('Plate/aircraft source timestamp skew during capture')
    plate,rotation=poses[0]
    if rotation.as_matrix()[2,2]<np.cos(np.deg2rad(5)):raise ValueError('Rest the plate flat on the flight floor before capture')
    floor=float(plate[2]-np.abs(rotation.as_matrix()[2])@(np.asarray(c.plate_size_m)/2))
    config.settings['floor_height_m']=floor
    from experiments.sanity_plate_transport.flight_model import PlateFlightModel
    from experiments.sanity_plate_transport.flight_reference import FlightReference
    from experiments.sanity_plate_transport.deployment import PlateSupervisor
    model=PlateFlightModel(c,config.models);reference=FlightReference(model)
    state=np.zeros(model.nx)
    for start,(position,body) in zip([0,13,37,61],poses):
        state[start:start+3]=position;state[start+3:start+7]=body.as_quat()[[3,0,1,2]]
    supervisor=PlateSupervisor(model,reference,config.settings)
    separation=min(np.linalg.norm(poses[i][0]-poses[k][0]) for i in range(1,4) for k in range(1,i))
    if separation<c.minimum_drone_separation_m:raise ValueError('Captured aircraft separation is below the configured plate limit')
    try:supervisor.update(state,0.)
    except ValueError as error:
        if 'formation' in str(error):raise ValueError(str(error)+'; --drones must follow the actual cable mounts; check --mount-yaw-deg against the plate rigid-body axes') from error
        raise
    expected,_=reference.sample(0.)
    for i,drone in enumerate(config.drones):details[drone['frame']]['expected_xy_m']=expected[13+24*i:15+24*i].tolist()
    return dict(floor_height_m=floor,poses=details,plate_initial_yaw_rad=reference.yaw,
        minimum_separation_m=float(separation),
        ground_formation_check_skipped=supervisor.ground_formation_check_skipped,initial_formation_error_m=supervisor.initial_formation_error_m.tolist(),
        maximum_start_height_m=config.settings.get('maximum_start_height_m',.08),landing_body_heights_m=supervisor.landing_height.tolist(),
        mount_order=[d['frame'] for d in config.drones],scope='Stationary mocap capture and existing ground/formation guards; no command transport opened')


def capture(config, duration_s, timeout_s):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    frames=[config.mocap['plate_frame'],*[d['frame'] for d in config.drones]];samples={frame:[] for frame in frames}
    rclpy.init();node=Node('sanity_plate_ground_capture')
    def receive(message):
        receipt=time.time_ns()
        for tf in message.transforms:
            if tf.child_frame_id not in samples:continue
            p,q=tf.transform.translation,tf.transform.rotation
            samples[tf.child_frame_id].append(dict(parent_frame=tf.header.frame_id,stamp_ns=tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec,
                receipt_unix_ns=receipt,position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w]))
            samples[tf.child_frame_id]=[r for r in samples[tf.child_frame_id] if receipt-r['receipt_unix_ns']<=int(duration_s*1e9)]
    node.create_subscription(TFMessage,config.mocap['topic'],receive,qos_profile_sensor_data)
    deadline=time.monotonic()+timeout_s;error='Waiting for all four bodies'
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02);now=time.time_ns()
            recent={frame:[r for r in rows if now-r['receipt_unix_ns']<=int(duration_s*1e9)] for frame,rows in samples.items()}
            missing=[frame for frame,rows in recent.items() if len(rows)<20 or rows[-1]['stamp_ns']-rows[0]['stamp_ns']<(duration_s-.1)*1e9]
            if missing:
                error='Waiting for stationary /tf bodies: '+', '.join(missing)
                continue
            try:return captured_setup(config,recent,now),recent
            except ValueError as exc:error=str(exc)
        raise TimeoutError(error)
    finally:node.destroy_node();rclpy.shutdown()


def main(argv=None):
    parser=argparse.ArgumentParser(description='Shared plate pickup, hover or circle-tilt mission, placement and landing using commissioned hover identities; default is offline plan only')
    parser.add_argument('--workspace',type=Path,required=True);parser.add_argument('--registry',type=Path);parser.add_argument('--base-profile',type=Path)
    parser.add_argument('--drones',nargs=3,default=['sanity09','sanity15','sanity24'],metavar=('MOUNT0','MOUNT120','MOUNT240'))
    parser.add_argument('--plate-frame',help='Motive plate frame (default from base profile: plate_payload)');parser.add_argument('--controller',choices=['pd','mpc'])
    parser.add_argument('--mission',choices=['hover','circle_tilt'],default='hover')
    parser.add_argument('--height',type=float,default=.5,help='Plate centre height above the captured floor, metres (default0.5; aircraft about1.0m)')
    parser.add_argument('--hover-seconds',type=float,default=5.);parser.add_argument('--cable-length-mm',type=float,nargs='+')
    parser.add_argument('--mount-yaw-deg',type=float,default=0.,help='Mount0 angle about plate-frame+Z, degrees; the other mounts follow at+120/+240')
    parser.add_argument('--skip-ground-formation-check',action='store_true',help='Allow a wider ground footprint; retain cable-reach, floor, spacing, attitude and tracking checks')
    parser.add_argument('--skip-flight-separation-check',action='store_true',help='Disable the in-flight separation emergency stop; initial spacing and active repulsion remain, but collision protection is reduced')
    parser.add_argument('--frames-confirmed',action='store_true');parser.add_argument('--geometry-confirmed',action='store_true')
    parser.add_argument('--capture-duration-s',type=float,default=2.);parser.add_argument('--capture-timeout-s',type=float,default=20.)
    parser.add_argument('--output',type=Path)
    mode=parser.add_mutually_exclusive_group();mode.add_argument('--plan-only',action='store_true');mode.add_argument('--sim',action='store_true');mode.add_argument('--shadow',action='store_true');mode.add_argument('--fly',action='store_true')
    args=parser.parse_args(argv);root=args.workspace.resolve();sys.path.insert(0,str(root/'loco_mjx'))
    args.controller=args.controller or ('pd' if args.mission=='hover' else 'mpc')
    if args.mission!='hover' and args.controller!='mpc':parser.error('Circle tilt requires the coupled MPC controller')
    if not math.isfinite(args.capture_duration_s) or not .75<=args.capture_duration_s<=5 or not math.isfinite(args.capture_timeout_s) or not args.capture_duration_s+1<=args.capture_timeout_s<=60:
        parser.error('Capture duration0.75..5s; timeout duration+1..60s')
    package=root/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh'
    registry_path=(args.registry or package/'config/fleet_hover_registry.yaml').resolve();base_path=(args.base_profile or package/'config/sanity_plate_108g.json').resolve()
    registry=yaml.safe_load(registry_path.read_text());base=json.loads(base_path.read_text())
    args.plate_frame=args.plate_frame or base['mocap']['plate_frame']
    profile=commissioning_profile(base,registry,args.drones,args.plate_frame,args.controller,args.height,args.hover_seconds,args.frames_confirmed,args.geometry_confirmed)
    profile['supervision']['hover_only']=args.mission=='hover'
    profile['supervision']['skip_ground_formation_check']=args.skip_ground_formation_check
    profile['supervision']['skip_flight_separation_check']=args.skip_flight_separation_check
    from experiments.sanity_plate_transport.config import load_config
    geometry_path=Path(base['mission_profile']);geometry_path=geometry_path if geometry_path.is_absolute() else root/geometry_path
    geometry=load_config(geometry_path).dictionary();geometry['initial_height_m']=args.height
    if all('cable_length_m' in d for d in profile['drones']):geometry['cable_lengths_m']=[d['cable_length_m'] for d in profile['drones']]
    if all('aircraft_and_cable_mass_g' in d for d in profile['drones']):
        geometry['drone_masses_kg']=[d['aircraft_and_cable_mass_g']*.001-geometry['cable_mass_kg'] for d in profile['drones']]
    if args.mission=='circle_tilt':
        motion=load_config(root/'loco_mjx/experiments/sanity_plate_transport/configs/circle_tilt.json').dictionary()
        for key in ['trajectory','orientation_mode','path_duration_s','path_radius_m','path_vertical_amplitude_m','plate_tilt_amplitude_deg','plate_yaw_amplitude_deg','duration_s','maximum_plate_tilt_deg']:
            geometry[key]=motion[key]
    if not math.isfinite(args.mount_yaw_deg):parser.error('Mount yaw must be finite degrees')
    if args.mount_yaw_deg:
        geometry['plate_attachments_m']=Rotation.from_euler('z',args.mount_yaw_deg,degrees=True).apply(load_config(geometry_path).attachments).tolist()
    if args.cable_length_mm is not None:
        if len(args.cable_length_mm) not in (1,3) or not np.isfinite(args.cable_length_mm).all() or min(args.cable_length_mm)<=0:parser.error('Supply one shared or three positive cable lengths in mm')
        geometry['cable_lengths_m']=np.repeat(args.cable_length_mm,3).tolist() if len(args.cable_length_mm)==1 else args.cable_length_mm
        geometry['cable_lengths_m']=(np.asarray(geometry['cable_lengths_m'])/1000.).tolist()
    suffix=datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    output=(args.output or root/'outputs'/('sanity_plate_hover_'+suffix)).resolve();output.mkdir(parents=True,exist_ok=False)
    profile['mission_profile']=str(output/'geometry.json');(output/'geometry.json').write_text(json.dumps(geometry,indent=2))
    profile_path=output/'profile.json';profile_path.write_text(json.dumps(profile,indent=2))
    config=PlateDeploymentConfig(profile_path,root)
    plan=dict(mode='fly' if args.fly else 'shadow' if args.shadow else 'sim' if args.sim else 'plan',controller=args.controller,mission=args.mission,
        skip_ground_formation_check=args.skip_ground_formation_check,
        skip_flight_separation_check=args.skip_flight_separation_check,
        mount_order=[dict(mount_index=i,plate_mount_position_m=config.physical.attachments[i].tolist(),frame=d['frame'],receiver_id=d['id'],calibration=d['calibration'],
            cable_length_mm=float(config.physical.lengths[i]*1000.),cable_length_provisional=bool(d.get('cable_length_provisional',False)) if args.cable_length_mm is None else False,
            aircraft_and_cable_mass_g=float((config.physical.masses[i]+config.physical.cable_mass_kg)*1000.),aircraft_mass_g=float(config.physical.masses[i]*1000.),
            attachment_body_m=config.models[i]['attachment_body_m']) for i,d in enumerate(config.drones)],
        plate_assembly_mass_g=config.physical.plate_mass_kg*1000.,plate_size_mm=(np.asarray(config.physical.plate_size_m)*1000.).tolist(),
        plate_frame=args.plate_frame,plate_height_above_floor_m=args.height,hover_s=args.hover_seconds,cable_lengths_m=config.physical.lengths.tolist(),
        geometry_confirmed=config.data['geometry_confirmed'],frames_confirmed=config.mocap['frames_confirmed'],
        registry=str(registry_path),registry_sha256=hashlib.sha256(registry_path.read_bytes()).hexdigest(),base_profile=str(base_path),base_sha256=hashlib.sha256(base_path.read_bytes()).hexdigest(),
        phase_sequence=['PD_takeoff','PD_pickup','PD_climb','PD_hover' if args.controller=='pd' else 'MPC_stationary_hover',*(['circle_tilt'] if args.mission=='circle_tilt' else []),'place_plate','land'],
        hardware_output=args.fly,clock_preflight=json.loads(os.environ['SANITY_CLOCK_PREFLIGHT_JSON']) if os.environ.get('SANITY_CLOCK_PREFLIGHT_JSON') else None)
    (output/'plan.json').write_text(json.dumps(plan,indent=2));print(json.dumps(plan,indent=2),flush=True)
    if args.fly:config.require_hardware()
    if args.sim:
        command=[str(root/'.venv-sanity-passage/bin/python'),'-u','-m','experiments.sanity_plate_transport.flight_validate','--workspace',str(root),'--profile',str(profile_path),'--output',str(output/'simulation')]
        if args.controller=='pd':command.append('--pd-only')
    elif args.fly or args.shadow:
        from .mission_clock import record_clock_preflight
        record_clock_preflight(output,required=True)
        print('Keep the plate resting on the floor and all three aircraft stationary; capturing four bodies.',flush=True)
        proof,samples=capture(config,args.capture_duration_s,args.capture_timeout_s)
        (output/'capture.json').write_text(json.dumps(dict(proof=proof,samples=samples),indent=2))
        profile['supervision']['floor_height_m']=proof['floor_height_m'];profile_path.write_text(json.dumps(profile,indent=2))
        command=[sys.executable,'-u','-m','sanity_zenoh.plate_sequence','--ros-args','-p','workspace:='+str(root),'-p','profile:='+str(profile_path),'-p','output:='+str(output/'flight'),
            '-p','enable_output:='+str(args.fly).lower(),'-p','no_transport:='+str(not args.fly).lower()]
    else:
        print('Plan saved to '+str(output)+'; no ROS, clock changes or aircraft connection.',flush=True)
        return
    environment={**os.environ,'PYTHONPATH':str(root/'loco_mjx')+os.pathsep+os.environ.get('PYTHONPATH',''),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
    os.execve(command[0],command,environment)


if __name__=='__main__':main()
