#!/usr/bin/env python3
"""Resolve local lab configuration without ROS, network or device access."""
from pathlib import Path
import argparse
import ipaddress
import json
import math
import yaml

ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh'


def resolve(path):
    path=Path(path).expanduser()
    return path.resolve() if path.is_absolute() else (ROOT/path).resolve()


def quaternion(value):
    return isinstance(value,list) and len(value)==4 and all(type(x) in (int,float) and math.isfinite(x) for x in value) and abs(sum(x*x for x in value)-1.)<1e-3


def load_lab(path):
    lab=json.loads(Path(path).read_text())
    for key in ('host_ip','mocap_ntp_server'):
        ip=ipaddress.IPv4Address(lab[key])
        if ip.is_unspecified or ip.is_multicast:raise ValueError('Explicit lab IPv4 required')
    if not quaternion(lab['world_from_map_xyzw']):raise ValueError('Unit world/map quaternion required')
    for mode in ('pid','indi'):
        device=lab[mode]
        if type(device['device']) is not int or not 1<=device['device']<=255 or not device['tracked_frame']:raise ValueError('Explicit receiver identity/frame required')
        if not math.isfinite(device['ground_m']) or not quaternion(device['marker_to_base_xyzw']):raise ValueError('Finite ground height and unit marker calibration required')
        xyz=device['marker_to_base_xyz']
        if len(xyz)!=3 or not all(math.isfinite(x) for x in xyz):raise ValueError('Finite marker offset required')
        if not resolve(device['collective_profile']).is_file():raise ValueError('Missing collective calibration profile')
    if len(lab['indi']['expected_uid'])!=24:raise ValueError('Expected FC MSP UID must have24 hex characters')
    bytes.fromhex(lab['indi']['expected_uid'])
    return lab


def configuration(lab,recipe):
    family=recipe['family'];key='pid' if family in ('legacy','pitch') else 'indi'
    hardware=lab[key]
    if family in ('legacy','pitch'):
        name='pitch_loop_sequence.yaml' if family=='pitch' else 'sanity09_mpc_5ms.yaml' if recipe.get('high_bank') else 'circle_sequence.yaml'
        result=yaml.safe_load((PACKAGE/'config'/name).read_text());p=result['sanity_sequence']['ros__parameters']
        p.update(drone_id=hardware['device'],listen=f'udp/{lab["host_ip"]}:7447',telemetry_listen=f'tcp/{lab["host_ip"]}:7448',
            tf_topic=lab['tf_topic'],parent_frame=lab['parent_frame'],tracked_frame=hardware['tracked_frame'],
            world_from_map_xyzw=lab['world_from_map_xyzw'],marker_to_base_xyz=hardware['marker_to_base_xyz'],
            marker_to_base_xyzw=lab.get('pid_flip_marker_to_base_xyzw',hardware['marker_to_base_xyzw']) if family=='pitch' else hardware['marker_to_base_xyzw'])
        p['ground_root_height_m']=hardware['ground_m']
        return result
    result=json.loads((PACKAGE/'config/sanity15_indi_mpc_constant_arclength.json').read_text())
    result['device']=hardware['device'];result['listen']=f'tcp/{lab["host_ip"]}:7447'
    result['collective_profile']=str(resolve(hardware['collective_profile']))
    result['limits'].update(expected_uid=hardware.get('expected_uid',lab['indi']['expected_uid']),ground_m=hardware['ground_m'])
    result['mocap'].update(topic=lab['tf_topic'],parent_frame=lab['parent_frame'],tracked_frame=hardware['tracked_frame'],
        world_from_map_xyzw=lab['world_from_map_xyzw'],marker_to_base_xyz=hardware['marker_to_base_xyz'],marker_to_base_xyzw=hardware['marker_to_base_xyzw'])
    result['estimator_logging']['clock_endpoint']=hardware.get('clock_endpoint',lab['indi']['clock_endpoint'])
    # Builder selects the predictor; resolve the unchanged common measured-mass XML.
    result['mpc']['model']=str(ROOT/'loco_mjx/loco_mjx/environments/robots/sanity_ctbr/data/plane_sanity15.xml')
    return result


def write_config(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():raise FileExistsError(path)
    path.write_text(yaml.safe_dump(value,sort_keys=False) if path.suffix=='.yaml' else json.dumps(value,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lab',type=Path,required=True);parser.add_argument('--recipe',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from recipes import RECIPES
    write_config(args.output,configuration(load_lab(args.lab),RECIPES[args.recipe]))
    print(args.output.resolve())

if __name__=='__main__':main()
