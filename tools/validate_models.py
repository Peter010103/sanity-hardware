#!/usr/bin/env python3
"""Offline geometry, evidence, inertia, force-direction and scalar dynamics checks.

Does not access hardware. Checks URDF mass/inertia in PyBullet, balanced rotor
forces and roll/pitch signs. These idealised tests do not validate flight control,
yaw reaction torque, aerodynamic interactions or sim-to-real performance.
"""
import argparse
import csv
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sanity_actuator import SanityModel, SpinningModel

def read(rel):
    return json.loads((ROOT / rel).read_text())

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def put(out, name, value):
    (out / name).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')

def mesh_vertices(path):
    return np.array([[float(v) for v in line.split()[1:4]] for line in path.read_text().splitlines() if line.startswith('v ')])

def mass_properties(rel='models/sanity.urdf'):
    doc=ET.parse(ROOT/rel).getroot()
    inertial=doc.find("link[@name='base_link']/inertial")
    assert inertial is not None
    mass=float(inertial.find('mass').attrib['value'])
    origin=inertial.find('origin')
    com=np.array([float(x) for x in origin.attrib['xyz'].split()])
    assert np.array_equal([float(x) for x in origin.attrib['rpy'].split()],[0,0,0])
    a={k:float(v) for k,v in inertial.find('inertia').attrib.items()}
    tensor=np.array([[a['ixx'],a['ixy'],a['ixz']],
                     [a['ixy'],a['iyy'],a['iyz']],
                     [a['ixz'],a['iyz'],a['izz']]])
    assert mass>0 and np.isfinite(tensor).all() and np.isfinite(com).all()
    for link in doc.findall('link'):
        if link.attrib['name']=='base_link':continue
        virtual=link.find('inertial')
        assert virtual is not None and float(virtual.find('mass').attrib['value'])==0
        assert all(float(v)==0 for v in virtual.find('inertia').attrib.values())
    return mass,com,tensor


def geometry(rel):
    path = ROOT / rel
    doc = ET.parse(path).getroot()
    links = {e.attrib['name'] for e in doc.findall('link')}
    joints = doc.findall('joint')
    children = {e.find('child').attrib['link'] for e in joints}
    assert len(links)==5 and len(joints)==4 and links-children=={'base_link'}
    assert all(e.attrib['type']=='fixed' for e in joints)
    assert 'world' not in links and len(doc.findall('.//inertial')) == len(links)
    mass_properties(rel)
    vertices=[]; collision=[]; refs=[]
    for link in doc.findall('link'):
        for kind in ['visual','collision']:
            for shape in link.findall(kind):
                mesh=shape.find('geometry/mesh')
                assert mesh is not None
                assert mesh.attrib.get('scale','1 1 1')=='1 1 1'
                file=(path.parent / mesh.attrib['filename']).resolve()
                assert file.is_relative_to(ROOT) and file.is_file()
                refs.append(str(file.relative_to(ROOT)))
                for line in file.read_text().splitlines():
                    if line.startswith('mtllib '):
                        mtl=file.parent / line.split(maxsplit=1)[1]
                        assert mtl.is_file()
                        for ml in mtl.read_text().splitlines():
                            if ml.startswith(('map_', 'bump ', 'disp ')):
                                texture=(mtl.parent / ml.split(maxsplit=1)[1]).resolve()
                                assert texture.is_relative_to(ROOT) and texture.is_file()
                (vertices if kind=='visual' else collision).append(mesh_vertices(file))
    v=np.concatenate(vertices); c=np.concatenate(collision)
    assert np.all(v.min(0)>=c.min(0)-1e-9) and np.all(v.max(0)<=c.max(0)+1e-9)
    dims=v.max(0)-v.min(0)
    assert np.allclose(dims,[.111918702,.111918702,.0295],atol=1e-8,rtol=0)
    forces=[]
    for j in joints:
        r=np.array([float(x) for x in j.find('origin').attrib['xyz'].split()])
        moment=np.cross(r,[0,0,1])
        assert np.allclose(moment,[r[1],-r[0],0])
        forces.append(dict(frame=j.find('child').attrib['link'], position_m=r.tolist(),
                           unit_thrust_N=[0,0,1], moment_about_model_origin_Nm=moment.tolist(),
                           yaw_reaction_moment_Nm=None))
    rotor_parameters=read('parameters/actuators.json')['rotor_frames']
    assert {frame['frame'] for frame in forces} == set(rotor_parameters)
    for frame in forces:
        assert np.allclose(np.abs(frame['position_m'][:2]),.0284610479427585,atol=1e-10,rtol=0)
    return dict(file=rel, links=len(links), fixed_joints=len(joints), root='base_link',
                world_joint=False,inertial_tags=True, mesh_references=refs,
                dimensions_m=dims.tolist(), visual_enclosed_by_collision=True, geometric_force_signs=forces)

def native_import(rel, out, name):
    import pybullet as pb
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    mass,com,J=mass_properties(rel)
    client=pb.connect(pb.DIRECT)
    try:
        body=pb.loadURDF(str(ROOT/rel),useFixedBase=False,
                        flags=pb.URDF_USE_INERTIA_FROM_FILE,physicsClientId=client)
        assert pb.getNumJoints(body,physicsClientId=client)==4
        assert len(pb.getVisualShapeData(body,physicsClientId=client))==6
        dynamics=pb.getDynamicsInfo(body,-1,physicsClientId=client)
        link_masses=[pb.getDynamicsInfo(body,i,physicsClientId=client)[0] for i in range(4)]
        assert math.isclose(dynamics[0],mass,abs_tol=1e-12) and link_masses==[0]*4
        assert np.allclose(dynamics[2],np.diag(J),rtol=0,atol=1e-12)
        assert np.allclose(dynamics[3],com,rtol=0,atol=1e-12)
        view=pb.computeViewMatrix(cameraEyePosition=[.17,-.20,.15],cameraTargetPosition=[0,0,-.005],cameraUpVector=[0,0,1])
        projection=pb.computeProjectionMatrixFOV(fov=40,aspect=4/3,nearVal=.001,farVal=2)
        camera=pb.getCameraImage(960,720,view,projection,renderer=pb.ER_TINY_RENDERER,
                    lightDirection=[1,-1,2],physicsClientId=client)
        rgba=np.asarray(camera[2]).reshape(720,960,4)
        object_pixels=int(np.sum(np.asarray(camera[4])>=0))
        assert object_pixels>1000
        plt.imsave(out/(name+'.png'),rgba)

        # Apply forces explicitly: rotor-frame joints do not create lift.
        rotor_positions=[np.array([float(x) for x in joint.find('origin').attrib['xyz'].split()])
                         for joint in ET.parse(ROOT/rel).getroot().findall('joint')]
        pb.changeDynamics(body,-1,linearDamping=0,angularDamping=0,physicsClientId=client)
        pb.setGravity(0,0,-9.80665,physicsClientId=client)
        pb.setTimeStep(.002,physicsClientId=client)
        start=np.array(pb.getBasePositionAndOrientation(body,physicsClientId=client)[0])
        for _ in range(250):
            for r in rotor_positions:
                pb.applyExternalForce(body,-1,[0,0,mass*9.80665/4],r.tolist(),
                                      pb.LINK_FRAME,physicsClientId=client)
            pb.stepSimulation(physicsClientId=client)
        position,orientation=pb.getBasePositionAndOrientation(body,physicsClientId=client)
        drift=float(np.linalg.norm(np.array(position)-start))
        assert drift<1e-9 and np.allclose(orientation,[0,0,0,1],atol=1e-9)

        # Each rotor's lever arm produces the expected angular acceleration.
        pb.setGravity(0,0,0,physicsClientId=client)
        dt=.0001;pb.setTimeStep(dt,physicsClientId=client)
        responses=[]
        for i,r in enumerate(rotor_positions):
            pb.resetBasePositionAndOrientation(body,com.tolist(),[0,0,0,1],physicsClientId=client)
            pb.resetBaseVelocity(body,[0,0,0],[0,0,0],physicsClientId=client)
            force=np.array([0,0,.1])
            pb.applyExternalForce(body,-1,force.tolist(),r.tolist(),pb.LINK_FRAME,physicsClientId=client)
            pb.stepSimulation(physicsClientId=client)
            linear,angular=pb.getBaseVelocity(body,physicsClientId=client)
            expected=np.linalg.solve(J,np.cross(r-com,force))
            actual=np.array(angular)/dt
            assert np.allclose(actual,expected,rtol=1e-6,atol=1e-8)
            assert linear[2]>0
            responses.append(dict(rotor=i+1,angular_acceleration_rad_s2=actual.tolist()))
        return dict(file=rel,loaded=True,joints=4,visual_shapes=6,visible_object_pixels=object_pixels,
                    useFixedBase=False,total_mass_kg=mass+sum(link_masses),
                    rotor_link_masses_kg=link_masses,CoM_m=com.tolist(),
                    inertia_diagonal_kg_m2=list(dynamics[2]),render=name+'.png',
                    simulation_steps=254,balanced_force_duration_s=.5,balanced_force_drift_m=drift,
                    rotor_force_responses=responses,
                    scope='Idealised force-balance and moment-sign tests; no yaw reaction torque or flight controller.')
    finally:
        pb.disconnect(client)


def scalar_checks(out):
    model=SanityModel()
    mass,_,_=mass_properties(); g=9.80665; weight=mass*g
    k=model.force['k_N_per_rad_s_squared']
    omega=math.sqrt(weight/(4*k))
    assert abs(sum(model.thrusts([omega]*4))-weight)<1e-12
    # Continuous interpolation of the fitted count-to-speed map, not an integer hardware setpoint.
    V=7.8
    lo,hi=1250.,1450.
    for _ in range(60):
        mid=(lo+hi)/2
        if sum(model.thrusts(model.steady_omega([mid]*4,V)))>weight: hi=mid
        else: lo=mid
    command=(lo+hi)/2
    rotor=SpinningModel(model,[command]*4,V)
    h=v=0.; dt=.001; rows=[]
    for n in range(2000):
        t=n*dt
        u=command+(10 if .5<=t<.7 else 0)
        w=rotor.step(dt,[u]*4,V)
        thrust=sum(model.thrusts(w)); acc=thrust/mass-g
        h+=v*dt+.5*acc*dt*dt; v+=acc*dt
        rows.append(dict(time_s=t+dt,command_equivalent_counts=u,voltage_V=V,
                         collective_thrust_N=thrust,vertical_acceleration_m_s2=acc,
                         vertical_velocity_m_s=v,height_m=h))
    drift=max(abs(r['height_m']) for r in rows[:500])
    assert drift<1e-10 and max(r['vertical_acceleration_m_s2'] for r in rows)>0
    assert all(math.isfinite(x) for r in rows for x in r.values())
    assert abs(rows[-1]['collective_thrust_N']-weight)<1e-10
    with (out/'scalar_vertical_smoke.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    return dict(status='passed algebraic/scalar consistency only',mass_kg=mass,gravity_m_s2=g,weight_N=weight,
                equal_speed_hover_omega_rad_s=omega,equal_speed_hover_RPM=omega*60/(2*math.pi),
                voltage_V=V,common_command_equivalent_counts=command,
                duration_s=2,dt_s=dt,hover_only_height_drift_m=drift,positive_pulse_acceleration=True,
                assumptions=['confirmed mass from models/sanity.urdf',
                             'vertical axis held fixed; no attitude, yaw or drag model',
                             'constant supplied voltage; no battery dynamics',
                             'all rotor forces upward; physical mapping unverified',
                             'fractional count interpolation for offline force balance, not a commanded hardware test'],
                full_aircraft_hover_or_flight_validation=False)

def evidence():
    counts={}
    model=read('data/processed/actuator/fit_report.json')
    for directory,files in model['source_sha256'].items():
        for name,expected in files.items():
            assert digest(ROOT/'data/raw'/directory/name)==expected,(directory,name)
    counts['model_source_hashes']=sum(len(v) for v in model['source_sha256'].values())
    raw=read('data/provenance/raw_manifest.json')
    for item in raw:
        assert digest(ROOT/item['path'])==item['sha256'],item['path']
    counts['raw_hashes']=len(raw)
    for item in read('models/sources/inventory.json'):
        assert digest(ROOT/'models/sources'/item['source_name'])==item['sha256']
    counts['original_asset_hashes']=len(read('models/sources/inventory.json'))
    mass,com,J=mass_properties()
    assert mass==0.06947 and np.array_equal(com,[0,0,0])
    assert np.array_equal(J,np.diag([3.32e-5,4.28e-5,5.43e-5]))
    assert np.allclose(J,J.T) and np.all(np.linalg.eigvalsh(J)>0)
    eig=np.linalg.eigvalsh(J); margins=eig.sum()-2*eig
    assert np.all(margins>=0)
    return dict(**counts,mass_kg=mass,CoM_m=com.tolist(),
                selected_inertia_diagonal_kg_m2=np.diag(J).tolist(),
                inertia_symmetric=True,inertia_positive_definite=True,triangle_margins_kg_m2=margins.tolist(),
                mass_in_URDF=True,rotor_frames_massless=True,
                configuration='measured mass; operator-selected bifilar working tensor; CoM placement assumed')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'build/validation')
    args=parser.parse_args(); out=args.output; out.mkdir(parents=True,exist_ok=True)
    models=['models/sanity.urdf']
    results=dict(geometry=[geometry(x) for x in models],
                 physical_and_evidence=evidence(),
                 native_imports=[native_import(x,out,'native_'+str(i)) for i,x in enumerate(models)],
                 scalar_simulation=scalar_checks(out),
                 yaw_dynamic_sign_check='not performed: physical directions and reaction coefficient unknown',
                 hardware='existing bench evidence only; no hardware access during validation; no flight-policy validation')
    put(out,'contribution_validation.json',results)
    versions={name:importlib.metadata.version(name) for name in ['numpy','scipy','matplotlib','pybullet']}
    versions.update(python=platform.python_version(),node=subprocess.check_output(['node','--version'],text=True).strip(),
                    platform=platform.platform(),
                    commands=['node tools/build_geometry.mjs','python3 tools/reduce_loadcell.py',
                              'python3 tools/fit_actuators.py --bootstrap 100','python3 tools/verify_refit.py',
                              'python3 tools/validate_models.py','python3 tools/validate_repo.py'],
                    dependencies_note='tools installed externally; no dependency binaries vendored')
    put(out,'toolchain_current.json',versions)
    print(json.dumps(dict(geometry='passed',native_import_and_force_checks='passed',physical='mass, CoM and inertia match selected URDF parameters',
                         scalar_simulation='passed; not 6-DoF',source_hashes=results['physical_and_evidence']),indent=2))

if __name__=='__main__':
    main()
