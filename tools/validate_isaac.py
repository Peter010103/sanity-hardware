#!/usr/bin/env python3
"""Native Isaac Sim/Isaac Lab import, rendering and GPU dynamics checks.

Run with the Python environment containing Isaac Sim and Isaac Lab:
    python tools/validate_isaac.py
Outputs (including report.json) go to build/validation/isaac/.
The rotor mapping and signed yaw coefficient below are test fixtures, not
hardware identification. No hardware or flight-policy test is performed.
"""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=ROOT/'build/validation/isaac')
parser.add_argument('--device', default='cuda:0')
args = parser.parse_args()
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)
(out/'report.json').unlink(missing_ok=True)

from isaaclab.app import AppLauncher
app = AppLauncher(headless=True, enable_cameras=True, device=args.device).app

import csv
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
import shutil
import tempfile
import traceback

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import torch
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
import isaaclab.sim as sim_utils
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.utils.math import quat_apply
from sanity_actuator import SanityModel, SpinningModel
from sanity_isaac import SanityIsaacActuator, apply_to_isaaclab

# Deliberately nontrivial mapping exercises output-to-frame allocation.
FRAMES = ['rotor_3_front_left', 'rotor_1_rear_right',
          'rotor_4_rear_left', 'rotor_2_front_right']
DT, N, G = .001, 8, 9.80665
report = dict(status='running', date_utc=datetime.now(timezone.utc).isoformat(),
              versions={n: importlib.metadata.version(n) for n in ['isaacsim','isaaclab','torch','warp-lang']},
              device=args.device, physics_dt_s=DT, environments=N,
              test_output_to_frame=FRAMES, mapping_is_hardware_identification=False,
              hardware_access=False, flight_policy_validation=False,
              input_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in [ROOT/'models/sanity.urdf', ROOT/'parameters/actuators.json',
                                      ROOT/'sanity_isaac.py', Path(__file__).resolve()]},
              checks={})


def save_report():
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')


def passed(name, **values):
    report['checks'][name] = dict(status='passed', **values)
    save_report()
    print('PASS', name, values, flush=True)


class InertialUrdfConverter(UrdfConverter):
    def _get_urdf_import_config(self):
        cfg = super()._get_urdf_import_config()
        cfg.set_import_inertia_tensor(True)
        return cfg


def inspect_geometry(usd_path, source):
    stage = Usd.Stage.Open(str(usd_path))
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
    assert not any(p.IsA(UsdPhysics.Joint) for p in stage.Traverse())
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    assert len(bodies) == 1
    cache = UsdGeom.XformCache()
    cloud, collisions, materials = [], [], set()
    for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(p)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        transform = np.asarray(cache.GetLocalToWorldTransform(p))
        assert np.linalg.det(transform[:3,:3]) > 0
        points = np.c_[points, np.ones(len(points))] @ transform
        ancestor, is_collision = p, False
        while ancestor and not ancestor.IsPseudoRoot():
            is_collision |= ancestor.HasAPI(UsdPhysics.CollisionAPI)
            ancestor = ancestor.GetParent()
        target = collisions if is_collision else cloud
        target.append(points[:,:3])
        material, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
        if material: materials.add(str(material.GetPath()))
    assert cloud and collisions
    source_visual, source_collision = [], []
    for p in source.glob('meshes/**/*.obj'):
        points = np.array([[float(v) for v in l.split()[1:4]] for l in p.read_text().splitlines() if l.startswith('v ')])
        (source_collision if 'collision' in p.parts else source_visual).append(points)
    visual, reference = np.concatenate(cloud), np.concatenate(source_visual)
    collision, reference_collision = np.concatenate(collisions), np.concatenate(source_collision)
    errors = [cKDTree(a).query(b)[0].max() for a,b in [(visual,reference),(reference,visual),
                                                                   (collision,reference_collision),(reference_collision,collision)]]
    assert max(errors) < 1e-7, errors
    assert len(materials) >= 6
    passed('relocated_urdf_and_usd_geometry', visual_meshes=len(cloud), collision_meshes=len(collisions),
           dimensions_m=np.ptp(visual,axis=0).tolist(), max_vertex_error_m=max(errors),
           bound_materials=len(materials), free_rigid_bodies=1, joints=0)


def run():
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=args.device, gravity=(0,0,-G)))
    with tempfile.TemporaryDirectory(prefix='sanity_isaac_') as directory:
        source = Path(directory)/'models'
        shutil.copytree(ROOT/'models', source)
        cfg = UrdfConverterCfg(asset_path=str(source/'sanity.urdf'), usd_dir=str(out/'converted'),
                               fix_base=False, merge_fixed_joints=True, joint_drive=None,
                               make_instanceable=False, force_usd_conversion=True)
        converter = InertialUrdfConverter(cfg)
        # Move the complete generated asset, then remove its original export directory.
        # USD references must survive independently of both conversion and source paths.
        relocated = out/'usd'
        if relocated.exists(): shutil.rmtree(relocated)
        shutil.move(str(out/'converted'), relocated)
        usd_path = relocated/Path(converter.usd_path).name
        inspect_geometry(usd_path, source)
    # The temporary URDF/OBJ directory no longer exists when PhysX loads the asset.
    for i in range(N):
        sim_utils.create_prim(f'/World/env_{i}', 'Xform')
    robot = RigidObject(RigidObjectCfg(
        prim_path='/World/env_.*/Sanity',
        spawn=sim_utils.UsdFileCfg(usd_path=str(usd_path), rigid_props=sim_utils.RigidBodyPropertiesCfg(
            linear_damping=0, angular_damping=0, disable_gravity=False,
            sleep_threshold=0, enable_gyroscopic_forces=True)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0,0,1))))
    light = sim_utils.DomeLightCfg(intensity=1200., color=(1.,1.,1.))
    light.func('/World/Light', light)
    sim.reset()
    assert robot.is_initialized and robot.num_instances == N and robot.num_bodies == 1
    body_ids, _ = robot.find_bodies('^base_link$')
    assert body_ids == [0]
    report['force_api'] = 'permanent_wrench_composer' if hasattr(robot, 'permanent_wrench_composer') else 'set_external_force_and_torque'
    if args.device.startswith('cuda'):report['gpu'] = torch.cuda.get_device_name()
    params = SanityModel()
    motors = SanityIsaacActuator(N, DT, output_to_frame=FRAMES, device=args.device,
                                 omit_reaction_torque=True, delay_s=params.L)
    mass = motors.mass_kg
    J = torch.tensor([5.75e-5,6.64e-5,9.31e-5], device=args.device)
    actual_mass = robot.root_physx_view.get_masses().cpu().numpy()
    actual_J = robot.root_physx_view.get_inertias().cpu().numpy().reshape(N,3,3)
    com = robot.root_physx_view.get_coms().cpu().numpy()
    np.testing.assert_allclose(actual_mass, mass, rtol=1e-6)
    np.testing.assert_allclose(actual_J, np.broadcast_to(np.diag(J.cpu().numpy()),(N,3,3)), rtol=1e-6, atol=1e-12)
    np.testing.assert_allclose(com[:,:3], 0, atol=1e-9)
    np.testing.assert_allclose(com[:,3:], np.tile([0,0,0,1],(N,1)), atol=1e-9)
    passed('physx_mass_inertia_com', mass_kg=float(actual_mass[0,0]),
           inertia_kg_m2=actual_J[0].tolist(), com_m=com[0,:3].tolist())
    state = torch.zeros((N,13), device=args.device)
    state[:,0] = torch.arange(N,device=args.device)*.4
    state[:,2], state[:,3] = 1, 1

    def reset(ids=None, orientation=None):
        ids = torch.arange(N,device=args.device) if ids is None else ids
        s = state[ids].clone()
        if orientation is not None:s[:,3:7] = orientation
        robot.write_root_state_to_sim(s, env_ids=ids)
        robot.reset(ids)
        motors.reset([1250]*4, 7.8, env_ids=ids)
        robot.update(DT)

    def tick(output):
        apply_to_isaaclab(robot, output, body_ids=body_ids)
        robot.write_data_to_sim()
        sim.step(render=False)
        robot.update(DT)
        assert torch.isfinite(robot.data.root_state_w).all()

    reset()
    zero = motors.wrench_from_speeds([0]*4)
    for _ in range(200):tick(zero)
    drop = (state[:,2]-robot.data.root_pos_w[:,2]).cpu().numpy()
    velocity = robot.data.root_lin_vel_w[:,2].cpu().numpy()
    np.testing.assert_allclose(velocity, -G*.2, atol=2e-5)
    np.testing.assert_allclose(drop, .5*G*DT**2*200*201, atol=3e-5)
    passed('free_fall', duration_s=.2, mean_drop_m=float(drop.mean()), max_velocity_error_m_s=float(np.max(abs(velocity+G*.2))))

    reset()
    hover = motors.wrench_from_speeds([math.sqrt(mass*G/(4*motors.k))]*4)
    for _ in range(500):tick(hover)
    drift = (robot.data.root_pos_w-state[:,:3]).abs().max().item()
    assert drift < 5e-5
    assert robot.data.root_ang_vel_b.abs().max() < 1e-6
    passed('ideal_hover_balance', duration_s=.5, max_position_drift_m=drift)

    reset()
    speed = torch.zeros((N,4),device=args.device)
    for i in range(4):speed[i,i] = math.sqrt(.1/motors.k)
    single = motors.wrench_from_speeds(speed)
    tick(single)
    expected_ang = single.torque_b_Nm[:,0]/J*DT
    torch.testing.assert_close(robot.data.root_ang_vel_b, expected_ang, rtol=2e-3, atol=1e-6)
    expected_v = single.force_b_N[:,0]/mass*DT
    expected_v[:,2] -= G*DT
    torch.testing.assert_close(robot.data.root_lin_vel_w, expected_v, rtol=2e-3, atol=1e-6)
    passed('individual_rotor_roll_pitch', angular_velocity_rad_s=robot.data.root_ang_vel_b[:4].cpu().tolist(),
           max_error_rad_s=(robot.data.root_ang_vel_b-expected_ang).abs().max().item())

    reset()
    yaw = SanityIsaacActuator(N,DT,output_to_frame=FRAMES,device=args.device,
                              reaction_torque_ratio_m=[.01,-.01,.01,-.01])
    speed.zero_();speed[0,0]=math.sqrt(.1/motors.k);speed[1,1]=math.sqrt(.1/motors.k)
    output = yaw.wrench_from_speeds(speed)
    tick(output)
    expected = output.torque_b_Nm[:,0]/J*DT
    torch.testing.assert_close(robot.data.root_ang_vel_b, expected, rtol=2e-3, atol=1e-6)
    passed('synthetic_signed_yaw', yaw_velocity_rad_s=robot.data.root_ang_vel_b[:2,2].cpu().tolist(),
           synthetic_ratio_m=.01, measured_yaw_validation=False)

    angle = math.pi/6
    q = torch.tensor([math.cos(angle/2),0,math.sin(angle/2),0],device=args.device)
    reset(orientation=q)
    tick(hover)
    expected = torch.tensor([G*math.sin(angle)*DT,0,G*(math.cos(angle)-1)*DT],device=args.device).repeat(N,1)
    torch.testing.assert_close(robot.data.root_lin_vel_w, expected, rtol=2e-3, atol=1e-6)
    passed('body_local_tilted_thrust', pitch_deg=30, velocity_m_s=robot.data.root_lin_vel_w[0].cpu().tolist())

    reset()
    scalar = [SpinningModel(params,[1250]*4,7.8) for _ in range(N)]
    rows=[];max_speed_error=0.;max_velocity_error=0.
    for i in range(300):
        command = 1250 if i < 50 or i >= 200 else 1450
        voltage = 7.8 - .0002*i
        before_q = robot.data.root_quat_w.clone()
        before_v = robot.data.root_lin_vel_w.clone()
        output = motors.step([command]*4,voltage)
        reference = torch.tensor([s.step(DT,[command]*4,voltage) for s in scalar],device=args.device)
        max_speed_error = max(max_speed_error,(output.omega_rad_s-reference).abs().max().item())
        expected_v = before_v + quat_apply(before_q,output.force_b_N[:,0])/mass*DT
        expected_v[:,2] -= G*DT
        tick(output)
        max_velocity_error = max(max_velocity_error,(robot.data.root_lin_vel_w-expected_v).abs().max().item())
        rows.append([i*DT,command,voltage,*output.omega_rad_s[0].cpu().tolist(),
                     *output.force_b_N[0,0].cpu().tolist(),*output.torque_b_Nm[0,0].cpu().tolist(),
                     *robot.data.root_state_w[0].cpu().tolist()])
    assert max_speed_error < .02
    assert max_velocity_error < 2e-5, max_velocity_error
    with (out/'step_response.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['command_time_s','command_counts','voltage_V']+
          [f'omega_{i}_rad_s' for i in range(1,5)]+['Fx_N','Fy_N','Fz_N','Mx_Nm','My_Nm','Mz_Nm']+
          ['x_m','y_m','z_m','qw','qx','qy','qz','vx_m_s','vy_m_s','vz_m_s','wx_rad_s','wy_rad_s','wz_rad_s'])
        writer.writerows(rows)
    passed('actuator_steps_through_physx', duration_s=.3, commands=[1250,1450,1250],
           max_scalar_speed_difference_rad_s=max_speed_error, max_one_step_velocity_error_m_s=max_velocity_error)

    # Flush a pending command in only two environments, preserving the other six.
    reset()
    for _ in range(7):tick(motors.step([1450]*4,7.8))
    unreset_state=robot.data.root_state_w[1].clone()
    ids=torch.tensor([0,2],device=args.device)
    reset(ids)
    torch.testing.assert_close(robot.data.root_state_w[1],unreset_state)
    commands=torch.full((N,4),1450.,device=args.device);commands[ids]=1250
    output=motors.step(commands,7.8)
    low=motors.steady_omega([1250]*4,7.8)
    torch.testing.assert_close(output.omega_rad_s[ids],low[ids],rtol=1e-6,atol=.002)
    tick(output)
    # Explicit zero wrench must remove previously buffered forces and moments.
    reset();tick(hover);tick(zero)
    torch.testing.assert_close(robot.data.root_lin_vel_w[:,2],torch.full((N,),-G*DT,device=args.device),atol=1e-6,rtol=1e-4)
    passed('partial_resets_and_zero_wrench', reset_env_ids=[0,2], independent_environments=N)

    reset()
    import omni.replicator.core as rep
    camera=rep.create.camera(position=(.17,-.17,1.15),look_at=(0,0,1),clipping_range=(.001,100))
    product=rep.create.render_product(camera,(960,720))
    annotator=rep.AnnotatorRegistry.get_annotator('rgb');annotator.attach([product])
    for _ in range(24):sim.render()
    rgb=np.asarray(annotator.get_data())
    assert rgb.shape[:2]==(720,960) and rgb[:,:,:3].std()>5
    Image.fromarray(rgb).save(out/'render.png')
    passed('native_render', resolution=[960,720], file='render.png')
    annotator.detach()
    report['status']='passed'
    save_report()
    print('NATIVE ISAAC VALIDATION PASSED:',out/'report.json',flush=True)


exit_code = 0
try:
    run()
except BaseException as error:
    exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
    report['status']='failed'
    report['error']=traceback.format_exc()
    save_report()
    traceback.print_exc()
    raise
finally:
    simulation = sim_utils.SimulationContext.instance()
    if simulation is not None:
        # Avoid the interactive stop callback waiting for a user in a headless run.
        simulation._disable_app_control_on_stop_handle = True
        simulation.stop()
        simulation.clear_all_callbacks()
    # Kit fast shutdown terminates Python; preserve the validation exit status.
    app.app.post_quit(exit_code)
    app.close(wait_for_replicator=False)
