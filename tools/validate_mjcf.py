#!/usr/bin/env python3
"""Check the MuJoCo conversion, relocated assets and explicit rotor-force inputs."""
import json
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from build_mjcf import build, origin

ROOT = Path(__file__).resolve().parents[1]


def vertices(path):
    return np.array([[float(x) for x in line.split()[1:4]]
                     for line in path.read_text().splitlines() if line.startswith('v ')])


def main():
    output = ROOT / 'build/validation'
    output.mkdir(parents=True, exist_ok=True)
    assert (ROOT / 'models/sanity.xml').read_text() == build(), 'Rebuild the XML from the URDF.'
    urdf = ET.parse(ROOT / 'models/sanity.urdf').getroot()
    base = urdf.find("link[@name='base_link']")
    inertial = base.find('inertial')
    mass = float(inertial.find('mass').get('value'))
    com, rotation = origin(inertial)
    a = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
    tensor = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']],
                       [a['ixz'], a['iyz'], a['izz']]])
    tensor = rotation.as_matrix() @ tensor @ rotation.as_matrix().T
    moments = np.linalg.eigvalsh(tensor)
    assert np.all(moments > 0) and np.all(moments.sum() - 2 * moments > 0)

    # Only the XML and its assets are copied: no source-tree path may be needed.
    with tempfile.TemporaryDirectory(prefix='sanity-mjcf-') as folder:
        clean = Path(folder)
        shutil.copy2(ROOT / 'models/sanity.xml', clean / 'sanity.xml')
        shutil.copytree(ROOT / 'models/meshes', clean / 'meshes')
        xml = ET.parse(clean / 'sanity.xml').getroot()
        for item in xml.findall('./asset/mesh'):
            asset = (clean / item.get('file')).resolve()
            assert asset.is_relative_to(clean) and asset.is_file()
        m = mujoco.MjModel.from_xml_path(str(clean / 'sanity.xml'))
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        assert (m.nbody, m.nq, m.nv, m.nu, m.nmesh, m.nsite) == (2, 7, 6, 8, 7, 4)
        assert m.njnt == 1 and m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
        assert np.isclose(m.body_mass.sum(), mass, rtol=0, atol=1e-12)
        assert np.allclose(m.body_ipos[1], com, rtol=0, atol=1e-12)
        q = m.body_iquat[1]
        r = Rotation.from_quat(q, scalar_first=True).as_matrix()
        assert np.allclose(r @ np.diag(m.body_inertia[1]) @ r.T, tensor, rtol=0, atol=1e-12)

        # MuJoCo recentres mesh vertices internally. Restore the compiled geom
        # transforms and compare both point sets to catch offsets/mirrors/scales.
        errors = {}
        bounds = {}
        for kind in ['visual', 'collision']:
            points = []
            for i, shape in enumerate(base.findall(kind)):
                geom = m.geom(f'{kind}_{i}').id
                mesh_id = m.geom_dataid[geom]
                start, count = m.mesh_vertadr[mesh_id], m.mesh_vertnum[mesh_id]
                matrix = d.geom_xmat[geom].reshape(3, 3)
                assert np.isclose(np.linalg.det(matrix), 1)
                actual = m.mesh_vert[start:start + count] @ matrix.T + d.geom_xpos[geom]
                mesh = shape.find('geometry/mesh')
                expected = vertices(ROOT / 'models' / mesh.get('filename'))
                expected *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                p, r = origin(shape)
                expected = expected @ r.as_matrix().T + p
                error = max(cKDTree(expected).query(actual)[0].max(),
                            cKDTree(actual).query(expected)[0].max())
                assert error < 2e-8, (kind, i, error)
                errors[f'{kind}_{i}'] = float(error)
                points.append(actual)
                if kind == 'visual':
                    color = np.fromstring(shape.find('material/color').get('rgba'), sep=' ')
                    assert np.allclose(m.mat_rgba[m.geom_matid[geom]], color, atol=1e-7)
                    assert m.geom_contype[geom] == m.geom_conaffinity[geom] == 0
                else:
                    assert m.geom_contype[geom] == m.geom_conaffinity[geom] == 1
            points = np.concatenate(points)
            bounds[kind] = (points.min(0), points.max(0))
        assert np.all(bounds['visual'][0] >= bounds['collision'][0] - 2e-8)
        assert np.all(bounds['visual'][1] <= bounds['collision'][1] + 2e-8)

        positions = []
        for joint in urdf.findall('joint'):
            site = m.site(joint.find('child').get('link')).id
            pos, rotation = origin(joint)
            assert np.allclose(d.site_xpos[site], pos, rtol=0, atol=1e-12)
            assert np.allclose(d.site_xmat[site].reshape(3, 3), rotation.as_matrix(), atol=1e-12)
            positions.append(pos)

        # Free fall confirms that the base is not welded to the world.
        gravity = m.opt.gravity.copy()
        assert np.allclose(d.qacc[:3], gravity, atol=1e-12)
        mujoco.mj_step(m, d)
        assert d.qpos[2] < 0

        mujoco.mj_resetData(m, d)
        d.ctrl[:4] = mass * -gravity[2] / 4
        for _ in range(250):
            mujoco.mj_step(m, d)
        drift = float(np.linalg.norm(d.qpos[:3]))
        assert drift < 1e-10 and np.allclose(d.qpos[3:], [1, 0, 0, 0], atol=1e-10)

        m.opt.gravity[:] = 0
        responses = []
        for i, pos in enumerate(positions):
            mujoco.mj_resetData(m, d)
            d.ctrl[i] = .1
            mujoco.mj_forward(m, d)
            expected = np.linalg.solve(tensor, np.cross(pos - com, [0, 0, .1]))
            assert np.allclose(d.qacc[:3], [0, 0, .1 / mass], atol=1e-10)
            assert np.allclose(d.qacc[3:], expected, atol=1e-10)
            responses.append(d.qacc[3:].tolist())
            for sign in [-1, 1]:
                mujoco.mj_resetData(m, d)
                d.ctrl[i + 4] = sign * .0001
                mujoco.mj_forward(m, d)
                assert np.allclose(d.qacc[:3], 0, atol=1e-10)
                assert np.allclose(d.qacc[3:], np.linalg.solve(tensor, [0, 0, sign * .0001]), atol=1e-10)
        # Rotor thrust follows body orientation instead of staying world-up.
        mujoco.mj_resetData(m, d)
        tilt = Rotation.from_euler('y', 45, degrees=True)
        d.qpos[3:] = tilt.as_quat(scalar_first=True)
        d.ctrl[:4] = .1
        mujoco.mj_forward(m, d)
        assert np.allclose(d.qacc[:3], tilt.apply([0, 0, .4]) / mass, atol=1e-10)

        mujoco.mj_resetData(m, d)
        m.opt.gravity[:] = gravity
        mujoco.mj_forward(m, d)
        m.vis.global_.offwidth, m.vis.global_.offheight = 800, 600
        m.vis.headlight.ambient[:] = .7
        m.vis.headlight.diffuse[:] = .9
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0, 0, 0]
        camera.distance, camera.azimuth, camera.elevation = .24, 135, -30
        view = mujoco.MjvOption()
        view.sitegroup[:] = 0
        with mujoco.Renderer(m, height=600, width=800) as renderer:
            for collision in [False, True]:
                view.geomgroup[:] = 0
                view.geomgroup[1] = 1
                view.geomgroup[3] = int(collision)
                renderer.update_scene(d, camera=camera, scene_option=view)
                frame = renderer.render().copy()
                assert frame.std() > 5
                name = 'mjcf_collision.png' if collision else 'mjcf_visual.png'
                Image.fromarray(frame).save(output / name)

    report = dict(mujoco_version=mujoco.__version__, relocated_asset_load=True,
                  free_base=True, mass_kg=mass, com_m=com.tolist(), inertia_kg_m2=tensor.tolist(),
                  compiled_mesh_max_errors_m=errors, visual_collision_alignment=True,
                  rotor_sites_match=True, free_fall=True, hover_duration_s=.5, hover_drift_m=drift,
                  single_rotor_angular_acceleration_rad_s2=responses,
                  signed_reaction_torque_inputs=True, body_frame_thrust_under_tilt=True,
                  hardware_yaw_coefficient_validated=False, flight_controller_model=False)
    (output / 'mjcf.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
