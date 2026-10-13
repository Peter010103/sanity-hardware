"""Offline scalar parity, force allocation, reset isolation and CPU/GPU checks."""
import importlib.util
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

from sanity_actuator import SanityModel, SpinningModel

HAS_TORCH = importlib.util.find_spec('torch') is not None
if HAS_TORCH:
    import torch
    from sanity_isaac import SanityIsaacActuator, apply_to_isaaclab

ROOT = Path(__file__).resolve().parents[1]
# Synthetic wiring for tests only. This does not establish the hardware map.
FRAMES = ['rotor_3_front_left', 'rotor_1_rear_right', 'rotor_4_rear_left', 'rotor_2_front_right']


@unittest.skipUnless(HAS_TORCH, 'Install PyTorch to check the optional Isaac adapter')
class IsaacActuatorTests(unittest.TestCase):
    def actuator(self, n=3, dt=.002, **kwargs):
        kwargs.setdefault('omit_reaction_torque', True)
        kwargs.setdefault('dtype', torch.float64)
        return SanityIsaacActuator(n, dt, output_to_frame=FRAMES, **kwargs)

    def test_matches_scalar_with_fractional_delay_and_changing_voltage(self):
        reference = SanityModel()
        for delay in [0., .004, reference.L, .0008]:
            reference.L = delay
            batch = self.actuator(delay_s=delay)
            batch.reset([1250]*4, 7.8)
            scalar = [SpinningModel(reference, [1250]*4, 7.8) for _ in range(3)]
            for i in range(80):
                commands = [[1250 + ((i//3 + e + j) % 5)*50 for j in range(4)] for e in range(3)]
                voltages = [7.8 - .001*i + .03*e for e in range(3)]
                expected = [s.step(.002, c, v) for s, c, v in zip(scalar, commands, voltages)]
                actual = batch.step(commands, voltages)
                torch.testing.assert_close(actual.omega_rad_s, torch.tensor(expected, dtype=torch.float64), rtol=1e-12, atol=1e-9)
                torch.testing.assert_close(actual.thrust_N, batch.k*actual.omega_rad_s.square())

    def test_speed_lag_then_square_and_explicit_delay(self):
        batch = self.actuator(n=1, dt=.01, delay_s=.015)
        batch.reset([1250]*4, 7.8)
        low = batch.steady_omega([1250]*4, 7.8)
        high = batch.steady_omega([1450]*4, 7.8)
        torch.testing.assert_close(batch.step([1450]*4, 7.8).omega_rad_s, low)
        actual = batch.step([1450]*4, 7.8)
        expected = high + (low-high)*math.exp(-.005/batch.tau_s)
        torch.testing.assert_close(actual.omega_rad_s, expected, rtol=1e-12, atol=1e-9)
        torch.testing.assert_close(actual.thrust_N, batch.k*expected.square())

    def test_partial_reset_clears_only_selected_histories(self):
        batch = self.actuator(n=2, delay_s=.0193)
        reference = SanityModel(); reference.L = .0193
        scalar = [SpinningModel(reference, [1250]*4, 7.8) for _ in range(2)]
        batch.reset([1250]*4, 7.8)
        for _ in range(7):
            batch.step([1450]*4, 7.8)
            for s in scalar: s.step(.002, [1450]*4, 7.8)
        batch.reset([1350]*4, 7.8, env_ids=[0])
        scalar[0] = SpinningModel(reference, [1350]*4, 7.8)
        for _ in range(25):
            commands = [[1350]*4, [1450]*4]
            actual = batch.step(commands, 7.8).omega_rad_s
            expected = [s.step(.002, c, 7.8) for s, c in zip(scalar, commands)]
            torch.testing.assert_close(actual, torch.tensor(expected, dtype=torch.float64), rtol=1e-12, atol=1e-9)

    def test_force_balance_and_single_rotor_moment_signs(self):
        batch = self.actuator(n=1)
        hover = math.sqrt(batch.mass_kg*9.80665/(4*batch.k))
        output = batch.wrench_from_speeds([hover]*4)
        self.assertAlmostEqual(output.force_b_N[0, 0, 2].item(), batch.mass_kg*9.80665, places=12)
        torch.testing.assert_close(output.torque_b_Nm, torch.zeros_like(output.torque_b_Nm), atol=1e-12, rtol=0)
        arm = .0284610479
        positions = [(arm,arm), (-arm,-arm), (-arm,arm), (arm,-arm)]
        for i, (x,y) in enumerate(positions):
            speed = [0]*4; speed[i] = math.sqrt(.1/batch.k)
            output = batch.wrench_from_speeds(speed)
            torch.testing.assert_close(output.force_b_N[0,0], torch.tensor([0.,0.,.1], dtype=torch.float64))
            torch.testing.assert_close(output.torque_b_Nm[0,0], torch.tensor([y*.1,-x*.1,0.], dtype=torch.float64))

    def test_com_rotor_axis_rotation_and_signed_reaction_moment(self):
        # A deliberately altered fixture tests transforms without editing the asset.
        tree = ET.parse(ROOT/'models/sanity.urdf')
        tree.find("link[@name='base_link']/inertial/origin").set('xyz', '.001 .002 .003')
        tree.find("joint[@name='rotor_3_frame']/origin").set('rpy', f'0 {math.pi/2} 0')
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'test.urdf'; tree.write(path)
            batch=self.actuator(n=1, urdf_path=path, omit_reaction_torque=False,
                                reaction_torque_ratio_m=[.02,-.02,.02,-.02])
        output=batch.wrench_from_speeds([math.sqrt(.1/batch.k),0,0,0])
        # +Z rotated +90deg about Y gives +X. r x F plus +X reaction torque.
        expected=torch.tensor([.002,-.0003,-(.0284610479-.002)*.1], dtype=torch.float64)
        torch.testing.assert_close(output.force_b_N[0,0], torch.tensor([.1,0.,0.], dtype=torch.float64), atol=1e-12, rtol=0)
        torch.testing.assert_close(output.torque_b_Nm[0,0], expected, atol=1e-12, rtol=0)

    def test_invalid_inputs_and_unconfirmed_configuration_not_assumed(self):
        with self.assertRaises(ValueError):
            self.actuator(omit_reaction_torque=False)
        with self.assertRaises(ValueError):
            SanityIsaacActuator(1,.002,output_to_frame=[FRAMES[0]]*4,omit_reaction_torque=True)
        batch=self.actuator(n=1)
        with self.assertRaises(RuntimeError): batch.step([1350]*4,7.8)
        for commands, voltage in [([1000]*4,7.8),([1750]*4,7.8),([1350]*4,6.5),([float('nan')]*4,7.8)]:
            with self.assertRaises(ValueError): batch.reset(commands,voltage)
        with self.assertRaises(ValueError): batch.reset([1350]*4,7.8,initial_omega_rad_s=[0]*4)
        with self.assertRaises(ValueError): batch.wrench_from_speeds([6000]*4)

    def test_isaac_api_contract_uses_one_body_and_local_frame(self):
        batch=self.actuator(n=2,dtype=torch.float32)
        output=batch.wrench_from_speeds([2200]*4)
        calls=[]
        def record(**kwargs): calls.append(kwargs)
        for robot in [SimpleNamespace(set_external_force_and_torque=record),
                      SimpleNamespace(permanent_wrench_composer=SimpleNamespace(set_forces_and_torques=record))]:
            apply_to_isaaclab(robot,output,body_ids=[3])
            self.assertIs(calls[-1]['is_global'],False)
            self.assertEqual(calls[-1]['body_ids'],[3])
            self.assertEqual(calls[-1]['forces'].shape,(2,1,3))
            self.assertNotIn('positions',calls[-1])
        with self.assertRaises(ValueError): apply_to_isaaclab(robot,output,body_ids=[0,1])

    def test_cuda_batch_matches_cpu(self):
        if not torch.cuda.is_available(): self.skipTest('CUDA unavailable')
        cpu=self.actuator(n=4096,dtype=torch.float32,delay_s=.0193)
        gpu=self.actuator(n=4096,dtype=torch.float32,delay_s=.0193,device='cuda')
        commands=torch.linspace(1250,1450,4096*4).reshape(4096,4)
        voltage=torch.linspace(7.6,8.0,4096)
        for model in [cpu,gpu]: model.reset([1250]*4,7.8)
        for _ in range(30):
            a=cpu.step(commands,voltage)
            b=gpu.step(commands.to('cuda'),voltage.to('cuda'))
        for name in ['omega_rad_s','thrust_N','force_b_N','torque_b_Nm']:
            torch.testing.assert_close(getattr(a,name),getattr(b,name).cpu(),rtol=2e-5,atol=2e-6)


if __name__ == '__main__': unittest.main()
