"""Batched Sanity motor response and body wrench for Isaac Sim / Isaac Lab.

Requires PyTorch, not an Isaac installation. Commands are MSP bench counts in
hardware order M1..M4; forces are N and moments are Nm in URDF base_link axes.
Call reset() for already-spinning initial conditions, then step() once per
physics timestep. No joint-drive model, flight controller or hardware I/O.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import torch

from sanity_actuator import SanityModel

ROOT = Path(__file__).resolve().parent


@dataclass
class RotorOutput:
    omega_rad_s: torch.Tensor       # (environments, 4), M1..M4
    thrust_N: torch.Tensor          # (environments, 4), M1..M4
    force_b_N: torch.Tensor         # (environments, 1, 3), base_link axes
    torque_b_Nm: torch.Tensor       # (environments, 1, 3), about the CoM


class SanityIsaacActuator:
    """Fixed-step, vectorised implementation of the measured spinning model.

    output_to_frame lists the URDF frame for each of M1..M4. It has no guessed
    default. reaction_torque_ratio_m gives four SIGNED body reaction moments
    per unit thrust (Nm/N), in the same output order. Positive means a moment
    about the rotor's +Z axis. Supply it, or explicitly omit reaction torque.

    delay_s defaults to zero: the bench's apparent delay is not established as
    flight-path latency. Use SanityModel().L to reproduce the bench model.
    validate_inputs=False avoids device synchronisation for range checks; the
    caller then owns enforcement of the documented calibrated input ranges.
    """

    def __init__(self, num_envs, dt_s, *, output_to_frame, device='cpu',
                 reaction_torque_ratio_m=None, omit_reaction_torque=False,
                 delay_s=0.0, dtype=torch.float32, validate_inputs=True,
                 parameters_path=None, urdf_path=None):
        if not isinstance(num_envs, int) or num_envs <= 0:
            raise ValueError('num_envs must be a positive integer')
        if not math.isfinite(dt_s) or dt_s <= 0 or not math.isfinite(delay_s) or delay_s < 0:
            raise ValueError('Positive dt_s and nonnegative delay_s required')
        if dtype not in (torch.float32, torch.float64):
            raise ValueError('Use torch.float32 or torch.float64')
        self.num_envs, self.dt_s = num_envs, float(dt_s)
        self.device, self.dtype = torch.device(device), dtype
        self.validate_inputs = validate_inputs
        self.model = SanityModel(parameters_path)
        self.tau_s, self.delay_s = self.model.tau, float(delay_s)
        self.k = self.model.force['k_N_per_rad_s_squared']
        self.coefficients = self._tensor(self.model.speed['coefficients_a_b_c_rad_s'])

        urdf = ET.parse(urdf_path or ROOT / 'models/sanity.urdf').getroot()
        base = urdf.find("link[@name='base_link']")
        self.mass_kg = float(base.find('inertial/mass').get('value'))
        self.com_b_m = self._tensor([float(v) for v in base.find('inertial/origin').get('xyz').split()])
        joints = {j.find('child').get('link'): j for j in urdf.findall('joint')}
        frames = list(output_to_frame)
        if len(frames) != 4 or set(frames) != set(self.model.data['rotor_frames']):
            raise ValueError('output_to_frame must assign each of the four URDF rotor frames once to M1..M4')
        positions, axes = [], []
        for name in frames:
            joint = joints[name]
            if joint.get('type') != 'fixed' or joint.find('parent').get('link') != 'base_link':
                raise ValueError('Rotor frames must be fixed directly to base_link')
            origin = joint.find('origin')
            positions.append([float(v) for v in origin.get('xyz', '0 0 0').split()])
            r, p, y = [float(v) for v in origin.get('rpy', '0 0 0').split()]
            # URDF extrinsic XYZ rotation applied to the local thrust axis +Z.
            axes.append([math.cos(y)*math.sin(p)*math.cos(r) + math.sin(y)*math.sin(r),
                         math.sin(y)*math.sin(p)*math.cos(r) - math.cos(y)*math.sin(r),
                         math.cos(p)*math.cos(r)])
        self.output_to_frame = tuple(frames)
        self.positions_b_m, self.axes_b = self._tensor(positions), self._tensor(axes)
        self.levers_b_m = self.positions_b_m - self.com_b_m
        if reaction_torque_ratio_m is None:
            if not omit_reaction_torque:
                raise ValueError('Provide signed reaction_torque_ratio_m, or explicitly set omit_reaction_torque=True')
            reaction_torque_ratio_m = [0.0] * 4
        elif omit_reaction_torque:
            raise ValueError('Choose a torque ratio or omission, not both')
        self.reaction_torque_ratio_m = self._tensor(reaction_torque_ratio_m)
        if self.reaction_torque_ratio_m.shape != (4,) or not torch.isfinite(self.reaction_torque_ratio_m).all():
            raise ValueError('Four finite signed reaction torque/thrust ratios required')
        self.reaction_torque_omitted = omit_reaction_torque

        # A fractional delay crosses at most one old command boundary per step.
        steps = self.delay_s / self.dt_s
        if math.isclose(steps, round(steps), rel_tol=0, abs_tol=1e-12):
            steps = float(round(steps))
        self._whole_delay = math.floor(steps)
        self._fraction_s = (steps - self._whole_delay) * self.dt_s
        self._history = torch.zeros((self._whole_delay + 2, num_envs, 4), device=self.device, dtype=dtype)
        self._head = 0
        self._omega = torch.zeros((num_envs, 4), device=self.device, dtype=dtype)
        self._initialized = torch.zeros(num_envs, device=self.device, dtype=torch.bool)

    def _tensor(self, value):
        return torch.as_tensor(value, device=self.device, dtype=self.dtype)

    def _commands_voltage(self, commands, voltage_V, count):
        commands = self._tensor(commands)
        if commands.shape == (4,):
            commands = commands.expand(count, 4)
        if commands.shape != (count, 4):
            raise ValueError('commands must have shape (4,) or (environments, 4)')
        voltage = self._tensor(voltage_V)
        if voltage.ndim == 0:
            voltage = voltage.expand(count, 1)
        elif voltage.shape == (count,):
            voltage = voltage[:, None]
        if voltage.shape != (count, 1):
            raise ValueError('voltage_V must be scalar, (environments,) or (environments, 1)')
        if self.validate_inputs:
            lo, hi = self.model.speed['command_range_counts']
            vlo, vhi = self.model.speed['voltage_range_V']
            if not (torch.isfinite(commands).all() and ((commands >= lo) & (commands <= hi)).all()):
                raise ValueError('Spinning command map covers 1250..1450 MSP counts; no startup/stop model')
            if not (torch.isfinite(voltage).all() and ((voltage >= vlo) & (voltage <= vhi)).all()):
                raise ValueError('Voltage outside the measured command-map envelope')
        return commands, voltage

    def _steady(self, commands, voltage):
        q = (commands - 1350) / 100
        a, b, c = self.coefficients.unbind(dim=1)
        return (voltage / 8) * (a + b*q + c*q*q)

    def steady_omega(self, commands, voltage_V):
        """Steady speed in rad/s, with the same limits as SanityModel."""
        command, voltage = self._commands_voltage(commands, voltage_V, self.num_envs)
        return self._steady(command, voltage)

    @torch.no_grad()
    def reset(self, commands, voltage_V, *, env_ids=None, initial_omega_rad_s=None):
        """Reset selected environments and flush their delayed commands.

        Inputs describe only the selected environments, or broadcast from one
        four-value command and scalar voltage. Default initial speed is steady.
        """
        ids = torch.arange(self.num_envs, device=self.device) if env_ids is None else torch.as_tensor(
            env_ids, device=self.device, dtype=torch.long)
        if ids.ndim != 1 or (ids < 0).any() or (ids >= self.num_envs).any() or ids.unique().numel() != ids.numel():
            raise ValueError('env_ids must be distinct valid environment indices')
        command, voltage = self._commands_voltage(commands, voltage_V, ids.numel())
        omega = self._steady(command, voltage) if initial_omega_rad_s is None else self._tensor(initial_omega_rad_s)
        if omega.shape == (4,):
            omega = omega.expand(ids.numel(), 4)
        if omega.shape != (ids.numel(), 4) or not torch.isfinite(omega).all() or (omega <= 0).any():
            raise ValueError('Initial speed must contain four positive finite values per selected environment')
        self._check_speeds(omega)
        self._omega[ids] = omega
        self._history[:, ids] = command
        self._initialized[ids] = True

    def _check_speeds(self, omega):
        if self.validate_inputs:
            lo, hi = self.model.force['measured_speed_range_rad_s']
            valid = torch.isfinite(omega) & ((omega == 0) | ((omega >= lo) & (omega <= hi)))
            if not valid.all():
                raise ValueError('Nonzero rotor speed outside the measured static envelope')

    def wrench_from_speeds(self, omega_rad_s):
        """Static square law and force allocation; does not advance motor state."""
        omega = self._tensor(omega_rad_s)
        if omega.shape == (4,):
            omega = omega.expand(self.num_envs, 4)
        if omega.shape != (self.num_envs, 4):
            raise ValueError('omega_rad_s must have shape (4,) or (environments, 4)')
        self._check_speeds(omega)
        thrust = self.k * omega.square()
        forces = thrust[..., None] * self.axes_b
        torques = torch.linalg.cross(self.levers_b_m.expand_as(forces), forces, dim=-1)
        torques = torques + (thrust * self.reaction_torque_ratio_m)[..., None] * self.axes_b
        return RotorOutput(omega.clone(), thrust, forces.sum(dim=1, keepdim=True), torques.sum(dim=1, keepdim=True))

    @torch.no_grad()
    def step(self, commands, voltage_V):
        """Advance by dt_s; return endpoint speed, thrust and body-frame wrench.

        Commands are held over this physics step. Voltage acts immediately and
        is held constant over the step, matching the scalar reference model.
        """
        command, voltage = self._commands_voltage(commands, voltage_V, self.num_envs)
        if self.validate_inputs and not self._initialized.all():
            raise RuntimeError('Call reset() for every environment before stepping')
        self._history[self._head] = command
        size = self._history.shape[0]
        for offset, duration in [(self._whole_delay + 1, self._fraction_s),
                                 (self._whole_delay, self.dt_s - self._fraction_s)]:
            if duration > 0:
                delayed = self._history[(self._head - offset) % size]
                target = self._steady(delayed, voltage)
                self._omega = target + (self._omega - target) * math.exp(-duration / self.tau_s)
        self._head = (self._head + 1) % size
        return self.wrench_from_speeds(self._omega)


def apply_to_isaaclab(robot, output, *, body_ids):
    """Buffer a wrench on ONE base body; call robot.write_data_to_sim() next.

    Uses the current wrench composer or the Isaac Lab 2.3 external-wrench API.
    The imported base axes must match the URDF and forces must act at its CoM.
    Do not also apply these forces at rotor positions: moments are already summed.
    """
    if len(body_ids) != 1:
        raise ValueError('Apply the summed wrench to exactly one base body')
    if output.force_b_N.dtype != torch.float32 or output.torque_b_Nm.dtype != torch.float32:
        raise ValueError('Isaac Lab physics inputs require float32 tensors')
    if hasattr(robot, 'permanent_wrench_composer'):
        setter = robot.permanent_wrench_composer.set_forces_and_torques
    else:
        setter = robot.set_external_force_and_torque
    setter(forces=output.force_b_N, torques=output.torque_b_Nm,
           body_ids=body_ids, is_global=False)
