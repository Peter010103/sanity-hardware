"""Explicit R2 high-speed reference preparation, without hardware or network I/O.

The high-speed levels retain the Gerono geometry and requested peak speed. Their
reference limits are derived from the existing analytic triangle bounds at the
requested two-second ramp, rather than reusing the 1.5 m/s reference limits.
The screen is a model calculation, not a flight qualification.
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation

from sanity_zenoh.indi_lemniscate import IndiShapedAngularLemniscate, ButterflyReference

SPEED_LEVELS = (3., 4., 5., 6.)


def reference_limits(speed, radius=2., ramp=2., width_m=None,vertical_amplitude_m=0.):
    width=radius if width_m is None else width_m
    if any(isinstance(x,bool) or not math.isfinite(x) or x<=0 for x in (speed,radius,ramp,width)):raise ValueError("Positive finite geometry, speed and ramp required")
    if isinstance(vertical_amplitude_m,bool) or not math.isfinite(vertical_amplitude_m):raise ValueError("Finite signed vertical amplitude required")
    w2=width**2+4.*vertical_amplitude_m**2
    omega = speed / (math.sqrt(2.) * radius if width==radius and vertical_amplitude_m==0. else math.sqrt(radius**2+w2))
    b1,b2,b3 = radius*np.sqrt([2.,5.,17.]) if width==radius and vertical_amplitude_m==0. else np.sqrt([radius**2+w2,radius**2+4.*w2,radius**2+16.*w2])
    # Round outward, including the floating-point inversion in the ramp solver.
    acceleration = b2 * omega**2 + b1 * omega * (15./8.) / ramp
    jerk = b3 * omega**3 + 3.*b2*omega**2*(15./8.)/ramp + b1*omega*(10./math.sqrt(3.))/ramp**2
    return dict(maximum_acceleration_m_s2=float(acceleration*(1.+1e-12)),
                maximum_jerk_m_s3=float(jerk*(1.+1e-12)))


def make_reference(speed, center=(0., 0., 1.), *, width_m=None,butterfly_height_m=0.,butterfly_cycles=3):
    if isinstance(speed, bool) or speed not in SPEED_LEVELS:
        raise ValueError('Explicit high-speed lemniscate levels are 3,4,5,6 m/s')
    if butterfly_height_m:
        return ButterflyReference(center,2.,2. if width_m is None else width_m,speed,2.,cycles=butterfly_cycles,height_amplitude_m=butterfly_height_m,
            **reference_limits(speed,width_m=width_m,vertical_amplitude_m=butterfly_height_m))
    return IndiShapedAngularLemniscate(center, 2., speed, 2., laps=3,
                                      width_m=width_m,**reference_limits(speed,width_m=width_m))


def screen_reference(speed, dt=.002, *, mass=.07151, eta=.953, width_m=None,butterfly_height_m=0.,butterfly_cycles=3):
    """Sample the full reference and solve its fixed-yaw thrust/drag equilibrium.

    Uses the fitted body drag (.587,.820,0)/s with full 3D rotation. The rates
    are finite differences of that equilibrium attitude, not jerk/g proxies.
    This remains a zero-attitude-error calculation without rate-loop dynamics,
    tracking corrections, motor allocation or thrust transients.
    """
    curve = make_reference(speed,width_m=width_m,butterfly_height_m=butterfly_height_m,butterfly_cycles=butterfly_cycles)
    t = np.linspace(0., curve.duration, math.ceil(curve.duration/dt)+1)
    samples = [curve.sample(float(value)) for value in t]
    v, a, j = (np.asarray([sample[key] for sample in samples])
               for key in ('velocity', 'acceleration', 'jerk'))
    gravity = np.array([0., 0., 9.81])
    force = a + gravity
    x = np.broadcast_to([1., 0., 0.], force.shape)
    for _ in range(80):
        b3 = force / np.linalg.norm(force, axis=1)[:, None]
        b2 = np.cross(b3, x); b2 /= np.linalg.norm(b2, axis=1)[:, None]
        b1 = np.cross(b2, b3)
        rotation = np.stack([b1, b2, b3], axis=2)
        body_velocity = np.einsum('nji,nj->ni', rotation, v)
        drag = np.einsum('nij,nj->ni', rotation, body_velocity * [.587, .820, 0.])
        updated = a + gravity + drag
        residual = float(np.max(np.linalg.norm(updated-force, axis=1)))
        force = .5 * (force + updated)
        if residual < 1e-11: break
    if residual > 1e-7:
        raise ValueError('Drag/attitude equilibrium did not converge')
    thrust = np.linalg.norm(force, axis=1) * mass / eta
    tilt = np.degrees(np.arccos(np.clip(rotation[:, 2, 2], -1., 1.)))
    increments = np.einsum('nji,njk->nik', rotation[:-1], rotation[1:])
    rates = Rotation.from_matrix(increments).as_rotvec() / np.diff(t)[:, None]
    axis_rates = np.max(np.abs(rates), axis=0)
    ordinary_violations = []
    if max(tilt) > 30.: ordinary_violations.append('MPC tilt limit 30 deg')
    if max(thrust) > mass*9.81*1.5: ordinary_violations.append('airborne thrust cap 1.5 weight')
    if max(axis_rates) > 2.3: ordinary_violations.append('SZC4 selected-INDI body-rate cap 2.3 rad/s')
    return dict(scope='Sampled fixed-yaw full-rotation reference equilibrium with fitted body drag; not measured flight or closed-loop guarantee',
        speed_m_s=float(speed), radius_m=2.,width_m=curve.width, footprint_xy_m=[4.,curve.width], center_xy_m=[0.,0.], height_m=1., laps=curve.laps,butterfly_cycles=butterfly_cycles if butterfly_height_m else None,butterfly_height_m=butterfly_height_m,
        duration_s=curve.duration, effective_ramp_s=curve.ramp, sample_dt_max_s=float(max(np.diff(t))),
        speed_peak_m_s=float(max(np.linalg.norm(v,axis=1))),
        acceleration_peak_m_s2=float(max(np.linalg.norm(a,axis=1))),
        jerk_peak_m_s3=float(max(np.linalg.norm(j,axis=1))),
        tilt_peak_deg=float(max(tilt)), thrust_peak_N=float(max(thrust)),
        thrust_peak_weight=float(max(thrust)/(mass*9.81)), body_rate_axis_peak_rad_s=axis_rates.tolist(),
        equilibrium_residual_m_s2=residual, ordinary_envelope_violations=ordinary_violations,
        reference_design=curve.design_metadata,
        qualification='offline model study; high-speed small-angle identified rate response is unvalidated')
