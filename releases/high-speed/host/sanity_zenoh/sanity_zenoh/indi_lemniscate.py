"""Pure angular lemniscate references and full-INDI mission routing."""
from dataclasses import asdict, dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

from sanity_zenoh.full_indi_mission import FullIndiMission


@dataclass(frozen=True)
class LemniscateOptions:
    center_xy_m: tuple = (0., 0.)
    radius_m: float = 1.
    peak_speed_m_s: float = .8
    ramp_s: float = 2.
    transfer_speed_m_s: float = .4
    entry_dwell_s: float = 1.
    entry_timeout_s: float = 10.
    entry_position_m: float = .15
    entry_speed_m_s: float = .25
    entry_tilt_deg: float = 6.
    final_hold_s: float = 2.
    laps: int = 1

    def __post_init__(self):
        center = np.asarray(self.center_xy_m, dtype=float)
        values = [value for key, value in vars(self).items() if key != 'center_xy_m']
        if center.shape != (2,) or not np.isfinite(center).all() or not all(type(value) is not bool and math.isfinite(value) and value > 0 for value in values):
            raise ValueError('Finite XY center and positive finite trajectory settings required')
        if self.entry_timeout_s <= max(self.entry_dwell_s, self.final_hold_s) or self.transfer_speed_m_s > .4 or self.peak_speed_m_s > 1.:
            raise ValueError('Entry timeout must exceed dwell; transfer<=0.4m/s and lap<=1m/s')
        if type(self.laps) is not int or self.laps < 1:
            raise ValueError('A positive integer lap count is required')
        object.__setattr__(self, 'center_xy_m', tuple(float(value) for value in center))


class QuinticTransfer:
    def __init__(self, start, end, maximum_speed=.4):
        self.start, self.end = np.array(start, dtype=float), np.array(end, dtype=float)
        if self.start.shape != (3,) or self.end.shape != (3,) or not np.isfinite([self.start, self.end]).all() or not math.isfinite(maximum_speed) or maximum_speed <= 0:
            raise ValueError('Finite transfer endpoints and positive speed required')
        self.delta = self.end - self.start
        self.duration = max(2., 1.875 * float(np.linalg.norm(self.delta)) / maximum_speed)

    def sample(self, elapsed):
        if not math.isfinite(elapsed):raise ValueError('Finite reference time required')
        u = float(np.clip(elapsed / self.duration, 0., 1.))
        position = self.start + self.delta * (u ** 3 * (10. - 15. * u + 6. * u * u))
        velocity = self.delta * 30. * u * u * (1. - u) ** 2 / self.duration
        acceleration = self.delta * 60. * u * (1. - u) * (1. - 2. * u) / self.duration ** 2
        jerk = self.delta * 60. * (1. - 6. * u + 6. * u * u) / self.duration ** 3 if 0. <= elapsed <= self.duration else np.zeros(3)
        return dict(position=position.tolist(), velocity=velocity.tolist(), acceleration=acceleration.tolist(), jerk=jerk.tolist(), phase='transfer')


class IndiShapedTransfer:
    """C3 straight-line transfer with analytic speed/acceleration/jerk bounds."""
    def __init__(self,start,end,maximum_speed=.4,maximum_acceleration_m_s2=1.5,maximum_jerk_m_s3=2.):
        self.start,self.end=np.asarray(start,dtype=float),np.asarray(end,dtype=float)
        limits=(maximum_speed,maximum_acceleration_m_s2,maximum_jerk_m_s3)
        if (self.start.shape!=(3,) or self.end.shape!=(3,) or not np.isfinite([self.start,self.end]).all()
                or any(isinstance(v,bool) or not math.isfinite(v) or v<=0 for v in limits)):
            raise ValueError('Finite transfer endpoints and positive shaped-reference limits required')
        self.delta=self.end-self.start;distance=float(np.linalg.norm(self.delta))
        # Exact maxima of derivatives of 35u^4-84u^5+70u^6-20u^7.
        max_first,max_second,max_third=35./16.,84.*math.sqrt(5.)/25.,52.5
        self.duration=max(2.,max_first*distance/maximum_speed,
            math.sqrt(max_second*distance/maximum_acceleration_m_s2),
            (max_third*distance/maximum_jerk_m_s3)**(1./3.))
        self.design_metadata=dict(kind='indi_shaped_transfer',duration_s=self.duration,
            speed_bound_m_s=max_first*distance/self.duration,
            acceleration_bound_m_s2=max_second*distance/self.duration**2,
            jerk_bound_m_s3=max_third*distance/self.duration**3,
            scope='C3 reference transfer; not measured aircraft tracking')

    def sample(self,elapsed):
        if not math.isfinite(elapsed):raise ValueError('Finite transfer time required')
        u=float(np.clip(elapsed/self.duration,0.,1.))
        s=u**4*(35.-84.*u+70.*u**2-20.*u**3)
        ds=140.*u**3*(1.-u)**3
        dds=420.*u**2*(1.-u)**2*(1.-2.*u)
        ddds=840.*u*(1.-u)*(1.-5.*u+5.*u**2)
        return dict(position=(self.start+self.delta*s).tolist(),velocity=(self.delta*ds/self.duration).tolist(),
            acceleration=(self.delta*dds/self.duration**2).tolist(),jerk=(self.delta*ddds/self.duration**3).tolist(),phase='transfer')


class AngularLemniscate:
    def __init__(self, center, radius=1., peak_speed=1., ramp_s=2., *, laps=1):
        self.center = np.array(center, dtype=float)
        if self.center.shape != (3,) or not np.isfinite(self.center).all() or not all(math.isfinite(value) and value > 0 for value in (radius, peak_speed, ramp_s)):
            raise ValueError('Finite center and positive radius, speed and ramp required')
        self.radius, self.ramp = float(radius), float(ramp_s)
        if type(laps) is not int or laps < 1:raise ValueError('A positive integer lap count is required')
        self.laps = laps
        self.total_angle = 2. * math.pi * laps
        self.omega = peak_speed / (math.sqrt(2.) * radius)
        self.cruise = self.total_angle / self.omega - self.ramp
        if self.cruise < 0:raise ValueError('Angular ramps cannot exceed the one-lap angle')
        self.duration = self.cruise + 2. * self.ramp

    def sample(self, elapsed):
        if not math.isfinite(elapsed):raise ValueError('Finite reference time required')
        t = float(np.clip(elapsed, 0., self.duration))
        if t < self.ramp or t > self.duration - self.ramp:
            rising = t < self.ramp
            u = (t if rising else self.duration - t) / self.ramp
            integral = 2.5 * u ** 4 - 3. * u ** 5 + u ** 6
            theta = self.omega * self.ramp * integral if rising else self.total_angle - self.omega * self.ramp * integral
            speed = self.omega * u ** 3 * (10. - 15. * u + 6. * u * u)
            acceleration = (1. if rising else -1.) * self.omega * 30. * u * u * (1. - u) ** 2 / self.ramp
            angular_jerk = self.omega * 60. * u * (1. - u) * (1. - 2. * u) / self.ramp ** 2
        else:
            theta = self.omega * (t - .5 * self.ramp)
            speed, acceleration, angular_jerk = self.omega, 0., 0.
        position = self.center + self.radius * np.array([math.sin(theta), .5 * math.sin(2. * theta), 0.])
        first = self.radius * np.array([math.cos(theta), math.cos(2. * theta), 0.])
        second = self.radius * np.array([-math.sin(theta), -2. * math.sin(2. * theta), 0.])
        third = self.radius * np.array([-math.cos(theta), -4. * math.cos(2. * theta), 0.])
        velocity = first * speed
        linear_acceleration = second * speed ** 2 + first * acceleration
        jerk = third * speed ** 3 + 3. * second * speed * acceleration + first * angular_jerk
        if elapsed <= 0. or elapsed >= self.duration:
            position, velocity, linear_acceleration, jerk = self.center.copy(), np.zeros(3), np.zeros(3), np.zeros(3)
        return dict(position=position.tolist(), velocity=velocity.tolist(), acceleration=linear_acceleration.tolist(), jerk=jerk.tolist(),
                    phase='lemniscate', theta_rad=theta, lap_progress=theta / (2. * math.pi), completed_laps=min(self.laps, int(theta / (2. * math.pi))))


class IndiShapedAngularLemniscate(AngularLemniscate):
    """Same Gerono curve/cruise speed, with analytically bounded ramp demands.

    For p(theta), derivative norm bounds are R*sqrt(2), R*sqrt(5),
    R*sqrt(17). Angular speed uses the existing quintic smoothstep, whose
    first/second derivative maxima are 15/8 and 10/sqrt(3). Triangle bounds
    on p''*theta_dot^2+p'*theta_ddot and the corresponding jerk determine
    a sufficient ramp duration; sampled extrema are not used as guarantees.

    These are reference acceleration/jerk bounds, not motor, measured tracking
    or sensing guarantees. Peak path speed and all three laps are retained.
    """
    def __init__(self, center, radius=3.2, peak_speed=1.5, ramp_s=2., *, laps=3,
                 maximum_acceleration_m_s2=1.5, maximum_jerk_m_s3=2.):
        values=(radius,peak_speed,ramp_s,maximum_acceleration_m_s2,maximum_jerk_m_s3)
        if any(isinstance(v,bool) or not math.isfinite(v) or v<=0 for v in values):
            raise ValueError('Finite positive shaped-reference geometry and limits required')
        omega=peak_speed/(math.sqrt(2.)*radius)
        b1,b2,b3=radius*np.sqrt([2.,5.,17.])
        s1,s2=15./8.,10./math.sqrt(3.)
        cruise_acceleration=b2*omega**2;cruise_jerk=b3*omega**3
        if maximum_acceleration_m_s2<=cruise_acceleration or maximum_jerk_m_s3<=cruise_jerk:
            raise ValueError('Requested cruise peak speed cannot satisfy shaped reference limits')
        ramp_acceleration=b1*omega*s1/(maximum_acceleration_m_s2-cruise_acceleration)
        # J >= B3*w^3 + 3*B2*w^2*S1/ramp + B1*w*S2/ramp^2.
        margin=maximum_jerk_m_s3-cruise_jerk
        linear=3.*b2*omega**2*s1;constant=b1*omega*s2
        ramp_jerk=(linear+math.sqrt(linear**2+4.*margin*constant))/(2.*margin)
        ramp=max(float(ramp_s),float(ramp_acceleration),float(ramp_jerk))
        super().__init__(center,radius,peak_speed,ramp,laps=laps)
        self.design_metadata=dict(schedule='indi_shaped_angular',requested_ramp_s=float(ramp_s),
            effective_ramp_s=ramp,maximum_acceleration_m_s2=float(maximum_acceleration_m_s2),
            maximum_jerk_m_s3=float(maximum_jerk_m_s3),
            conservative_acceleration_bound_m_s2=float(cruise_acceleration+b1*omega*s1/ramp),
            conservative_jerk_bound_m_s3=float(cruise_jerk+linear/ramp+constant/ramp**2),
            angular_cruise_rad_s=float(omega),
            reference_rate_proxy_limit_rad_s=float(maximum_jerk_m_s3/9.81),
            rate_proxy_scope='Small-tilt fixed-yaw jerk/g proxy; not actual MPC/FC rates',
            geometry_and_peak_speed_preserved=True,
            sensing_scope='Reference shaping does not establish mocap coverage; selected radius and center determine geometry')


class LemniscateIndiMission(FullIndiMission):
    trajectory_kind = 'angular_lemniscate'
    options_class = LemniscateOptions
    tracking_phase = 'lemniscate'

    def __init__(self, *args, trajectory_options=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.trajectory_options = self.options_class(**(trajectory_options or {}))
        options = self.trajectory_options
        self.lap_duration_s = self._make_curve(np.r_[options.center_xy_m, 0.]).duration
        self.trajectory_stable_since = None
        self.segment = None

    def _make_curve(self, center):
        options = self.trajectory_options
        return AngularLemniscate(center, options.radius_m, options.peak_speed_m_s, options.ramp_s, laps=options.laps)

    def _path_start(self):
        return np.r_[self.trajectory_options.center_xy_m, self.home[2]]

    def _make_transfer(self,start,end):
        return QuinticTransfer(start,end,self.trajectory_options.transfer_speed_m_s)

    def _begin_lap_segment(self, now):
        self.segment = self._make_curve(np.r_[self.trajectory_options.center_xy_m, self.home[2]])
        self._phase(self.tracking_phase, now)

    def diagnostics(self, output):
        result = super().diagnostics(output)
        result['trajectory'] = dict(kind=self.trajectory_kind, phase=self.phase, options=asdict(self.trajectory_options), lap_duration_s=self.lap_duration_s,
                                    duration_scope='whole_speed_ramped_lap_segment', requested_laps=self.trajectory_options.laps)
        design=getattr(self.segment,'design_metadata',None)
        if design is not None:result['trajectory']['reference_design']=dict(design)
        return result

    def _flight_reference(self, now):
        if self.phase in ('transfer_to_center', self.tracking_phase, 'return_home'):
            reference = self.segment.sample(now - self.phase_started)
            reference['phase'] = self.phase
            return reference
        if self.phase in ('trajectory_entry', 'trajectory_hold'):
            return dict(position=self.home.tolist(), velocity=[0.] * 3, acceleration=[0.] * 3, phase=self.phase)
        return super()._flight_reference(now)

    def tick(self, now, state, control, **kwargs):
        output = super().tick(now, state, control, **kwargs)
        if self.done or self.phase == 'landing':return output
        options = self.trajectory_options
        if self.phase == 'indi_hover':
            self._phase('trajectory_entry', now)
            self.trajectory_stable_since = None
        if self.phase in ('trajectory_entry', 'trajectory_hold'):
            sample = np.asarray(state, dtype=float)
            rotation = Rotation.from_quat(sample[[4, 5, 6, 3]])
            tilt = math.degrees(math.acos(float(np.clip(rotation.as_matrix()[2, 2], -1., 1.))))
            stable = np.linalg.norm(sample[:3] - self.home) <= options.entry_position_m and np.linalg.norm(sample[7:10]) <= options.entry_speed_m_s and tilt <= options.entry_tilt_deg
            self.trajectory_stable_since = (now if self.trajectory_stable_since is None else self.trajectory_stable_since) if stable else None
        if self.phase == 'trajectory_entry':
            if self.trajectory_stable_since is not None and now - self.trajectory_stable_since >= options.entry_dwell_s:
                start = self._path_start()
                self.segment = self._make_transfer(self.home,start)
                self._phase('transfer_to_center', now)
            elif now - self.phase_started >= options.entry_timeout_s:
                self.request_landing('trajectory_entry_timeout')
        elif self.phase in ('transfer_to_center', self.tracking_phase, 'return_home') and now - self.phase_started >= self.segment.duration:
            if self.phase == 'transfer_to_center':
                self._begin_lap_segment(now)
            elif self.phase == self.tracking_phase:
                self.segment = self._make_transfer(self._path_start(),self.home)
                self._phase('return_home', now)
            else:
                self._phase('trajectory_hold', now)
                self.trajectory_stable_since = None
        elif self.phase == 'trajectory_hold':
            settled = self.trajectory_stable_since is not None and now - self.trajectory_stable_since >= options.entry_dwell_s
            if now - self.phase_started >= options.final_hold_s and settled:
                self.request_landing('test_complete')
            elif now - self.phase_started >= options.entry_timeout_s:
                self.request_landing('return_not_settled')
        output['phase'] = self.phase
        return output
