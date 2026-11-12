import importlib.util
import math
from pathlib import Path
import numpy as np


class PatternMission:
    def __init__(self, pattern, speed, radius=.5, laps=2, position_gain=1., command_speed_limit=1., ramp_s=2., hold_s=2., velocity_gain=0., acceleration_lead_s=0., vertical_gain=1., vertical_command_limit=.2, command_limit_mode="horizontal", yaw_gain=1., yaw_rate_limit=.6):
        if pattern not in ("circle", "lemniscate"):
            raise ValueError("Expected circle or lemniscate")
        values = (speed, radius, laps, position_gain, command_speed_limit, ramp_s, hold_s, vertical_gain, vertical_command_limit, yaw_gain, yaw_rate_limit)
        if not all(math.isfinite(v) and v > 0 for v in values) or laps != int(laps):
            raise ValueError("Mission parameters must be positive and finite; laps must be integral")
        if not all(math.isfinite(v) and v >= 0 for v in (velocity_gain, acceleration_lead_s)):
            raise ValueError("Velocity gain and acceleration lead must be finite and nonnegative")
        if command_limit_mode not in ("horizontal", "norm"):
            raise ValueError("Command limit mode must be horizontal or norm")
        if speed > command_speed_limit:
            raise ValueError("Reference peak speed exceeds command speed limit; select an explicit higher limit")
        self.vertical_gain, self.vertical_command_limit = vertical_gain, vertical_command_limit
        self.command_limit_mode = command_limit_mode
        self.yaw_gain, self.yaw_rate_limit = yaw_gain, yaw_rate_limit
        self.velocity_gain, self.acceleration_lead_s = velocity_gain, acceleration_lead_s
        self.pattern, self.speed, self.radius, self.laps = pattern, speed, radius, laps
        self.position_gain, self.command_speed_limit = position_gain, command_speed_limit
        self.ramp_s, self.hold_s = ramp_s, hold_s
        self.angles = np.linspace(0, 2 * np.pi, 20001)
        derivative = np.column_stack([radius * np.cos(self.angles), radius * (np.sin(self.angles) if pattern == "circle" else np.cos(2 * self.angles))])
        norms = np.linalg.norm(derivative, axis=1)
        self.arc = np.r_[0, np.cumsum((norms[1:] + norms[:-1]) * np.diff(self.angles) / 2)]
        self.length = float(self.arc[-1])
        self.distance = self.length * laps
        if speed * ramp_s > self.distance:
            raise ValueError("Mission too short for the requested speed and ramps")
        self.moving_s = self.distance / speed + ramp_s
        self.duration = self.moving_s + 2 * hold_s


    def reset(self):
        pass


    def reference(self, time_s):
        t = time_s - self.hold_s
        if t <= 0:
            distance, speed, phase = 0., 0., "settle"
        elif t < self.ramp_s:
            speed = self.speed * t / self.ramp_s
            distance, phase = .5 * speed * t, "accelerate"
        elif t < self.moving_s - self.ramp_s:
            speed, distance, phase = self.speed, self.speed * (t - self.ramp_s / 2), "cruise"
        elif t < self.moving_s:
            remaining = self.moving_s - t
            speed = self.speed * remaining / self.ramp_s
            distance, phase = self.distance - .5 * speed * remaining, "decelerate"
        else:
            distance, speed, phase = self.distance, 0., "hold"
        theta = float(np.interp(distance % self.length, self.arc, self.angles))
        r = self.radius
        position = np.array([r * np.sin(theta), r * (1 - np.cos(theta)) if self.pattern == "circle" else .5 * r * np.sin(2 * theta)])
        tangent = np.array([r * np.cos(theta), r * np.sin(theta) if self.pattern == "circle" else r * np.cos(2 * theta)])
        norm = np.linalg.norm(tangent)
        velocity = speed * tangent / norm
        second = np.array([-r * np.sin(theta), r * np.cos(theta) if self.pattern == "circle" else -2 * r * np.sin(2 * theta)])
        tangential_acceleration = self.speed / self.ramp_s * (1 if phase == "accelerate" else -1 if phase == "decelerate" else 0)
        acceleration = tangential_acceleration * tangent / norm + speed ** 2 * (second / norm ** 2 - tangent * np.dot(tangent, second) / norm ** 4)
        return {"position_xy": position.tolist(), "velocity_xy": velocity.tolist(), "acceleration_xy": acceleration.tolist(), "speed_m_s": speed,
                "distance_m": distance, "phase": phase}


    def command(self, time_s, imu_position, rotation, target_height, imu_velocity_world=None):
        reference = self.reference(time_s)
        vertical_velocity = reference.get("vertical_velocity_m_s", 0.) + self.vertical_gain * (target_height + reference.get("height_offset_m", 0.) - imu_position[2])
        velocity = np.r_[np.array(reference["velocity_xy"]) + self.position_gain * (np.array(reference["position_xy"]) - imu_position[:2]),
                         np.clip(vertical_velocity, -self.vertical_command_limit, self.vertical_command_limit)]
        if self.velocity_gain:
            if imu_velocity_world is None:
                raise ValueError("Velocity feedback requires the current IMU world velocity")
            velocity[:2] += self.velocity_gain * (np.asarray(reference["velocity_xy"]) - np.asarray(imu_velocity_world)[:2])
        velocity[:2] += self.acceleration_lead_s * np.asarray(reference["acceleration_xy"])
        axes = slice(None) if self.command_limit_mode == "norm" else slice(0, 2)
        speed = np.linalg.norm(velocity[axes])
        limited = bool(speed > self.command_speed_limit or abs(vertical_velocity) > self.vertical_command_limit)
        if speed > self.command_speed_limit:
            velocity[axes] *= self.command_speed_limit / speed
        yaw = rotation.as_euler("xyz")[2]
        goal = [*rotation.inv().apply(velocity), float(np.clip(-self.yaw_gain * yaw, -self.yaw_rate_limit, self.yaw_rate_limit))]
        return goal, limited


    def metadata(self):
        return {"pattern": self.pattern, "cruise_speed_m_s": self.speed, "radius_m": self.radius,
                "laps": self.laps, "path_length_per_lap_m": self.length, "planned_duration_s": self.duration,
                "ramp_s": self.ramp_s, "hold_s": self.hold_s, "position_gain_per_s": self.position_gain,
                "velocity_gain": self.velocity_gain, "acceleration_lead_s": self.acceleration_lead_s,
                "horizontal_command_limit_m_s": self.command_speed_limit, "yaw_target_rad": 0.,
                "command_limit_mode": self.command_limit_mode, "vertical_gain_per_s": self.vertical_gain,
                "vertical_command_limit_m_s": self.vertical_command_limit, "yaw_gain_per_s": self.yaw_gain, "yaw_rate_limit_rad_s": self.yaw_rate_limit,
                "parameterization": "Numerical arc length: constant tangential cruise speed, linear acceleration/deceleration",
                "curve": "x=R sin(theta), y=R(1-cos(theta))" if self.pattern == "circle" else "Gerono lemniscate: x=R sin(theta), y=(R/2)sin(2 theta)"}



class DeploymentMission(PatternMission):
    def __init__(self, pattern, speed, radius=.5, laps=2, command_speed_limit=0., fallback_speed_limit=0., ramp_s=2., max_reference_acceleration=3.5, auto_radius=False, reference_tilt_limit_deg=20.):
        if not math.isfinite(reference_tilt_limit_deg) or not 0 < reference_tilt_limit_deg < 45:
            raise ValueError('Reference tilt limit must be finite and between 0 and 45 degrees')
        self.reference_tilt_limit_deg = reference_tilt_limit_deg
        values = (speed, radius, ramp_s, max_reference_acceleration)
        if not all(math.isfinite(x) and x > 0 for x in values) or speed > 3.:
            raise ValueError('Deployment requires speed in (0,3] m/s and positive finite radius, ramp and acceleration budget')
        if pattern not in ('circle', 'lemniscate'):
            raise ValueError('Expected circle or lemniscate')
        if not all(math.isfinite(x) and x >= 0 for x in (command_speed_limit, fallback_speed_limit)):
            raise ValueError('Speed limits must be finite and nonnegative; zero selects automatic limits')
        self.requested_radius = radius
        self.auto_radius = bool(auto_radius)
        self.max_reference_acceleration = max_reference_acceleration
        angles = np.linspace(0., 2*np.pi, 20001)
        first = np.column_stack([np.cos(angles), np.sin(angles) if pattern == 'circle' else np.cos(2*angles)])
        second = np.column_stack([-np.sin(angles), np.cos(angles) if pattern == 'circle' else -2*np.sin(2*angles)])
        unit_curvature = np.max(abs(first[:,0]*second[:,1]-first[:,1]*second[:,0])/np.linalg.norm(first,axis=1)**3)
        tangential = speed/ramp_s
        if tangential >= max_reference_acceleration:
            raise ValueError('Speed/ramp exceeds acceleration budget; increase --ramp-time')
        self.minimum_radius = float(unit_curvature*speed**2/math.sqrt(max_reference_acceleration**2-tangential**2))
        if auto_radius:
            radius = max(radius, self.minimum_radius*(1+1e-9))
        if radius < self.minimum_radius*(1-1e-9):
            raise ValueError(f'{pattern} at {speed:g} m/s needs radius >= {self.minimum_radius:.3f} m for the {max_reference_acceleration:g} m/s^2 acceleration budget; increase --radius or use --auto-radius')
        limit = min(3.,max(.8,1.25*speed)) if command_speed_limit == 0 else command_speed_limit
        if not speed <= limit <= 3.:
            raise ValueError('Command-speed limit must cover reference speed and stay within the declared 3 m/s policy range')
        self.fallback_speed_limit = max(1.,1.25*speed) if fallback_speed_limit == 0 else fallback_speed_limit
        if self.fallback_speed_limit <= speed:
            raise ValueError('Fallback speed limit must exceed the planned cruise speed')
        super().__init__(pattern,speed,radius,laps,position_gain=.75,velocity_gain=.5,acceleration_lead_s=.15,
            command_speed_limit=limit,vertical_gain=1.,vertical_command_limit=.8,command_limit_mode='norm',
            yaw_gain=1.5,yaw_rate_limit=1.,ramp_s=ramp_s)
        self.reference_acceleration_bound = float(math.hypot(unit_curvature*speed**2/radius,tangential))
        if math.degrees(math.atan2(self.reference_acceleration_bound,9.81)) >= self.reference_tilt_limit_deg:
            raise ValueError(f'Reference acceleration consumes the {self.reference_tilt_limit_deg:g} degree planning tilt envelope; lower the acceleration budget or increase radius/ramp')
        self.transfer_s = max(6.,1.875*(radius if pattern=='circle' else 0.)/.5)
        self.minimum_mission_time = 22.5+self.transfer_s+self.duration
        self.landing_reserve_s = 30.+max(4.,(radius if pattern=='circle' else 0.)/.12)
        self.last_goal = None


    def world_velocity(self, reference, imu_position, imu_velocity_world):
        position, velocity = np.asarray(imu_position),np.asarray(imu_velocity_world)
        target, feedforward, acceleration = [np.asarray(reference[k]) for k in ('position','velocity','acceleration')]
        if any(x.shape != (3,) or not np.isfinite(x).all() for x in (position,velocity,target,feedforward,acceleration)):
            raise ValueError('Finite world position/velocity/reference vectors required')
        raw = feedforward+np.array([self.position_gain,self.position_gain,self.vertical_gain])*(target-position)
        raw[:2] += self.velocity_gain*(feedforward[:2]-velocity[:2])+self.acceleration_lead_s*acceleration[:2]
        desired = raw.copy()
        desired[2] = np.clip(desired[2],-self.vertical_command_limit,self.vertical_command_limit)
        desired *= min(1.,self.command_speed_limit/max(np.linalg.norm(desired),1e-9))
        self.last_goal = {'raw_world_velocity_m_s':raw.tolist(),'selected_world_velocity_m_s':desired.tolist(),
            'limited':bool(np.any(abs(raw-desired)>1e-12)),'norm_limit_m_s':self.command_speed_limit,
            'vertical_limit_m_s':self.vertical_command_limit}
        return desired


    def metadata(self):
        result = super().metadata()
        samples = [self.reference(t) for t in np.linspace(0.,self.duration,4001)]
        perfect_goal = [np.r_[np.asarray(r['velocity_xy'])+.15*np.asarray(r['acceleration_xy']),0.] for r in samples]
        norms = np.linalg.norm(perfect_goal,axis=1)
        bounds = [[-self.radius,-self.radius],[self.radius,self.radius]] if self.pattern=='circle' else [[-self.radius,-.5*self.radius],[self.radius,.5*self.radius]]
        result.update(curve='x=R cos(theta), y=R sin(theta); start=(R,0)' if self.pattern=='circle' else 'x=R sin(theta), y=(R/2)sin(2 theta); start=(0,0)',requested_radius_m=self.requested_radius,auto_radius=self.auto_radius,
            minimum_radius_m=self.minimum_radius,reference_acceleration_bound_m_s2=self.reference_acceleration_bound,
            reference_tilt_bound_deg=math.degrees(math.atan2(self.reference_acceleration_bound,9.81)),
            reference_tilt_limit_deg=self.reference_tilt_limit_deg,
            acceleration_budget_m_s2=self.max_reference_acceleration,fallback_speed_limit_m_s=self.fallback_speed_limit,
            command_norm_limit_m_s=self.command_speed_limit,world_xy_bounds_relative_to_origin_m=bounds,
            pd_transfer_s=self.transfer_s,minimum_nominal_mission_time_s=self.minimum_mission_time,nominal_landing_reserve_s=self.landing_reserve_s,
            braking_time_at_cruise_s=max(2.,1.875*self.speed/1.),
            braking_distance_at_cruise_m=.5*self.speed*max(2.,1.875*self.speed/1.),
            peak_nominal_outer_goal_m_s=float(norms.max()),
            nominal_outer_goal_clipped_fraction=float(np.mean(norms>self.command_speed_limit)),
            braking_scope='Normal path ramps to rest; manual landing/fallback uses a velocity-continuous PD braking reference. Flight space must also cover stopping and recovery.',
            policy_scope='Command norm <=3 m/s is a declared deployment interface limit, not validation of any checkpoint at that speed.')
        return result


class StarlingFigureEightMission(PatternMission):
    def __init__(self, period, amplitude_x=1., amplitude_y=.5, altitude_amplitude=.2, laps=2, hold_s=10., **kwargs):
        if not all(math.isfinite(v) and v > 0 for v in (period, amplitude_x, amplitude_y)) or not math.isfinite(altitude_amplitude) or altitude_amplitude < 0:
            raise ValueError("Starling period/amplitudes must be finite and positive; altitude amplitude may be zero")
        peak_speed = 2 * math.pi / period * math.hypot(amplitude_x, amplitude_y)
        kwargs.setdefault("yaw_gain", 1.5)
        kwargs.setdefault("yaw_rate_limit", 1.)
        super().__init__("lemniscate", peak_speed, radius=amplitude_x, laps=laps, hold_s=hold_s, **kwargs)
        if 2 * math.pi / period * altitude_amplitude > self.vertical_command_limit:
            raise ValueError("Reference vertical speed exceeds vertical command limit")
        if self.command_limit_mode == "norm" and 2 * math.pi / period * math.sqrt(amplitude_x ** 2 + amplitude_y ** 2 + altitude_amplitude ** 2) > self.command_speed_limit:
            raise ValueError("Reference 3D speed exceeds command speed limit")
        self.period, self.amplitude_x, self.amplitude_y = period, amplitude_x, amplitude_y
        self.altitude_amplitude = altitude_amplitude
        self.duration = self.hold_s + self.laps * self.period + 2.


    def reference(self, time_s):
        t = time_s - self.hold_s
        moving = 0 <= t < self.laps * self.period
        theta = 2 * math.pi * max(0., min(t, self.laps * self.period)) / self.period
        omega = 2 * math.pi / self.period
        velocity = np.array([self.amplitude_x * math.cos(theta), self.amplitude_y * math.cos(2 * theta)]) * omega if moving else np.zeros(2)
        return {"position_xy": [self.amplitude_x * math.sin(theta), .5 * self.amplitude_y * math.sin(2 * theta)],
                "velocity_xy": velocity.tolist(),
                "acceleration_xy": [-self.amplitude_x * math.sin(theta) * omega ** 2, -2 * self.amplitude_y * math.sin(2 * theta) * omega ** 2] if moving else [0., 0.],
                "height_offset_m": self.altitude_amplitude * math.sin(theta),
                "vertical_velocity_m_s": self.altitude_amplitude * math.cos(theta) * omega if moving else 0.,
                "vertical_acceleration_m_s2": -self.altitude_amplitude * math.sin(theta) * omega ** 2 if moving else 0.,
                "speed_m_s": float(np.linalg.norm(velocity)), "phase": "cruise" if moving else "settle" if t < 0 else "hold"}


    def metadata(self):
        metadata = super().metadata()
        for name in ("radius_m", "path_length_per_lap_m", "ramp_s", "cruise_speed_m_s"):
            metadata.pop(name)
        metadata.update(pattern="starling_eight", period_s=self.period, amplitude_x_m=self.amplitude_x,
                        amplitude_y_m=self.amplitude_y, altitude_amplitude_m=self.altitude_amplitude,
                        reference_peak_horizontal_speed_m_s=self.speed,
                        reference_peak_3d_speed_m_s=2 * math.pi / self.period * math.sqrt(self.amplitude_x ** 2 + self.amplitude_y ** 2 + self.altitude_amplitude ** 2),
                        parameterization="Starling figure_eight_test.py: uniform phase; speed varies with phase; no start/stop ramp",
                        curve="x=Ax sin(theta), y=(Ay/2)sin(2 theta), z=z0+Az sin(theta)")
        return metadata



class ReportedStarlingMission(StarlingFigureEightMission):
    def __init__(self, period, *args, integral_gain=.15, integral_limit=.3, feedforward_lead_s=None, **kwargs):
        kwargs.setdefault("position_gain", 1.)
        kwargs.setdefault("vertical_gain", .7)
        kwargs.setdefault("command_speed_limit", .8)
        kwargs.setdefault("vertical_command_limit", .8)
        kwargs.setdefault("command_limit_mode", "norm")
        super().__init__(period, *args, **kwargs)
        if self.velocity_gain or self.acceleration_lead_s:
            raise ValueError("Reported controller uses its original velocity feedforward, not velocity feedback or acceleration lead")
        if self.command_limit_mode != "norm" or self.vertical_command_limit < self.command_speed_limit:
            raise ValueError("Reported Starling controller requires a 3D norm limit without a tighter separate vertical cap")
        feedforward_lead_s = (.4 if period <= 20 else 0.) if feedforward_lead_s is None else feedforward_lead_s
        if not all(math.isfinite(v) and v >= 0 for v in (integral_gain, integral_limit, feedforward_lead_s)):
            raise ValueError("Integral settings and feedforward lead must be finite and nonnegative")
        self.integral_gain, self.integral_limit, self.feedforward_lead_s = integral_gain, integral_limit, feedforward_lead_s
        source = Path(__file__).resolve().parents[1] / "figure_eight_test.py"
        spec = importlib.util.spec_from_file_location("reported_starling_controller", source)
        self.reported = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.reported)
        self.reset()


    def reset(self):
        self.integral = np.zeros(3)


    def command(self, time_s, imu_position, rotation, target_height, imu_velocity_world=None):
        reference = self.reference(time_s)
        position = np.r_[reference["position_xy"], target_height + reference["height_offset_m"]]
        velocity = np.r_[reference["velocity_xy"], reference["vertical_velocity_m_s"]]
        t = time_s - self.hold_s
        if 0 <= t < self.laps * self.period and self.feedforward_lead_s:
            velocity = self.reported.figure_eight_reference(min(t + self.feedforward_lead_s, self.laps * self.period), self.amplitude_x, self.amplitude_y, self.period, target_height, self.altitude_amplitude)[1]
        raw_velocity = velocity + np.array([self.position_gain, self.position_gain, self.vertical_gain]) * (position - imu_position)
        goal = self.reported.outer_loop_command(np.asarray(imu_position), rotation.as_matrix(), position, velocity, "hold", self.position_gain, self.yaw_gain, self.command_speed_limit, self.yaw_rate_limit,
                                               integral=self.integral, integral_gain=self.integral_gain, integral_limit=self.integral_limit, dt=.02, vertical_gain=self.vertical_gain)
        limited = bool(np.linalg.norm(raw_velocity + self.integral_gain * np.r_[self.integral[:2], 0.]) > self.command_speed_limit)
        return goal.tolist(), limited


    def metadata(self):
        metadata = super().metadata()
        metadata.update(outer_controller="reported_starling", integral_gain_per_s2=self.integral_gain,
                        integral_limit_m_s=self.integral_limit, feedforward_lead_s=self.feedforward_lead_s,
                        controller_source="experiments/figure_eight_test.py:outer_loop_command",
                        dt_s=.02)
        return metadata
