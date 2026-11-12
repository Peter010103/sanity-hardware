import math
import numpy as np


def smooth_fraction(value):
    value = np.clip(value, 0., 1.)
    # Near one, floating-point cancellation can put the polynomial a few
    # ulps above one even with a bounded input. Keep blend weights inside
    # their mathematical range before control-command validation.
    return np.clip(value ** 3 * (10 - 15 * value + 6 * value ** 2), 0., 1.)


class TakeoffReference:
    def __init__(self, ground_height, target_height=1., settle_s=.5, spool_s=1., climb_s=10.):
        if not all(math.isfinite(v) for v in (ground_height, target_height, settle_s, spool_s, climb_s)) or target_height <= ground_height or min(settle_s, spool_s, climb_s) <= 0:
            raise ValueError('Takeoff height/times must be finite with positive durations and an upward climb')
        self.ground_height, self.target_height = ground_height, target_height
        self.settle_s, self.spool_s, self.climb_s = settle_s, spool_s, climb_s
        self.climb_start = settle_s + spool_s
        self.climb_end = self.climb_start + climb_s
        self.handover_ready_time = self.climb_end


    def sample(self, time_s):
        u = float(np.clip((time_s - self.climb_start) / self.climb_s, 0., 1.))
        distance = self.target_height - self.ground_height
        velocity = distance * 30 * u ** 2 * (1 - u) ** 2 / self.climb_s
        acceleration = distance * 60 * u * (1 - u) * (1 - 2 * u) / self.climb_s ** 2
        return {'position': [0., 0., self.ground_height + distance * smooth_fraction(u)],
                'velocity': [0., 0., velocity], 'acceleration': [0., 0., acceleration],
                'phase': 'ground_settle' if time_s < self.settle_s else 'spool' if time_s < self.climb_start else 'climb' if time_s < self.climb_end else 'hover'}


class TakeoffCircleReference(TakeoffReference):
    def __init__(self, ground_height, circle, target_height=1., climb_s=10., transfer_s=6., arrival_hold_s=2.):
        super().__init__(ground_height, target_height, climb_s=climb_s)
        if not all(math.isfinite(v) and v > 0 for v in (transfer_s, arrival_hold_s)) or circle.pattern not in ('circle','lemniscate'):
            raise ValueError('A circle/lemniscate mission and positive finite transfer/arrival times are required')
        self.circle = circle
        self.start_xy = np.array([circle.radius,0.]) if circle.pattern=='circle' else np.zeros(2)
        self.transfer_s = transfer_s
        self.transfer_start = self.climb_end + 2.
        self.arrival_time = self.transfer_start + transfer_s
        self.handover_ready_time = self.arrival_time + arrival_hold_s
        self.orbit_start = None


    def sample(self, time_s):
        reference = super().sample(time_s)
        if time_s < self.transfer_start:
            return reference
        if time_s < self.arrival_time:
            u = (time_s - self.transfer_start) / self.transfer_s
            reference.update(position=[*(self.start_xy*smooth_fraction(u)), self.target_height],
                             velocity=[*(self.start_xy*30*u**2*(1-u)**2/self.transfer_s),0.],
                             acceleration=[*(self.start_xy*60*u*(1-u)*(1-2*u)/self.transfer_s**2),0.],
                             phase='reposition')
        elif self.orbit_start is None or time_s < self.orbit_start:
            reference.update(position=[*self.start_xy,self.target_height], velocity=[0.] * 3,
                             acceleration=[0.] * 3, phase=self.circle.pattern+'_start_hold')
        else:
            path = self.circle.reference(time_s - self.orbit_start)
            if self.circle.pattern=='circle':
                xy=[self.circle.radius-path['position_xy'][1],path['position_xy'][0]]
                velocity=[-path['velocity_xy'][1],path['velocity_xy'][0]]
                acceleration=[-path['acceleration_xy'][1],path['acceleration_xy'][0]]
            else:
                xy,velocity,acceleration=path['position_xy'],path['velocity_xy'],path['acceleration_xy']
            reference.update(position=[*xy,self.target_height],velocity=[*velocity,0.],acceleration=[*acceleration,0.],
                             phase=self.circle.pattern+'_'+path['phase'],path_distance_m=path['distance_m'],
                             reference_laps=path['distance_m']/self.circle.length)
        return reference


class LevelLiftoff:
    def __init__(self, clearance_m=.05, airborne_dwell_s=.1, blend_s=.5, ground_rate_limit_rad_s=.15):
        values = (clearance_m, airborne_dwell_s, blend_s, ground_rate_limit_rad_s)
        if not all(math.isfinite(v) and v > 0 for v in values) or ground_rate_limit_rad_s > 2.3:
            raise ValueError('Positive finite liftoff clearance, dwell, blend and rate limit required')
        self.clearance_m, self.airborne_dwell_s, self.blend_s, self.ground_rate_limit = values
        self.airborne_since = self.blend_start = self.last_time = None
        self.preflight_since = self.preflight_time = None
        self.complete = False
        self.last = {}
        self.preflight_status = {}


    def preflight(self, state, time_s):
        state = np.asarray(state, dtype=float)
        if state.shape != (13,) or not np.isfinite(state).all() or not math.isfinite(time_s):
            raise ValueError('Finite 13-element mocap state and time required for liftoff readiness')
        quaternion = state[3:7]
        norm = np.linalg.norm(quaternion)
        if abs(norm - 1.) > .05:
            raise ValueError('Unit attitude quaternion required for liftoff readiness')
        quaternion = quaternion / norm
        tilt = math.degrees(math.acos(np.clip(1. - 2. * np.dot(quaternion[1:3], quaternion[1:3]), -1., 1.)))
        speed, rate = np.linalg.norm(state[7:10]), np.linalg.norm(state[10:13])
        passed = bool(tilt <= 10. and speed <= .1 and rate <= .3)
        if self.preflight_time is None or not 0 <= time_s - self.preflight_time <= .04 + 1e-9:
            self.preflight_since = None
        self.preflight_time = time_s
        self.preflight_since = (time_s if self.preflight_since is None else self.preflight_since) if passed else None
        dwell = 0. if self.preflight_since is None else time_s - self.preflight_since
        ready = passed and dwell >= .5 - 1e-9
        self.preflight_status = {'ready': ready, 'tilt_deg': tilt, 'speed_m_s': float(speed), 'angular_rate_rad_s': float(rate),
                                 'dwell_s': dwell, 'limits': {'tilt_deg': 10., 'speed_m_s': .1, 'angular_rate_rad_s': .3, 'dwell_s': .5}}
        return ready


    def update(self, time_s, height_m, climb_start_s):
        if not all(math.isfinite(v) for v in (time_s, height_m, climb_start_s)):
            raise ValueError('Finite liftoff elapsed time, height and climb start required')
        if self.last_time is not None and time_s < self.last_time:
            raise ValueError('Liftoff clock must not go backwards; reset for a new takeoff')
        if not self.complete:
            if self.last_time is None or time_s - self.last_time > .04 + 1e-9:
                self.airborne_since = self.blend_start = None
            if time_s < climb_start_s or height_m < .6 * self.clearance_m:
                self.airborne_since = self.blend_start = None
            elif self.blend_start is None:
                self.airborne_since = (time_s if self.airborne_since is None else self.airborne_since) if height_m >= self.clearance_m else None
                if self.airborne_since is not None and time_s - self.airborne_since >= self.airborne_dwell_s - 1e-9:
                    self.blend_start = time_s
        self.last_time = time_s
        weight = 1. if self.complete else 0. if self.blend_start is None else float(smooth_fraction((time_s - self.blend_start) / self.blend_s))
        self.complete = weight >= 1.
        self.last = {'phase': 'complete' if self.complete else 'horizontal_blend' if self.blend_start is not None else 'level_liftoff',
                     'horizontal_weight': weight, 'height_above_initial_ground_m': height_m, 'airborne_since_s': self.airborne_since,
                     'clearance_m': self.clearance_m, 'airborne_dwell_s': self.airborne_dwell_s, 'blend_s': self.blend_s,
                     'roll_pitch_rate_limit_rad_s': None if self.complete else self.ground_rate_limit + (2.3 - self.ground_rate_limit) * weight}
        return weight


class PositionPD:
    def __init__(self, mass, gravity=9.81, position_gains=(2., 2., 4.), velocity_gains=(2., 2., 3.), attitude_gain=6., integral_gains=(0., 0., 0.), integral_acceleration_limits=(.45, .45, .8), *, liftoff_options=None):
        self.mass, self.gravity = mass, gravity
        self.kp, self.kd = np.asarray(position_gains), np.asarray(velocity_gains)
        self.ki = np.asarray(integral_gains, dtype=float)
        self.integral_limits = np.asarray(integral_acceleration_limits, dtype=float)
        self.attitude_gain = attitude_gain
        if not all(np.isfinite(v).all() for v in (self.kp, self.kd, self.ki, self.integral_limits, mass, gravity, attitude_gain)) or any(v.shape != (3,) for v in (self.kp, self.kd, self.ki, self.integral_limits)) or min(mass, gravity, attitude_gain) <= 0 or np.any(self.kp <= 0) or np.any(self.kd <= 0) or np.any(self.ki < 0) or np.any(self.integral_limits < 0) or np.any((self.ki > 0) & (self.integral_limits <= 0)):
            raise ValueError('PD gains must be positive; integral gains/limits finite and nonnegative with positive limits on enabled axes')
        if liftoff_options is not None and not isinstance(liftoff_options, dict):
            raise ValueError('Liftoff options must be a dictionary of LevelLiftoff settings')
        options = {} if liftoff_options is None else dict(liftoff_options)
        if any(isinstance(value, bool) for value in options.values()):
            raise ValueError('Liftoff options require positive finite numbers')
        try:
            liftoff = LevelLiftoff(**options)
        except TypeError as exc:
            raise ValueError('Unknown or nonnumeric LevelLiftoff option') from exc
        self.liftoff_options = {'clearance_m': liftoff.clearance_m, 'airborne_dwell_s': liftoff.airborne_dwell_s,
                                'blend_s': liftoff.blend_s, 'ground_rate_limit_rad_s': liftoff.ground_rate_limit}
        self.reset()


    def reset(self):
        self.integral_acceleration = np.zeros(3)
        self.previous_integral_acceleration = np.zeros(3)
        self.integrating = False
        self.last = {}
        # Diagnostic-only count is monotonic for this controller object so a
        # control-state reset cannot be mistaken for a fresh PD evaluation.
        self.command_count = getattr(self, 'command_count', 0)
        self.liftoff = LevelLiftoff(**self.liftoff_options)


    def takeoff_command(self, position, velocity, rotation, reference, time_s, height_m, climb_start_s, yaw=0., dt=.02, integrate=False):
        weight = self.liftoff.update(time_s, height_m, climb_start_s)
        liftoff_yaw = yaw
        if not self.liftoff.complete:
            measured_yaw = rotation.as_euler('xyz')[2]
            liftoff_yaw = measured_yaw + weight * math.atan2(math.sin(yaw - measured_yaw), math.cos(yaw - measured_yaw))
        command = self.command(position, velocity, rotation, reference, yaw=liftoff_yaw, dt=dt,
                               integrate=integrate and self.liftoff.complete, horizontal_weight=weight,
                               roll_pitch_rate_limit=self.liftoff.last['roll_pitch_rate_limit_rad_s'])
        if weight == 0.:
            command[3] = 0.
            self.last['requested_ctbr'] = command.tolist()
        self.last['liftoff'] = dict(self.liftoff.last)
        return command


    def command(self, position, velocity, rotation, reference, yaw=0., dt=.02, integrate=False, horizontal_weight=1., roll_pitch_rate_limit=None):
        position, velocity = np.asarray(position), np.asarray(velocity)
        target, target_velocity, feedforward = (np.asarray(reference[key]) for key in ('position', 'velocity', 'acceleration'))
        if not all(v.shape == (3,) and np.isfinite(v).all() for v in (position, velocity, target, target_velocity, feedforward)) or not math.isfinite(dt) or not 0 < dt <= .04 or not math.isfinite(yaw):
            raise ValueError('Expected finite three-axis state/reference and a positive integration step of at most40ms')
        error = target - position
        if not math.isfinite(horizontal_weight) or not 0 <= horizontal_weight <= 1 or (roll_pitch_rate_limit is not None and (not math.isfinite(roll_pitch_rate_limit) or roll_pitch_rate_limit <= 0)):
            raise ValueError('Horizontal weight in [0,1] and positive finite liftoff rate limit required')
        base_acceleration = feedforward + self.kp * error + self.kd * (target_velocity - velocity)
        before = self.integral_acceleration.copy()
        candidate = np.clip(before + self.ki * error * dt, -self.integral_limits, self.integral_limits) if integrate else before.copy()
        raw = base_acceleration + candidate
        bounded = raw.copy()
        bounded[2] = np.clip(bounded[2], -2., 2.)
        bounded[:2] *= min(1., 1.5 / max(np.linalg.norm(bounded[:2]), 1e-9))
        blocked = (raw - bounded) * (candidate - before) > 1e-12
        self.integral_acceleration = np.where(blocked, before, candidate)
        self.previous_integral_acceleration = before
        self.integrating = bool(integrate)
        acceleration = base_acceleration + self.integral_acceleration
        acceleration[2] = np.clip(acceleration[2], -2., 2.)
        acceleration[:2] *= min(1., 1.5 / max(np.linalg.norm(acceleration[:2]), 1e-9))
        acceleration[:2] *= horizontal_weight
        specific_force = acceleration + [0., 0., self.gravity]
        desired_z = specific_force / np.linalg.norm(specific_force)
        heading = np.array([math.cos(yaw), math.sin(yaw), 0.])
        desired_y = np.cross(desired_z, heading)
        desired_y /= np.linalg.norm(desired_y)
        desired_rotation = np.column_stack([np.cross(desired_y, desired_z), desired_y, desired_z])
        current = rotation.as_matrix()
        if not np.isfinite(current).all():
            self.integral_acceleration = before
            raise ValueError('Finite measured attitude required')
        skew = .5 * (desired_rotation.T @ current - current.T @ desired_rotation)
        rates = -self.attitude_gain * np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
        if roll_pitch_rate_limit is not None:
            rates[:2] *= min(1., roll_pitch_rate_limit / max(np.linalg.norm(rates[:2]), 1e-9))
        command = np.r_[self.mass * np.dot(specific_force, current[:, 2]), rates]
        self.command_count += 1
        self.last = {'integrating':self.integrating, 'position_error_m':error.tolist(),
                     'command_count':self.command_count,
                     'base_acceleration_m_s2':base_acceleration.tolist(),
                     'integral_acceleration_before_m_s2':before.tolist(),
                     'integral_acceleration_candidate_m_s2':candidate.tolist(),
                     'acceleration_requested_m_s2':raw.tolist(),
                     'acceleration_limit_candidate_m_s2':bounded.tolist(),
                     'horizontal_weight':float(horizontal_weight),
                     'desired_body_to_world':desired_rotation.tolist(),
                     'measured_body_to_world':current.tolist(),
                     'integral_acceleration_used_m_s2':self.integral_acceleration.tolist(),
                     'integral_acceleration_next_m_s2':self.integral_acceleration.tolist(),
                     'acceleration_anti_windup_axes':blocked.tolist(), 'command_anti_windup':False,
                     'acceleration_command_m_s2':acceleration.tolist(), 'requested_ctbr':command.tolist()}
        return command


    def accept_output(self, selected_ctbr):
        selected = np.asarray(selected_ctbr)
        if selected.shape != (4,) or not np.isfinite(selected).all():
            raise ValueError('Finite selected CTBR required for integral anti-windup')
        if self.integrating and np.any(abs(selected - self.last['requested_ctbr']) > 1e-8):
            self.integral_acceleration = self.previous_integral_acceleration.copy()
            self.last['command_anti_windup'] = True
        self.last['integral_acceleration_next_m_s2'] = self.integral_acceleration.tolist()
        self.last['selected_ctbr'] = selected.tolist()


class TakeoffSupervisor:
    def __init__(self, mode, reference, ctrlrange, history_length_steps, dt=.02, blend_s=2., dwell_s=1., handover_position_tolerance=.06, fallback_position_tolerance=.25, fallback_speed_limit=.5, handover_thrust_tolerance=.15, fallback_tilt_limit_deg=20., hard_tilt_limit_deg=45.):
        if mode not in ('policy', 'pd', 'hybrid') or min(dt, blend_s, dwell_s) <= 0:
            raise ValueError('Expected policy, pd or hybrid with positive update/blend/dwell times')
        if not (math.isfinite(handover_position_tolerance) and math.isfinite(fallback_position_tolerance)
                and 0 < handover_position_tolerance < fallback_position_tolerance):
            raise ValueError('Position tolerances must be finite, positive and ordered: handover < fallback')
        if not math.isfinite(fallback_speed_limit) or fallback_speed_limit <= .10:
            raise ValueError('Fallback speed limit must be finite and exceed the 0.1 m/s handover limit')
        if not math.isfinite(handover_thrust_tolerance) or handover_thrust_tolerance <= 0:
            raise ValueError('Handover thrust tolerance must be finite and positive')
        if not math.isfinite(hard_tilt_limit_deg) or not math.isfinite(fallback_tilt_limit_deg) or not 5 < fallback_tilt_limit_deg < hard_tilt_limit_deg <= 80:
            raise ValueError('Tilt limits must be finite and ordered:5 degree handover < fallback < hard <=80 degrees')
        self.fallback_tilt_limit_deg = fallback_tilt_limit_deg
        self.handover_thrust_tolerance = handover_thrust_tolerance
        self.fallback_speed_limit = fallback_speed_limit
        self.handover_position_tolerance = handover_position_tolerance
        self.fallback_position_tolerance = fallback_position_tolerance
        self.mode, self.reference = mode, reference
        self.ctrlrange = np.asarray(ctrlrange, dtype=float)
        self.history_length = history_length_steps
        self.dt, self.blend_s, self.dwell_s = dt, blend_s, dwell_s
        self.stable_since = self.blend_start = self.policy_start = None
        self.fallback = False
        self.previous = np.zeros(4)


    def step(self, time_s, position_error, velocity, tilt_deg, clearance, pd_ctbr, policy_ctbr, history_age, policy_age_s=0., state_age_s=0.):
        pd = np.asarray(pd_ctbr, dtype=float)
        policy = np.asarray(policy_ctbr, dtype=float) if policy_ctbr is not None else np.full(4, np.nan)
        if pd.shape != (4,) or not np.isfinite(pd).all() or not np.isfinite(np.r_[time_s, position_error, velocity, tilt_deg, clearance]).all():
            raise ValueError('Valid measured state and PD output are required; caller must stop on stale/invalid state')
        if not math.isfinite(state_age_s) or not 0 <= state_age_s <= .04:
            raise ValueError('Fresh measured state is required; caller must stop stale output')
        valid_policy = math.isfinite(policy_age_s) and 0 <= policy_age_s <= .04 and policy.shape == (4,) and np.isfinite(policy).all() and np.all(policy >= self.ctrlrange[:, 0]) and np.all(policy <= self.ctrlrange[:, 1])
        position_error = np.asarray(position_error)
        state_gate = np.linalg.norm(position_error) < self.handover_position_tolerance and np.linalg.norm(velocity) < .10 and tilt_deg < 5 and clearance > .15
        agreement = valid_policy and abs(policy[0] - pd[0]) < self.handover_thrust_tolerance and np.linalg.norm(policy[1:] - pd[1:]) < .5
        ready = time_s >= self.reference.handover_ready_time and state_gate and agreement and history_age >= self.history_length
        if self.mode == 'hybrid' and not self.fallback:
            if self.blend_start is None:
                self.stable_since = (time_s if self.stable_since is None else self.stable_since) if ready else None
                if self.stable_since is not None and time_s - self.stable_since >= self.dwell_s:
                    self.blend_start = time_s
            elif not valid_policy or np.linalg.norm(position_error) > self.fallback_position_tolerance or np.linalg.norm(velocity) > self.fallback_speed_limit or tilt_deg > self.fallback_tilt_limit_deg or clearance < .10:
                self.fallback = True
        weight = 1. if self.mode == 'policy' else float(smooth_fraction((time_s - self.blend_start) / self.blend_s)) if self.blend_start is not None and not self.fallback else 0.
        if self.mode == 'policy' and not valid_policy:
            raise ValueError('Direct policy output is invalid')
        selected = (1 - weight) * pd + weight * policy if valid_policy else pd
        if time_s < self.reference.settle_s:
            selected = np.zeros(4)
            phase = 'ground_settle'
        elif time_s < self.reference.climb_start:
            selected *= smooth_fraction((time_s - self.reference.settle_s) / self.reference.spool_s)
            phase = 'spool'
        elif self.fallback:
            phase = 'fallback_pd'
        elif weight >= 1:
            phase = 'policy'
            if self.policy_start is None:
                self.policy_start = time_s
        elif self.blend_start is not None:
            phase = 'blend'
        else:
            phase = 'pd_' + self.reference.sample(time_s)['phase']
        command = np.clip(selected, self.ctrlrange[:, 0], self.ctrlrange[:, 1])
        altered = bool(np.any(abs(command - selected) > 1e-10))
        self.previous = command.copy()
        return command, {'phase': phase, 'policy_weight': weight, 'handover_ready': bool(ready),
                         'blend_start_s': self.blend_start, 'policy_start_s': self.policy_start,
                         'fallback': self.fallback, 'command_limited': altered}
