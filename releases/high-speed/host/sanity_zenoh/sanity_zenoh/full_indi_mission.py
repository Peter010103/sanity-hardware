"""Ground-acknowledged INDI takeoff, hover and landing, without planned PID return.

Pure mission logic: no ROS, network, devices or actuator side effects. The FC
arms in PID for a low-request ground warmup; actual INDI authority must be
acknowledged before the takeoff trajectory is allowed to begin.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

from sanity_zenoh.indi_hover_mission import IndiHoverMission


@dataclass(frozen=True)
class GroundIndiLimits:
    thrust_weight: float = .4
    spool_s: float = 1.5
    warmup_s: float = .6
    eligible_dwell_s: float = .5
    maximum_wait_s: float = 8.
    maximum_height_m: float = .04
    start_climb_on_ack: bool = False
    launch_boost_weight: float = 0.
    launch_boost_max_s: float = .5
    launch_slew_up_N_s: float = 3.

    def __post_init__(self):
        if type(self.start_climb_on_ack) is not bool:
            raise ValueError('start_climb_on_ack must be boolean')
        if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and v > 0
                   for key,v in vars(self).items() if key not in ('start_climb_on_ack','launch_boost_weight')):
            raise ValueError('Positive finite ground INDI limits required')
        if (not isinstance(self.launch_boost_weight,(int,float)) or isinstance(self.launch_boost_weight,bool)
                or not math.isfinite(self.launch_boost_weight) or self.launch_boost_weight<0):
            raise ValueError('Finite nonnegative launch boost weight required')
        if self.launch_boost_max_s>.6 or self.launch_slew_up_N_s>4:
            raise ValueError('Launch boost is bounded to0.6s and upward slew4N/s')
        if self.launch_boost_weight and (not 1.<=self.launch_boost_weight<=1.25 or not self.start_climb_on_ack):
            raise ValueError('Active launch boost requires1–1.25weight and immediate climb on acknowledgement')
        if not .35 <= self.thrust_weight <= (.7 if self.start_climb_on_ack else .4):
            raise ValueError('Ground spinup is bounded to35–40 percent weight, or70 percent with immediate climb on ACK')
        if self.warmup_s < .5 or self.eligible_dwell_s < .5:
            raise ValueError('Ground filter/RPM warmup and eligibility dwell require at least0.5s')
        if not .03 <= self.maximum_height_m <= .05:
            raise ValueError('Ground engagement height bound must be3–5cm')
        if self.maximum_wait_s <= self.spool_s + self.warmup_s + self.eligible_dwell_s:
            raise ValueError('Ground wait must accommodate spool, warmup and eligibility dwell')


class FullIndiMission(IndiHoverMission):
    """Healthy airborne outputs retain INDI; only fault fallback lowers request."""
    full_indi = True
    ground_phases = ('ground_spinup', 'ground_indi_request')
    ground_request_phase = 'ground_indi_request'
    ground_reference_phase = 'ground_indi_warmup'
    ground_error_prefix = 'ground_indi'
    ground_lift_reason = 'ground_lift_before_indi_ack'

    def __init__(self, limits, pd, reference_factory, ground=None, *, spool_leveling=True):
        if limits.pid_only:
            raise ValueError('Full INDI flight cannot use pid_only limits')
        if type(spool_leveling) is not bool:
            raise ValueError('spool_leveling must be boolean')
        super().__init__(limits, pd, reference_factory)
        self.spool_leveling = spool_leveling
        self.ground = ground or GroundIndiLimits()
        self.ground_started = None
        self.fallback = False
        self.indi_confirmed_at = None
        self.launch_boost_started = None
        self.launch_boost_stop_reason = None
        self.launch_boost_active = False
        self.launch_boost_log = dict(reason='awaiting_ack' if self.ground.launch_boost_weight else 'disabled')
        self.last_envelope_violation = None

    @property
    def ground_thrust_N(self):
        return self.limits.mass_kg * 9.81 * self.ground.thrust_weight

    def diagnostics(self,output):
        result=super().diagnostics(output)
        result['ground_entry']=dict(thrust_weight=self.ground.thrust_weight,start_climb_on_ack=self.ground.start_climb_on_ack,
            acknowledgement_monotonic_s=self.indi_confirmed_at,reference_clock_origin_monotonic_s=self.armed_at)
        result['launch_boost']={**self.launch_boost_log,'configured_weight':self.ground.launch_boost_weight,
            'active':self.launch_boost_active,'started_monotonic_s':self.launch_boost_started,
            'permanent_stop_reason':self.launch_boost_stop_reason,'selected_thrust_N':output['thrust_N'],
            'slew_up_limit_N_s':self.ground.launch_slew_up_N_s if self.launch_boost_active else .5,
            'slew_down_limit_N_s':.5}
        result['flight_envelope_guard']=dict(active=self._airborne_envelope(),last_violation=self.last_envelope_violation)
        return result

    def _end_launch_boost(self,reason):
        self.launch_boost_active=False
        if self.launch_boost_started is not None and self.launch_boost_stop_reason is None:
            self.launch_boost_stop_reason=reason
        self.launch_boost_log['reason']=self.launch_boost_stop_reason or ('disabled' if not self.ground.launch_boost_weight else reason)

    def stop(self,reason):
        self._end_launch_boost('stopped:'+reason)
        super().stop(reason)

    def _land(self,now,state,reason):
        self._end_launch_boost('landing:'+reason)
        super()._land(now,state,reason)

    def _update_launch_boost(self,now,control,request):
        if not self.ground.launch_boost_weight or self.launch_boost_started is None or self.launch_boost_stop_reason is not None:
            return
        liftoff=getattr(self.pd,'liftoff',None)
        if self.done or self.phase!='takeoff' or self.fallback or self.pending_landing:
            self._end_launch_boost('mission_left_takeoff')
        elif not (request and control.get('requested') is True and self._ground_acknowledged(control) and control.get('permission') and control.get('supported') and not control.get('fault_latched')):
            self._end_launch_boost('indi_authority_unavailable')
        elif liftoff is not None and (liftoff.blend_start is not None or liftoff.complete):
            self._end_launch_boost('confirmed_liftoff')
        elif now-self.launch_boost_started>=self.ground.launch_boost_max_s-1e-9:
            self._end_launch_boost('timeout')
        else:
            self.launch_boost_active=True
            self.launch_boost_log.update(reason='active',elapsed_s=now-self.launch_boost_started)

    def request_landing(self, reason='operator_landing'):
        self._end_launch_boost('landing_requested:'+reason)
        if self.phase in ('waiting', 'arming', *self.ground_phases):
            self.stop(reason)
        elif not self.done:
            self.pending_landing = self.pending_landing or reason

    def _fault_landing(self, now, state, reason):
        self.fallback = True
        if self.reason in (None, 'test_complete'):
            self.reason = reason
        if self.phase != 'landing':
            self._land(now, state, self.reason)

    def _bounded_output(self, dt, thrust, rates, *, request):
        active=self.launch_boost_active and self.phase=='takeoff' and request and not self.fallback
        floor=min(self.ground.launch_boost_weight,self.limits.maximum_thrust_weight)*self.limits.mass_kg*9.81 if active else 0.
        self.launch_boost_log.update(pd_requested_thrust_N=float(thrust),requested_floor_N=floor,
            requested_thrust_N=max(float(thrust),floor),previous_selected_thrust_N=self.last_thrust)
        thrust=max(float(thrust),floor)
        thrust = float(np.clip(thrust, 0., self.limits.mass_kg * 9.81 * self.limits.maximum_thrust_weight))
        up=self.ground.launch_slew_up_N_s if active else .5
        thrust = float(np.clip(thrust, self.last_thrust - .5 * min(dt, .04), self.last_thrust + up * min(dt, .04)))
        rates = np.clip(rates, -self.limits.maximum_rate_rad_s, self.limits.maximum_rate_rad_s)
        self.last_thrust = thrust
        return self.output(True, thrust, rates, request)

    def _flight_reference(self, now):
        if self.phase == 'takeoff':
            ref = self.reference.sample(now - self.armed_at)
            ref['position'][:2] = self.home[:2].tolist()
            return ref
        if self.phase == 'landing':
            return self.landing.sample(now)
        return {'position': self.home.tolist(), 'velocity': [0.] * 3, 'acceleration': [0.] * 3}

    def _airborne_envelope(self):
        return dict(soft_position_m=self.limits.soft_position_m, hard_position_m=self.limits.hard_position_m,
                    soft_tilt_deg=self.limits.soft_tilt_deg, hard_tilt_deg=self.limits.hard_tilt_deg,
                    maximum_speed_m_s=2.)

    def _select_airborne_command(self, now, state, reference, dt, thrust, rates, *, request):
        return self._bounded_output(dt, thrust, rates, request=request)

    def _ground_entry_eligible(self, control, at_target):
        return at_target and control.get('eligible') and control.get('rpm_mask') == 15

    def _ground_acknowledged(self, control):
        return control.get('mode') == 'indi' and control.get('indi_weight') == 1 and control.get('requested', True) and control.get('reason') != 'idle'

    def _ground_confirmed(self, now):
        self.indi_confirmed_at = now
        self.armed_at = now-self.reference.climb_start if self.ground.start_climb_on_ack else now
        self._phase('takeoff', now)
        if self.ground.launch_boost_weight and self.launch_boost_started is None and self.launch_boost_stop_reason is None:
            self.launch_boost_started=now

    def _ground_request(self):
        return self.phase == self.ground_request_phase

    def _airborne_fault_reason(self, control):
        if control.get('fault_latched'):
            return 'fc_indi_fault_' + str(control.get('fault'))
        if not self.fallback and (not control.get('permission') or not control.get('supported') or control.get('mode') != 'indi' or control.get('indi_weight') != 1 or control.get('reason') == 'idle'):
            return 'indi_authority_lost'
        return None

    def _airborne_request(self):
        return not self.fallback

    def tick(self, now, state, control, *, mocap_age_s=0., controller_age_s=0., receiver_ready=True):
        self.launch_boost_active=False
        self.launch_boost_log.update(pd_requested_thrust_N=None,requested_floor_N=0.,requested_thrust_N=None)
        self._tick_pd_start_count=getattr(self.pd,'command_count',0)
        if not math.isfinite(now):
            raise ValueError('Finite monotonic time required')
        if self.done:
            return self.output()
        if self.first_tick is None:
            self.first_tick = now
        dt = .02 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        if dt <= 0:
            self.stop('clock_not_monotonic');return self.output()
        if self.phase not in ('waiting', 'arming') and dt > .04:
            self.request_landing('control_deadline')
            if self.done:return self.output()
        state = np.asarray(state, dtype=float) if state is not None else None
        valid = state is not None and state.shape == (13,) and np.isfinite(state).all() and abs(np.linalg.norm(state[3:7]) - 1.) < .001
        fresh = valid and math.isfinite(mocap_age_s) and 0 <= mocap_age_s < .04
        control_fresh = control is not None and math.isfinite(controller_age_s) and 0 <= controller_age_s < self.limits.controller_status_max_age_s
        if not fresh or not control_fresh or not receiver_ready:
            if self.phase != 'waiting':self.stop('state_or_receiver_unavailable')
            else:self.ready_since = None
            if self.phase == 'waiting' and now - self.first_tick > 20:self.stop('startup_timeout')
            return self.output()
        if control.get('fc_uid_msp') != self.limits.expected_uid:
            self.stop('fc_identity_mismatch');return self.output()
        if control.get('benchmark_active', False):
            self.stop('benchmark_active');return self.output()
        if now - self.first_tick > self.limits.maximum_run_s:
            self.stop('maximum_runtime');return self.output()
        if self.phase == 'waiting' and now - self.first_tick > 20:
            self.stop('startup_timeout');return self.output()
        rotation = Rotation.from_quat(state[[4, 5, 6, 3]])
        tilt = math.degrees(math.acos(float(np.clip(rotation.as_matrix()[2, 2], -1., 1.))))
        self.last_state = state.copy()
        if self.phase == 'waiting':
            period = control.get('control_period_us')
            rate = np.asarray(control.get('gyro_native_rad_s', state[10:13]), dtype=float)
            stationary_rate = rate.shape == (3,) and np.isfinite(rate).all() and np.linalg.norm(rate) < .2
            ready = (not control.get('armed') and control.get('permission') and control.get('supported') and
                     control.get('request_aux') == 3 and control.get('mode') == 'pid' and control.get('indi_weight') == 0 and
                     not control.get('requested', False) and not control.get('fault_latched') and self.limits.control_period_matches(period) and
                     abs(state[2] - self.limits.ground_m) < .03 and tilt < self.limits.maximum_startup_tilt_deg and
                     np.linalg.norm(state[7:10]) < .1 and stationary_rate)
            self.ready_since = (now if self.ready_since is None else self.ready_since) if ready else None
            if self.ready_since is not None and now - self.ready_since >= 1.:
                self.home = np.r_[state[:2], self.limits.ground_m + self.limits.height_m]
                self.yaw = rotation.as_euler('xyz')[2]
                self.pd.reset()
                self.reference = self.reference_factory(self.limits.ground_m, self.home[2], self.limits.settle_s, self.limits.spool_s, self.limits.climb_s)
                self._phase('arming', now)
            return self.output(enabled=self.phase == 'arming')
        if self.phase == 'arming':
            if control.get('armed'):
                self.ever_armed = True
                self.ground_started = now
                self._phase('ground_spinup', now)
            elif now - self.phase_started > 3.:
                self.stop('arming_timeout');return self.output()
            else:return self.output(enabled=True)
        if not control.get('armed'):
            self.stop('unexpected_fc_disarm');return self.output()
        ground_ref = {'position': [*self.home[:2], self.limits.ground_m], 'velocity': [0.] * 3, 'acceleration': [0.] * 3, 'phase': self.ground_reference_phase}
        if self.phase in self.ground_phases:
            self.last_reference = ground_ref
            # This is a measured ground-phase condition, before any INDI ACK.
            # Do not continue a warmup/engagement attempt after unintended lift.
            if state[2] - self.limits.ground_m > self.ground.maximum_height_m:
                self.stop(self.ground_lift_reason);return self.output()
            if (tilt > self.limits.soft_tilt_deg or np.linalg.norm(state[:3] - ground_ref['position']) > self.limits.soft_position_m):
                self.stop('ground_flight_envelope');return self.output()
            if control.get('takeoff_guard_triggered'):
                self.stop('fc_takeoff_guard');return self.output()
            if not control.get('permission') or not control.get('supported') or control.get('fault_latched'):
                self.stop(self.ground_error_prefix + '_unavailable');return self.output()
            elapsed_ground = now - self.ground_started
            if elapsed_ground > self.ground.maximum_wait_s:
                self.stop(self.ground_error_prefix + '_entry_not_ready');return self.output()
            if self.phase == 'ground_spinup':
                if control.get('mode') != 'pid' or control.get('indi_weight') != 0 or control.get('requested', False):
                    self.stop('unexpected_ground_authority');return self.output()
                at_target = elapsed_ground >= self.ground.spool_s + self.ground.warmup_s
                eligible = self._ground_entry_eligible(control, at_target)
                self.ready_since = (now if self.ready_since is None else self.ready_since) if eligible else None
                if self.ready_since is not None and now - self.ready_since >= self.ground.eligible_dwell_s:
                    self._phase(self.ground_request_phase, now)
            if self.phase == self.ground_request_phase:
                if self._ground_acknowledged(control):
                    self._ground_confirmed(now)  # Clock starts only on actual native acknowledgement.
                elif now - self.phase_started > self.limits.transition_timeout_s:
                    self.stop(self.ground_error_prefix + '_engagement_not_acknowledged');return self.output()
            if self.phase in self.ground_phases:
                u = float(np.clip(elapsed_ground / self.ground.spool_s, 0., 1.))
                thrust = self.ground_thrust_N * u ** 3 * (10. - 15. * u + 6. * u * u)
                return self._bounded_output(dt, thrust, np.zeros(3), request=self._ground_request())
        # All healthy airborne mission phases retain request high. A reported
        # fault/authority loss permits the existing native fallback and PD landing.
        # Evaluate hard position against the existing flight reference BEFORE
        # a fault/operator landing resets that reference to the measured state.
        ref = self._flight_reference(now)
        error = float(np.linalg.norm(state[:3] - ref['position']))
        envelope = self._airborne_envelope()
        speed = float(np.linalg.norm(state[7:10]))
        hard_criteria=[name for name,exceeded in (
            ('tilt',tilt>envelope['hard_tilt_deg']),('position',error>envelope['hard_position_m']),
            ('altitude',state[2]>self.limits.maximum_altitude_m),('speed',speed>envelope.get('maximum_speed_m_s',2.))) if exceeded]
        if hard_criteria:
            self.last_envelope_violation=dict(level='hard',criteria=hard_criteria,monotonic_s=now,
                tilt_deg=tilt,position_error_m=error,speed_m_s=speed,altitude_m=float(state[2]),limits=dict(envelope))
            self.stop('hard_flight_envelope');return self.output()
        if control.get('takeoff_guard_triggered'):
            self.stop('fc_takeoff_guard');return self.output()
        # Native IDLE can release INDI as the final ground spool reaches zero.
        # Treat it as normal completion only AFTER measured touchdown latched;
        # the same idle/authority loss while airborne remains a fault landing.
        ground_idle = (self.phase == 'landing' and self.landing.touch_time is not None and
                       abs(state[2] - self.limits.ground_m) <= self.landing.exit_height_m and
                       control.get('reason') == 'idle' and not control.get('fault_latched'))
        if ground_idle:
            self.stop(self.reason or 'test_complete');return self.output()
        fault_reason = self._airborne_fault_reason(control)
        if fault_reason is not None:self._fault_landing(now, state, fault_reason)
        if self.pending_landing and self.phase != 'landing':
            self._land(now, state, self.pending_landing)
        elapsed = now - self.armed_at
        ref = self._flight_reference(now)
        error = float(np.linalg.norm(state[:3] - ref['position']))
        if self.phase != 'landing' and (tilt > envelope['soft_tilt_deg'] or error > envelope['soft_position_m']):
            self.last_envelope_violation=dict(level='soft',criteria=[name for name,exceeded in (
                ('tilt',tilt>envelope['soft_tilt_deg']),('position',error>envelope['soft_position_m'])) if exceeded],
                monotonic_s=now,tilt_deg=tilt,position_error_m=error,speed_m_s=speed,
                altitude_m=float(state[2]),limits=dict(envelope))
            self._land(now, state, 'soft_flight_envelope')
        if self.phase == 'takeoff' and elapsed >= self.reference.climb_end:
            self._phase('indi_hover', now)
        if self.phase == 'indi_hover' and now - self.phase_started >= self.limits.indi_hold_s:
            self._land(now, state, 'test_complete')
        if self.phase == 'landing':
            ref = self.landing.sample(now)
            if self.landing.update_touchdown(now, state, tilt):
                self.stop(self.reason or 'test_complete');return self.output()
            if now > self.landing.timeout:
                self.stop('landing_timeout');return self.output()
        self.last_reference = ref
        preclimb = self.phase == 'takeoff' and elapsed < self.reference.climb_start
        if preclimb:
            # Preserve ground RPM/INDI authority: never restart the legacy
            # zero-thrust settle/spool after the on-ground INDI acknowledgement.
            u = float(np.clip((elapsed - self.limits.settle_s) / self.limits.spool_s, 0., 1.))
            blend = u ** 3 * (10. - 15. * u + 6. * u * u)
            thrust = self.ground_thrust_N + (self.limits.mass_kg * 9.81 * .7 - self.ground_thrust_N) * blend
            if self.spool_leveling:
                level = self.pd.takeoff_command(state[:3], state[7:10], rotation, ref, elapsed, state[2] - self.limits.ground_m,
                                                self.reference.climb_start, yaw=self.yaw, dt=min(dt, .04), integrate=False)
                rates = level[1:]
            else:
                rates = np.zeros(3)
        else:
            if self.phase == 'landing':
                command = self.pd.command(state[:3], state[7:10], rotation, ref, yaw=self.yaw, dt=min(dt, .04), integrate=True)
                command[0] *= self.landing.spool(now)
            else:
                command = self.pd.takeoff_command(state[:3], state[7:10], rotation, ref, elapsed, state[2] - self.limits.ground_m,
                                                  self.reference.climb_start, yaw=self.yaw, dt=min(dt, .04), integrate=True)
            thrust, rates = command[0], command[1:]
        request=self._airborne_request()
        self._update_launch_boost(now,control,request)
        result = self._select_airborne_command(now, state, ref, dt, thrust, rates, request=request)
        if not preclimb or self.spool_leveling:
            self.pd.accept_output(np.r_[result['thrust_N'], result['rates_rad_s']])
        return result
