"""Benchmark-only emergency braking and a bounded landing-time reserve.

This proposal has no I/O. Ordinary takeoff/settled landing retain 0.6 rad/s.
The PID butterfly comparison explicitly uses emergency2rad/s after admission.
State, identity, speed, position and attitude guards retain their values.
"""
import math
import numpy as np

from sanity_zenoh.emergency_landing import EmergencyLanding


class EmergencyBrakingMixin:
    landing_reserve_s = 25.

    @property
    def emergency_braking_rate_rad_s(self):
        options=self.trajectory_options
        reviewed=(self.inner_controller=='pid' and options.reference_kind=='butterfly'
                  and options.peak_speed_m_s in (3.,4.,5.) and options.radius_m==2.
                  and options.width_m==3. and options.butterfly_height_m==.3)
        return 2. if reviewed else self.limits.maximum_rate_rad_s

    def _emergency_braking_enabled(self):
        return (self.inner_controller == 'pid' and self.benchmark_envelope_active
                and self.trajectory_options.peak_speed_m_s >= 3.)

    def _land(self, now, state, reason):
        speed = float(np.linalg.norm(np.asarray(state)[7:10]))
        if self._emergency_braking_enabled() and (reason != 'test_complete' or speed > .25):
            self._end_launch_boost('landing:' + reason)
            self.reason = self.reason or reason
            self.landing = EmergencyLanding(now, state, self.limits.ground_m, self.reason,
                mass=self.limits.mass_kg, thrust_N=self.last_thrust,
                thrust_weight=getattr(self.benchmark_envelope, 'mpc_thrust_weight', 1.5),
                yaw=self.yaw, rate_limit=self.emergency_braking_rate_rad_s,
                previous_selected_ctbr=getattr(self, '_previous_selected_ctbr', None))
            self._phase('landing', now)
        else:
            super()._land(now, state, reason)

    def _update_landing_state(self, now, state):
        if isinstance(self.landing, EmergencyLanding):
            try:
                self.landing.advance(now, state)
            except ValueError:
                self.stop('emergency_landing_state_error')
                return
            if self.landing.failure_reason:
                self.stop(self.landing.failure_reason)
        else:
            super()._update_landing_state(now, state)

    def _handle_runtime_expiry(self, now, state):
        if not self._emergency_braking_enabled():
            return super()._handle_runtime_expiry(now, state)
        if self.phase != 'landing':
            self.request_landing('maximum_runtime')
        elif now > min(self.landing.timeout, self.landing.start + self.landing_reserve_s):
            self.stop('landing_timeout')

    def tick(self, now, state, control, **kwargs):
        if (math.isfinite(now) and self.first_tick is not None and not self.done
                and self._emergency_braking_enabled() and self.phase != 'landing'
                and now - self.first_tick >= self.limits.maximum_run_s - self.landing_reserve_s):
            self.request_landing('maximum_runtime')
        output = super().tick(now, state, control, **kwargs)
        if output['enabled']:
            # _prepare_mpc_tick clears diagnostic selection before a new
            # landing is considered. Retain the actual previous output here.
            self._previous_selected_ctbr = np.r_[output['thrust_N'], output['rates_rad_s']]
        return output

    def _select_airborne_command(self, now, state, reference, dt, thrust, rates, *, request):
        if self.phase == 'landing' and isinstance(self.landing, EmergencyLanding):
            from scipy.spatial.transform import Rotation
            rotation = Rotation.from_quat(np.asarray(state)[[4, 5, 6, 3]])
            command = self.landing.command(now, state, rotation, min(dt, .04), self.last_thrust)
            if command is not None:
                self.last_thrust = float(command[0])
                return self.output(True, command[0], command[1:], request)
        return super()._select_airborne_command(now, state, reference, dt, thrust, rates, request=request)

    def diagnostics(self, output):
        result = super().diagnostics(output)
        result['emergency_braking_configuration'] = dict(rate_limit_rad_s=self.emergency_braking_rate_rad_s,
            ordinary_PD_rate_limit_rad_s=self.limits.maximum_rate_rad_s,
            scope='Explicit PID comparison: emergency2rad/s for admitted3/4/5butterfly; ordinarytakeoff/settledlanding0.6; guardrate/collective lag are documented PID engineering assumptions')
        if isinstance(getattr(self, 'landing', None), EmergencyLanding):
            result['emergency_landing'] = self.landing.diagnostics()
        return result
