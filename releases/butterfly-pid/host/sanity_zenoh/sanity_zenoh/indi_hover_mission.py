"""Single-aircraft PD takeoff, acknowledged INDI hover, PID return and landing.

This module has no network/ROS/hardware side effects. Freshness and authority
are explicit inputs so fault cases can be tested without arming anything.
"""
from dataclasses import dataclass
import copy
import math
from typing import ClassVar
import numpy as np
from scipy.spatial.transform import Rotation
from sanity_zenoh.landing import Landing

@dataclass(frozen=True)
class HoverLimits:
    runtime_ceiling_s: ClassVar[float] = 120.
    allow_signed_ground: ClassVar[bool] = False
    expected_uid: str
    pid_only: bool=False
    ground_m: float=.0152
    height_m: float=.5
    mass_kg: float=.06947
    settle_s: float=.5
    spool_s: float=1.5
    climb_s: float=5.
    pid_hold_s: float=2.
    indi_hold_s: float=5.
    transition_timeout_s: float=1.5
    maximum_run_s: float=60.
    maximum_altitude_m: float=1.
    soft_position_m: float=.25
    hard_position_m: float=.75
    soft_tilt_deg: float=15.
    hard_tilt_deg: float=35.
    maximum_rate_rad_s: float=.6
    maximum_thrust_weight: float=1.5
    maximum_startup_tilt_deg: float=6.
    controller_status_max_age_s: float=.15
    expected_control_period_us: int=500

    def __post_init__(self):
        if len(self.expected_uid)!=24 or len(bytes.fromhex(self.expected_uid))!=12:
            raise ValueError('Explicit 12-byte FC MSP UID required')
        if type(self.pid_only) is not bool:raise ValueError('pid_only must be boolean')
        if type(self.expected_control_period_us) is not int or self.expected_control_period_us not in (500,1000):
            raise ValueError('Explicit expected control period must be500us or1000us')
        if self.allow_signed_ground and (isinstance(self.ground_m,bool) or not math.isfinite(self.ground_m)):
            raise ValueError('Finite world-frame ground coordinate required')
        values=[v for k,v in vars(self).items() if k not in ('expected_uid','pid_only') and not (k=='ground_m' and self.allow_signed_ground)]
        if not all(math.isfinite(v) and v>0 for v in values):raise ValueError('Positive finite hover limits required')
        maximum_height=1.0
        if not .2<=self.height_m<=maximum_height or self.ground_m+self.height_m>=self.maximum_altitude_m:
            raise ValueError(f'Hover height .2..{maximum_height}m must be below the absolute altitude limit')
        if not self.soft_position_m<self.hard_position_m or not 5<self.soft_tilt_deg<self.hard_tilt_deg<=45:
            raise ValueError('Ordered soft/hard position and tilt limits required')
        if self.maximum_rate_rad_s>1 or self.maximum_thrust_weight>1.8 or self.maximum_run_s>self.runtime_ceiling_s:
            raise ValueError('Initial hover envelope is deliberately bounded')
        if not .1<=self.controller_status_max_age_s<=.3:
            raise ValueError('Controller diagnostic age must be0.1–0.3 seconds')
        if self.maximum_startup_tilt_deg>10 or self.maximum_startup_tilt_deg>=self.soft_tilt_deg:
            raise ValueError('Startup tilt allowance must be at most10 degrees and below the flight tilt limit')

    def control_period_matches(self, period_us):
        """Match the selected profile, retaining the original +/-10% allowance."""
        return (isinstance(period_us,(int,float)) and not isinstance(period_us,bool) and
                math.isfinite(period_us) and
                .9*self.expected_control_period_us<=period_us<=1.1*self.expected_control_period_us)

class IndiHoverMission:
    def __init__(self, limits, pd, reference_factory):
        self.limits=limits;self.pd=pd;self.reference_factory=reference_factory
        self.phase='waiting';self.reason=None;self.done=False
        self.first_tick=None;self.last_tick=None;self.ready_since=None
        self.phase_started=None;self.armed_at=None;self.last_state=None
        self.home=None;self.yaw=0.;self.reference=None;self.landing=None
        self.last_reference=None
        self.last_thrust=0.;self.ever_armed=False;self.pending_landing=None
        self._tick_pd_start_count=0

    def diagnostics(self, output):
        """Copy live internal state; explicitly mark a tick with no PD evaluation.

        Commands and control arithmetic are untouched. Old PD.last values are
        never presented as a new desired attitude during ground/spool/stop.
        Integral state is sampled directly even when the PD did not run.
        """
        count=getattr(self.pd,'command_count',0)
        evaluated=count!=self._tick_pd_start_count
        return {'version':1,'pd_evaluated_this_tick':evaluated,
                'pd_command_count':count,'world_frame':'configured_world_z_up',
                'attitude_matrix_convention':'body_to_configured_world',
                'integral_acceleration_state_m_s2':self.pd.integral_acceleration.tolist(),
                'integral_limits_m_s2':self.pd.integral_limits.tolist(),
                'pd':copy.deepcopy(self.pd.last) if evaluated else None,
                'selected_ctbr':[output['thrust_N'],*output['rates_rad_s']],
                'selected_enabled':output['enabled'],
                'expected_control_period_us':self.limits.expected_control_period_us}

    def stop(self, reason):
        self.reason=reason;self.phase='stopped';self.done=True

    def request_landing(self, reason='operator_landing'):
        if self.phase in ('waiting','arming') and not self.ever_armed:
            self.stop(reason)
        elif not self.done:self.pending_landing=self.pending_landing or reason

    def _phase(self, name, now):
        self.phase=name;self.phase_started=now;self.ready_since=None

    def _land(self, now, state, reason):
        self.reason=self.reason or reason
        self.landing=Landing(now,state[:3],self.home[:2],self.limits.ground_m,self.reason,
            policy_weight=0.,velocity=state[7:10],exit_height_m=.04,exit_dwell_s=.3,speed_profile='quick')
        self._phase('landing',now)

    def output(self, enabled=False, thrust=0., rates=(0.,0.,0.), request=False):
        return {'enabled':bool(enabled),'thrust_N':float(thrust),'rates_rad_s':[float(x) for x in rates],
                'indi_requested':bool(request and enabled),'phase':self.phase,'reason':self.reason,'done':self.done}

    def tick(self, now, state, control, *, mocap_age_s=0., controller_age_s=0., receiver_ready=True):
        self._tick_pd_start_count=getattr(self.pd,'command_count',0)
        if not math.isfinite(now):raise ValueError('Finite monotonic time required')
        if self.done:return self.output()
        if self.first_tick is None:self.first_tick=now
        dt=.02 if self.last_tick is None else now-self.last_tick
        self.last_tick=now
        if dt<=0: self.stop('clock_not_monotonic');return self.output()
        live=self.phase not in ('waiting','arming')
        if live and dt>.04:self.request_landing('control_deadline')
        state=np.asarray(state,dtype=float) if state is not None else None
        valid_state=state is not None and state.shape==(13,) and np.isfinite(state).all() and abs(np.linalg.norm(state[3:7])-1)<.001
        fresh=valid_state and math.isfinite(mocap_age_s) and 0<=mocap_age_s<.04
        control_fresh=control is not None and math.isfinite(controller_age_s) and 0<=controller_age_s<self.limits.controller_status_max_age_s
        if not fresh or not control_fresh or not receiver_ready:
            if self.phase!='waiting':self.stop('state_or_receiver_unavailable')
            else:self.ready_since=None
            if self.phase=='waiting' and now-self.first_tick>20:self.stop('startup_timeout')
            return self.output()
        if control.get('fc_uid_msp')!=self.limits.expected_uid:
            self.stop('fc_identity_mismatch');return self.output()
        if control.get('benchmark_active',False):self.stop('benchmark_active');return self.output()
        if self.limits.pid_only and live:
            if control.get('mode')!='pid' or control.get('indi_weight')!=0 or control.get('requested',False):
                self.stop('pid_only_authority_changed');return self.output()
            if control.get('permission'):self.request_landing('indi_permission_changed')
        if live and not control.get('armed'):
            self.stop('unexpected_fc_disarm');return self.output()
        if now-self.first_tick>self.limits.maximum_run_s:
            self.stop('maximum_runtime');return self.output()
        if self.phase=='waiting' and now-self.first_tick>20:
            self.stop('startup_timeout');return self.output()
        rotation=Rotation.from_quat(state[[4,5,6,3]])
        tilt=math.degrees(math.acos(float(np.clip(rotation.as_matrix()[2,2],-1,1))))
        self.last_state=state.copy()
        if self.phase=='waiting':
            permission_ok=not control.get('permission') if self.limits.pid_only else control.get('permission')
            ready=not control.get('armed') and permission_ok and control.get('supported') and control.get('request_aux')==3 and control.get('mode')=='pid' and control.get('indi_weight')==0 and not control.get('requested',False) and not control.get('fault_latched')
            period=control.get('control_period_us')
            ready=ready and self.limits.control_period_matches(period)
            # The FC gyro directly measures body motion; differentiating the
            # mocap quaternion can report motion from optical pose jitter.
            # Legacy/offline fixtures without FC gyro retain their state rate.
            startup_rate=np.asarray(control.get('gyro_native_rad_s',state[10:13]),dtype=float)
            stationary_rate=startup_rate.shape==(3,) and np.isfinite(startup_rate).all() and np.linalg.norm(startup_rate)<.2
            ready=ready and abs(state[2]-self.limits.ground_m)<.03 and tilt<self.limits.maximum_startup_tilt_deg and np.linalg.norm(state[7:10])<.1 and stationary_rate
            self.ready_since=(now if self.ready_since is None else self.ready_since) if ready else None
            if self.ready_since is not None and now-self.ready_since>=1.:
                self.home=np.r_[state[:2],self.limits.ground_m+self.limits.height_m]
                self.yaw=rotation.as_euler('xyz')[2];self.pd.reset()
                self.reference=self.reference_factory(self.limits.ground_m,self.home[2],
                    self.limits.settle_s,self.limits.spool_s,self.limits.climb_s)
                self._phase('arming',now)
            return self.output(enabled=self.phase=='arming')
        if self.phase=='arming':
            if control.get('armed'):
                self.ever_armed=True;self.armed_at=now;self._phase('takeoff',now)
            elif now-self.phase_started>3.:
                self.stop('arming_timeout');return self.output()
            else:return self.output(enabled=True)
        elapsed=now-self.armed_at
        if self.phase=='takeoff':
            ref=self.reference.sample(elapsed)
            # TakeoffReference supplies a vertical trajectory at x=y=0.
            # Translate it to the latched launch position before both the
            # position-envelope check and the position controller consume it.
            ref['position'][:2]=self.home[:2].tolist()
        elif self.phase=='landing':ref=self.landing.sample(now)
        else:ref={'position':self.home.tolist(),'velocity':[0.]*3,'acceleration':[0.]*3}
        error=float(np.linalg.norm(state[:3]-np.asarray(ref['position'])))
        if tilt>self.limits.hard_tilt_deg or error>self.limits.hard_position_m or state[2]>self.limits.maximum_altitude_m or np.linalg.norm(state[7:10])>2.:
            self.stop('hard_flight_envelope');return self.output()
        if self.phase not in ('landing','return_pid') and (tilt>self.limits.soft_tilt_deg or error>self.limits.soft_position_m):
            self.request_landing('soft_flight_envelope')
        if control.get('takeoff_guard_triggered'):
            self.stop('fc_takeoff_guard');return self.output()
        if self.phase in ('enter_indi','indi_hover') and control.get('fault_latched'):
            self.request_landing('fc_indi_fault_'+str(control.get('fault')))
        if self.phase=='indi_hover' and (control.get('mode')!='indi' or control.get('indi_weight')!=1):
            self.request_landing('indi_authority_lost')
        if self.pending_landing and self.phase not in ('return_pid','landing'):
            self.reason=self.reason or self.pending_landing;self._phase('return_pid',now)
        if self.phase=='takeoff' and elapsed>=self.reference.climb_end:
            self._phase('pid_hover',now)
        if self.phase=='pid_hover':
            if self.limits.pid_only:
                if now-self.phase_started>=self.limits.pid_hold_s:
                    self._land(now,state,'test_complete')
                # Continue using the same bounded PD and landing paths below;
                # this profile never visits enter_indi / indi_hover.
            else:
                settled=error<.06 and np.linalg.norm(state[7:10])<.15 and tilt<8 and np.linalg.norm(state[10:13])<.4 and control.get('eligible') and control.get('mode')=='pid'
                self.ready_since=(now if self.ready_since is None else self.ready_since) if settled else None
                if now-self.phase_started>=self.limits.pid_hold_s and self.ready_since is not None and now-self.ready_since>=.5:
                    self._phase('enter_indi',now)
                elif now-self.phase_started>self.limits.pid_hold_s+8.:
                    self.reason='indi_entry_not_ready';self._phase('return_pid',now)
        if self.phase=='enter_indi':
            if control.get('mode')=='indi' and control.get('indi_weight')==1:
                self._phase('indi_hover',now)
            elif now-self.phase_started>self.limits.transition_timeout_s:
                self.reason='indi_engagement_not_acknowledged';self._phase('return_pid',now)
        if self.phase=='indi_hover' and now-self.phase_started>=self.limits.indi_hold_s:
            self._phase('return_pid',now)
        if self.phase=='return_pid':
            if control.get('mode')=='pid' and control.get('indi_weight')==0:
                self._land(now,state,self.reason or 'test_complete')
            elif now-self.phase_started>self.limits.transition_timeout_s:
                # Fresh state/control still permits a PD landing while the FC
                # processes the low request; never freeze outputs awaiting ACK.
                self._land(now,state,self.reason or 'pid_return_not_acknowledged')
        if self.phase=='landing':
            ref=self.landing.sample(now)
            if self.landing.update_touchdown(now,state,tilt):
                self.stop(self.reason or 'test_complete');return self.output()
            if now>self.landing.timeout:
                self.stop('landing_timeout');return self.output()
        self.last_reference=ref
        if self.phase!='landing' and elapsed<self.reference.climb_start:
            factor=float(np.clip((elapsed-self.limits.settle_s)/self.limits.spool_s,0,1))
            thrust=self.limits.mass_kg*9.81*.7*factor**3*(10-15*factor+6*factor*factor);rates=np.zeros(3)
        else:
            command=self.pd.takeoff_command(state[:3],state[7:10],rotation,ref,elapsed,
                state[2]-self.limits.ground_m,self.reference.climb_start,yaw=self.yaw,
                dt=min(dt,.04),integrate=True)
            if self.phase=='landing':
                command=self.pd.command(state[:3],state[7:10],rotation,ref,yaw=self.yaw,dt=min(dt,.04),integrate=True)
            thrust=float(command[0]);rates=command[1:]
            if self.phase=='landing':thrust*=self.landing.spool(now)
        thrust=float(np.clip(thrust,0,self.limits.mass_kg*9.81*self.limits.maximum_thrust_weight))
        # Limit command change, not the emergency disabled output above.
        thrust=float(np.clip(thrust,self.last_thrust-.5*min(dt,.04),self.last_thrust+.5*min(dt,.04)))
        self.last_thrust=thrust
        rates=np.clip(rates,-self.limits.maximum_rate_rad_s,self.limits.maximum_rate_rad_s)
        self.pd.accept_output(np.r_[thrust,rates])
        return self.output(True,thrust,rates,self.phase in ('enter_indi','indi_hover'))
