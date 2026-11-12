"""Pure emergency brake then in-place landing; no I/O or mission admission.

State13 is p_world3,q_wxyz4,v_world3,omega_world3. advance() alone moves
an independently seeded guard model. sample()/diagnostics() never consume
measurements or advance it. This model is an engineering guard, not evidence
that a selected braking rate fits the room.
"""
from collections import deque
import copy
import math
import numpy as np
from scipy.spatial.transform import Rotation
from sanity_zenoh.landing import Landing


def _state(value):
    state=np.asarray(value,dtype=float)
    if state.shape!=(13,) or not np.isfinite(state).all() or abs(np.linalg.norm(state[3:7])-1.)>.001:
        raise ValueError('Finite state13 with unit wxyz quaternion required')
    return state.copy()


def _rotation(state):
    return Rotation.from_quat(state[[4,5,6,3]])


class PidEmergencyLanding:
    rate_tau_s=.05
    collective_tau_s=.025
    command_delay_s=.035
    braking_limit_s=8.
    landing_limit_s=25.
    thrust_slew_N_s=4.
    drag_per_s=np.array([.587,.820,0.])

    def __init__(self,start,state13,ground,reason,mass,thrust_N,thrust_weight,
                 yaw=0.,rate_limit=.6,eta=.953,previous_selected_ctbr=None):
        state=_state(state13)
        numbers=(start,ground,mass,thrust_N,thrust_weight,yaw,rate_limit,eta)
        if not all(np.isfinite(x) and not isinstance(x,(bool,np.bool_)) for x in numbers):
            raise ValueError('Finite emergency landing parameters required')
        if mass<=0 or thrust_N<0 or thrust_weight<=0 or not 0<rate_limit<=2. or not 0<eta<=1:
            raise ValueError('Positive physical limits and rate cap at most2.0rad/s required')
        self.start=float(start);self.ground=float(ground);self.reason=str(reason)
        self.mass=float(mass);self.eta=float(eta);self.rate_limit=float(rate_limit)
        self.maximum_thrust_N=self.mass*9.81*float(thrust_weight)
        self.initial_state=state.copy();self.initial_thrust_N=float(thrust_N)
        self.height=float(state[2]);self.requested_yaw=float(yaw)
        matrix=_rotation(state).as_matrix()
        self.yaw=math.atan2(matrix[1,0],matrix[0,0]) if np.linalg.norm(matrix[:2,0])>1e-8 else float(yaw)
        self.timeout=self.start+self.landing_limit_s
        self.braking_deadline=self.start+self.braking_limit_s
        self.last_advance=self.start;self.advance_count=0;self.last_nominal_cells=0;self.settled_since=None
        self.phase='emergency_brake';self.fail_reason=None;self.landing=None;self.last_settling_checks=None
        self.stopping_anchor=None;self.latched_at=None;self.last_actual_state=state.copy()
        self.last_command=None;self.last_command_time=None;self.last_command_detail=None
        self.nominal_position=state[:3].copy();self.nominal_velocity=state[7:10].copy()
        self.nominal_rotation=_rotation(state)
        self.nominal_body_rate=self.nominal_rotation.inv().apply(state[10:13])
        self.nominal_thrust=float(thrust_N);self.nominal_acceleration=np.zeros(3)
        # Prior selected rates are unknown at entry; retain measured initial
        # rates until the first new braking selection reaches the nominal plant.
        self.previous_selected_ctbr=None
        if previous_selected_ctbr is not None:
            previous=np.asarray(previous_selected_ctbr,dtype=float)
            if previous.shape!=(4,) or not np.isfinite(previous).all() or previous[0]<0:
                raise ValueError('Finite previous selected CTBR4 with nonnegative thrust required')
            self.previous_selected_ctbr=previous.copy()
            # Historical normal-MPC rates may exceed the new braking cap.
            # This is a delayed prior input, never a new emergency output.
            self.nominal_active_command=previous.copy()
            self.nominal_active_command[0]=float(thrust_N)
        else:self.nominal_active_command=np.r_[thrust_N,self.nominal_body_rate]
        self.initial_nominal_active_command=self.nominal_active_command.copy()
        selected,detail=self._control(state[:3],state[7:10],self.nominal_rotation,.02,thrust_N)
        self.nominal_selected_command=selected
        self.nominal_queue=deque([(self.start+self.command_delay_s,selected.copy())])
        if detail.get('failure'):self.fail_reason=detail['failure'];self.phase='emergency_failed'

    @property
    def failure_reason(self):
        return self.fail_reason

    @property
    def active_braking(self):
        return self.landing is None

    @property
    def touch_time(self):
        return None if self.landing is None else self.landing.touch_time

    @property
    def exit_height_m(self):
        return .04 if self.landing is None else self.landing.exit_height_m

    def _control(self,position,velocity,rotation,dt,previous_thrust):
        horizontal=-2.5*np.asarray(velocity[:2])
        horizontal*=min(1.,6./max(float(np.linalg.norm(horizontal)),1e-12))
        az=float(np.clip(4.*(self.height-position[2])-3.*velocity[2],-2.,2.))
        specific=np.r_[horizontal,9.81+az]
        desired_z=specific/np.linalg.norm(specific)
        heading=np.array([math.cos(self.yaw),math.sin(self.yaw),0.])
        desired_y=np.cross(desired_z,heading);desired_y/=np.linalg.norm(desired_y)
        desired=np.column_stack([np.cross(desired_y,desired_z),desired_y,desired_z])
        current=rotation.as_matrix();rzz=float(current[2,2])
        error=Rotation.from_matrix(current.T@desired).as_rotvec()
        rates=np.clip(6.*error,-self.rate_limit,self.rate_limit)
        detail=dict(horizontal_acceleration_target_m_s2=horizontal.tolist(),vertical_acceleration_target_m_s2=az,
                    desired_body_to_world=desired.tolist(),body_log_error_rad=error.tolist(),actual_Rzz=rzz)
        if rzz<=1e-6:
            detail['failure']='emergency_nonpositive_vertical_authority'
            return np.zeros(4),detail
        drag_world=current@(self.drag_per_s*(current.T@np.asarray(velocity)))
        unclipped_vertical_specific=9.81+az+float(drag_world[2])
        vertical_specific=max(0.,unclipped_vertical_specific)
        detail.update(drag_world_m_s2=drag_world.tolist(),
            unclipped_required_vertical_force_N=self.mass*unclipped_vertical_specific,
            required_vertical_force_N=self.mass*vertical_specific,
            vertical_force_numerator_limited=bool(unclipped_vertical_specific<0.))
        raw=self.mass*vertical_specific/(self.eta*rzz)
        target=float(np.clip(raw,0.,self.maximum_thrust_N))
        thrust=float(np.clip(target,previous_thrust-self.thrust_slew_N_s*dt,
                             previous_thrust+self.thrust_slew_N_s*dt))
        thrust=float(np.clip(thrust,0.,self.maximum_thrust_N))
        detail.update(requested_thrust_N=raw,capped_thrust_N=target,selected_thrust_N=thrust,
            thrust_cap_overrode_slew=bool(abs(thrust-previous_thrust)>self.thrust_slew_N_s*dt+1e-9))
        return np.r_[thrust,rates],detail

    def _advance_nominal(self,now):
        interval=now-self.last_advance
        # Normal20ms ticks use two10ms cells. Fresh but late ticks retain the
        # mission's existing landing-request behavior: bounded adaptive work,
        # never a new gap-triggered hard stop or unbounded catch-up loop.
        # Delayed held inputs are selected at cell midpoints. No later measured
        # state is read by this independently propagated model.
        cells=min(16,max(1,int(math.ceil(interval/.01-1e-9))))
        self.last_nominal_cells=cells
        h=interval/cells
        for index in range(cells):
            midpoint=self.last_advance+(index+.5)*h
            while self.nominal_queue and self.nominal_queue[0][0]<=midpoint+1e-12:
                _,self.nominal_active_command=self.nominal_queue.popleft()
            target=self.nominal_active_command
            old_rate=self.nominal_body_rate.copy();old_thrust=self.nominal_thrust
            self.nominal_body_rate+=(-math.expm1(-h/self.rate_tau_s))*(target[1:]-self.nominal_body_rate)
            self.nominal_thrust+=(-math.expm1(-h/self.collective_tau_s))*(target[0]-self.nominal_thrust)
            mid_rotation=self.nominal_rotation*Rotation.from_rotvec(.25*(old_rate+self.nominal_body_rate)*h)
            matrix=mid_rotation.as_matrix()
            acceleration=(matrix[:,2]*(self.eta*.5*(old_thrust+self.nominal_thrust)/self.mass)
                          -[0.,0.,9.81]-matrix@(self.drag_per_s*(matrix.T@self.nominal_velocity)))
            self.nominal_position+=self.nominal_velocity*h+.5*acceleration*h*h
            self.nominal_velocity+=acceleration*h
            self.nominal_rotation=self.nominal_rotation*Rotation.from_rotvec(.5*(old_rate+self.nominal_body_rate)*h)
            self.nominal_acceleration=acceleration
        selected,detail=self._control(self.nominal_position,self.nominal_velocity,self.nominal_rotation,
                                      interval,self.nominal_selected_command[0])
        self.nominal_selected_command=selected
        self.nominal_queue.append((now+self.command_delay_s,selected.copy()))
        if detail.get('failure'):
            self.fail_reason='emergency_nominal_vertical_authority';self.phase='emergency_failed'

    def advance(self,now,state):
        state=_state(state)
        if not math.isfinite(now):raise ValueError('Finite control time required')
        if now<self.last_advance-1e-10:raise ValueError('Emergency control time moved backwards')
        if abs(now-self.last_advance)<=1e-10:return
        if self.fail_reason is not None:return
        if now>=self.timeout:
            self.fail_reason='emergency_landing_timeout';self.phase='emergency_failed';return
        if self.active_braking and now>=self.braking_deadline:
            self.fail_reason='emergency_braking_timeout';self.phase='emergency_failed';return
        interval=now-self.last_advance
        self.last_actual_state=state.copy()
        if self.active_braking:
            self._advance_nominal(now)
            if self.fail_reason is not None:return
            tilt=math.degrees(math.acos(float(np.clip(_rotation(state).as_matrix()[2,2],-1.,1.))))
            self.last_settling_checks=dict(horizontal_speed=bool(np.linalg.norm(state[7:9])<.2),
                tilt=bool(tilt<10.),vertical_speed=bool(abs(state[9])<.3),
                height_error=bool(abs(state[2]-self.height)<.2))
            settled=all(self.last_settling_checks.values())
            self.settled_since=(now if self.settled_since is None else self.settled_since) if settled else None
            self.phase='emergency_settle' if settled else 'emergency_brake'
            if self.settled_since is not None and now-self.settled_since>=.3-1e-9:
                # The normal mission checks its reference after advance(). Do
                # not hide an existing hard guard breach by reanchoring the
                # descent to a measured stopping position on this same tick.
                if np.linalg.norm(state[:3]-self.nominal_position)>1.5:
                    self.fail_reason='emergency_braking_guard_error';self.phase='emergency_failed'
                    return
                self.stopping_anchor=state[:3].copy();self.latched_at=float(now)
                self.landing=Landing(now,state[:3],state[:2],self.ground,self.reason,policy_weight=0.,
                    velocity=np.zeros(3),exit_height_m=.04,exit_dwell_s=.3,speed_profile='quick')
                self.phase=self.landing.sample(now)['phase']
        else:self.phase=self.landing.sample(now)['phase']
        self.last_advance=float(now);self.advance_count+=1
        self.last_command=None;self.last_command_time=None

    def sample(self,now):
        if not math.isfinite(now):raise ValueError('Finite reference time required')
        if self.landing is not None:return self.landing.sample(now)
        elapsed=max(0.,float(now)-self.last_advance)
        # Pure held-velocity preview only. The selected current guard is the
        # independently propagated force-model state, not a measured anchor.
        position=self.nominal_position+self.nominal_velocity*elapsed
        return dict(position=position.tolist(),velocity=self.nominal_velocity.tolist(),
                    acceleration=self.nominal_acceleration.tolist(),phase=self.phase)

    def command(self,now,state,rotation,dt,previous_thrust):
        if self.landing is not None:return None
        _state(state)
        if not all(math.isfinite(v) for v in (now,dt,previous_thrust)) or not 0<dt<=.040000001 or previous_thrust<0:
            raise ValueError('Finite positive bounded control interval and nonnegative prior thrust required')
        if abs(now-self.last_advance)>1e-9 and self.fail_reason is None:
            raise ValueError('advance(now,state) must precede command for this accepted tick')
        if self.last_command_time is not None and abs(now-self.last_command_time)<=1e-10:return self.last_command.copy()
        if self.fail_reason is not None:result=np.zeros(4);detail=dict(failure=self.fail_reason)
        else:
            state=self.last_actual_state
            # Bind attitude and all feedback to the same accepted advance()
            # state; a mutable caller Rotation cannot mix packet generations.
            result,detail=self._control(state[:3],state[7:10],_rotation(state),dt,previous_thrust)
            if detail.get('failure'):self.fail_reason=detail['failure'];self.phase='emergency_failed'
        self.last_command=result.copy();self.last_command_time=float(now);self.last_command_detail=copy.deepcopy(detail)
        return result.copy()

    def update_touchdown(self,now,state,tilt):
        return False if self.landing is None else self.landing.update_touchdown(now,state,tilt)

    def spool(self,now):
        return 1. if self.landing is None else self.landing.spool(now)

    def weight(self,now):
        return 0.

    def diagnostics(self,now=None):
        return dict(version=1,phase=self.phase,active_braking=self.active_braking,fail_reason=self.fail_reason,
            start_s=self.start,last_advance_s=self.last_advance,advance_count=self.advance_count,last_nominal_cells=self.last_nominal_cells,
            braking_deadline_s=self.braking_deadline,timeout_s=self.timeout,settled_since_s=self.settled_since,
            latched_at_s=self.latched_at,height_target_m=self.height,heading_target_rad=self.yaw,settling_checks=copy.deepcopy(self.last_settling_checks),
            requested_yaw_rad=self.requested_yaw,initial_state=self.initial_state.tolist(),initial_thrust_N=self.initial_thrust_N,
            previous_selected_ctbr=None if self.previous_selected_ctbr is None else self.previous_selected_ctbr.tolist(),
            initial_nominal_active_ctbr=self.initial_nominal_active_command.tolist(),
            stopping_anchor=None if self.stopping_anchor is None else self.stopping_anchor.tolist(),
            nominal_guard=dict(position=self.nominal_position.tolist(),velocity=self.nominal_velocity.tolist(),
                quaternion_wxyz=self.nominal_rotation.as_quat()[[3,0,1,2]].tolist(),body_rate=self.nominal_body_rate.tolist(),
                thrust_N=self.nominal_thrust,acceleration=self.nominal_acceleration.tolist(),
                selected_ctbr=self.nominal_selected_command.tolist(),delay_s=self.command_delay_s,
                rate_tau_s=self.rate_tau_s,collective_tau_s=self.collective_tau_s),
            last_command=None if self.last_command is None else self.last_command.tolist(),
            command_detail=copy.deepcopy(self.last_command_detail),limits=dict(rate_rad_s=self.rate_limit,
                thrust_N=self.maximum_thrust_N,thrust_slew_N_s=self.thrust_slew_N_s,horizontal_decel_m_s2=6.,height_accel_m_s2=2.),
            guard_scope='PID branch engineering guard: first-order rate50ms,collective25ms,totaldelay35ms; independent initial-state model, no measured-velocity anchor updates; hard error1.5unchanged')


# Local PID branch contract; implementation is isolated from active INDI.
EmergencyLanding = PidEmergencyLanding
