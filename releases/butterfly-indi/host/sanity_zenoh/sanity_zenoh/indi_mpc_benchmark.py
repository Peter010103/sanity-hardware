"""Explicit paired MPC-outer/INDI-inner benchmark; no I/O in mission creation."""
from dataclasses import dataclass
import importlib.util
import json
import math
from pathlib import Path
import sys
from collections.abc import Mapping
import time
from typing import ClassVar

import numpy as np
from scipy.spatial.transform import Rotation

from sanity_zenoh.indi_hover_mission import HoverLimits
from sanity_zenoh.indi_lemniscate import AngularLemniscate, IndiShapedAngularLemniscate, ButterflyReference, IndiShapedTransfer, LemniscateIndiMission


@dataclass(frozen=True)
class BenchmarkHoverLimits(HoverLimits):
    runtime_ceiling_s: ClassVar[float] = 180.
    allow_signed_ground: ClassVar[bool] = True


@dataclass(frozen=True)
class BenchmarkTrajectory:
    pattern: str = 'lemniscate'
    reference_kind: str = 'gerono'
    butterfly_height_m: float = 0.
    butterfly_cycles: int = 3
    schedule: str = 'constant_arclength'
    center_xy_m: tuple = (0., 0.)
    radius_m: float = 3.2
    width_m: float | None = None
    peak_speed_m_s: float = 1.5
    ramp_s: float = 2.
    laps: int = 3
    transfer_speed_m_s: float = .4
    entry_dwell_s: float = 1.
    entry_timeout_s: float = 10.
    entry_position_m: float = .15
    entry_speed_m_s: float = .25
    entry_tilt_deg: float = 6.
    final_hold_s: float = 2.
    maximum_acceleration_m_s2: float = 1.5
    maximum_jerk_m_s3: float = 2.

    def __post_init__(self):
        if self.reference_kind not in ('gerono','butterfly') or isinstance(self.butterfly_height_m,bool) or not math.isfinite(self.butterfly_height_m) or self.butterfly_height_m<0 or type(self.butterfly_cycles) is not int or self.butterfly_cycles<1:
            raise ValueError('Explicit reference kind and finite butterfly height/cycles required')
        if self.reference_kind=='butterfly' and (self.butterfly_height_m<=0 or self.pattern!='lemniscate' or self.schedule!='indi_shaped_angular'):
            raise ValueError('Butterfly requires positive height and shaped angular Gerono planes')
        if self.reference_kind=='gerono' and self.butterfly_height_m!=0.:raise ValueError('Positive butterfly height requires explicit reference_kind')
        if self.width_m is not None and (isinstance(self.width_m,bool) or not math.isfinite(self.width_m) or self.width_m<=0 or self.pattern!="lemniscate" or self.schedule=="constant_arclength"):
            raise ValueError("Explicit positive width requires an angular lemniscate")
        center=np.asarray(self.center_xy_m,dtype=float)
        values=[value for key,value in vars(self).items() if key not in ('center_xy_m','schedule','pattern','width_m','reference_kind','butterfly_height_m')]
        if center.shape!=(2,) or not np.isfinite(center).all() or not all(type(value) is not bool and math.isfinite(value) and value>0 for value in values):
            raise ValueError('Finite benchmark geometry/timings required')
        if self.schedule not in ('constant_arclength','variable_angular','indi_shaped_angular') or type(self.laps) is not int or (self.laps!=3 if self.reference_kind=='gerono' else self.laps!=2*self.butterfly_cycles):
            raise ValueError('Explicit constant_arclength, variable_angular or indi_shaped_angular three-lap schedule required')
        if self.pattern not in ('lemniscate','circle') or (self.pattern=='circle' and self.schedule!='constant_arclength'):
            raise ValueError('Explicit circle constant_arclength or lemniscate schedule required')
        if self.radius_m not in ({'lemniscate':(2.,3.2),'circle':(1.5,)}[self.pattern]) or self.peak_speed_m_s not in (.8,1.,1.5,3.,4.,5.,6.) or self.ramp_s!=2. or self.transfer_speed_m_s>.4 or self.entry_timeout_s<=max(self.entry_dwell_s,self.final_hold_s):
            raise ValueError('Benchmark requires reviewed geometry, speed levels and bounded timings')
        if self.peak_speed_m_s>=3.:
            from sanity_zenoh.high_speed_lemniscate import reference_limits
            expected=reference_limits(self.peak_speed_m_s,width_m=self.width_m,vertical_amplitude_m=self.butterfly_height_m)
            if self.pattern!='lemniscate' or self.schedule!='indi_shaped_angular' or self.radius_m!=2. or any(getattr(self,key)!=value for key,value in expected.items()):
                raise ValueError('High-speed R2 references require explicit derived acceleration/jerk bounds')
        object.__setattr__(self,'center_xy_m',tuple(float(value) for value in center))


@dataclass(frozen=True)
class BenchmarkEnvelope:
    soft_position_m: float = 1.
    hard_position_m: float = 1.5
    soft_tilt_deg: float = 40.
    hard_tilt_deg: float = 60.
    mpc_rate_limit_rad_s: float = 2.3
    blend_s: float = 2.
    maximum_speed_m_s: float = 2.
    mpc_thrust_weight: float = 1.5

    def __post_init__(self):
        if any(type(value) is bool for value in vars(self).values()) or tuple(vars(self).values()) not in ((1.,1.5,30.,45.,2.3,2.,2.,1.5),(1.,1.5,35.,45.,2.3,2.,4.,1.5),(1.,1.5,40.,60.,2.3,2.,2.,1.5),(1.,1.5,40.,60.,2.3,2.,4.,1.5),(1.,1.5,50.,60.,9.5,2.,5.,1.8),(1.,1.5,60.,75.,9.5,2.,5.,1.8),(1.,1.5,62.,70.,9.5,2.,6.5,2.2),(1.,1.5,72.,82.,9.5,2.,6.5,2.2),(1.,1.5,72.,80.,9.5,2.,7.5,2.9),(1.,1.5,78.,85.,9.5,2.,7.5,2.9)):
            raise ValueError('Explicit reviewed trajectory envelope tuple required; command limits remain separate')


class HistoricalArcReference:
    def __init__(self,center,options,pattern_class):
        self.center=np.array(center,dtype=float)
        self.pattern=pattern_class(options.pattern,options.peak_speed_m_s,radius=options.radius_m,laps=options.laps,
                                   command_speed_limit=options.peak_speed_m_s,ramp_s=options.ramp_s,hold_s=2.,ramp_profile='linear')
        self.duration=self.pattern.duration
        self.offset=np.array([0.,-options.radius_m if options.pattern=='circle' else 0.,0.])

    def sample(self,elapsed):
        if not math.isfinite(elapsed):raise ValueError('Finite reference time required')
        reference=self.pattern.reference(elapsed)
        return dict(position=(self.center+self.offset+np.r_[reference['position_xy'],0.]).tolist(),velocity=[*reference['velocity_xy'],0.],
                    acceleration=[*reference['acceleration_xy'],0.],phase='constant_arclength',
                    lap_progress=reference['distance_m']/self.pattern.length)


from sanity_zenoh.emergency_braking_mission import EmergencyBrakingMixin


class MpcBenchmarkMission(EmergencyBrakingMixin, LemniscateIndiMission):
    trajectory_kind='paired_mpc_gerono'
    mpc_benchmark=True
    worker_required=True
    options_class=BenchmarkTrajectory

    def __init__(self,*args,loco_root,mpc_settings=None,benchmark_envelope=None,inner_controller='indi',**kwargs):
        if inner_controller not in ('pid','indi'):raise ValueError('Explicit inner_controller must be pid or indi')
        self._inner_controller=inner_controller
        if inner_controller=='pid':
            self.ground_request_phase='ground_pid_confirm'
            self.ground_reference_phase='ground_pid_warmup'
            self.ground_error_prefix='ground_pid'
            self.ground_lift_reason='ground_lift_before_pid_ack'
            self.ground_phases=('ground_spinup',self.ground_request_phase)
        self.native_inner_confirmed_at=None;self.pid_confirmed_at=None;self.last_inner_status=None
        self.loco_root=Path(loco_root).resolve()
        spec=importlib.util.spec_from_file_location('paired_benchmark_patterns',self.loco_root/'experiments/sanity_ctbr_sim2sim/missions.py')
        patterns=importlib.util.module_from_spec(spec);spec.loader.exec_module(patterns);self.pattern_class=patterns.PatternMission
        self.benchmark_envelope=BenchmarkEnvelope(**(benchmark_envelope or {}))
        self.mpc_settings=dict(mpc_settings or {})
        if set(self.mpc_settings)-{'config','model'}:raise ValueError('Unknown MPC benchmark setting')
        super().__init__(*args,**kwargs)
        options=self.trajectory_options
        e=self.benchmark_envelope
        expected={3.:(40.,60.,2.3,4.,1.5),4.:(60.,75.,9.5,5.,1.8),5.:(72.,82.,9.5,6.5,2.2),6.:(78.,85.,9.5,7.5,2.9)}.get(options.peak_speed_m_s)
        archival={3.:(35.,45.,2.3,4.,1.5),4.:(50.,60.,9.5,5.,1.8),5.:(62.,70.,9.5,6.5,2.2),6.:(72.,80.,9.5,7.5,2.9)}.get(options.peak_speed_m_s)
        actual=(e.soft_tilt_deg,e.hard_tilt_deg,e.mpc_rate_limit_rad_s,e.maximum_speed_m_s,e.mpc_thrust_weight)
        if expected is not None:
            if not (inner_controller=='indi' and options.pattern=='lemniscate' and options.schedule=='indi_shaped_angular' and options.radius_m==2. and actual in (expected,archival)):
                raise ValueError('High-speed mission requires its exact reviewed R2 INDI speed/envelope tuple')
        elif actual not in ((30.,45.,2.3,2.,1.5),(40.,60.,2.3,2.,1.5)):
            raise ValueError('High-speed airborne/braking limits cannot apply to an ordinary benchmark')
        if self.trajectory_options.reference_kind=='butterfly':self.trajectory_kind='paired_mpc_butterfly'
        self.tracking_phase=self.trajectory_options.pattern
        if self.tracking_phase=='circle':self.trajectory_kind='paired_mpc_circle'
        self.minimum_runtime_s=max(120,math.ceil(self.lap_duration_s+60.))
        if not self.minimum_runtime_s<=self.limits.maximum_run_s<=180. or self.limits.maximum_rate_rad_s!=.6 or self.limits.maximum_thrust_weight!=1.5:
            raise ValueError('Benchmark requires derived runtime<=180s and unchanged PD0.6rad/s/1.5weight caps')
        self.mpc_candidate=None;self.mpc_health={'valid':False,'reason':'no_worker_result'}
        self.mpc_weight=0.;self.mpc_blend_started=None;self.benchmark_envelope_active=False
        self.mpc_entry_stable_since=None;self.mpc_selection={}
        self.mpc_selected_count=0;self.mpc_clip_count=0

    @property
    def inner_controller(self):
        return self._inner_controller

    def _ground_entry_eligible(self,control,at_target):
        if self.inner_controller=='indi':return super()._ground_entry_eligible(control,at_target)
        return at_target and control.get('rpm_mask')==15 and self._ground_acknowledged(control) and control.get('permission') and control.get('supported') and not control.get('fault_latched')

    def _ground_acknowledged(self,control):
        if self.inner_controller=='indi':return super()._ground_acknowledged(control)
        return control.get('mode')=='pid' and control.get('indi_weight')==0 and control.get('requested') is False and control.get('reason')!='idle'

    def _ground_confirmed(self,now):
        self.native_inner_confirmed_at=now
        if self.inner_controller=='indi':return super()._ground_confirmed(now)
        self.pid_confirmed_at=now
        self.armed_at=now
        self._phase('takeoff',now)

    def _ground_request(self):
        return self.inner_controller=='indi' and super()._ground_request()

    def _airborne_fault_reason(self,control):
        if self.inner_controller=='indi':return super()._airborne_fault_reason(control)
        if control.get('fault_latched'):return 'fc_controller_fault_'+str(control.get('fault'))
        if not self.fallback and (not control.get('permission') or not control.get('supported') or not self._ground_acknowledged(control)):
            return 'pid_authority_lost'
        return None

    def _airborne_request(self):
        return self.inner_controller=='indi' and super()._airborne_request()

    def _make_curve(self,center):
        options=self.trajectory_options
        if options.reference_kind=='butterfly':
            return ButterflyReference(center,options.radius_m,options.radius_m if options.width_m is None else options.width_m,
                options.peak_speed_m_s,options.ramp_s,cycles=options.butterfly_cycles,height_amplitude_m=options.butterfly_height_m,
                maximum_acceleration_m_s2=options.maximum_acceleration_m_s2,maximum_jerk_m_s3=options.maximum_jerk_m_s3)
        if options.schedule=='constant_arclength':return HistoricalArcReference(center,options,self.pattern_class)
        if options.schedule=='indi_shaped_angular':
            return IndiShapedAngularLemniscate(center,options.radius_m,options.peak_speed_m_s,options.ramp_s,laps=options.laps,
                maximum_acceleration_m_s2=options.maximum_acceleration_m_s2,maximum_jerk_m_s3=options.maximum_jerk_m_s3,width_m=options.width_m)
        return AngularLemniscate(center,options.radius_m,options.peak_speed_m_s,options.ramp_s,laps=options.laps,width_m=options.width_m)

    def _path_start(self):
        start=super()._path_start()
        if self.trajectory_options.pattern=='circle':start[1]-=self.trajectory_options.radius_m
        return start

    def _make_transfer(self,start,end):
        options=self.trajectory_options
        if options.schedule=='indi_shaped_angular':
            return IndiShapedTransfer(start,end,options.transfer_speed_m_s,
                options.maximum_acceleration_m_s2,options.maximum_jerk_m_s3)
        return super()._make_transfer(start,end)

    def _begin_lap_segment(self,now):
        self.segment=self._make_curve(np.r_[self.trajectory_options.center_xy_m,self.home[2]])
        self._phase('mpc_entry',now)
        self.mpc_entry_stable_since=None

    def _flight_reference(self,now):
        if self.phase in ('mpc_entry','mpc_blend'):
            return dict(position=self._path_start().tolist(),velocity=[0.]*3,acceleration=[0.]*3,phase=self.phase)
        return super()._flight_reference(now)

    def reference_preview(self,now,preview_times,state):
        if self.reference is None or self.home is None:
            return [[list(state[:3]),[0.]*3,[0.]*3] for _ in preview_times]
        return [[self._flight_reference(now+float(offset))[key] for key in ('position','velocity','acceleration')] for offset in preview_times]

    def set_mpc_result(self,candidate,health):
        self.mpc_candidate=candidate
        self.mpc_health=dict(health)

    def _airborne_envelope(self):
        if not self.benchmark_envelope_active:return super()._airborne_envelope()
        e=self.benchmark_envelope
        return dict(soft_position_m=e.soft_position_m,hard_position_m=e.hard_position_m,soft_tilt_deg=e.soft_tilt_deg,hard_tilt_deg=e.hard_tilt_deg,maximum_speed_m_s=e.maximum_speed_m_s)

    def _select_airborne_command(self,now,state,reference,dt,thrust,rates,*,request):
        if self.phase not in ('mpc_blend',self.tracking_phase,'return_home','trajectory_hold') or self.mpc_weight<=0 or self.pending_landing or self.fallback:
            if self.phase=='landing' or self.fallback:self.mpc_weight=0.
            return super()._select_airborne_command(now,state,reference,dt,thrust,rates,request=request)
        requested=np.asarray(self.mpc_candidate['ctbr'],dtype=float)
        cap=self.benchmark_envelope.mpc_rate_limit_rad_s
        bounded=requested.copy();bounded[0]=np.clip(bounded[0],0.,self.limits.mass_kg*9.81*self.benchmark_envelope.mpc_thrust_weight)
        bounded[1:]=np.clip(bounded[1:],-cap,cap)
        if self.mpc_weight<1.:
            pd=super()._select_airborne_command(now,state,reference,dt,thrust,rates,request=request)
            actual=(1.-self.mpc_weight)*np.r_[pd['thrust_N'],pd['rates_rad_s']]+self.mpc_weight*bounded
        else:actual=bounded
        self.last_thrust=float(actual[0])
        self.mpc_selection=dict(requested_ctbr=requested.tolist(),bounded_mpc_ctbr=bounded.tolist(),selected_ctbr=actual.tolist(),
                                command_clipped=bool(np.any(bounded!=requested)),weight=self.mpc_weight)
        self.mpc_selected_count+=1;self.mpc_clip_count+=int(self.mpc_selection['command_clipped'])
        return self.output(True,actual[0],actual[1:],request)

    def _record_inner_status(self,control):
        self.last_inner_status={key:control.get(key) for key in ('mode','indi_weight','requested','permission','armed')} if control is not None else None

    def _prepare_mpc_tick(self,now,control):
        self.mpc_selection={}
        self._record_inner_status(control)
        active=self.mpc_weight>0 or self.phase in ('mpc_blend',self.tracking_phase,'return_home','trajectory_hold')
        if self.pending_landing or self.phase=='landing':self.mpc_weight=0.
        elif active and not self.mpc_health.get('valid'):
            self.request_landing('mpc_'+self.mpc_health.get('reason','unavailable'))
            self.mpc_weight=0.
        elif self.phase=='mpc_blend':
            self.mpc_weight=float(np.clip((now-self.mpc_blend_started)/self.benchmark_envelope.blend_s,0.,1.))

    def _update_mpc_admission(self,now,state):
        if self.phase=='mpc_entry':
            sample=np.asarray(state,dtype=float);tilt=math.degrees(math.acos(float(np.clip(Rotation.from_quat(sample[[4,5,6,3]]).as_matrix()[2,2],-1,1))))
            center=self._path_start()
            eligible=self.mpc_health.get('valid') and np.linalg.norm(sample[:3]-center)<.06 and np.linalg.norm(sample[7:10])<.1 and tilt<5.
            self.mpc_entry_stable_since=(now if self.mpc_entry_stable_since is None else self.mpc_entry_stable_since) if eligible else None
            if self.mpc_entry_stable_since is not None and now-self.mpc_entry_stable_since>=1.:
                self.mpc_blend_started=now;self.benchmark_envelope_active=True;self._phase('mpc_blend',now)
            elif now-self.phase_started>=self.trajectory_options.entry_timeout_s:self.request_landing('mpc_entry_not_ready')
        elif self.phase=='mpc_blend' and self.mpc_weight>=1.:
            self._phase(self.tracking_phase,now)

    def tick(self,now,state,control,**kwargs):
        self._prepare_mpc_tick(now,control)
        output=super().tick(now,state,control,**kwargs)
        if self.done or self.phase=='landing':return output
        self._update_mpc_admission(now,state)
        output['phase']=self.phase
        return output

    def diagnostics(self,output):
        result=super().diagnostics(output)
        result['inner_controller']=self.inner_controller
        result['native_inner_status']=self.last_inner_status
        result['native_inner_confirmed_at']=self.native_inner_confirmed_at
        result['pid_confirmed_at']=self.pid_confirmed_at
        result['indi_confirmed_at']=self.indi_confirmed_at
        result['mpc']=dict(health=dict(self.mpc_health),weight=self.mpc_weight,selection=dict(self.mpc_selection),
                           benchmark_envelope_active=self.benchmark_envelope_active,airborne_envelope=self._airborne_envelope(),
                           selected_count=self.mpc_selected_count,clipped_count=self.mpc_clip_count,
                           solver=self.mpc_candidate.get('mpc') if self.mpc_candidate is not None else None)
        return result


class MpcWorkerRuntime:
    """Ordered stdio worker only. Construction is explicit, never in the factory."""
    def __init__(self,mission,output,event=None):
        from sanity_zenoh.async_policy import AsyncPolicy
        self.event=event or (lambda record:None)
        root=mission.loco_root;settings=mission.mpc_settings
        config=Path(settings.get('config',root.parent/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_mpc.json')).resolve()
        model=Path(settings.get('model',root/'loco_mjx/environments/robots/sanity_ctbr/data/plane.xml')).resolve()
        source=Path(__file__).resolve().parents[3]/'mpc_controller.py'
        destination=Path(output);destination.mkdir(parents=True,exist_ok=False)
        self.worker=AsyncPolicy(Path(sys.executable),source,config,destination,controller='mpc',mpc_model=model)
        try:
            self.info=dict(self.worker.info);self.preview_times=self.info['preview_times_s'];self.next_tick=0;self.pending_tick=None
            self.rate_prediction_enabled=self.info.get('rate_response_configuration',{}).get('enabled',False)
            self.prediction_lead_s=.02+float(self.info.get('command_delay_s',0.)) if self.rate_prediction_enabled else 0.
            self.pid=self.worker.actor.process.pid;self.closed=False;self.last_health={};self.runtime_error=None
            if not math.isclose(self.info['mass_kg'],mission.limits.mass_kg,abs_tol=1e-8) or not math.isclose(self.info['policy_dt_s'],.02):
                raise ValueError('MPC model mass/control period differs from the explicit benchmark')
            self.event(dict(kind='mpc_worker_started',pid=self.pid,info=self.info))
        except BaseException:
            self.worker.close()
            raise

    def prepare(self,mission,now,state,mocap_age_s):
        if self.pending_tick is not None:raise RuntimeError('MPC observation needs exactly one selected-command commit')
        valid=state is not None and np.asarray(state).shape==(13,) and np.isfinite(state).all() and math.isfinite(mocap_age_s) and 0<=mocap_age_s<.04
        if valid and not self.worker.error and self.runtime_error is None:
            stamp=int(round((now-mocap_age_s)*1e9))
            try:
                self.worker.observe(dict(tick=self.next_tick,state_stamp_ns=stamp,state=list(state),goal=None,
                                         reference_preview=mission.reference_preview(now+getattr(self,'prediction_lead_s',0.),self.preview_times,state),
                                         **({'prediction_context':dict(target_stamp_ns=int(round((now+self.prediction_lead_s)*1e9)),asynchronous_selection_lead_s=.02,nominal_command_delay_s=self.prediction_lead_s-.02)} if getattr(self,'rate_prediction_enabled',False) else {})))
                self.pending_tick=self.next_tick;self.next_tick+=1
            except Exception as exc:self.runtime_error=str(exc)
        self.refresh(mission,now)

    def refresh(self,mission,now):
        reason='no_result';age=None;result_tick=None;total_ms=None;result=None
        try:
            result=self.worker.latest()
            if self.worker.error or self.runtime_error is not None:reason='worker_error'
            elif result is not None:
                if not isinstance(result,Mapping):raise ValueError('Result mapping required')
                result_tick=result.get('tick');mpc=result.get('mpc')
                stamp=result['state_stamp_ns']
                if type(stamp) is not int or type(result_tick) is not int or not isinstance(mpc,Mapping):
                    raise ValueError('Typed result timestamp/tick/solver mapping required')
                age=(int(round(now*1e9))-stamp)*1e-9
                total_ms=mpc.get('total_ms')
                if isinstance(total_ms,bool) or not isinstance(total_ms,(int,float)):
                    raise ValueError('Numeric compute duration required')
                ctbr=np.asarray(result.get('ctbr'),dtype=float)
                bounds=np.asarray(self.info['ctrlrange'],dtype=float)
                tick=self.pending_tick if self.pending_tick is not None else self.next_tick-1
                if result_tick not in (tick,tick-1):reason='tick_age'
                elif not 0<=age<=.04+1e-12:reason='source_age'
                elif result.get('valid') is not True or mpc.get('success') is not True:reason='invalid_solver'
                elif not math.isfinite(total_ms) or not 0<=total_ms<=25.:reason='compute_deadline'
                elif ctbr.shape!=(4,) or not np.isfinite(ctbr).all():reason='nonfinite_result'
                elif bounds.shape!=(4,2) or not np.isfinite(bounds).all():raise ValueError('Solver bounds required')
                elif np.any(ctbr<bounds[:,0]-1e-6) or np.any(ctbr>bounds[:,1]+1e-6):reason='solver_bounds'
                else:reason='ok'
        except (KeyError,TypeError,ValueError,OverflowError):
            reason='malformed_result'
        health=dict(valid=reason=='ok',reason=reason,source_age_s=age if age is not None and math.isfinite(age) else None,result_tick=result_tick if type(result_tick) is int else None,
                    state_time_domain='host_monotonic_ns; state timestamp reconstructed from measured age duration',
                    total_ms=total_ms if isinstance(total_ms,(int,float)) and not isinstance(total_ms,bool) and math.isfinite(total_ms) else None)
        self.last_health=health;mission.set_mpc_result(result if reason=='ok' else None,health)
        return health

    def commit(self,output):
        if self.pending_tick is None:return
        selected=[output['thrust_N'],*output['rates_rad_s']]
        try:
            self.worker.commit(dict(tick=self.pending_tick,selected_ctbr=selected,
                                    **({'controller_context':dict(selected_stamp_ns=time.monotonic_ns())} if getattr(self,'rate_prediction_enabled',False) else {})))
            self.event(dict(kind='mpc_selected_command',tick=self.pending_tick,selected_ctbr=selected,health=dict(self.last_health)))
        except Exception as exc:
            self.runtime_error=str(exc);self.event(dict(kind='mpc_commit_error',tick=self.pending_tick,error=str(exc)))
        self.pending_tick=None

    def close(self):
        if self.closed:return
        self.worker.close();self.closed=True
        self.event(dict(kind='mpc_worker_closed',pid=self.pid,exit_code=self.worker.actor.process.poll(),error=self.worker.error))
