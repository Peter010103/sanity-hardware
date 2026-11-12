"""Explicit bounded control-identification missions; pure factory/reference APIs."""
import copy
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from sanity_zenoh.full_indi_mission import FullIndiMission
from sanity_zenoh.indi_mpc_benchmark import MpcBenchmarkMission

TESTS=('pd-hover','roll-pulse','pitch-pulse','yaw-pulse','roll-sine','pitch-sine','mpc-hover')


def smoothstep(u):
    u=float(np.clip(u,0.,1.))
    return u*u*u*(10.+u*(-15.+6.*u))


@dataclass(frozen=True)
class DiagnosticOptions:
    test: str
    sine_profile: str='baseline'

    def __post_init__(self):
        if self.test not in TESTS:raise ValueError('Explicit reviewed control diagnostic required')
        if self.sine_profile not in ('baseline','extended') or (self.sine_profile!='baseline' and not self.test.endswith('-sine')):
            raise ValueError('Sine profile must be baseline or extended for a sine test')

    @property
    def sine_frequencies(self):
        return [.5,1.,2.,3.,4.] if self.sine_profile=='extended' else [.5,1.,2.]

    @property
    def sine_amplitude(self):return .30 if self.sine_profile=='extended' else .15

    @property
    def duration_s(self):
        if self.test.endswith('-pulse'):return 13.2
        if self.test.endswith('-sine'):return 6.+8.*len(self.sine_frequencies)-2.
        return 10.

    @property
    def metadata(self):
        result=dict(test=self.test,duration_s=self.duration_s,reference='stationary_launch_home',
                    rate_frame='host CTBR body FLU',fc_rate_frame='FC native body FRD; compare via diag(1,-1,-1)',
                    fc_demand_source='Blackbox demand_native p46..48; not available in SIC host snapshot',
                    final_hold='at least2s plus continuous1s measured settling;10s timeout, separate from nominal duration',
                    outer='mpc' if self.test=='mpc-hover' else 'pd',pre_s=3. if '-' in self.test and self.test not in ('pd-hover','mpc-hover') else 0.,
                    post_s=3. if self.test.endswith(('-pulse','-sine')) else 0.)
        if self.test.endswith('-pulse'):
            result['excitation']=dict(axis=self.test.split('-')[0],amplitude_rad_s=.10 if self.test=='yaw-pulse' else .15,
                signs=[1,-1,1,-1],pulse_s=.30,edge_s=.05,recovery_s=2.,edge='quintic_smoothstep',recovery_scope='timed zero injection; not measured settling')
        elif self.test.endswith('-sine'):
            result['excitation']=dict(axis=self.test.split('-')[0],amplitude_rad_s=self.sine_amplitude,frequencies_hz=self.sine_frequencies,
                segment_s=6.,taper_s=1.,recovery_s=2.,edge='quintic_smoothstep',recovery_scope='timed zero injection; not measured settling')
            if self.sine_profile!='baseline':result['sine_profile']=self.sine_profile
        return result


def rate_injection(test,elapsed,sine_profile='baseline'):
    """Pure requested additive rate; schedule never advances on sampling."""
    options=DiagnosticOptions(test,sine_profile)
    if not math.isfinite(elapsed):raise ValueError('Finite diagnostic time required')
    result=np.zeros(3)
    if elapsed<0 or elapsed>=options.duration_s:return result
    axis={'roll':0,'pitch':1,'yaw':2}.get(test.split('-')[0])
    if axis is None:return result
    amplitude=options.sine_amplitude if test.endswith('-sine') else (.10 if test=='yaw-pulse' else .15)
    local=elapsed-3.
    if test.endswith('-pulse'):
        for index,sign in enumerate((1,-1,1,-1)):
            t=local-index*2.3
            if 0<=t<.30:
                envelope=smoothstep(t/.05)*smoothstep((.30-t)/.05)
                result[axis]=sign*amplitude*envelope
                break
    elif test.endswith('-sine'):
        for index,frequency in enumerate(options.sine_frequencies):
            t=local-index*8.
            if 0<=t<6.:
                envelope=smoothstep(t)*smoothstep(6.-t)
                result[axis]=amplitude*envelope*math.sin(2.*math.pi*frequency*t)
                break
    return result


class StationaryReference:
    def __init__(self,position,duration):self.position=np.array(position,dtype=float);self.duration=duration
    def sample(self,elapsed):
        if not math.isfinite(elapsed):raise ValueError('Finite reference time required')
        return dict(position=self.position.tolist(),velocity=[0.]*3,acceleration=[0.]*3,phase='stationary')


class DiagnosticMission(MpcBenchmarkMission):
    trajectory_kind='control_diagnostic'

    def __init__(self,*args,diagnostic_options,**kwargs):
        if not isinstance(diagnostic_options,dict) or 'test' not in diagnostic_options:raise ValueError('Explicit diagnostic metadata required')
        self.diagnostic_options=DiagnosticOptions(diagnostic_options['test'],diagnostic_options.get('sine_profile','baseline'))
        if diagnostic_options!=self.diagnostic_options.metadata:raise ValueError('Diagnostic parameters differ from the reviewed fixed recipe')
        self.worker_required=self.diagnostic_options.test=='mpc-hover'
        super().__init__(*args,**kwargs)
        self.tracking_phase='diagnostic_mpc_hover' if self.worker_required else 'diagnostic_pd'
        self.trajectory_kind='control_diagnostic'
        self.diagnostic_stable_since=None
        self.excitation={}
        if (self.limits.soft_position_m,self.limits.hard_position_m,self.limits.soft_tilt_deg,self.limits.hard_tilt_deg)!=(.6,.75,15.,35.):
            raise ValueError('Diagnostics preserve the original PD position/tilt guards')

    def _make_curve(self,center):
        return StationaryReference(center,self.diagnostic_options.duration_s)

    def _path_start(self):return self.home.copy()

    def _flight_reference(self,now):
        if self.phase in ('takeoff','landing'):return FullIndiMission._flight_reference(self,now)
        return dict(position=self.home.tolist(),velocity=[0.]*3,acceleration=[0.]*3,phase='stationary_launch_home')

    def _select_airborne_command(self,now,state,reference,dt,thrust,rates,*,request):
        if self.worker_required:
            return super()._select_airborne_command(now,state,reference,dt,thrust,rates,request=request)
        base=np.asarray(rates,dtype=float)
        elapsed=now-self.phase_started
        injection=rate_injection(self.diagnostic_options.test,elapsed,self.diagnostic_options.sine_profile) if self.phase==self.tracking_phase and not self.pending_landing and not self.fallback else np.zeros(3)
        requested=base+injection
        result=FullIndiMission._select_airborne_command(self,now,state,reference,dt,thrust,requested,request=request)
        baseline=np.clip(base,-self.limits.maximum_rate_rad_s,self.limits.maximum_rate_rad_s)
        selected=np.asarray(result['rates_rad_s'])
        self.excitation=dict(base_ctbr=[float(thrust),*base.tolist()],requested_injection_rad_s=injection.tolist(),
            uncapped_sum_ctbr=[float(thrust),*requested.tolist()],baseline_capped_rates_rad_s=baseline.tolist(),
            applied_injection_rad_s=(selected-baseline).tolist(),selected_ctbr=[result['thrust_N'],*selected.tolist()],
            rate_clipped=(selected!=requested).tolist(),elapsed_s=elapsed if self.phase==self.tracking_phase else None)
        return result

    def tick(self,now,state,control,**kwargs):
        self.excitation=dict(base_ctbr=None,requested_injection_rad_s=[0.]*3,applied_injection_rad_s=[0.]*3,
                             uncapped_sum_ctbr=None,rate_clipped=[False]*3)
        if self.worker_required:self._prepare_mpc_tick(now,control)
        else:
            self.mpc_selection={};self._record_inner_status(control)
        output=FullIndiMission.tick(self,now,state,control,**kwargs)
        self.excitation['selected_ctbr']=[output['thrust_N'],*output['rates_rad_s']]
        if self.done or self.phase=='landing':return output
        if self.phase=='indi_hover':
            self._phase('diagnostic_entry',now);self.diagnostic_stable_since=None
        if self.phase in ('diagnostic_entry','diagnostic_settle','trajectory_hold'):
            sample=np.asarray(state,dtype=float)
            tilt=math.degrees(math.acos(float(np.clip(Rotation.from_quat(sample[[4,5,6,3]]).as_matrix()[2,2],-1.,1.))))
            stable=np.linalg.norm(sample[:3]-self.home)<=.15 and np.linalg.norm(sample[7:10])<=.25 and tilt<=6.
            self.diagnostic_stable_since=(now if self.diagnostic_stable_since is None else self.diagnostic_stable_since) if stable else None
        if self.phase=='diagnostic_entry':
            if self.diagnostic_stable_since is not None and now-self.diagnostic_stable_since>=1.:
                self._phase('mpc_entry' if self.worker_required else self.tracking_phase,now)
                self.mpc_entry_stable_since=None
            elif now-self.phase_started>=10.:self.request_landing('diagnostic_entry_not_settled')
        elif self.phase in ('mpc_entry','mpc_blend'):
            self._update_mpc_admission(now,state)
        elif self.phase==self.tracking_phase and now-self.phase_started>=self.diagnostic_options.duration_s:
            self._phase('trajectory_hold' if self.worker_required else 'diagnostic_settle',now)
            self.diagnostic_stable_since=None
        elif self.phase in ('diagnostic_settle','trajectory_hold'):
            settled=self.diagnostic_stable_since is not None and now-self.diagnostic_stable_since>=1.
            if now-self.phase_started>=2. and settled:self.request_landing('test_complete')
            elif now-self.phase_started>=10.:self.request_landing('diagnostic_return_not_settled')
        output['phase']=self.phase
        return output

    def diagnostics(self,output):
        result=super().diagnostics(output)
        result.pop('trajectory',None)
        result['mpc']['required']=self.worker_required
        result['mpc']['usage']='stationary hover only' if self.worker_required else 'unused for this PD diagnostic'
        result['diagnostic']=dict(self.diagnostic_options.metadata,phase=self.phase,worker_required=self.worker_required,
            active_outer='mpc' if self.mpc_weight==1. else 'pd_mpc_blend' if self.mpc_weight>0 else 'pd',excitation=copy.deepcopy(self.excitation))
        return result


def build_diagnostic_config(base_config,*,inner,test,ground_m,sine_profile='baseline'):
    from sanity_zenoh.indi_hover import make_mission
    from sanity_zenoh.mpc_benchmark_config import workspace_root
    if inner not in ('pid','indi'):raise ValueError('Explicit native inner pid or indi required')
    if isinstance(ground_m,bool) or not math.isfinite(ground_m):raise ValueError('Finite freshly measured ground coordinate required')
    options=DiagnosticOptions(test,sine_profile);config=copy.deepcopy(base_config)
    if config.get('flight_profile')!='indi_mpc_benchmark':raise ValueError('Explicit common benchmark base profile required')
    config['inner_controller']=inner;config['limits']['ground_m']=float(ground_m);config['limits']['maximum_run_s']=120.
    config['mpc']['config']=str(Path(__file__).resolve().parents[1]/'config'/('sanity15_mpc_indi_rate_model.json' if inner=='indi' else 'sanity09_mpc.json'))
    config['ground_indi'].update(thrust_weight=.4,start_climb_on_ack=inner=='indi',launch_boost_weight=1.2 if inner=='indi' else 0.,launch_boost_max_s=.5,launch_slew_up_N_s=3.)
    config.pop('trajectory',None);config.pop('benchmark_provenance',None)
    config['diagnostic']=options.metadata
    takeoff='PID ground warmup to0.4weight then guarded INDI ACK starts climb with bounded1.2weight launch floor, at most0.5s and3N/s upward slew; liftoff ends boost permanently' if inner=='indi' else 'retained PID ground warmup to0.4weight and legacy takeoff timing, launch boost disabled'
    config['commissioning']=f'Explicit {test} diagnostic with native {inner.upper()} inner; {takeoff}, stable measured PD hover, stationary launch-home reference and normal landing. Fresh disabled-bench floor required; no gains or FC changes.'
    config['takeoff_provenance']=dict(ground_thrust_weight=config['ground_indi']['thrust_weight'],start_climb_on_ack=config['ground_indi']['start_climb_on_ack'],
        launch_boost_weight=config['ground_indi']['launch_boost_weight'],launch_boost_max_s=config['ground_indi']['launch_boost_max_s'],launch_slew_up_N_s=config['ground_indi']['launch_slew_up_N_s'],scope='host-only bounded launch candidate; existing SZC4 and FC firmware; not flight qualified')
    make_mission(config,workspace_root()/'loco_mjx')
    return config
