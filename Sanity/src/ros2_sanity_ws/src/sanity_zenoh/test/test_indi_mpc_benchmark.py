from sanity_zenoh.standalone_paths import load_json
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from sanity_zenoh.indi_hover import make_mission
from sanity_zenoh.indi_mpc_benchmark import MpcBenchmarkMission, MpcWorkerRuntime

ROOT=next(path for path in Path(__file__).resolve().parents if (path/'loco_mjx').is_dir())
PACKAGE=Path(__file__).resolve().parents[1]


def configuration(schedule='constant_arclength'):
    return load_json(PACKAGE / 'config' / f'sanity15_indi_mpc_{schedule}.json')


def airborne(stage='lemniscate',valid=True):
    mission=make_mission(configuration(),ROOT/'loco_mjx')
    mission.home=np.array([.8,.1,mission.limits.ground_m+1.])
    mission.reference=mission.reference_factory(mission.limits.ground_m,mission.home[2],.5,1.5,5.)
    mission.first_tick=0.;mission.last_tick=9.98;mission.armed_at=1.;mission.ever_armed=True;mission.pd.liftoff.complete=True
    mission.segment=mission._make_curve(np.r_[0.,0.,mission.home[2]])
    mission._phase(stage,10.);mission.benchmark_envelope_active=True;mission.mpc_weight=1. if stage=='lemniscate' else 0.
    mission.mpc_blend_started=10.
    state=np.zeros(13);state[3]=1.;state[:3]=mission._flight_reference(10.)['position']
    control=dict(fc_uid_msp=mission.limits.expected_uid,permission=True,supported=True,armed=True,
                 mode='indi',indi_weight=1.,requested=True,eligible=True,fault_latched=False,fault='ok',reason='ok',rpm_mask=15)
    mission.set_mpc_result(dict(ctbr=[.75,1.2,-1.1,.1],mpc=dict(success=True,total_ms=1.)),dict(valid=valid,reason='ok' if valid else 'no_result'))
    return mission,state,control


def test_factory_and_preview_do_not_start_worker_or_mutate_phase(monkeypatch):
    import subprocess
    monkeypatch.setattr(subprocess,'Popen',lambda *a,**k:pytest.fail('Factory must not start a worker'))
    mission,state,_=airborne()
    before=(mission.phase,mission.phase_started,mission.mpc_weight,id(mission.segment))
    first=mission.reference_preview(10.,[0.,.1,1.],state)
    for _ in range(3):assert mission.reference_preview(10.,[0.,.1,1.],state)==first
    assert before==(mission.phase,mission.phase_started,mission.mpc_weight,id(mission.segment))


def test_constant_reference_is_exact_frozen_historical_recipe_at_translated_origin():
    source=ROOT/'tests/fixtures/historical_missions.py'
    spec=importlib.util.spec_from_file_location('historical_comparison_patterns',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    frozen=module.PatternMission('lemniscate',1.5,radius=3.2,laps=3,command_speed_limit=1.5,ramp_s=2.,hold_s=2.)
    mission=make_mission(configuration(),ROOT/'loco_mjx');curve=mission._make_curve([0.,0.,1.])
    assert curve.duration==frozen.duration==pytest.approx(45.0222302087)
    for time in np.linspace(-.1,curve.duration+.1,1001):
        old,new=frozen.reference(float(time)),curve.sample(float(time))
        assert new['position'][:2]==old['position_xy']
        assert new['velocity'][:2]==old['velocity_xy']
        assert new['acceleration'][:2]==old['acceleration_xy']


def test_mpc_selected_before_pd_caps_with_common_physical_clipping_and_no_extra_slew():
    mission,state,control=airborne();mission.last_thrust=.2
    mission.mpc_candidate['ctbr']=[.8,1.2,-1.1,2.5]
    output=mission.tick(10.,state,control)
    assert output['thrust_N']==.8 and output['rates_rad_s']==[1.2,-1.1,2.3]
    assert mission.mpc_selection['command_clipped']
    assert mission.pd.last['selected_ctbr'] if 'selected_ctbr' in mission.pd.last else True
    assert mission.pd.last['command_anti_windup']


def test_invalid_result_on_first_blend_tick_lands_without_none_candidate_crash():
    mission,state,control=airborne('mpc_blend',valid=False);mission.mpc_candidate=None
    output=mission.tick(10.02,state,control)
    assert output['phase']=='landing' and mission.reason=='mpc_no_result'
    assert mission.mpc_weight==0. and max(abs(value) for value in output['rates_rad_s'])<=.6


def test_benchmark_hard_envelope_stays_latched_through_fault_landing():
    mission,state,control=airborne();state[3:7]=Rotation.from_euler('x',40.,degrees=True).as_quat()[[3,0,1,2]]
    control.update(fault_latched=True,fault='rpm')
    output=mission.tick(10.,state,control)
    assert output['phase']=='landing' and mission.reason=='fc_indi_fault_rpm'
    assert mission._airborne_envelope()['hard_tilt_deg']==60.
    assert max(abs(value) for value in output['rates_rad_s'])<=.6


def test_takeoff_keeps_original_envelope_and_hard_checks_precede_operator_landing():
    mission,state,control=airborne('takeoff');mission.benchmark_envelope_active=False
    state[:3]=mission.home;state[3:7]=Rotation.from_euler('x',40.,degrees=True).as_quat()[[3,0,1,2]]
    assert not mission.tick(10.,state,control)['enabled']
    mission,state,control=airborne();state[0]+=1.51;mission.request_landing('operator_landing')
    assert not mission.tick(10.,state,control)['enabled']


@pytest.mark.parametrize('stage',['mpc_blend','lemniscate','return_home','trajectory_hold'])
@pytest.mark.parametrize('fault',['worker_error','source_age','invalid_solver','compute_deadline','nonfinite_result'])
def test_active_bad_result_requests_same_tick_bounded_pd_landing(stage,fault):
    mission,state,control=airborne(stage);mission.mpc_weight=.5
    mission.set_mpc_result(None,dict(valid=False,reason=fault))
    output=mission.tick(10.,state,control)
    assert output['phase']=='landing' and mission.reason=='mpc_'+fault
    assert output['indi_requested'] and max(abs(x) for x in output['rates_rad_s'])<=.6


class FakeWorker:
    error=None
    def __init__(self,result):self.result=result;self.commits=[]
    def latest(self):return self.result
    def commit(self,record):self.commits.append(record)


def fake_runtime():
    runtime=MpcWorkerRuntime.__new__(MpcWorkerRuntime)
    result=dict(tick=10,state_stamp_ns=9_990_000_000,valid=True,ctbr=[.7,.1,.2,0.],mpc=dict(success=True,total_ms=1.))
    runtime.worker=FakeWorker(result);runtime.runtime_error=None;runtime.pending_tick=10;runtime.next_tick=11
    runtime.info=dict(ctrlrange=[[0.,3.2],[-2.3,2.3],[-2.3,2.3],[-2.6,2.6]])
    runtime.event=lambda record:None;runtime.last_health={}
    return runtime


@pytest.mark.parametrize('mutation,reason',[
    (dict(tick=8),'tick_age'),(dict(state_stamp_ns=9_950_000_000),'source_age'),
    (dict(valid=False),'invalid_solver'),(dict(ctbr=[.7,float('nan'),0.,0.]),'nonfinite_result'),
    (dict(ctbr=[.7,3.,0.,0.]),'solver_bounds'),(dict(mpc=dict(success=True,total_ms=25.1)),'compute_deadline')])
def test_real_admission_function_rejects_bad_stale_and_late_solver_result(mutation,reason):
    mission,_,_=airborne();runtime=fake_runtime();runtime.worker.result.update(mutation)
    assert runtime.refresh(mission,10.)['reason']==reason and mission.mpc_candidate is None


def test_actual_selected_command_committed_once_and_duplicate_is_noop():
    runtime=fake_runtime();output=dict(thrust_N=.72,rates_rad_s=[.3,-.2,0.])
    runtime.commit(output);runtime.commit(output)
    assert runtime.worker.commits==[dict(tick=10,selected_ctbr=[.72,.3,-.2,0.])]


@pytest.mark.parametrize('failure',['missing_info','mass','event'])
def test_runtime_constructor_closes_started_worker_on_every_later_failure(tmp_path,monkeypatch,failure):
    from sanity_zenoh import async_policy
    closed=[]
    class Fake:
        def __init__(self,*a,**k):
            self.info={} if failure=='missing_info' else dict(preview_times_s=[0.,.1],mass_kg=.2 if failure=='mass' else configuration()['limits']['mass_kg'],policy_dt_s=.02)
            self.actor=SimpleNamespace(process=SimpleNamespace(pid=123))
        def close(self):closed.append(True)
    monkeypatch.setattr(async_policy,'AsyncPolicy',Fake)
    mission=make_mission(configuration(),ROOT/'loco_mjx')
    event=(lambda record:(_ for _ in ()).throw(RuntimeError('event failure'))) if failure=='event' else None
    with pytest.raises((KeyError,ValueError,RuntimeError)):MpcWorkerRuntime(mission,tmp_path/'worker',event)
    assert closed==[True]


def test_async_constructor_closes_actor_if_post_start_initialization_fails(tmp_path,monkeypatch):
    from sanity_zenoh import async_policy
    closed=[]
    class FakeActor:
        def __init__(self,*a,**k):self.info={}
        def close(self):closed.append(True)
    monkeypatch.setattr(async_policy,'ActorProcess',FakeActor)
    monkeypatch.setattr(async_policy.threading.Thread,'start',lambda self:(_ for _ in ()).throw(RuntimeError('thread failure')))
    with pytest.raises(RuntimeError):async_policy.AsyncPolicy('python','source','config',tmp_path)
    assert closed==[True]


@pytest.mark.parametrize('mutation',[{'state_stamp_ns':None},{'mpc':None},{'mpc':[]},
    {'mpc':{'success':True,'total_ms':None}},{'mpc':{'success':True,'total_ms':'1'}},
    {'ctbr':['bad',0,0,0]},{'tick':'10'}])
def test_malformed_worker_results_land_instead_of_raising(mutation):
    mission,state,control=airborne();runtime=fake_runtime();runtime.worker.result.update(mutation)
    assert runtime.refresh(mission,10.)['reason']=='malformed_result'
    assert mission.tick(10.,state,control)['phase']=='landing'
    assert mission.reason=='mpc_malformed_result'


def test_missing_timestamp_and_nonmapping_worker_results_are_invalid():
    mission,_,_=airborne();runtime=fake_runtime();runtime.worker.result.pop('state_stamp_ns')
    assert runtime.refresh(mission,10.)['reason']=='malformed_result'
    runtime.worker.result=['bad']
    assert runtime.refresh(mission,10.)['reason']=='malformed_result'


@pytest.mark.parametrize('tick',[float('nan'),float('inf'),[],{}])
def test_malformed_result_diagnostics_remain_strict_json(tick):
    mission,_,_=airborne();runtime=fake_runtime();runtime.worker.result['tick']=tick
    health=runtime.refresh(mission,10.)
    assert health['reason']=='malformed_result' and health['result_tick'] is None
    json.dumps(health,allow_nan=False)


@pytest.mark.parametrize('key',['mpc','benchmark_envelope'])
def test_benchmark_settings_cannot_silently_apply_to_other_profiles(key):
    config=configuration();config['flight_profile']='indi_full_flight';config['estimator_logging']['flight_profile']='indi_full_flight'
    with pytest.raises(ValueError,match='explicit benchmark'):make_mission(config,ROOT/'loco_mjx')


@pytest.mark.parametrize('mutation',[{'laps':1},{'radius_m':2.1},{'peak_speed_m_s':2.},{'unknown':1}])
def test_benchmark_geometry_is_explicit_and_reviewed(mutation):
    config=configuration();config['trajectory'].update(mutation)
    with pytest.raises((ValueError,TypeError)):make_mission(config,ROOT/'loco_mjx')


@pytest.mark.parametrize('inner',['pid','indi'])
def test_small_shaped_lemniscate_preserves_speed_laps_and_reference_limits(inner):
    from sanity_zenoh.mpc_benchmark_config import build_config
    base=configuration()
    config=build_config(base,pattern='lemniscate',inner=inner,speed=1.5,
                        schedule='indi_shaped_angular',radius_m=2.,ground_m=.037)
    mission=make_mission(config,ROOT/'loco_mjx')
    curve=mission._make_curve([0.,0.,1.037])
    samples=[curve.sample(float(t)) for t in np.linspace(0,curve.duration,10001)]
    positions=np.asarray([r['position'] for r in samples])
    np.testing.assert_allclose(np.ptp(positions[:,:2],axis=0),[4.,2.],atol=1e-5)
    assert max(np.linalg.norm(r['velocity']) for r in samples)==pytest.approx(1.5,abs=1e-6)
    assert samples[-1]['lap_progress']==pytest.approx(3.)
    assert max(np.linalg.norm(r['acceleration']) for r in samples)<=1.5
    assert curve.design_metadata['conservative_jerk_bound_m_s3']<=2.+1e-12
    assert curve.design_metadata['conservative_acceleration_bound_m_s2']<=1.5+1e-12
    assert config['benchmark_provenance']['footprint_xy_m']==[4.,2.]
    assert config['limits']['mass_kg']==.07151
    assert base['trajectory']['radius_m']==3.2
    assert build_config(base,pattern='lemniscate',inner=inner,speed=1.5,
                        ground_m=.037)['trajectory']['radius_m']==3.2


@pytest.mark.parametrize('radius',[0.,-2.,float('nan'),float('inf'),True,1.5,2.1])
def test_lemniscate_size_rejects_unsupported_geometry(radius):
    from sanity_zenoh.mpc_benchmark_config import build_config
    with pytest.raises(ValueError):
        build_config(configuration(),pattern='lemniscate',inner='indi',speed=1.5,
                     schedule='indi_shaped_angular',radius_m=radius,ground_m=.037)


def pid_airborne(stage='lemniscate'):
    config=load_json(PACKAGE / 'config/sanity15_indi_mpc_constant_arclength_pid.json')
    mission=make_mission(config,ROOT/'loco_mjx');template,state,control=airborne(stage)
    for key in ('home','reference','first_tick','last_tick','armed_at','ever_armed','segment','phase','phase_started','benchmark_envelope_active','mpc_weight','mpc_blend_started'):
        setattr(mission,key,getattr(template,key))
    mission.pd.liftoff.complete=True;control.update(mode='pid',indi_weight=0.,requested=False)
    mission.set_mpc_result(template.mpc_candidate,template.mpc_health)
    return mission,state,control


def test_pid_constructor_selection_is_read_only_and_not_normal_pid_permission_profile():
    from dataclasses import replace
    from sanity_zenoh.indi_hover import controller_permission_matches
    mission,_,_=pid_airborne()
    assert mission.inner_controller=='pid' and mission.limits.pid_only is False
    assert controller_permission_matches(mission.limits,{'permission':True})
    assert not controller_permission_matches(mission.limits,{'permission':False})
    normal_pid=replace(mission.limits,pid_only=True)
    assert controller_permission_matches(normal_pid,{'permission':False})
    assert not controller_permission_matches(normal_pid,{'permission':True})
    with pytest.raises(AttributeError):mission.inner_controller='indi'
    config=configuration();config['inner_controller']='blend'
    with pytest.raises(ValueError):make_mission(config,ROOT/'loco_mjx')


def test_pid_native_ground_warmup_never_requests_or_fakes_indi_ack_and_does_not_require_indi_eligibility():
    config=load_json(PACKAGE / 'config/sanity15_indi_mpc_constant_arclength_pid.json');mission=make_mission(config,ROOT/'loco_mjx')
    state=np.zeros(13);state[2]=mission.limits.ground_m;state[3]=1.
    control=dict(fc_uid_msp=mission.limits.expected_uid,permission=True,supported=True,armed=False,mode='pid',indi_weight=0.,requested=False,request_aux=3,eligible=False,rpm_mask=15,fault_latched=False,fault='ok',reason='ok',control_period_us=1000.)
    phases=[]
    for tick in range(300):
        output=mission.tick(tick*.02,state,control);phases.append(mission.phase)
        assert not output['indi_requested'] and mission.indi_confirmed_at is None
        if output['enabled']:control['armed']=True
        if mission.phase=='takeoff':break
    assert phases[-1]=='takeoff' and 'ground_spinup' in phases and 'ground_indi_request' not in phases
    assert mission.pid_confirmed_at==mission.native_inner_confirmed_at==mission.armed_at
    assert mission.armed_at>=mission.ground_started+mission.ground.spool_s+mission.ground.warmup_s+mission.ground.eligible_dwell_s
    diag=mission.diagnostics(output)
    assert diag['inner_controller']=='pid' and diag['native_inner_status']['mode']=='pid' and diag['indi_confirmed_at'] is None


@pytest.mark.parametrize('mutation',[dict(mode='indi'),dict(mode='to_indi'),dict(mode='to_pid'),dict(indi_weight=.01),dict(requested=True),dict(requested=None)])
def test_pid_ack_cannot_be_satisfied_by_indi_or_unrequested_status(mutation):
    mission,_,control=pid_airborne();control.update(mutation)
    assert not mission._ground_acknowledged(control)


@pytest.mark.parametrize('stage',['takeoff','trajectory_entry','transfer_to_center','mpc_entry','mpc_blend','lemniscate','return_home','trajectory_hold'])
def test_native_pid_inner_remains_pid_requested_false_across_airborne_phases(stage):
    mission,state,control=pid_airborne(stage)
    if stage=='takeoff':state[:3]=mission.home
    output=mission.tick(10.,state,control)
    assert output['enabled'] and not output['indi_requested']
    assert mission.reason is None


@pytest.mark.parametrize('mutation',[dict(mode='indi'),dict(mode='to_indi'),dict(indi_weight=.1),dict(requested=True),dict(permission=False)])
def test_native_pid_authority_loss_lands_without_requesting_indi(mutation):
    mission,state,control=pid_airborne();control.update(mutation)
    output=mission.tick(10.,state,control)
    assert output['phase']=='landing' and mission.reason=='pid_authority_lost'
    assert not output['indi_requested'] and max(abs(x) for x in output['rates_rad_s'])<=.6


@pytest.mark.parametrize('mutation,expected',[({'permission':False},'ground_pid_unavailable'),({'fault_latched':True},'ground_pid_unavailable'),({'mode':'to_indi'},'unexpected_ground_authority'),({'requested':True},'unexpected_ground_authority'),({'rpm_mask':0},None)])
def test_native_pid_ground_faults_and_missing_rpm_never_enter_takeoff(mutation,expected):
    mission,state,control=pid_airborne('ground_spinup');mission.ground_started=7.;state[:3]=[*mission.home[:2],mission.limits.ground_m]
    control.update(mutation);output=mission.tick(10.,state,control)
    assert mission.phase!='takeoff' and not output['indi_requested']
    assert mission.reason==expected and mission.pid_confirmed_at is None


def test_matched_inner_selection_keeps_outer_command_and_bounds_identical_for_identical_input():
    indi,state,indi_status=airborne();pid,_,pid_status=pid_airborne()
    left=indi.tick(10.,state,indi_status);right=pid.tick(10.,state,pid_status)
    assert left['thrust_N']==right['thrust_N'] and left['rates_rad_s']==right['rates_rad_s']
    assert left['indi_requested'] and not right['indi_requested']


@pytest.mark.parametrize('pattern,schedule',[('lemniscate','constant_arclength'),('lemniscate','variable_angular'),('circle','constant_arclength')])
@pytest.mark.parametrize('speed',[.8,1.,1.5])
@pytest.mark.parametrize('inner',['pid','indi'])
def test_generic_builder_reviewed_grid_is_finite_bounded_and_has_derived_runtime(pattern,schedule,speed,inner):
    from sanity_zenoh.mpc_benchmark_config import build_config
    config=build_config(configuration(),pattern=pattern,inner=inner,speed=speed,schedule=schedule,ground_m=.037)
    mission=make_mission(config,ROOT/'loco_mjx');curve=mission._make_curve([0.,0.,1.037])
    samples=[curve.sample(float(t)) for t in np.linspace(0,curve.duration,601)]
    assert np.isfinite([[*r['position'],*r['velocity'],*r['acceleration']] for r in samples]).all()
    assert max(np.linalg.norm(r['velocity']) for r in samples)<=speed+1e-12
    assert config['limits']['maximum_run_s']==max(120,np.ceil(curve.duration+60))<=180
    assert mission.inner_controller==inner and mission.tracking_phase==pattern
    assert config['mpc']['model']==configuration()['mpc']['model']
    assert config['benchmark_envelope']==configuration()['benchmark_envelope']
    predictor=load_json(Path(config['mpc']['config']))
    assert predictor.get('rate_response',{}).get('enabled',False) is (inner=='indi')


def circle_airborne(stage='circle',inner='indi'):
    from sanity_zenoh.mpc_benchmark_config import build_config
    config=build_config(configuration(),pattern='circle',inner=inner,speed=1.5,ground_m=.037)
    mission=make_mission(config,ROOT/'loco_mjx');template,state,control=airborne(stage)
    for key in ('home','reference','first_tick','last_tick','armed_at','ever_armed','phase','phase_started','benchmark_envelope_active','mpc_weight','mpc_blend_started'):
        setattr(mission,key,getattr(template,key))
    mission.segment=mission._make_curve([0.,0.,mission.home[2]])
    mission.pd.liftoff.complete=True;state[:3]=mission._flight_reference(10.)['position']
    if inner=='pid':control.update(mode='pid',indi_weight=0.,requested=False)
    mission.set_mpc_result(template.mpc_candidate,template.mpc_health)
    return mission,state,control


def test_circle_world_center_and_transfer_return_start_are_consistent_and_c2_at_segment_joins():
    from sanity_zenoh.indi_lemniscate import QuinticTransfer
    mission,state,_=circle_airborne();start=mission._path_start()
    assert start.tolist()==[0.,-1.5,mission.home[2]]
    transfer=QuinticTransfer(mission.home,start);returned=QuinticTransfer(start,mission.home)
    assert transfer.sample(transfer.duration)['position']==mission.segment.sample(0.)['position']==start.tolist()
    np.testing.assert_allclose(returned.sample(0.)['position'],mission.segment.sample(mission.segment.duration)['position'],atol=2e-14,rtol=0.)
    for reference in [transfer.sample(transfer.duration),returned.sample(0.),mission.segment.sample(0.),mission.segment.sample(mission.segment.duration)]:
        assert reference['velocity']==[0.]*3 and reference['acceleration']==[0.]*3
    dense=np.array([mission.segment.sample(t)['position'] for t in np.linspace(0,mission.segment.duration,1001)])
    np.testing.assert_allclose(np.mean([dense.min(axis=0),dense.max(axis=0)],axis=0),[0.,0.,mission.home[2]],atol=2e-5)
    assert mission._flight_reference(10.)['position']==start.tolist()


def test_circle_mpc_admission_holds_at_path_start_and_reference_samples_are_pure():
    mission,state,control=circle_airborne('mpc_entry');mission.benchmark_envelope_active=False
    assert mission._flight_reference(10.)['position']==mission._path_start().tolist()
    before=(mission.phase,mission.phase_started)
    for _ in range(5):mission.reference_preview(10.,[0.,.1,1.],state)
    assert before==(mission.phase,mission.phase_started)
    for tick in range(53):mission.tick(10.+tick*.02,state,control)
    assert mission.phase=='mpc_blend'
    assert mission.benchmark_envelope_active


@pytest.mark.parametrize('inner',['pid','indi'])
def test_circle_tracking_and_fault_landing_use_real_circle_phase(inner):
    mission,state,control=circle_airborne(inner=inner)
    assert mission.tick(10.,state,control)['phase']=='circle'
    mission.set_mpc_result(None,dict(valid=False,reason='source_age'))
    output=mission.tick(10.02,state,control)
    assert output['phase']=='landing' and mission.reason=='mpc_source_age'
    assert output['indi_requested'] is (inner=='indi')


def test_benchmark_runtime_extension_does_not_relax_normal_hover_limits():
    from sanity_zenoh.indi_hover_mission import HoverLimits
    from sanity_zenoh.mpc_benchmark_config import build_config
    values=configuration()['limits'];values['maximum_run_s']=169.
    with pytest.raises(ValueError):HoverLimits(**values)
    config=build_config(configuration(),pattern='lemniscate',inner='indi',speed=.8,schedule='variable_angular',ground_m=.037)
    assert config['limits']['maximum_run_s']==169.
    config['limits']['maximum_run_s']=120.
    with pytest.raises(ValueError):make_mission(config,ROOT/'loco_mjx')
    with pytest.raises(ValueError):build_config(configuration(),pattern='circle',inner='pid',speed=1.5,schedule='variable_angular',ground_m=.037)


@pytest.mark.parametrize('ground',[0.,-.05])
def test_benchmark_world_ground_can_be_zero_or_negative_without_fake_epsilon(ground):
    from sanity_zenoh.mpc_benchmark_config import build_config
    from sanity_zenoh.indi_hover_mission import HoverLimits
    config=build_config(configuration(),pattern='circle',inner='pid',speed=1.,ground_m=ground)
    assert config['limits']['ground_m']==ground
    mission=make_mission(config,ROOT/'loco_mjx')
    assert mission.limits.ground_m==ground and mission.limits.height_m==1.
    with pytest.raises(ValueError):HoverLimits(**config['limits'])


def test_measured_sanity15_mass_matches_pd_and_mpc_model():
    import xml.etree.ElementTree as ET
    for path in (PACKAGE/'config').glob('sanity15_indi_mpc_*.json'):
        config=load_json(path)
        mission=make_mission(config,ROOT/'loco_mjx')
        model=ET.parse(config['mpc']['model']).getroot()
        mass=sum(float(x.attrib['mass']) for x in model.findall('.//body/inertial'))
        assert mass==pytest.approx(.07151)
        assert mission.limits.mass_kg==mass
        assert mission.pd.mass==mass


def test_indi_prediction_moves_reference_to_same_command_epoch_and_commits_selected_output():
    runtime=fake_runtime();mission,state,_=airborne()
    runtime.pending_tick=None;runtime.next_tick=10
    runtime.rate_prediction_enabled=True;runtime.prediction_lead_s=.035
    observations=[];runtime.worker.observe=observations.append
    preview_epochs=[]
    mission.reference_preview=lambda now,offsets,state:preview_epochs.append(now) or [[[0.,0.,1.],[0.,0.,0.],[0.,0.,0.]]]*len(offsets)
    runtime.preview_times=[0.,.05]
    runtime.prepare(mission,10.,state,.005)
    assert preview_epochs==[10.035]
    assert observations[0]['state_stamp_ns']==9_995_000_000
    assert observations[0]['prediction_context']['target_stamp_ns']==10_035_000_000
    runtime.commit(dict(thrust_N=.72,rates_rad_s=[.2,-.1,0]))
    selected=runtime.worker.commits[-1]
    assert selected['selected_ctbr']==[.72,.2,-.1,0.]
    assert type(selected['controller_context']['selected_stamp_ns']) is int


def test_disabled_bench_never_starts_an_unused_mpc_worker(monkeypatch,tmp_path):
    import sanity_zenoh.indi_mpc_benchmark as benchmark
    from sanity_zenoh.indi_hover import start_mpc_runtime
    mission=make_mission(configuration(),ROOT/'loco_mjx')
    monkeypatch.setattr(benchmark,'MpcWorkerRuntime',lambda *_:pytest.fail('Disabled-only bench must not start a solver'))
    assert start_mpc_runtime(mission,tmp_path/'mpc',disabled_only=True) is None
