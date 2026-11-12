"""Pure candidate mission.tick integration: no workers, ROS, network or hardware."""
import copy
from functools import lru_cache
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from sanity_zenoh.indi_hover import make_mission
from sanity_zenoh.indi_mpc_benchmark import MpcBenchmarkMission
from sanity_zenoh.emergency_landing import EmergencyLanding

OUT=Path(__file__).resolve().parents[1]
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'loco_mjx').is_dir())
_spec=importlib.util.spec_from_file_location('emergency_candidate_launcher',OUT/'run_high_speed_lemniscate.py')
_launcher=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_launcher)

@lru_cache(maxsize=None)
def config(speed):
    args=SimpleNamespace(test='trajectory',pattern='lemniscate',inner='pid',speed=float(speed),
                         schedule='indi_shaped_angular',radius_m=2.,width_m=3.,butterfly_height_m=.3,butterfly_cycles=3)
    if speed<3.:args.width_m=None;args.butterfly_height_m=0.
    return _launcher.make_config(args,0.)


def airborne(speed=3.,*,now=10.,admitted=True,stage='lemniscate',velocity=(1.,0.,0.)):
    m=make_mission(copy.deepcopy(config(speed)),ROOT/'loco_mjx')
    assert isinstance(m,MpcBenchmarkMission)
    assert Path(__import__('sanity_zenoh.indi_mpc_benchmark',fromlist=['x']).__file__).is_relative_to(OUT)
    m.home=np.array([.2,.1,m.limits.ground_m+m.limits.height_m])
    m.reference=m.reference_factory(m.limits.ground_m,m.home[2],.5,1.5,5.)
    m.first_tick=0.;m.last_tick=now-.02;m.armed_at=0.;m.ever_armed=True;m.pd.liftoff.complete=True
    m.segment=m._make_curve(np.r_[0.,0.,m.home[2]])
    m._phase(stage,now);m.benchmark_envelope_active=admitted;m.mpc_weight=1. if admitted else 0.
    m.mpc_blend_started=now-2.;m.last_thrust=.8
    s=np.zeros(13);s[3]=1.;s[:3]=m._flight_reference(now)['position'];s[7:10]=velocity
    c=dict(fc_uid_msp=m.limits.expected_uid,permission=True,supported=True,armed=True,mode='pid',indi_weight=0.,
           requested=False,eligible=True,fault_latched=False,fault='ok',reason='ok',rpm_mask=15)
    m.set_mpc_result(dict(ctbr=[.8,1.2,-.7,.15],mpc=dict(success=True,total_ms=1.)),dict(valid=True,reason='ok'))
    return m,s,c


def tick(m,now,s,c,**kw):
    return m.tick(now,s,c,mocap_age_s=kw.pop('mocap_age_s',.004),
        controller_age_s=kw.pop('controller_age_s',.02),receiver_ready=kw.pop('receiver_ready',True),**kw)

@pytest.mark.parametrize('speed',[3.,4.,5.])
@pytest.mark.parametrize('trigger',['operator','worker_invalid','native_fault'])
def test_actual_admitted_mission_enters_brake_and_bounds_outputs(speed,trigger):
    m,s,c=airborne(speed)
    selected=tick(m,10.,s,c)
    previous=np.r_[selected['thrust_N'],selected['rates_rad_s']]
    assert selected['enabled'] and max(abs(previous[1:]))>.6
    assert m.mpc_selection['selected_ctbr']==previous.tolist()
    if trigger=='operator':m.request_landing('operator_landing')
    elif trigger=='worker_invalid':m.set_mpc_result(None,dict(valid=False,reason='worker_error'))
    else:c.update(fault_latched=True,fault='rpm')
    out=tick(m,10.02,s,c)
    assert out['enabled'] and isinstance(m.landing,EmergencyLanding)
    # The real _prepare_mpc_tick clears diagnostic selection. The retained
    # ACTUAL output must nevertheless reach the nominal delayed-input seed.
    assert m.mpc_selection=={}
    np.testing.assert_array_equal(m.landing.previous_selected_ctbr,previous)
    np.testing.assert_array_equal(m.landing.initial_nominal_active_command[1:],previous[1:])
    assert max(abs(np.asarray(out['rates_rad_s'])))<=2.
    assert out['thrust_N']<=m.landing.maximum_thrust_N
    assert abs(out['thrust_N']-previous[0])<=4.*.02+1e-10
    assert out['indi_requested'] is False
    assert m.fallback==(trigger=='native_fault')
    for index in range(2,7):
        out=tick(m,10.+index*.02,s,c)
        assert out['enabled'] and max(abs(np.asarray(out['rates_rad_s'])))<=2.
        assert out['indi_requested'] is False

@pytest.mark.parametrize('speed',[.8,1.5])
@pytest.mark.parametrize('trigger',['operator','worker_invalid','native_fault'])
def test_ordinary_profiles_keep_existing_landing(speed,trigger):
    m,s,c=airborne(speed)
    if trigger=='operator':m.request_landing('operator_landing')
    elif trigger=='worker_invalid':m.set_mpc_result(None,dict(valid=False,reason='worker_error'))
    else:c.update(fault_latched=True,fault='rpm')
    out=tick(m,10.,s,c)
    assert out['enabled'] and m.phase=='landing'
    assert not isinstance(m.landing,EmergencyLanding)
    assert max(abs(np.asarray(out['rates_rad_s'])))<=.6

@pytest.mark.parametrize('speed',[3.,4.,5.])
def test_pre_admission_and_normal_settled_completion_do_not_activate_braking(speed):
    m,s,c=airborne(speed,admitted=False,stage='trajectory_entry')
    m.request_landing('operator_landing');out=tick(m,10.,s,c)
    assert out['enabled'] and not isinstance(m.landing,EmergencyLanding)
    m,s,c=airborne(speed,velocity=(.1,0.,0.))
    m.request_landing('test_complete');out=tick(m,10.,s,c)
    assert out['enabled'] and not isinstance(m.landing,EmergencyLanding)


def test_runtime_reserve_starts_landing_before_global_deadline():
    m,s,c=airborne(3.);reserve=m.limits.maximum_run_s-25.
    m.last_tick=reserve-.02;m.phase_started=reserve
    out=tick(m,reserve,s,c)
    assert out['enabled'] and m.phase=='landing' and m.reason=='maximum_runtime'
    assert isinstance(m.landing,EmergencyLanding)
    assert m.landing.start==reserve and m.landing.timeout==reserve+25.
    assert reserve<m.limits.maximum_run_s


def test_one_fixed_landing_deadline_survives_global_deadline_and_repeated_requests():
    m,s,c=airborne(3.,now=110.,velocity=(0.,0.,0.))
    m.request_landing('operator_landing');out=tick(m,110.,s,c)
    landing=m.landing;fixed=landing.timeout
    for index in range(1,18):
        out=tick(m,110.+index*.02,s,c)
        assert out['enabled']
    assert not landing.active_braking
    # Stay slightly above ground instead of satisfying touchdown, so this test
    # can exercise the fixed grace rather than normal earlier completion.
    for now in (m.limits.maximum_run_s+.02,fixed-.02):
        s[:3]=landing.sample(now)['position'];s[2]+=.10;s[7:10]=0.
        m.last_tick=now-.02
        m.request_landing('operator_repeat');c.update(fault_latched=True,fault='rpm')
        out=tick(m,now,s,c)
        assert out['enabled'] and m.landing is landing and landing.timeout==fixed
        assert not out['indi_requested']
    m.last_tick=fixed-.01
    out=tick(m,fixed+.01,s,c)
    assert not out['enabled'] and m.landing is landing and landing.timeout==fixed
    assert m.reason in ('emergency_landing_timeout','landing_timeout')

@pytest.mark.parametrize('fault',['tilt','speed','position','altitude','state_stale','controller_stale','receiver','identity','bad_quaternion'])
@pytest.mark.parametrize('already_braking',[False,True])
def test_original_hard_guards_still_disable_before_or_during_braking(fault,already_braking):
    m,s,c=airborne(4.)
    now=10.
    if already_braking:
        m.request_landing('operator_landing');assert tick(m,now,s,c)['enabled'];now+=.02
    kw={}
    if fault=='tilt':s[3:7]=Rotation.from_euler('x',76.,degrees=True).as_quat()[[3,0,1,2]]
    elif fault=='speed':s[7:10]=[5.01,0.,0.]
    elif fault=='position':
        s[:3]=m._flight_reference(now)['position'];s[0]+=1.51
    elif fault=='altitude':s[2]=m.limits.maximum_altitude_m+.01
    elif fault=='state_stale':kw['mocap_age_s']=.04
    elif fault=='controller_stale':kw['controller_age_s']=m.limits.controller_status_max_age_s
    elif fault=='receiver':kw['receiver_ready']=False
    elif fault=='identity':c['fc_uid_msp']='0'*24
    elif fault=='bad_quaternion':s[3:7]=0.
    m.request_landing('operator_landing')
    out=tick(m,now,s,c,**kw)
    assert not out['enabled'],(fault,already_braking,m.diagnostics(out))
    expected='hard_flight_envelope' if fault in ('tilt','speed','position','altitude') else 'fc_identity_mismatch' if fault=='identity' else 'state_or_receiver_unavailable'
    assert m.reason==expected


def test_failed_braking_settle_is_reported_through_real_mission():
    m,s,c=airborne(3.);m.request_landing('operator_landing');assert tick(m,10.,s,c)['enabled']
    m.last_tick=17.98;out=tick(m,18.,s,c)
    assert not out['enabled'] and m.reason=='emergency_braking_timeout'
    assert m.landing.failure_reason=='emergency_braking_timeout'


def test_settling_handoff_cannot_mask_existing_hard_guard_breach():
    m,s,c=airborne(3.,velocity=(0.,0.,0.));m.request_landing('operator_landing')
    assert tick(m,10.,s,c)['enabled']
    for index in range(1,16):assert tick(m,10.+index*.02,s,c)['enabled']
    assert m.landing.landing is None
    s[0]+=1.51
    out=tick(m,10.32,s,c)
    assert not out['enabled'] and m.reason=='emergency_braking_guard_error'
    assert m.landing.landing is None and m.landing.stopping_anchor is None


@pytest.mark.parametrize('trigger',['operator','worker_invalid','native_fault'])
def test_pre_admission_trigger_keeps_original_admission_and_landing_policy(trigger):
    m,s,c=airborne(4.,admitted=False,stage='mpc_entry')
    if trigger=='operator':m.request_landing('operator_landing')
    elif trigger=='worker_invalid':m.set_mpc_result(None,dict(valid=False,reason='worker_error'))
    else:c.update(fault_latched=True,fault='rpm')
    out=tick(m,10.,s,c)
    assert out['enabled'] and not isinstance(m.landing,EmergencyLanding)
    if trigger=='worker_invalid':assert m.phase=='mpc_entry'
    else:assert m.phase=='landing'
    assert out['indi_requested'] is False


def test_real_preview_and_fresh_late_tick_do_not_reanchor_or_reset_brake():
    m,s,c=airborne(4.);m.request_landing('operator_landing');assert tick(m,10.,s,c)['enabled']
    landing=m.landing;before=copy.deepcopy(landing.diagnostics())
    first=m.reference_preview(10.,[0.,.02,1.],s)
    for _ in range(5):assert m.reference_preview(10.,[0.,.02,1.],s)==first
    assert before==landing.diagnostics()
    out=tick(m,10.1,s,c)
    assert out['enabled'] and m.landing is landing
    assert landing.start==10. and landing.timeout==35. and landing.last_nominal_cells==10
    assert max(abs(np.asarray(out['rates_rad_s'])))<=2.
