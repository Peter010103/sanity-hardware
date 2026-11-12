"""Exercise production mission.tick through airborne, braking and fault guards."""
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from sanity_zenoh.indi_hover import make_mission
from sanity_zenoh.mpc_benchmark_config import build_config

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'loco_mjx').is_dir())
PACKAGE=ROOT/'releases/high-speed/host/sanity_zenoh'


def airborne(speed=3.,*,admitted=True):
    base=json.loads((PACKAGE/'config/sanity15_indi_mpc_constant_arclength.json').read_text())
    config=build_config(base,pattern='lemniscate',inner='indi',speed=speed,schedule='indi_shaped_angular',radius_m=2.,ground_m=.03)
    mission=make_mission(config,ROOT/'loco_mjx');mission.home=np.array([0.,0.,1.03])
    mission.reference=mission.reference_factory(.03,1.03,.5,1.5,5.)
    mission.first_tick=0.;mission.last_tick=9.98;mission.armed_at=1.;mission.ever_armed=True;mission.pd.liftoff.complete=True
    mission.segment=mission._make_curve(mission.home);mission._phase('lemniscate' if admitted else 'takeoff',10.)
    mission.benchmark_envelope_active=admitted;mission.mpc_weight=float(admitted);mission.mpc_blend_started=8.
    mission.set_mpc_result(dict(ctbr=[.75,.1,-.1,0.],mpc=dict(success=True,total_ms=1.)),dict(valid=True,reason='ok'))
    state=np.r_[mission._flight_reference(10.)['position'],1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
    control=dict(fc_uid_msp=mission.limits.expected_uid,permission=True,supported=True,armed=True,mode='indi',indi_weight=1.,requested=True,eligible=True,fault_latched=False,fault='ok',reason='ok',rpm_mask=15)
    return mission,state,control


def tilt(state,degrees):state[3:7]=Rotation.from_euler('x',degrees,degrees=True).as_quat()[[3,0,1,2]]



@pytest.mark.parametrize('speed,cap,soft,hard',[(4.,5.,60.,75.),(5.,6.5,72.,82.),(6.,7.5,78.,85.)])
def test_exact_reviewed_profiles_preserve_target_and_allow_expected_state(speed,cap,soft,hard):
    mission,state,control=airborne(speed);state[7]=speed+.1;tilt(state,soft-1.)
    output=mission.tick(10.,state,control)
    assert output['enabled'] and output['phase']=='lemniscate'
    e=mission._airborne_envelope()
    assert e['maximum_speed_m_s']==cap and e['soft_tilt_deg']==soft and e['hard_tilt_deg']==hard

@pytest.mark.parametrize('speed,cap',[(4.,5.),(5.,6.5),(6.,7.5)])
@pytest.mark.parametrize('reason',['operator_landing','mpc_invalid_solver','rpm'])
def test_caps_latch_through_actual_fault_braking(speed,cap,reason):
    mission,state,control=airborne(speed);state[7]=speed
    if reason=='rpm':control.update(fault_latched=True,fault='rpm')
    else:mission.request_landing(reason)
    output=mission.tick(10.,state,control)
    assert output['enabled'] and output['phase']=='landing'
    assert mission._airborne_envelope()['maximum_speed_m_s']==cap
    assert mission.tick(10.02,state,control)['enabled']
    assert max(abs(v) for v in output['rates_rad_s'])<=.6

@pytest.mark.parametrize('speed,cap,soft,hard',[(4.,5.,60.,75.),(5.,6.5,72.,82.),(6.,7.5,78.,85.)])
@pytest.mark.parametrize('kind',['speed','tilt','position','altitude'])
def test_real_hard_violation_still_disables(speed,cap,soft,hard,kind):
    mission,state,control=airborne(speed)
    if kind=='speed':state[7]=cap+.001
    elif kind=='tilt':tilt(state,hard+.01)
    elif kind=='position':state[0]+=1.501
    else:state[2]=mission.limits.maximum_altitude_m+.001
    output=mission.tick(10.,state,control)
    assert not output['enabled'] and output['reason']=='hard_flight_envelope'
    assert kind in mission.last_envelope_violation['criteria']

@pytest.mark.parametrize('speed',[4.,5.,6.])
@pytest.mark.parametrize('fault',['mocap','controller_age','identity','native_guard'])
def test_unrelated_hard_faults_still_disable(speed,fault):
    mission,state,control=airborne(speed);options={}
    if fault=='mocap':options['mocap_age_s']=.041
    elif fault=='controller_age':options['controller_age_s']=.301
    elif fault=='identity':control['fc_uid_msp']='00'*12
    else:control['takeoff_guard_triggered']=True
    output=mission.tick(10.,state,control,**options)
    assert not output['enabled'] and output['done']

@pytest.mark.parametrize('speed',[4.,5.,6.])
def test_pre_admission_stays_two_mps(speed):
    mission,state,control=airborne(speed,admitted=False);state[7]=2.001
    assert not mission.tick(10.,state,control)['enabled']
    assert mission.last_envelope_violation['limits']['maximum_speed_m_s']==2.

@pytest.mark.parametrize('speed,cap',[(4.,5.),(5.,6.5),(6.,7.5)])
def test_soft_guard_still_requests_landing_and_preserves_speed_cap(speed,cap):
    mission,state,control=airborne(speed);tilt(state,mission.benchmark_envelope.soft_tilt_deg+.01);state[7]=speed
    output=mission.tick(10.,state,control)
    assert output['enabled'] and output['phase']=='landing' and output['reason']=='soft_flight_envelope'
    assert mission._airborne_envelope()['maximum_speed_m_s']==cap

@pytest.mark.parametrize('speed',[4.,5.,6.])
def test_arbitrary_or_wrong_speed_envelope_is_rejected(speed):
    base=json.loads((PACKAGE/'config/sanity15_indi_mpc_constant_arclength.json').read_text())
    config=build_config(base,pattern='lemniscate',inner='indi',speed=speed,schedule='indi_shaped_angular',radius_m=2.,ground_m=.03)
    config['benchmark_envelope']['maximum_speed_m_s']+=.1
    with pytest.raises(ValueError):make_mission(config,ROOT/'loco_mjx')
    config=build_config(base,pattern='lemniscate',inner='indi',speed=speed,schedule='indi_shaped_angular',radius_m=2.,ground_m=.03)
    config['trajectory']['peak_speed_m_s']={4.:5.,5.:6.,6.:4.}[speed]
    from sanity_zenoh.high_speed_lemniscate import reference_limits
    config['trajectory'].update(reference_limits(config['trajectory']['peak_speed_m_s']))
    with pytest.raises(ValueError):make_mission(config,ROOT/'loco_mjx')

@pytest.mark.parametrize('speed',[.8,1.,1.5])
def test_lower_speed_benchmarks_keep_ordinary_limits(speed):
    mission,state,control=airborne(speed)
    e=mission._airborne_envelope()
    assert e['maximum_speed_m_s']==2. and e['soft_tilt_deg']==40. and e['hard_tilt_deg']==60.
    state[7]=2.001
    assert not mission.tick(10.,state,control)['enabled']
