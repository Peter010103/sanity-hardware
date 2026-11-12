from sanity_zenoh.standalone_paths import load_json
"""Exercise production mission.tick through airborne, braking and fault guards."""
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from sanity_zenoh.indi_hover import make_mission
from sanity_zenoh.mpc_benchmark_config import build_config

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'loco_mjx').is_dir())
PACKAGE=Path(__file__).resolve().parents[1]


def airborne(speed=3.,*,admitted=True):
    base=load_json(PACKAGE / 'config/sanity15_indi_mpc_constant_arclength.json')
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


def test_actual_reported_states_do_not_trigger_three_mps_envelope():
    mission,state,control=airborne();state[7]=2.045;tilt(state,30.792)
    output=mission.tick(10.,state,control)
    assert output['enabled'] and output['phase']=='lemniscate' and output['reason'] is None
    assert mission._airborne_envelope()['maximum_speed_m_s']==4.


@pytest.mark.parametrize('reason',['operator_landing','mpc_invalid_solver','rpm'])
def test_speed_headroom_stays_latched_through_fault_braking(reason):
    mission,state,control=airborne();state[7]=3.2
    if reason=='rpm':control.update(fault_latched=True,fault='rpm')
    else:mission.request_landing(reason)
    output=mission.tick(10.,state,control)
    assert output['enabled'] and output['phase']=='landing'
    assert mission._airborne_envelope()['maximum_speed_m_s']==4.
    state[7]=2.045
    output=mission.tick(10.02,state,control)
    assert output['enabled'] and output['phase']=='landing'
    assert max(abs(v) for v in output['rates_rad_s'])<=.6


def test_soft_tilt_still_lands_and_braking_above_two_remains_enabled():
    mission,state,control=airborne();state[7]=3.2;tilt(state,41.)
    output=mission.tick(10.,state,control)
    assert output['enabled'] and output['reason']=='soft_flight_envelope'
    assert mission.last_envelope_violation['criteria']==['tilt']
    state[7]=2.045;tilt(state,23.095)
    assert mission.tick(10.02,state,control)['enabled']


@pytest.mark.parametrize('kind',['speed','tilt','position','altitude'])
def test_true_hard_guards_still_disable_with_identified_criterion(kind):
    mission,state,control=airborne()
    if kind=='speed':state[7]=4.001
    elif kind=='tilt':tilt(state,60.01)
    elif kind=='position':state[0]+=1.501
    else:state[2]=mission.limits.maximum_altitude_m+.001
    output=mission.tick(10.,state,control)
    assert not output['enabled'] and output['reason']=='hard_flight_envelope'
    assert kind in mission.last_envelope_violation['criteria']


@pytest.mark.parametrize('speed,admitted',[(1.5,True),(3.,False)])
def test_normal_profile_and_three_mps_takeoff_keep_two_mps_limit(speed,admitted):
    mission,state,control=airborne(speed,admitted=admitted);state[7]=2.001
    assert not mission.tick(10.,state,control)['enabled']
    assert mission.last_envelope_violation['limits']['maximum_speed_m_s']==2.


@pytest.mark.parametrize('fault',['mocap','controller_age','identity','native_guard'])
def test_unrelated_hard_faults_keep_their_actions(fault):
    mission,state,control=airborne();options={}
    if fault=='mocap':options['mocap_age_s']=.041
    elif fault=='controller_age':options['controller_age_s']=.301
    elif fault=='identity':control['fc_uid_msp']='00'*12
    else:control['takeoff_guard_triggered']=True
    output=mission.tick(10.,state,control,**options)
    assert not output['enabled'] and output['done']


def test_three_mps_envelope_cannot_leak_into_other_benchmark_profiles():
    mission,_,_=airborne()
    config=load_json(PACKAGE / 'config/sanity15_indi_mpc_constant_arclength.json')
    config['benchmark_envelope'].update(soft_tilt_deg=35.,maximum_speed_m_s=4.)
    with pytest.raises(ValueError):make_mission(config,ROOT/'loco_mjx')
