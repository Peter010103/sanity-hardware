from sanity_zenoh.standalone_paths import load_json
"""Mission sequencing and loss-of-state tests; no ROS/network/hardware."""
import copy
import json
from pathlib import Path
import numpy as np
import pytest
from sanity_zenoh.indi_hover import make_mission, simulate
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'loco_mjx').is_dir())
CONFIG=load_json(Path(__file__).resolve().parents[1] / 'config/sanity15_indi_hover.json')

def fixture():
    mission=make_mission(CONFIG,ROOT/'loco_mjx')
    state=np.zeros(13);state[2]=mission.limits.ground_m;state[3]=1
    control=dict(fc_uid_msp=mission.limits.expected_uid,permission=True,supported=True,request_aux=3,
        mode='pid',indi_weight=0.,eligible=True,armed=False,fault_latched=False,fault='ok',control_period_us=500.)
    return mission,state,control

def advance_to_armed():
    m,s,c=fixture()
    for i in range(52):o=m.tick(i*.02,s,c)
    assert o['enabled'] and not o['indi_requested'] and o['thrust_N']==0
    c['armed']=True;m.tick(1.04,s,c)
    return m,s,c

@pytest.mark.parametrize('scale',[.98,1.,1.02])
def test_complete_order_and_bounded_commands(scale):
    result=simulate(CONFIG,ROOT/'loco_mjx',mass_scale=scale)
    assert result['completed'] and result['reason']=='test_complete'
    assert result['phases']==['waiting','arming','takeoff','pid_hover','enter_indi','indi_hover','return_pid','landing','stopped']
    for r in result['records']:
        o=r['output']
        assert 0<=o['thrust_N']<=CONFIG.get('mass_kg',.06947)*9.81*1.5
        assert max(abs(x) for x in o['rates_rad_s'])<=.6
        assert not o['indi_requested'] or (o['enabled'] and o['phase'] in ('enter_indi','indi_hover'))
    assert not result['records'][-1]['output']['enabled']

@pytest.mark.parametrize('initial_xy',[(-.17,.01),(2.,-3.)])
def test_launch_translation_does_not_change_commands_or_envelopes(initial_xy):
    # A hover must behave identically anywhere in the mocap world frame.
    origin=simulate(CONFIG,ROOT/'loco_mjx')
    shifted=simulate(CONFIG,ROOT/'loco_mjx',initial_xy=initial_xy)
    assert shifted['reason']==origin['reason']=='test_complete'
    assert len(shifted['records'])==len(origin['records'])
    for a,b in zip(origin['records'],shifted['records']):
        assert a['output']['phase']==b['output']['phase']
        np.testing.assert_allclose(b['state'][:2],np.asarray(a['state'][:2])+initial_xy,atol=1e-10)
        np.testing.assert_allclose(b['state'][2:],a['state'][2:],atol=1e-10)
        np.testing.assert_allclose(b['output']['rates_rad_s'],a['output']['rates_rad_s'],atol=1e-10)
        assert b['output']['thrust_N']==pytest.approx(a['output']['thrust_N'],abs=1e-10)

@pytest.mark.parametrize('fault,reason',[('mocap_loss','state_or_receiver_unavailable'),('rpm_loss','fc_indi_fault_rpm')])
def test_simulated_faults_stop_or_return_then_land(fault,reason):
    r=simulate(CONFIG,ROOT/'loco_mjx',fault=fault)
    assert r['completed'] and r['reason']==reason
    if fault=='rpm_loss':assert r['phases'][-3:]==['return_pid','landing','stopped']
    assert not r['records'][-1]['output']['enabled']

@pytest.mark.parametrize('field,value',[('permission',False),('supported',False),('request_aux',2),('armed',True),('fault_latched',True),('control_period_us',250),('control_period_us',None)])
def test_startup_interlocks_never_arm(field,value):
    m,s,c=fixture();c[field]=value
    for i in range(1100):assert not m.tick(i*.02,s,c)['enabled']
    assert m.done

def test_wrong_identity_and_missing_startup_data():
    m,s,c=fixture();c['fc_uid_msp']='00'*12
    assert not m.tick(0,s,c)['enabled'] and m.reason=='fc_identity_mismatch'
    m,s,c=fixture()
    for i in range(1100):assert not m.tick(i*.02,None,None)['enabled']
    assert m.done and m.reason=='startup_timeout'

@pytest.mark.parametrize('fault',[dict(mocap_age_s=.04),dict(controller_age_s=.15),dict(receiver_ready=False)])
def test_live_missing_measurements_disables_immediately(fault):
    m,s,c=advance_to_armed();o=m.tick(1.06,s,c,**fault)
    assert m.done and not o['enabled'] and not o['indi_requested']

def test_fc_disarm_never_rearms_and_hard_envelope_stops():
    m,s,c=advance_to_armed();c['armed']=False
    assert not m.tick(1.06,s,c)['enabled'] and m.reason=='unexpected_fc_disarm'
    m,s,c=advance_to_armed();s[2]=2
    assert not m.tick(1.06,s,c)['enabled'] and m.reason=='hard_flight_envelope'

def test_cancel_before_arming_and_during_takeoff():
    m,s,c=fixture();m.request_landing();assert not m.tick(0,s,c)['enabled']
    m,s,c=advance_to_armed();m.request_landing()
    o=m.tick(1.06,s,c)
    assert not o['indi_requested'] and m.phase=='landing'

def test_unacknowledged_engagement_does_not_claim_indi():
    m,s,c=advance_to_armed();m.phase='enter_indi';m.phase_started=1.04
    s[2]=m.home[2];m.last_tick=2.58
    o=m.tick(2.6,s,c)
    assert not o['indi_requested'] and m.phase=='landing' and m.reason=='indi_engagement_not_acknowledged'

@pytest.mark.parametrize('scale',[.9,1.1])
def test_large_model_mismatch_exits_to_landing_without_engaging_indi(scale):
    r=simulate(CONFIG,ROOT/'loco_mjx',mass_scale=scale)
    assert r['completed'] and r['reason']=='soft_flight_envelope'
    assert 'indi_hover' not in r['phases'] and r['phases'][-2:]==['landing','stopped']
    assert not r['records'][-1]['output']['enabled']
