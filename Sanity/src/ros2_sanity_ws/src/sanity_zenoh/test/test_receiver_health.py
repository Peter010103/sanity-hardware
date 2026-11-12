"""Replay the observed157ms heartbeat gap through actual mission landing logic."""
from types import SimpleNamespace
import pytest
from test_controller_snapshot_age import airborne
from sanity_zenoh.receiver_health import receiver_readiness

def receiver(**changes):
    values=dict(session=9,mode=1,epoch=123,failsafe=False,fc_ready=True,
                fc_age_ms=18,voltage_mv=7850)
    values.update(changes)
    return SimpleNamespace(**values)

def classify(control, age=.15708239, **changes):
    args=dict(age_s=age,session=9,mode=1,controller=control,
              airborne=True,controller_fresh=True,pose_fresh=True)
    args.update(changes)
    return receiver_readiness(receiver(),**args)

@pytest.mark.parametrize('full',[False,True])
def test_exact_flight_gap_requests_landing_with_positive_thrust(full):
    m,state,c=airborne(full)
    h=classify(c)
    assert h['ready'] and h['landing_requested']
    m.request_landing(h['reason'])
    out=m.tick(30.,state,c,mocap_age_s=.006319259,
               controller_age_s=.022902713,receiver_ready=h['ready'])
    assert out['enabled'] and out['thrust_N']>0 and not out['done']
    assert out['phase']=='landing' and out['reason']=='receiver_status_delayed'
    # The next recorded heartbeat arrived26.6ms after the old stop. A fresh
    # update restores admission but does not cancel the commanded landing.
    h=classify(c,age=.002)
    assert h['ready'] and not h['landing_requested']
    out=m.tick(30.02,state,c,receiver_ready=h['ready'])
    assert out['enabled'] and out['phase']=='landing'

@pytest.mark.parametrize('full',[False,True])
@pytest.mark.parametrize('age',[.300,.350,1.])
def test_sustained_gap_still_stops(full,age):
    m,state,c=airborne(full);h=classify(c,age)
    assert not h['ready']
    out=m.tick(30.,state,c,receiver_ready=h['ready'])
    assert out['done'] and not out['enabled']

@pytest.mark.parametrize('condition',['airborne','controller_fresh','pose_fresh'])
def test_grace_requires_airborne_and_independent_fresh_feedback(condition):
    _,_,c=airborne(True)
    assert not classify(c,**{condition:False})['ready']

@pytest.mark.parametrize('age',[.15,.157,.299])
def test_startup_never_uses_airborne_grace(age):
    _,_,c=airborne(True)
    assert not classify(c,age,airborne=False)['ready']

@pytest.mark.parametrize('change',[dict(session=10),dict(mode=2),dict(epoch=124),
                                  dict(failsafe=True),dict(fc_ready=False),
                                  dict(fc_age_ms=150),dict(voltage_mv=6500)])
def test_explicit_receiver_faults_are_never_masked_by_grace(change):
    _,_,c=airborne(True)
    h=receiver_readiness(receiver(**change),age_s=.157,session=9,mode=1,
                         controller=c,airborne=True,controller_fresh=True,pose_fresh=True)
    assert not h['ready'] and not h['landing_requested']

@pytest.mark.parametrize('age',[-.001,float('nan'),float('inf')])
def test_bad_status_time_rejected(age):
    _,_,c=airborne(True)
    assert not classify(c,age)['ready']
