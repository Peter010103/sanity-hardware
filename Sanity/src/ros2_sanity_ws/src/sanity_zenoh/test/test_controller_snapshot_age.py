from sanity_zenoh.standalone_paths import load_json
"""Deterministic receiver interleaving; no sender construction or transport."""
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from sanity_zenoh.client import SanitySender
from sanity_zenoh.indi_hover import controller_feedback_from_diagnostics,make_mission

ROOT=next(path for path in Path(__file__).resolve().parents if (path/'loco_mjx').is_dir())
CONFIG=Path(__file__).resolve().parents[1]/'config'


def airborne(full):
    config=load_json(CONFIG / ('sanity15_full_indi_estimator_hover.json' if full else 'sanity15_indi_hover.json'))
    mission=make_mission(config,ROOT/'loco_mjx')
    mission.home=np.array([0.,0.,mission.limits.ground_m+mission.limits.height_m])
    mission.reference=mission.reference_factory(mission.limits.ground_m,mission.home[2],mission.limits.settle_s,mission.limits.spool_s,mission.limits.climb_s)
    mission.first_tick=0.;mission.last_tick=29.98;mission.armed_at=20.;mission.ever_armed=True
    mission._phase('indi_hover' if full else 'pid_hover',29.)
    state=np.zeros(13);state[:3]=mission.home;state[3]=1.
    control=dict(fc_uid_msp=mission.limits.expected_uid,permission=True,supported=True,request_aux=3,
                 mode='indi' if full else 'pid',indi_weight=1. if full else 0.,requested=full,
                 eligible=True,armed=True,fault_latched=False,fault='ok',reason='ok',rpm_mask=15,
                 control_period_us=mission.limits.expected_control_period_us,receiver_epoch=123,response_esp_ms=1000)
    return mission,state,control


def diagnostics_sender(control):
    sender=SanitySender.__new__(SanitySender)
    sender.lock=threading.RLock();sender.latest_controller_state=dict(control)
    sender.controller_state_time=29.99;sender.status=SimpleNamespace(epoch=123,receiver_ms=1000)
    sender.status_time=29.999
    return sender


@pytest.mark.parametrize('full',[False,True])
def test_receiver_timestamp_advance_after_atomic_diagnostics_cannot_make_snapshot_age_negative(monkeypatch,full):
    import sanity_zenoh.client as client
    mission,state,control=airborne(full);sender=diagnostics_sender(control)
    tick_now=30.;monkeypatch.setattr(client.time,'monotonic',lambda:30.0005)
    health=sender.controller_diagnostics(mission.limits.controller_status_max_age_s)
    assert health['fresh']
    # Deterministic RX callback interleaving after diagnostics and after tick_now.
    sender.controller_state_time=30.002
    sender.latest_controller_state['mode']='to_pid'
    snapshot,age=controller_feedback_from_diagnostics(health)
    assert age==pytest.approx(.0105) and snapshot['mode']==control['mode']
    assert tick_now-sender.controller_state_time<0  # The former live read races.
    output=mission.tick(tick_now,state,snapshot,controller_age_s=age)
    assert output['enabled'] and not output['done'] and mission.reason is None
    old,old_state,_=airborne(full)
    assert not old.tick(tick_now,old_state,snapshot,controller_age_s=tick_now-sender.controller_state_time)['enabled']
    assert old.reason=='state_or_receiver_unavailable'


@pytest.mark.parametrize('full',[False,True])
@pytest.mark.parametrize('fault',['host_stale','esp_stale','missing','epoch'])
def test_snapshot_binding_retains_host_esp_epoch_and_missing_rejection(monkeypatch,full,fault):
    import sanity_zenoh.client as client
    mission,state,control=airborne(full);sender=diagnostics_sender(control)
    if fault=='host_stale':sender.controller_state_time=29.5
    if fault=='esp_stale':sender.status.receiver_ms=1400
    if fault=='missing':sender.latest_controller_state=None
    if fault=='epoch':sender.status.epoch=124
    monkeypatch.setattr(client.time,'monotonic',lambda:30.0005)
    health=sender.controller_diagnostics(mission.limits.controller_status_max_age_s)
    snapshot,age=controller_feedback_from_diagnostics(health)
    assert not health['fresh'] and snapshot is None
    output=mission.tick(30.,state,snapshot,controller_age_s=age)
    assert output['done'] and not output['enabled'] and mission.reason=='state_or_receiver_unavailable'


@pytest.mark.parametrize('full',[False,True])
@pytest.mark.parametrize('fault',['state','controller','receiver'])
def test_loss_after_twenty_seconds_preserves_airborne_stop_reason(full,fault):
    mission,state,control=airborne(full);kwargs={}
    if fault=='state':state=None
    if fault=='controller':control=None
    if fault=='receiver':kwargs['receiver_ready']=False
    assert not mission.tick(30.,state,control,**kwargs)['enabled']
    assert mission.reason=='state_or_receiver_unavailable'


@pytest.mark.parametrize('full',[False,True])
def test_invalid_startup_still_times_out_while_waiting(full):
    mission,_,_=airborne(full);mission._phase('waiting',0.)
    assert not mission.tick(30.,None,None)['enabled']
    assert mission.reason=='startup_timeout'


@pytest.mark.parametrize('full',[False,True])
def test_fresh_bound_snapshot_does_not_mask_receiver_voltage_or_readiness_loss(monkeypatch,full):
    import sanity_zenoh.client as client
    mission,state,control=airborne(full);sender=diagnostics_sender(control)
    monkeypatch.setattr(client.time,'monotonic',lambda:30.0005)
    health=sender.controller_diagnostics(mission.limits.controller_status_max_age_s)
    assert health['fresh']
    sender.controller_state_time=30.002
    snapshot,age=controller_feedback_from_diagnostics(health)
    output=mission.tick(30.,state,snapshot,controller_age_s=age,receiver_ready=False)
    assert not output['enabled'] and mission.reason=='state_or_receiver_unavailable'


@pytest.mark.parametrize('full',[False,True])
def test_actual_receiver_stop_during_recovery_landing_is_not_relabelled_as_startup(full):
    mission,state,control=airborne(full)
    mission._land(29.,state,'soft_flight_envelope')
    assert mission.reason=='soft_flight_envelope'
    output=mission.tick(30.,state,control,receiver_ready=False)
    assert output['done'] and not output['enabled']
    assert mission.reason=='state_or_receiver_unavailable'
