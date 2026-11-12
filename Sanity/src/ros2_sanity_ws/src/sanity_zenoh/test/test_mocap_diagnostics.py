"""TF rejection/expiry diagnostics; no ROS, devices or network."""
import json
import math

import pytest

from sanity_zenoh.mocap import MocapEstimator, MocapOutlier, state_age
from sanity_zenoh.mocap_diagnostics import MocapDiagnostics, update_with_diagnostics, retain_fresh_packet_after_outlier


def feed(estimator, diagnostics, events, n, *, stamp=None, position=(0.,0.,1.),
         quaternion=(0.,0.,0.,1.), now=None):
    source=1_000_000_000+n*5_000_000 if stamp is None else stamp
    ros=source+1_000_000 if now is None else now
    return update_with_diagnostics(estimator,diagnostics,events.append,
        parent='map',child='sanity15',stamp_ns=source,position=position,
        quaternion=quaternion,ros_now_ns=ros,monotonic_ns=ros+10_000_000_000)


def fixture():
    return MocapEstimator('map','sanity15',frames_confirmed=True),MocapDiagnostics(),[]


def test_logs_all_native_rate_inputs_independent_of_forwarding_rate():
    estimator,d,events=fixture()
    for n in range(20):
        packet,error=feed(estimator,d,events,n)
        assert error is None and packet is not None
    assert len(events)==20 and d.inputs==d.accepted==20
    assert all(e['outcome']=='accepted' for e in events)
    assert events[0]['estimator_ready'] is False
    assert events[-1]['estimator_ready'] is True
    assert events[-1]['position_xyz_m']==[0.,0.,1.]
    assert events[-1]['quaternion_xyzw']==[0.,0.,0.,1.]
    assert events[-1]['raw_source_age_s']==pytest.approx(.001)
    json.dumps(events,allow_nan=False)


def test_repeated_transform_is_observable_and_cannot_refresh_accepted_age():
    estimator,d,events=fixture()
    for n in range(5):packet,_=feed(estimator,d,events,n)
    accepted=d.last_accepted.copy()
    ignored,error=feed(estimator,d,events,5,stamp=packet['stamp_ns'],now=packet['stamp_ns']+30_000_000)
    assert ignored is None and error is None
    assert d.last_accepted==accepted and d.ignored==1
    assert events[-1]['outcome']=='ignored'
    assert events[-1]['reason']=='nonincreasing_source_stamp'
    # A new callback with a frozen source stamp changes arrival statistics only.
    now=packet['stamp_ns']+41_000_000
    with pytest.raises(ValueError,match='expired'):state_age(packet,now)
    snapshot=d.snapshot(ros_now_ns=now,monotonic_ns=now+10_000_000_000,state_error='Mocap state expired')
    assert snapshot['last_accepted_source_age_s']==pytest.approx(.041)
    assert snapshot['control_state_error']=='Mocap state expired'
    assert snapshot['last_input_gap_s']<snapshot['last_accepted_gap_s']


def test_position_outlier_retains_raw_pose_and_exact_rejection_details():
    estimator,d,events=fixture()
    for n in range(5):packet,_=feed(estimator,d,events,n)
    accepted=d.last_accepted.copy()
    rejected,error=feed(estimator,d,events,5,position=(1.,0.,1.))
    assert rejected is None and isinstance(error,MocapOutlier)
    assert d.last_accepted==accepted and d.rejected==1
    last=events[-1]
    assert last['position_xyz_m']==[1.,0.,1.]
    assert last['rejection_diagnostics']['component']=='position'
    assert last['rejection_diagnostics']['speed_m_s']==pytest.approx(200.)
    assert last['reason']=='Implausible mocap sample rejected'
    snapshot=d.snapshot(ros_now_ns=1_050_000_000,monotonic_ns=11_050_000_000,
        state_error='no_current_packet',callback_error=str(error))
    assert snapshot['last_input_outcome']=='rejected'
    assert snapshot['last_rejection']['source_stamp_ns']==1_025_000_000
    assert snapshot['last_accepted_source_stamp_ns']==1_020_000_000


@pytest.mark.parametrize('quaternion,position',[
    ((float('nan'),0.,0.,1.),(0.,0.,1.)),
    ((0.,0.,0.,1.),(float('inf'),0.,1.)),
])
def test_nonfinite_raw_input_is_logged_as_null_without_breaking_recorder(quaternion,position):
    estimator,d,events=fixture()
    packet,error=feed(estimator,d,events,0,quaternion=quaternion,position=position)
    assert packet is None and isinstance(error,ValueError)
    text=json.dumps(events[-1],allow_nan=False)
    assert 'null' in text
    assert events[-1]['outcome']=='rejected'


def test_nonfinite_outlier_diagnostics_are_also_json_safe():
    d=MocapDiagnostics();error=MocapOutlier(10,5,float('inf'),float('nan'))
    event=d.input(parent='map',child='sanity15',stamp_ns=10,position=(0,0,1),
        quaternion=(0,0,0,1),ros_now_ns=11,monotonic_ns=11,error=error)
    assert event['rejection_diagnostics']['speed_m_s'] is None
    assert event['rejection_diagnostics']['angular_rate_rad_s'] is None
    json.dumps(event,allow_nan=False)


def test_stale_and_future_inputs_log_timestamp_reason_not_a_camera_claim():
    for age in [.041,-.003]:
        estimator,d,events=fixture()
        packet,error=feed(estimator,d,events,0,now=1_000_000_000+int(age*1e9))
        assert packet is None and error is not None
        assert events[-1]['raw_source_age_s']==pytest.approx(age)
        assert 'stale/future' in events[-1]['reason']
        assert d.last_accepted is None


def test_missing_stream_has_no_fabricated_accepted_pose():
    d=MocapDiagnostics()
    snapshot=d.snapshot(ros_now_ns=1,monotonic_ns=1,state_error='no_current_packet')
    assert snapshot['inputs']==0
    assert snapshot['last_input_gap_s'] is None
    assert snapshot['last_accepted_source_age_s'] is None
    json.dumps(snapshot,allow_nan=False)


def test_isolated_outlier_retains_fresh_control_packet_without_refreshing_time():
    estimator,d,events=fixture()
    for n in range(8):packet,_=feed(estimator,d,events,n)
    original=json.dumps(packet,sort_keys=True)
    rejected,error=feed(estimator,d,events,8,position=(1.,0.,1.))
    assert rejected is None
    retained=retain_fresh_packet_after_outlier(packet,error,packet['stamp_ns']+20_000_000)
    assert retained is packet
    assert json.dumps(retained,sort_keys=True)==original
    assert state_age(retained,packet['stamp_ns']+20_000_000)[1]==pytest.approx(.02)
    recovered,error=feed(estimator,d,events,9)
    assert error is None and recovered['ready']
    assert recovered['stamp_ns']>packet['stamp_ns']


@pytest.mark.parametrize('age_ms',[40,41,80])
def test_outlier_grace_never_extends_original_freshness_limit(age_ms):
    estimator,d,events=fixture()
    for n in range(8):packet,_=feed(estimator,d,events,n)
    error=MocapOutlier(packet['stamp_ns']+5_000_000,packet['stamp_ns'],200.,0.)
    assert retain_fresh_packet_after_outlier(packet,error,packet['stamp_ns']+age_ms*1_000_000) is None


def test_invalid_input_or_unready_packet_gets_no_outlier_grace():
    estimator,d,events=fixture();unready,_=feed(estimator,d,events,0)
    for n in range(1,8):packet,_=feed(estimator,d,events,n)
    error=MocapOutlier(packet['stamp_ns']+5_000_000,packet['stamp_ns'],200.,0.)
    assert retain_fresh_packet_after_outlier(unready,error,unready['stamp_ns']+5_000_000) is None
    assert retain_fresh_packet_after_outlier(None,error,packet['stamp_ns']) is None
    assert retain_fresh_packet_after_outlier(packet,ValueError('Invalid quaternion'),packet['stamp_ns']+5_000_000) is None
    assert retain_fresh_packet_after_outlier(packet,error,packet['receipt_ns']-1) is None


def test_production_tf_callback_rejects_jump_retains_previous_and_never_reforwards_it():
    # Exercise the actual nested ROS callback with small message/clock doubles;
    # no ROS installation or running middleware is required.
    import ast
    import time
    from pathlib import Path
    from types import SimpleNamespace as NS
    from scipy.spatial.transform import Rotation
    source=Path(__file__).resolve().parents[1]/'sanity_zenoh/indi_hover.py'
    tree=ast.parse(source.read_text())
    run=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run_ros')
    node_class=next(n for n in run.body if isinstance(n,ast.ClassDef) and n.name=='HoverNode')
    callback=next(n for n in node_class.body if isinstance(n,ast.FunctionDef) and n.name=='on_tf')
    estimator,d,events=fixture();forwarded=[];now=1_000_000_000
    node=NS(mocap=estimator,mocap_diagnostics=d,packet=None,last_error=None,
            mocap_stream=NS(send_estimate=lambda p,t:forwarded.append((p,t))),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=now)))
    scope=dict(update_with_diagnostics=update_with_diagnostics,
               retain_fresh_packet_after_outlier=retain_fresh_packet_after_outlier,
               state_age=state_age,time=time,recorder=NS(record=events.append))
    exec(compile(ast.Module(body=[callback],type_ignores=[]),str(source),'exec'),scope)
    def send(n,quaternion=(0.,0.,0.,1.)):
        nonlocal now
        stamp=1_000_000_000+n*5_000_000;now=stamp+1_000_000
        message=NS(transforms=[NS(header=NS(frame_id='map',stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9)),
            child_frame_id='sanity15',transform=NS(translation=NS(x=0.,y=0.,z=1.),
            rotation=NS(**dict(zip('xyzw',quaternion)))))])
        scope['on_tf'](node,message)
    for n in range(8):send(n)
    previous=node.packet;sent=len(forwarded)
    send(8,Rotation.from_euler('x',70,degrees=True).as_quat())
    assert node.packet is previous and len(forwarded)==sent
    assert events[-1]['kind']=='mocap_outlier_retained_previous'
    assert state_age(node.packet,now)[1]==pytest.approx(.006)
    send(9)
    assert node.packet is not previous and node.last_error is None
    assert len(forwarded)==sent+1
    for n in range(10,18):send(n,Rotation.from_euler('x',70,degrees=True).as_quat())
    assert node.packet is None
