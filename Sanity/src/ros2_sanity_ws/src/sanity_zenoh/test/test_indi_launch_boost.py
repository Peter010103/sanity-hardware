from sanity_zenoh.standalone_paths import load_json
"""Bounded launch floor belongs only to a fresh, acknowledged INDI takeoff."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from sanity_zenoh.full_indi_mission import GroundIndiLimits
from sanity_zenoh.indi_hover import make_mission,simulate
from sanity_zenoh.indi_diagnostics import build_diagnostic_config
from sanity_zenoh.mpc_benchmark_config import build_config
from test_full_indi_mission import ROOT

PACKAGE=Path(__file__).resolve().parents[1]
CURRENT=load_json(PACKAGE / 'config/sanity15_full_indi_estimator_hover_1khz.json')
BASE=load_json(PACKAGE / 'config/sanity15_indi_mpc_constant_arclength.json')


def request(config=None):
    mission=make_mission(copy.deepcopy(config or CURRENT),ROOT/'loco_mjx')
    state=np.zeros(13);state[2]=mission.limits.ground_m;state[3]=1.
    control=dict(fc_uid_msp=mission.limits.expected_uid,permission=True,supported=True,request_aux=3,
        mode='pid',indi_weight=0.,requested=False,eligible=True,armed=False,fault_latched=False,fault='ok',
        reason='ok',control_period_us=float(mission.limits.expected_control_period_us),rpm_mask=15)
    for tick in range(500):
        now=tick*.02;output=mission.tick(now,state,control)
        assert not mission.launch_boost_active
        assert output['thrust_N']<=.4*mission.limits.mass_kg*9.81+1e-12
        if output['phase']=='arming':control['armed']=True
        if output['indi_requested']:return mission,state,control,now,output
    raise AssertionError('Ground request not reached')


def acknowledged(config=None):
    mission,state,control,now,before=request(config)
    control.update(mode='indi',indi_weight=1.,requested=True)
    now+=.02;output=mission.tick(now,state,control)
    return mission,state,control,now,before,output


@pytest.mark.parametrize('fields',(
    {'launch_boost_weight':True},{'launch_boost_weight':float('nan')},{'launch_boost_weight':float('inf')},
    {'launch_boost_weight':-.1},{'launch_boost_weight':.99},{'launch_boost_weight':1.251},
    {'launch_boost_max_s':False},{'launch_boost_max_s':float('nan')},{'launch_boost_max_s':.601},
    {'launch_boost_max_s':0},{'launch_slew_up_N_s':True},{'launch_slew_up_N_s':float('inf')},
    {'launch_slew_up_N_s':4.01},{'launch_slew_up_N_s':0},
))
def test_invalid_launch_fields_fail_closed(fields):
    with pytest.raises(ValueError):GroundIndiLimits(start_climb_on_ack=True,**fields)


def test_active_boost_requires_immediate_ack_and_legacy_zero_is_disabled():
    with pytest.raises(ValueError):GroundIndiLimits(launch_boost_weight=1.2)
    assert GroundIndiLimits().launch_boost_weight==0.
    config=copy.deepcopy(CURRENT);config['ground_indi']['launch_boost_weight']=0.
    mission,state,control,now,before,output=acknowledged(config)
    assert not mission.launch_boost_active and mission.launch_boost_started is None
    assert output['thrust_N']-before['thrust_N']<=.5*.02+1e-12


def test_ack_starts_continuous_bounded_ramp_and_logging():
    mission,state,control,now,before,output=acknowledged()
    assert mission.launch_boost_active and output['phase']=='takeoff' and output['indi_requested']
    assert output['thrust_N']-before['thrust_N']==pytest.approx(3.*.02)
    assert mission.last_reference['position'][2]==pytest.approx(mission.limits.ground_m)
    assert mission.last_reference['velocity']==pytest.approx([0.,0.,0.],abs=1e-12)
    log=mission.diagnostics(output)['launch_boost']
    assert log['requested_floor_N']==pytest.approx(1.2*mission.limits.mass_kg*9.81)
    assert log['selected_thrust_N']==output['thrust_N'] and log['slew_up_limit_N_s']==3.
    assert log['slew_down_limit_N_s']==.5 and log['reason']=='active'
    previous=output['thrust_N']
    for tick in range(1,11):
        output=mission.tick(now+tick*.02,state,control)
        assert output['thrust_N']-previous<=3.*.02+1e-12
        assert output['thrust_N']<=mission.limits.maximum_thrust_weight*mission.limits.mass_kg*9.81
        previous=output['thrust_N']
    assert output['thrust_N']>=1.2*mission.limits.mass_kg*9.81-1e-12


def test_timeout_is_permanent_and_returns_to_ordinary_slew():
    mission,state,control,now,before,output=acknowledged()
    for tick in range(1,26):
        previous=output['thrust_N'];output=mission.tick(now+tick*.02,state,control)
    assert not mission.launch_boost_active and mission.launch_boost_stop_reason=='timeout'
    assert abs(output['thrust_N']-previous)<=.5*.02+1e-12
    assert mission.diagnostics(output)['launch_boost']['slew_up_limit_N_s']==.5
    mission.launch_boost_started=now+.5 # Even a changed timestamp cannot clear the stop latch.
    output=mission.tick(now+.52,state,control)
    assert not mission.launch_boost_active and mission.launch_boost_stop_reason=='timeout'


def test_confirmed_liftoff_ends_before_pd_horizontal_blend_and_never_restarts():
    config=build_diagnostic_config(BASE,inner='indi',test='pd-hover',ground_m=.03)
    mission,state,control,now,before,output=acknowledged(config)
    assert mission.pd.liftoff.clearance_m==.02 and mission.pd.liftoff.airborne_dwell_s==.04
    state[2]=mission.limits.ground_m+.025
    for tick in range(1,4):output=mission.tick(now+tick*.02,state,control)
    assert mission.pd.liftoff.blend_start is not None and not mission.pd.liftoff.complete
    assert not mission.launch_boost_active and mission.launch_boost_stop_reason=='confirmed_liftoff'
    state[2]=mission.limits.ground_m
    output=mission.tick(now+.08,state,control)
    assert mission.pd.liftoff.blend_start is None
    assert not mission.launch_boost_active and mission.launch_boost_stop_reason=='confirmed_liftoff'


@pytest.mark.parametrize('mutation',({'mode':'to_indi','indi_weight':.999},{'requested':False}))
def test_no_boost_before_full_ack(mutation):
    mission,state,control,now,_=request();control.update(mutation)
    for tick in range(1,10):
        output=mission.tick(now+tick*.02,state,control)
        assert not mission.launch_boost_active and mission.launch_boost_started is None
        assert output['thrust_N']<=mission.ground_thrust_N+1e-12


@pytest.mark.parametrize('mutation',({'fault_latched':True,'fault':'rpm'},{'mode':'pid','indi_weight':0.},{'requested':False},{'permission':False}))
def test_fault_or_authority_loss_stops_boost_and_cannot_restart(mutation):
    mission,state,control,now,before,output=acknowledged();control.update(mutation)
    output=mission.tick(now+.02,state,control)
    assert not mission.launch_boost_active and mission.launch_boost_stop_reason is not None
    control.update(mode='indi',indi_weight=1.,requested=True,permission=True,fault_latched=False)
    mission.tick(now+.04,state,control)
    assert not mission.launch_boost_active


def test_landing_request_cancels_boost_immediately_and_preserves_downward_slew():
    mission,state,control,now,before,output=acknowledged()
    mission.request_landing('operator')
    assert not mission.launch_boost_active
    previous=output['thrust_N'];output=mission.tick(now+.02,state,control)
    assert output['phase']=='landing' and not mission.launch_boost_active
    assert abs(output['thrust_N']-previous)<=.5*.02+1e-12


@pytest.mark.parametrize('stale',({'mocap_age_s':.04},{'controller_age_s':.31},{'receiver_ready':False}))
def test_stale_feedback_during_boost_keeps_existing_immediate_stop(stale):
    mission,state,control,now,before,output=acknowledged()
    output=mission.tick(now+.02,state,control,**stale)
    assert not output['enabled'] and output['thrust_N']==0 and not output['indi_requested']
    assert output['reason']=='state_or_receiver_unavailable'
    assert not mission.launch_boost_active and mission.launch_boost_stop_reason=='stopped:state_or_receiver_unavailable'


def test_floor_obeys_mission_absolute_thrust_bound():
    mission,state,control,now,before,output=acknowledged()
    mission.limits=replace(mission.limits,maximum_thrust_weight=1.1)
    for tick in range(1,16):output=mission.tick(now+tick*.02,state,control)
    assert output['thrust_N']<=1.1*mission.limits.mass_kg*9.81+1e-12
    assert mission.diagnostics(output)['launch_boost']['requested_floor_N']==pytest.approx(1.1*mission.limits.mass_kg*9.81)


@pytest.mark.parametrize('inner',('pid','indi'))
def test_builders_select_only_indi_launch_and_preserve_pid_ground_baseline(inner):
    for config in (build_diagnostic_config(BASE,inner=inner,test='pd-hover',ground_m=.03),
                   build_config(BASE,pattern='circle',inner=inner,speed=.8,ground_m=.03)):
        mission=make_mission(config,ROOT/'loco_mjx')
        assert mission.ground.thrust_weight==.4
        assert mission.ground.launch_boost_weight==(1.2 if inner=='indi' else 0.)
        assert mission.ground.start_climb_on_ack is (inner=='indi')
        assert config['mpc']['model']==BASE['mpc']['model'] and Path(config['mpc']['config']).name in ('sanity09_mpc.json','sanity15_mpc_indi_rate_model.json') and config['liftoff_options']==BASE['liftoff_options']
        assert not config.get('ground_support_transition',False)


@pytest.mark.parametrize('test',('pd-hover','roll-pulse','pitch-pulse','yaw-pulse','roll-sine','pitch-sine'))
def test_actual_1khz_diagnostic_simulations_complete_and_boost_only_after_ack(test):
    config=build_diagnostic_config(BASE,inner='indi',test=test,ground_m=.03)
    result=simulate(config,ROOT/'loco_mjx')
    assert result['completed'] and result['reason']=='test_complete'
    active=[row for row in result['records'] if row['outer_controller']['launch_boost']['active']]
    assert active and active[-1]['time_s']-active[0]['time_s']<.5
    assert all(row['output']['phase']=='takeoff' and row['controller']['mode']=='indi' and row['controller']['indi_weight']==1. for row in active)
    assert all(not row['outer_controller']['launch_boost']['active'] for row in result['records'] if row['output']['phase'] in ('ground_spinup','ground_indi_request','landing','stopped'))


def test_actual_fullhover_1khz_candidate_simulates_without_firmware_or_guard_change():
    result=simulate(CURRENT,ROOT/'loco_mjx')
    assert result['completed'] and result['reason']=='test_complete'
    assert all(row['controller']['control_period_us']==1000 for row in result['records'])
    mission=make_mission(CURRENT,ROOT/'loco_mjx')
    assert mission.pd.liftoff.clearance_m==.05 and mission.pd.liftoff.airborne_dwell_s==.1
