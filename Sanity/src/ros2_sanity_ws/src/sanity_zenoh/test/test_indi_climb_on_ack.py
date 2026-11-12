from sanity_zenoh.standalone_paths import load_json
"""Host-only takeoff schedule: safe ground ACK gates the existing climb."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from sanity_zenoh.full_indi_mission import GroundIndiLimits
from sanity_zenoh.indi_hover import make_mission,simulate,mission_plan
from sanity_zenoh.indi_diagnostics import build_diagnostic_config
from sanity_zenoh.mpc_benchmark_config import build_config
from test_full_indi_mission import CONFIG,ROOT

PACKAGE=Path(__file__).resolve().parents[1]
BASE=load_json(PACKAGE / 'config/sanity15_indi_mpc_constant_arclength.json')
CURRENT=load_json(PACKAGE / 'config/sanity15_full_indi_estimator_hover_1khz.json')


def candidate():
    config=copy.deepcopy(CURRENT)
    config['ground_indi'].update(thrust_weight=.7,launch_boost_weight=0.)
    mission=make_mission(config,ROOT/'loco_mjx')
    state=np.zeros(13);state[2]=mission.limits.ground_m;state[3]=1.
    control=dict(fc_uid_msp=mission.limits.expected_uid,permission=True,supported=True,request_aux=3,
        mode='pid',indi_weight=0.,requested=False,eligible=True,armed=False,fault_latched=False,fault='ok',
        reason='ok',control_period_us=float(mission.limits.expected_control_period_us),rpm_mask=15)
    return mission,state,control


def request():
    mission,state,control=candidate()
    for tick in range(500):
        now=tick*.02;output=mission.tick(now,state,control)
        if output['phase']=='arming':control['armed']=True
        if output['indi_requested']:
            return mission,state,control,now,output
    raise AssertionError('Ground INDI request not reached')


def acknowledged():
    mission,state,control,now,before=request()
    control.update(mode='indi',indi_weight=1.,requested=True)
    now+=.02;output=mission.tick(now,state,control)
    return mission,state,control,now,before,output


def test_high_ground_target_requires_explicit_boolean_candidate():
    with pytest.raises(ValueError):GroundIndiLimits(thrust_weight=.7)
    with pytest.raises(ValueError):GroundIndiLimits(thrust_weight=.701,start_climb_on_ack=True)
    with pytest.raises(ValueError):GroundIndiLimits(start_climb_on_ack=1)
    assert GroundIndiLimits().thrust_weight==.4 and not GroundIndiLimits().start_climb_on_ack
    assert GroundIndiLimits(thrust_weight=.7,start_climb_on_ack=True).thrust_weight==.7


def test_ack_first_reference_is_ground_zero_velocity_then_climbs_without_second_spool():
    mission,state,control,now,before,output=acknowledged()
    assert output['phase']=='takeoff' and output['indi_requested']
    assert mission.indi_confirmed_at==now
    assert now-mission.armed_at==pytest.approx(mission.reference.climb_start)
    assert mission.last_reference['position']==pytest.approx([*mission.home[:2],mission.limits.ground_m])
    assert mission.last_reference['velocity']==pytest.approx([0.,0.,0.],abs=1e-12)
    assert before['thrust_N']==pytest.approx(.7*mission.limits.mass_kg*9.81)
    assert 0<=output['thrust_N']-before['thrust_N']<=.5*.02+1e-12
    output=mission.tick(now+.02,state,control)
    assert mission.last_reference['position'][2]>mission.limits.ground_m
    assert mission.last_reference['velocity'][2]>0
    assert output['indi_requested'] and mission.pd.last['liftoff']['horizontal_weight']==0.
    assert np.linalg.norm(output['rates_rad_s'][:2])<=.15+1e-12


def test_request_partial_weight_and_unacknowledged_status_never_start_climb():
    mission,state,control,now,_=request()
    control.update(mode='to_indi',indi_weight=.999,requested=True)
    for tick in range(1,20):
        output=mission.tick(now+tick*.02,state,control)
        assert output['phase']=='ground_indi_request' and mission.armed_at is None
        assert mission.last_reference['position'][2]==mission.limits.ground_m
        assert output['thrust_N']<=.7*mission.limits.mass_kg*9.81+1e-12


def test_ground_height_guard_still_stops_before_ack():
    mission,state,control,now,_=request();state[2]+=mission.ground.maximum_height_m+.001
    output=mission.tick(now+.02,state,control)
    assert not output['enabled'] and output['reason']=='ground_lift_before_indi_ack'
    assert mission.armed_at is None


def test_ack_preserves_slew_memory_and_native_authority_through_existing_climb():
    mission,state,control,now,before,output=acknowledged()
    previous=output['thrust_N']
    for tick in range(1,60):
        output=mission.tick(now+tick*.02,state,control)
        assert abs(output['thrust_N']-previous)<=.5*.02+1e-12
        assert output['indi_requested'] and output['enabled']
        previous=output['thrust_N']


@pytest.mark.parametrize('axis,angle',(('x',5.),('y',-5.)))
def test_tilted_ground_ack_uses_existing_level_liftoff_rate_cap(axis,angle):
    mission,state,control,now,_=request()
    state[3:7]=Rotation.from_euler(axis,angle,degrees=True).as_quat()[[3,0,1,2]]
    control.update(mode='indi',indi_weight=1.,requested=True)
    output=mission.tick(now+.02,state,control)
    index=0 if axis=='x' else 1
    assert output['rates_rad_s'][index]*angle<0
    assert np.linalg.norm(output['rates_rad_s'][:2])<=.15+1e-12
    assert output['rates_rad_s'][2]==0.
    assert not mission.pd.liftoff.complete
    assert mission.pd.last['liftoff']['horizontal_weight']==0.


def test_pid_builders_restore_legacy_ground_recipe_and_clock():
    for config in (build_diagnostic_config(BASE,inner='pid',test='pd-hover',ground_m=.03),
                   build_config(BASE,pattern='circle',inner='pid',speed=.8,ground_m=.03)):
        assert config['ground_indi']['thrust_weight']==.4
        assert config['ground_indi']['start_climb_on_ack'] is False
        assert not config.get('ground_support_transition',False)
        mission=make_mission(config,ROOT/'loco_mjx');mission._ground_confirmed(5.)
        assert mission.armed_at==5.
        assert config['mpc']['model']==BASE['mpc']['model'] and Path(config['mpc']['config']).name in ('sanity09_mpc.json','sanity15_mpc_indi_rate_model.json') and config['liftoff_options']==BASE['liftoff_options']


@pytest.mark.parametrize('source',(CONFIG,CURRENT),ids=('500us_legacy_with_candidate','actual_1khz_profile'))
def test_existing_full_indi_simulation_completes_with_same_airborne_requests(source):
    config=copy.deepcopy(source);config['ground_indi'].update(thrust_weight=.7,start_climb_on_ack=True,launch_boost_weight=0.)
    result=simulate(config,ROOT/'loco_mjx')
    assert result['completed'] and result['reason']=='test_complete'
    first=next(row for row in result['records'] if row['output']['phase']=='takeoff')
    assert first['controller']['mode']=='indi' and first['controller']['indi_weight']==1.
    assert first['state'][2]-config['limits']['ground_m']<=config['ground_indi']['maximum_height_m']
    assert all(row['output']['indi_requested'] for row in result['records'] if row['output']['phase'] in ('takeoff','indi_hover','landing'))
    hover=[row for row in result['records'] if row['output']['phase']=='indi_hover']
    assert hover[-1]['time_s']-hover[0]['time_s']>=config['limits']['indi_hold_s']-.02-1e-9
    assert hover[0]['time_s']-first['time_s']==pytest.approx(make_mission(config,ROOT/'loco_mjx').limits.climb_s,abs=.02)


@pytest.mark.parametrize('test',('pd-hover','roll-pulse','pitch-pulse','yaw-pulse','roll-sine','pitch-sine'))
def test_complete_diagnostic_dry_run_preserves_excitation_duration(test):
    config=build_diagnostic_config(BASE,inner='indi',test=test,ground_m=.03)
    result=simulate(config,ROOT/'loco_mjx')
    assert result['reason']=='test_complete' and result['completed']
    excitation=[row for row in result['records'] if row['output']['phase']=='diagnostic_pd']
    assert excitation[-1]['time_s']-excitation[0]['time_s']>=config['diagnostic']['duration_s']-.02-1e-9
    assert all(row['output']['indi_requested'] for row in result['records'] if row['output']['phase'] in ('takeoff','diagnostic_pd','diagnostic_settle','landing'))


@pytest.mark.parametrize('inner',('pid','indi'))
@pytest.mark.parametrize('test',('pd-hover','roll-pulse','pitch-pulse','yaw-pulse','roll-sine','pitch-sine','mpc-hover'))
def test_public_diagnostic_plans_select_only_host_schedule_without_transport_change(inner,test):
    config=build_diagnostic_config(BASE,inner=inner,test=test,ground_m=.03)
    mission=make_mission(config,ROOT/'loco_mjx')
    assert mission.ground.start_climb_on_ack is (inner=='indi')
    assert mission.ground.thrust_weight==.4
    assert mission.ground.launch_boost_weight==(1.2 if inner=='indi' else 0.)
    assert not config.get('ground_support_transition',False)
    assert mission_plan(mission)


@pytest.mark.parametrize('pattern',('circle','lemniscate'))
def test_public_mpc_plan_has_pure_new_schedule_with_unchanged_model(pattern):
    config=build_config(BASE,pattern=pattern,inner='indi',speed=.8,ground_m=.03)
    mission=make_mission(config,ROOT/'loco_mjx')
    assert mission.ground.start_climb_on_ack and mission.ground.thrust_weight==.4 and mission.ground.launch_boost_weight==1.2
    assert config['mpc']['model']==BASE['mpc']['model'] and Path(config['mpc']['config']).name in ('sanity09_mpc.json','sanity15_mpc_indi_rate_model.json') and config['limits']['expected_control_period_us']==1000
    assert not config.get('ground_support_transition',False) and mission_plan(mission)
