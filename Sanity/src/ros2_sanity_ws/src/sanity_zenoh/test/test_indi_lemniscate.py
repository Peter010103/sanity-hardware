from sanity_zenoh.standalone_paths import load_json
import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from sanity_zenoh.indi_hover import make_mission
from sanity_zenoh.indi_lemniscate import AngularLemniscate, LemniscateIndiMission, QuinticTransfer

ROOT = next(path for path in Path(__file__).resolve().parents if (path / 'loco_mjx').is_dir())
CONFIG = load_json(Path(__file__).resolve().parents[1] / 'config/sanity15_full_indi_lemniscate_1khz.json')


def airborne(stage='trajectory_entry'):
    mission = make_mission(copy.deepcopy(CONFIG), ROOT / 'loco_mjx')
    mission.home = np.array([.7, -.2, mission.limits.ground_m + 1.])
    mission.reference = mission.reference_factory(mission.limits.ground_m, mission.home[2], .5, 1.5, 5.)
    mission.first_tick = 0.;mission.last_tick = 9.98;mission.armed_at = 1.;mission.ever_armed = True
    mission._phase(stage, 10.)
    mission.pd.liftoff.complete = True
    center = np.r_[mission.trajectory_options.center_xy_m, mission.home[2]]
    if stage == 'transfer_to_center':mission.segment = QuinticTransfer(mission.home, center)
    if stage == 'lemniscate':mission.segment = AngularLemniscate(center)
    if stage == 'return_home':mission.segment = QuinticTransfer(center, mission.home)
    state = np.zeros(13);state[3] = 1.;state[:3] = mission._flight_reference(10.)['position']
    control = dict(fc_uid_msp=mission.limits.expected_uid, permission=True, supported=True,
                   armed=True, mode='indi', indi_weight=1., requested=True, eligible=True,
                   fault_latched=False, fault='ok', reason='ok', rpm_mask=15)
    return mission, state, control


def test_analytic_reference_derivatives_and_limits():
    curve = AngularLemniscate([0., 0., 1.])
    assert curve.duration == pytest.approx(2 * math.pi * math.sqrt(2) + 2.)
    for time in np.linspace(.001, curve.duration - .001, 301):
        h = 1e-5
        current, lower, upper = curve.sample(time), curve.sample(time-h), curve.sample(time+h)
        np.testing.assert_allclose((np.array(upper['position'])-lower['position'])/(2*h), current['velocity'], atol=2e-8)
        np.testing.assert_allclose((np.array(upper['velocity'])-lower['velocity'])/(2*h), current['acceleration'], atol=2e-8)
        np.testing.assert_allclose((np.array(upper['acceleration'])-lower['acceleration'])/(2*h), current['jerk'], atol=2e-7)
    samples=[curve.sample(time) for time in np.linspace(0., curve.duration, 5001)]
    assert max(np.linalg.norm(sample['velocity']) for sample in samples) <= 1. + 1e-12
    assert max(np.linalg.norm(sample['acceleration']) for sample in samples) < 1.064
    assert max(np.linalg.norm(sample['jerk']) for sample in samples) < 2.83


@pytest.mark.parametrize('join', [0., 2., 2 * math.pi * math.sqrt(2), 2 * math.pi * math.sqrt(2) + 2.])
def test_lap_joins_are_c2_and_exactly_centered(join):
    curve = AngularLemniscate([0., 0., 1.]);a=curve.sample(join-1e-7);b=curve.sample(join+1e-7)
    for field in ('position','velocity','acceleration'):
        np.testing.assert_allclose(a[field], b[field], atol=1e-6)
    if join in (0., curve.duration):
        assert curve.sample(join)['position'] == [0., 0., 1.]
        assert curve.sample(join)['velocity'] == curve.sample(join)['acceleration'] == [0.,0.,0.]


def test_quintic_transfer_speed_and_c2_endpoints():
    segment=QuinticTransfer([.7,-.2,1.],[0.,0.,1.])
    for elapsed,point in ((0.,segment.start),(segment.duration,segment.end)):
        sample=segment.sample(elapsed)
        np.testing.assert_allclose(sample['position'],point,atol=1e-15)
        assert sample['velocity']==sample['acceleration']==[0.,0.,0.]
    assert max(np.linalg.norm(segment.sample(time)['velocity']) for time in np.linspace(0.,segment.duration,501))<=.4+1e-12


def test_three_continuous_laps_preserve_c2_internal_joins_and_only_stop_at_end():
    curve=AngularLemniscate([0.,0.,1.],peak_speed=.8,laps=3)
    assert curve.duration==pytest.approx(6.*math.pi/curve.omega+2.)
    assert curve.sample(0.)['completed_laps']==0
    for completed in (1,2):
        crossing=2.*math.pi*completed/curve.omega+1.
        lower,upper=curve.sample(crossing-1e-7),curve.sample(crossing+1e-7)
        for field in ('position','velocity','acceleration'):
            np.testing.assert_allclose(lower[field],upper[field],atol=1e-6)
        assert np.linalg.norm(curve.sample(crossing)['velocity'])==pytest.approx(.8)
        assert upper['completed_laps']==completed
    final=curve.sample(curve.duration)
    assert final['theta_rad']==6.*math.pi and final['completed_laps']==3 and final['lap_progress']==3.
    assert final['position']==[0.,0.,1.] and final['velocity']==final['acceleration']==[0.,0.,0.]


@pytest.mark.parametrize('laps',[0,-1,1.5,True,'3'])
def test_invalid_lap_counts_are_rejected_before_arming(laps):
    config=copy.deepcopy(CONFIG);config['trajectory']['laps']=laps
    with pytest.raises(ValueError):make_mission(config,ROOT/'loco_mjx')


@pytest.mark.parametrize('stage', ['trajectory_entry','transfer_to_center','lemniscate','return_home','trajectory_hold'])
def test_repeated_reference_samples_are_pure_and_detached(stage):
    mission, _, _=airborne(stage);before=(mission.phase,mission.phase_started,mission.trajectory_stable_since,id(mission.segment))
    expected=mission._flight_reference(10.2)
    for time in (10.2,10.3,10.1,10.2):mission._flight_reference(time)
    changed=mission._flight_reference(10.2);changed['position'][0]=999
    assert mission._flight_reference(10.2)==expected
    assert before==(mission.phase,mission.phase_started,mission.trajectory_stable_since,id(mission.segment))


def test_stable_entry_and_perfect_reference_route_return_and_settle_before_landing():
    mission,state,control=airborne();phases=[]
    for number in range(2200):
        now=10.+number*.02
        reference=mission._flight_reference(now)
        state[:3]=reference['position'];state[7:10]=reference['velocity']
        output=mission.tick(now,state,control)
        if not phases or phases[-1]!=output['phase']:phases.append(output['phase'])
        if output['phase']=='landing':break
        assert output['indi_requested'] and max(abs(x) for x in output['rates_rad_s'])<=.6
    assert phases==['trajectory_entry','transfer_to_center','lemniscate','return_home','trajectory_hold','landing']
    assert mission.reason=='test_complete'


def test_entry_and_return_timeout_land_with_distinct_reason():
    for stage,reason in (('trajectory_entry','trajectory_entry_timeout'),('trajectory_hold','return_not_settled')):
        mission,state,control=airborne(stage);state[0]+=.2
        for number in range(503):
            output=mission.tick(10.+number*.02,state,control)
            if output['phase']=='landing':break
        assert output['phase']=='landing' and mission.reason==reason


def test_entry_stability_requires_a_fresh_continuous_dwell():
    mission,state,control=airborne()
    for number in range(26):mission.tick(10.+number*.02,state,control)
    state[7]=.3;mission.tick(10.52,state,control)
    assert mission.trajectory_stable_since is None
    state[7]=0.
    for number in range(27,77):
        mission.tick(10.+number*.02,state,control)
        assert mission.phase=='trajectory_entry'
    mission.tick(11.54,state,control)
    assert mission.phase=='transfer_to_center'


@pytest.mark.parametrize('stage', ['trajectory_entry','transfer_to_center','lemniscate','return_home','trajectory_hold'])
@pytest.mark.parametrize('fault', ['stale','authority','operator','late'])
def test_base_fault_stop_and_landing_routes_are_retained(stage,fault):
    mission,state,control=airborne(stage);kwargs={};now=10.
    if fault=='stale':kwargs['mocap_age_s']=.04
    if fault=='authority':control.update(mode='pid',indi_weight=0.)
    if fault=='operator':mission.request_landing('operator_landing')
    if fault=='late':now=10.1
    output=mission.tick(now,state,control,**kwargs)
    if fault=='stale':assert output['phase']=='stopped' and not output['enabled']
    else:
        assert output['phase']=='landing'
        assert output['indi_requested'] is (fault!='authority')


def test_hard_position_is_checked_against_moving_reference_before_operator_landing():
    mission,state,control=airborne('lemniscate');state[0]+=.76;mission.request_landing()
    output=mission.tick(10.,state,control)
    assert not output['enabled'] and mission.reason=='hard_flight_envelope'


def test_trajectory_requires_explicit_full_profile_and_unknown_kind_is_rejected():
    assert isinstance(make_mission(CONFIG,ROOT/'loco_mjx'),LemniscateIndiMission)
    assert make_mission(CONFIG,ROOT/'loco_mjx').trajectory_options.peak_speed_m_s==.8
    config=copy.deepcopy(CONFIG);config['trajectory']['kind']='unrecognized'
    with pytest.raises(ValueError):make_mission(config,ROOT/'loco_mjx')


@pytest.mark.parametrize('setting,value',[('radius_m',float('nan')),('center_xy_m',[0.]),('peak_speed_m_s',True),('ramp_s',100.),('unknown',1.)])
def test_invalid_trajectory_settings_are_rejected_before_arming(setting,value):
    config=copy.deepcopy(CONFIG);config['trajectory'][setting]=value
    with pytest.raises(ValueError):make_mission(config,ROOT/'loco_mjx')
    config=copy.deepcopy(CONFIG);config.pop('flight_profile');config.pop('liftoff_options');config.pop('spool_leveling')
    with pytest.raises(ValueError):make_mission(config,ROOT/'loco_mjx')
