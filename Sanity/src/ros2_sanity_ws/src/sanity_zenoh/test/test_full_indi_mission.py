from sanity_zenoh.standalone_paths import load_json
"""Full-INDI mission contract and failure behavior; offline, no ROS/devices."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from sanity_zenoh.firmware_collective import FirmwareCollective
from sanity_zenoh.full_indi_mission import FullIndiMission, GroundIndiLimits
from sanity_zenoh.indi_hover import make_mission, mission_plan, simulate

ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'loco_mjx').is_dir())
CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
CONFIG = load_json(CONFIG_DIR / 'sanity15_full_indi_estimator_hover.json')


def fixture():
    m = make_mission(copy.deepcopy(CONFIG), ROOT / 'loco_mjx')
    s = np.zeros(13);s[2] = m.limits.ground_m;s[3] = 1.
    c = dict(fc_uid_msp=m.limits.expected_uid, permission=True, supported=True,
             request_aux=3, mode='pid', indi_weight=0., requested=False,
             eligible=True, armed=False, fault_latched=False, fault='ok',
             control_period_us=500., gyro_native_rad_s=[0., 0., 0.], rpm_mask=15)
    return m, s, c


def ground_request():
    m, s, c = fixture()
    for i in range(500):
        now = i * .02
        o = m.tick(now, s, c)
        if o['phase'] == 'arming':c['armed'] = True
        if o['indi_requested']:
            assert o['phase'] == 'ground_indi_request'
            return m, s, c, now, o
    raise AssertionError('Ground request not reached')


def acknowledged():
    m, s, c, now, old = ground_request()
    c.update(mode='indi', indi_weight=1., requested=True)
    now += .02;o = m.tick(now, s, c)
    assert o['phase'] == 'takeoff' and o['indi_requested']
    return m, s, c, now, old, o


def airborne():
    m, s, c, now, _, _ = acknowledged()
    m._phase('indi_hover', now)
    s[:3] = m.home
    return m, s, c, now


@pytest.mark.parametrize('scale', [.98, 1., 1.02])
def test_complete_flight_uses_indi_before_climb_through_landing(scale):
    result = simulate(CONFIG, ROOT / 'loco_mjx', mass_scale=scale)
    assert result['reason'] == 'test_complete'
    assert result['phases'] == ['waiting', 'arming', 'ground_spinup', 'ground_indi_request', 'takeoff', 'indi_hover', 'landing', 'stopped']
    rows = result['records']
    first = next(i for i,r in enumerate(rows) if r['output']['phase'] == 'takeoff')
    assert rows[first]['controller']['mode'] == 'indi' and rows[first]['controller']['indi_weight'] == 1.
    assert rows[first]['state'][2] - CONFIG['limits']['ground_m'] <= CONFIG['ground_indi']['maximum_height_m']
    for r in rows:
        o = r['output'];assert abs(o['thrust_N']) <= .06947 * 9.81 * 1.5 + 1e-12
        assert max(abs(x) for x in o['rates_rad_s']) <= .6
        if o['phase'] in ('takeoff', 'indi_hover', 'landing'):
            assert o['enabled'] and o['indi_requested']
            assert r['controller']['mode'] == 'indi' and r['controller']['indi_weight'] == 1.
        if o['phase'] == 'ground_spinup':
            assert not o['indi_requested'] and o['thrust_N'] <= .06947 * 9.81 * .4 + 1e-12
    assert not rows[-1]['output']['enabled'] and rows[-1]['state'][2] == pytest.approx(CONFIG['limits']['ground_m'])
    hover = [r for r in rows if r['output']['phase'] == 'indi_hover']
    assert hover[-1]['time_s'] - hover[0]['time_s'] >= 19.98
    assert result['peak_height_m'] - CONFIG['limits']['ground_m'] == pytest.approx(1., abs=.02)


def test_ground_request_follows_armed_low_request_and_warmup_dwell():
    m, s, c, now, o = ground_request()
    assert m.ever_armed and c['armed'] and c['requested'] is False
    assert now - m.ground_started >= m.ground.spool_s + m.ground.warmup_s + m.ground.eligible_dwell_s
    assert o['thrust_N'] == pytest.approx(m.ground_thrust_N)
    assert m.indi_confirmed_at is None and m.armed_at is None
    assert s[2] == m.limits.ground_m


def test_no_takeoff_from_request_or_partial_weight_without_actual_ack():
    m, s, c, now, _ = ground_request()
    c.update(mode='to_indi', indi_weight=.999, requested=True)
    for i in range(1, 30):
        o = m.tick(now + i * .02, s, c)
        assert o['phase'] == 'ground_indi_request' and o['indi_requested']
        assert o['thrust_N'] <= m.ground_thrust_N + 1e-12
        assert m.armed_at is None


def test_ground_ack_keeps_collective_continuous_and_never_resets_to_zero_spool():
    m, s, c, now, before, o = acknowledged()
    assert o['thrust_N'] == pytest.approx(before['thrust_N'])
    assert m.armed_at == m.indi_confirmed_at == now
    for i in range(1, 70):
        o = m.tick(now + i * .02, s, c)
        assert o['indi_requested'] and o['thrust_N'] >= m.ground_thrust_N - 1e-12
        assert o['phase'] == 'takeoff'


@pytest.mark.parametrize('axis,angle', [('x', 5.), ('x', -5.), ('y', 5.), ('y', -5.)])
def test_post_ack_settle_and_spool_level_attitude_with_existing_liftoff_bounds(axis, angle):
    m, s, c, now, before, output = acknowledged()
    s[3:7] = Rotation.from_euler(axis, angle, degrees=True).as_quat()[[3, 0, 1, 2]]
    rate_axis = 0 if axis == 'x' else 1
    for i in range(1, 100):
        output = m.tick(now + i * .02, s, c)
        diagnostics = m.diagnostics(output)
        assert output['phase'] == 'takeoff' and output['indi_requested']
        assert output['rates_rad_s'][rate_axis] * angle < 0
        assert np.linalg.norm(output['rates_rad_s'][:2]) <= .15 + 1e-12
        assert output['rates_rad_s'][2] == 0.
        assert diagnostics['pd_evaluated_this_tick']
        assert diagnostics['pd']['horizontal_weight'] == 0.
        assert not diagnostics['pd']['integrating']
        assert diagnostics['pd']['liftoff']['phase'] == 'level_liftoff'
        assert diagnostics['pd']['selected_ctbr'] == diagnostics['selected_ctbr']
        assert diagnostics['integral_acceleration_state_m_s2'] == [0., 0., 0.]
        assert output['thrust_N'] >= before['thrust_N']


def test_post_ack_leveling_ignores_xy_motion_and_yaw_error_without_integrating():
    a, s, c, now, _, _ = acknowledged()
    b, other, control, time, _, _ = acknowledged()
    rotation = Rotation.from_euler('xyz', [8., -4., 35.], degrees=True)
    s[3:7] = other[3:7] = rotation.as_quat()[[3, 0, 1, 2]]
    other[:2] += [.2, -.2]
    other[7:9] = [.3, -.4]
    for i in range(1, 100):
        first = a.tick(now + i * .02, s, c)
        second = b.tick(time + i * .02, other, control)
        assert first['rates_rad_s'] == second['rates_rad_s']
        assert first['rates_rad_s'][2] == 0.
        assert np.linalg.norm(first['rates_rad_s'][:2]) == pytest.approx(.15)
        assert first['thrust_N'] == second['thrust_N']
        assert a.pd.integral_acceleration.tolist() == b.pd.integral_acceleration.tolist() == [0., 0., 0.]


@pytest.mark.parametrize('yaw', [0., 35., -120., 179.])
def test_post_ack_level_attitude_has_zero_rates_at_any_measured_yaw(yaw):
    m, s, c, now, _, _ = acknowledged()
    s[3:7] = Rotation.from_euler('z', yaw, degrees=True).as_quat()[[3, 0, 1, 2]]
    for i in range(1, 100):
        output = m.tick(now + i * .02, s, c)
        assert output['rates_rad_s'] == [0., 0., 0.]
        assert m.pd.integral_acceleration.tolist() == [0., 0., 0.]
        assert m.pd.last['selected_ctbr'] == [output['thrust_N'], *output['rates_rad_s']]


def test_level_feedback_starts_only_after_actual_full_indi_ack():
    m, s, c, now, _ = ground_request()
    s[3:7] = Rotation.from_euler('x', 5., degrees=True).as_quat()[[3, 0, 1, 2]]
    c.update(mode='to_indi', indi_weight=.8, requested=True)
    output = m.tick(now + .02, s, c)
    assert output['phase'] == 'ground_indi_request'
    assert output['rates_rad_s'] == [0., 0., 0.]
    assert not m.diagnostics(output)['pd_evaluated_this_tick']
    c.update(mode='indi', indi_weight=1.)
    output = m.tick(now + .04, s, c)
    assert output['phase'] == 'takeoff' and output['rates_rad_s'][0] < 0.
    assert m.diagnostics(output)['pd_evaluated_this_tick']


def test_post_ack_leveling_does_not_start_translational_blend_before_existing_clearance_rule():
    m, s, c, now, _, _ = acknowledged()
    s[0] += .2
    for i in range(1, 121):
        output = m.tick(now + i * .02, s, c)
        assert m.pd.last['horizontal_weight'] == 0.
        assert m.pd.last['liftoff']['phase'] == 'level_liftoff'
        assert not m.pd.last['integrating']
    # Five centimeters of measured clearance, then the original continuous
    # 100ms airborne dwell, are still required before horizontal blending.
    s[2] = m.limits.ground_m + .051
    for i in range(121, 126):
        m.tick(now + i * .02, s, c)
        assert m.pd.last['horizontal_weight'] == 0.
    for i in range(126, 130):
        m.tick(now + i * .02, s, c)
    assert m.pd.last['liftoff']['phase'] == 'horizontal_blend'
    assert 0. < m.pd.last['horizontal_weight'] < 1.


@pytest.mark.parametrize('fault, reason', [('entry_denied', 'ground_indi_entry_not_ready'), ('engagement_unacknowledged', 'ground_indi_engagement_not_acknowledged')])
def test_denied_ground_engagement_never_climbs_and_disables(fault, reason):
    result = simulate(CONFIG, ROOT / 'loco_mjx', fault=fault)
    assert result['reason'] == reason
    assert 'takeoff' not in result['phases'] and 'indi_hover' not in result['phases']
    assert result['peak_height_m'] == pytest.approx(CONFIG['limits']['ground_m'])
    assert not result['records'][-1]['output']['enabled']


def test_unintended_ground_lift_disables_even_if_indi_ack_arrives_same_tick():
    m, s, c, now, _ = ground_request()
    s[2] += .041;c.update(mode='indi', indi_weight=1., requested=True)
    o = m.tick(now + .02, s, c)
    assert not o['enabled'] and not o['indi_requested']
    assert m.reason == 'ground_lift_before_indi_ack' and m.indi_confirmed_at is None


def test_missing_rpm_does_not_request_indi_on_ground():
    m, s, c = fixture();c['rpm_mask'] = 7
    for i in range(600):
        o = m.tick(i * .02, s, c)
        if o['phase'] == 'arming':c['armed'] = True
        assert not o['indi_requested']
    assert m.reason == 'ground_indi_entry_not_ready'


@pytest.mark.parametrize('fault, reason', [('rpm_loss', 'fc_indi_fault_rpm'), ('indi_authority_loss', 'indi_authority_lost')])
def test_native_fault_fallback_can_land_in_pid_without_planned_healthy_return(fault, reason):
    result = simulate(CONFIG, ROOT / 'loco_mjx', fault=fault)
    assert result['reason'] == reason and result['completed']
    assert 'return_pid' not in result['phases']
    assert result['phases'][-2:] == ['landing', 'stopped']
    landing = [r for r in result['records'] if r['output']['phase'] == 'landing']
    assert landing and all(not r['output']['indi_requested'] for r in landing)
    assert not result['records'][-1]['output']['enabled']


def test_operator_or_soft_envelope_landing_retains_indi_request():
    m, s, c, now = airborne();m.request_landing()
    o = m.tick(now + .02, s, c)
    assert o['phase'] == 'landing' and o['indi_requested'] and m.reason == 'operator_landing'
    m, s, c, now = airborne();s[0] += m.limits.soft_position_m + .01
    o = m.tick(now + .02, s, c)
    assert o['phase'] == 'landing' and o['indi_requested'] and m.reason == 'soft_flight_envelope'


def test_fault_during_planned_landing_changes_outcome_and_does_not_restart_reference():
    m, s, c, now = airborne();m.phase_started = now - m.limits.indi_hold_s - .02
    o = m.tick(now + .02, s, c);assert o['phase'] == 'landing' and o['indi_requested']
    old = m.landing;c.update(mode='pid', indi_weight=0., fault_latched=True, fault='rpm')
    o = m.tick(now + .04, s, c)
    assert o['phase'] == 'landing' and not o['indi_requested']
    assert m.landing is old and m.reason == 'fc_indi_fault_rpm'


@pytest.mark.parametrize('loss', [dict(mocap_age_s=.04), dict(controller_age_s=.3), dict(receiver_ready=False)])
def test_stale_live_measurements_disable_immediately(loss):
    m, s, c, now = airborne();o = m.tick(now + .02, s, c, **loss)
    assert not o['enabled'] and m.reason == 'state_or_receiver_unavailable'


def test_hard_tilt_and_unexpected_disarm_keep_existing_disable_semantics():
    m, s, c, now = airborne();s[3:7] = Rotation.from_euler('x', 36, degrees=True).as_quat()[[3, 0, 1, 2]]
    assert not m.tick(now + .02, s, c)['enabled'] and m.reason == 'hard_flight_envelope'
    m, s, c, now = airborne();c['armed'] = False
    assert not m.tick(now + .02, s, c)['enabled'] and m.reason == 'unexpected_fc_disarm'


@pytest.mark.parametrize('landing_trigger', ['authority_loss', 'operator'])
def test_landing_transition_cannot_hide_existing_hard_position_error(landing_trigger):
    m,s,c,now=airborne();s[0]+=.76
    if landing_trigger=='authority_loss':c.update(mode='pid',indi_weight=0.)
    else:m.request_landing()
    assert not m.tick(now+.02,s,c)['enabled'] and m.reason=='hard_flight_envelope'


def test_startup_uses_fc_gyro_and_legacy_fallback():
    m, s, c = fixture();s[10:13] = [3, 0, 0]
    for i in range(52):o = m.tick(i * .02, s, c)
    assert o['enabled'] and m.phase == 'arming'
    m, s, c = fixture();c.pop('gyro_native_rad_s');s[10:13] = [3, 0, 0]
    for i in range(52):assert not m.tick(i * .02, s, c)['enabled']


def test_profile_is_explicit_and_existing_hybrid_remains_separate():
    assert isinstance(make_mission(CONFIG, ROOT / 'loco_mjx'), FullIndiMission)
    assert 'ground INDI request and actual mode/weight acknowledgement' in mission_plan(make_mission(CONFIG, ROOT / 'loco_mjx'))
    hybrid = load_json(CONFIG_DIR / 'sanity15_indi_estimator_hover.json')
    assert not isinstance(make_mission(hybrid, ROOT / 'loco_mjx'), FullIndiMission)
    invalid = copy.deepcopy(CONFIG);invalid['limits']['pid_only'] = True
    with pytest.raises(ValueError):make_mission(invalid, ROOT / 'loco_mjx')
    invalid = copy.deepcopy(CONFIG);invalid['estimator_logging']['flight_profile'] = 'indi_hover'
    with pytest.raises(ValueError):make_mission(invalid, ROOT / 'loco_mjx')


def test_ground_collective_profile_remains_above_native_idle_threshold():
    profile = FirmwareCollective(CONFIG_DIR / 'sanity15_indi_collective.json')
    thrust = CONFIG['ground_indi']['thrust_weight'] * CONFIG['limits'].get('mass_kg', .06947) * 9.81
    for voltage in np.linspace(6.6, 8.5, 40):
        profile.set_voltage(float(voltage))
        assert profile.convert(15, True, thrust, (0., 0., 0.)).throttle >= 1100


def test_actual_profile_landing_spool_never_releases_indi_before_touchdown():
    result=simulate(CONFIG,ROOT/'loco_mjx')
    profile=FirmwareCollective(CONFIG_DIR/'sanity15_indi_collective.json');profile.set_voltage(8.)
    landing=[r for r in result['records'] if r['output']['phase']=='landing']
    assert any(r['mission_diagnostics']['touchdown_latched'] for r in landing)
    for r in landing:
        o=r['output'];rc=profile.convert(15,o['enabled'],o['thrust_N'],o['rates_rad_s']).throttle
        if not r['mission_diagnostics']['touchdown_latched']:
            assert rc>=1100 and r['controller']['mode']=='indi' and r['controller']['indi_weight']==1.
        if rc<1100:assert r['mission_diagnostics']['touchdown_latched']
    assert result['records'][-1]['mission_diagnostics']['touchdown_latched']


def test_native_idle_is_normal_only_after_latched_ground_touchdown():
    result=simulate(CONFIG,ROOT/'loco_mjx',fault='idle_after_touchdown')
    assert result['reason']=='test_complete'
    last=result['records'][-1]
    assert last['controller']['reason']=='idle' and last['mission_diagnostics']['touchdown_latched']
    assert last['controller']['sim_rc_throttle']<1100
    assert not last['output']['enabled']
    result=simulate(CONFIG,ROOT/'loco_mjx',fault='idle_before_touchdown')
    assert result['reason']=='indi_authority_lost'
    first=next(r for r in result['records'] if r['output']['phase']=='landing')
    assert not first['mission_diagnostics']['touchdown_latched'] and not first['output']['indi_requested']


@pytest.mark.parametrize('settings', [dict(thrust_weight=.7), dict(maximum_height_m=.06), dict(warmup_s=.1), dict(eligible_dwell_s=.1)])
def test_invalid_ground_limits_rejected(settings):
    with pytest.raises(ValueError):GroundIndiLimits(**settings)


@pytest.mark.parametrize('age,enabled',[(.149,True),(.2,True),(.299,True),(.301,False)])
def test_full_indi_tolerates_brief_diagnostic_status_gaps(age,enabled):
    m,s,c,now=airborne()
    out=m.tick(now+.02,s,c,controller_age_s=age)
    assert out['enabled']==enabled
    if enabled:assert out['indi_requested']
    else:assert m.reason=='state_or_receiver_unavailable'


@pytest.mark.parametrize('offset,phase,enabled',[(.5,'indi_hover',True),(.61,'landing',True),(.76,'stopped',False)])
def test_relaxed_soft_tracking_limit_keeps_hard_envelope(offset,phase,enabled):
    m,s,c,now=airborne();s[0]+=offset
    out=m.tick(now+.02,s,c)
    assert out['phase']==phase and out['enabled']==enabled


@pytest.mark.parametrize('fault',[dict(mocap_age_s=.04),dict(receiver_ready=False)])
def test_diagnostic_tolerance_does_not_relax_live_control_feedback(fault):
    m,s,c,now=airborne();out=m.tick(now+.02,s,c,controller_age_s=.2,**fault)
    assert not out['enabled'] and m.reason=='state_or_receiver_unavailable'


def test_logging_only_estimator_fault_does_not_gate_fresh_recording_stream():
    from sanity_zenoh.indi_hover import observation_logging_ready
    snap=dict(ready=False,records=20,dropped=0,error=None,state_record_age_s=.1,latest=dict(epoch=123,flags=6,fault=8))
    assert observation_logging_ready(snap,False,123)
    assert not observation_logging_ready(snap,True,123)
    assert not observation_logging_ready(snap,False,124)
    for key,value in [('state_record_age_s',.3),('dropped',1),('error','CRC error')]:
        bad=dict(snap);bad[key]=value
        assert not observation_logging_ready(bad,False,123)
