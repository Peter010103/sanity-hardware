"""Causal INDI response model; no ROS, worker, network or aircraft access."""
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'loco_mjx').is_dir())
sys.path.insert(0, str(ROOT/'loco_mjx/experiments/sanity_ctbr_sim2sim'))
from indi_rate_model import (RateModelHistory, continuous_matrices,
                             discrete_matrices, response, validate_configuration)


def configuration():
    return {'enabled': True,
            'roll': {'wn_rad_s': 26.3306582162, 'zeta': .39412051785, 'zero_s': .00867346508},
            'pitch': {'wn_rad_s': 21.6381140155, 'zeta': .3811939339, 'zero_s': .01426921288},
            'yaw': {'tau_s': .125, 'identified': False}}


def test_off_is_ideal_and_enabled_requires_all_typed_finite_parameters():
    assert validate_configuration() == {'enabled': False}
    np.testing.assert_equal(response({}, [0., 1.]), np.ones((2, 3)))
    for c in [None, {'enabled': 1}, {'enabled': False, 'roll': {}},
              {'enabled': True}, {'enabled': True, 'unrecognized': 1}]:
        if c is None:
            continue
        with pytest.raises(ValueError):
            RateModelHistory(c)
    for axis, field, bad in [('roll', 'wn_rad_s', float('nan')),
                             ('pitch', 'zeta', 0.), ('roll', 'zero_s', -.001),
                             ('yaw', 'identified', 0), ('yaw', 'tau_s', True)]:
        c = configuration(); c[axis][field] = bad
        with pytest.raises(ValueError):
            validate_configuration(c)


def test_realization_is_stable_unit_dc_and_matches_analytic_frequency_response():
    c = configuration(); a, b, output = continuous_matrices(c)
    assert np.max(np.real(np.linalg.eigvals(a))) < 0
    np.testing.assert_allclose(-output @ np.linalg.solve(a, b), np.eye(3), atol=1e-12)
    for f in [0., .5, 1., 2., 3., 4., 10.]:
        measured = output @ np.linalg.solve(2j*np.pi*f*np.eye(6)-a, b)
        np.testing.assert_allclose(np.diag(measured), response(c, f), atol=1e-12)
        np.testing.assert_allclose(measured-np.diag(np.diag(measured)), 0., atol=1e-12)


def test_exact_zoh_has_semigroup_property_and_no_direct_command_jump():
    c = configuration(); a, b, output = discrete_matrices(c, .1)
    short_a, short_b, _ = discrete_matrices(c, .02)
    x = np.zeros(6); u = np.array([1., -.5, .2])
    for _ in range(5):
        x = short_a @ x + short_b @ u
    np.testing.assert_allclose(x, b @ u, atol=1e-12)
    zero_a, zero_b, _ = discrete_matrices(c, 0.)
    np.testing.assert_equal(zero_a, np.eye(6)); np.testing.assert_equal(zero_b, np.zeros((6, 3)))
    np.testing.assert_equal(output @ np.zeros(6), np.zeros(3))
    for dt in [-.1, float('nan'), True, 3.]:
        with pytest.raises(ValueError):
            discrete_matrices(c, dt)


def test_step_matches_independent_underdamped_closed_form_and_yaw_exponential():
    c = configuration(); h = RateModelHistory(c)
    h.reset(0); h.commit(0, [1., 1., 1.])
    for ns in [0, 10_000_000, 50_000_000, 100_000_000, 200_000_000]:
        t = ns*1e-9; expected = []
        for axis in ('roll', 'pitch'):
            p = c[axis]; wn = p['wn_rad_s']; zeta = p['zeta']; wd = wn*np.sqrt(1-zeta*zeta)
            e = np.exp(-zeta*wn*t)
            x1 = 1-e*(np.cos(wd*t)+zeta*wn/wd*np.sin(wd*t))
            x2 = e*wn*wn/wd*np.sin(wd*t)
            expected.append(x1+p['zero_s']*x2)
        expected.append(1-np.exp(-t/c['yaw']['tau_s']))
        np.testing.assert_allclose(h.output_at(ns), expected, atol=1e-12)


def test_history_uses_selected_command_only_and_does_not_revise_past():
    h = RateModelHistory(configuration()); h.reset(1_000_000_000)
    h.commit(1_020_000_000, [.8, -.2, 0.])
    past = h.output_at(1_030_000_000)
    # A different future selected command cannot affect an earlier prediction.
    h.commit(1_040_000_000, [-.6, .3, 0.])
    np.testing.assert_array_equal(h.output_at(1_030_000_000), past)
    np.testing.assert_array_equal(h.command_at(1_030_000_000), [.8, -.2, 0.])
    np.testing.assert_array_equal(h.command_at(1_040_000_000), [-.6, .3, 0.])
    before = h.state_at(1_050_000_000); before[:] = 100.
    assert np.max(h.state_at(1_050_000_000)) < 100.


def test_uninitialized_old_future_nonmonotonic_and_invalid_input_are_not_zeroed():
    h = RateModelHistory(configuration())
    with pytest.raises(ValueError): h.state_at(0)
    with pytest.raises(ValueError): h.commit(0, [0., 0., 0.])
    h.reset(1_000_000_000); h.commit(1_020_000_000, [0., 0., 0.])
    for stamp in [999_999_999, 1_270_000_001, True, 1.1]:
        with pytest.raises(ValueError): h.state_at(stamp)
    with pytest.raises(ValueError): h.commit(1_010_000_000, [0., 0., 0.])
    for rates in [[0., 0.], [0., float('nan'), 0.], [11., 0., 0.]]:
        with pytest.raises(ValueError): h.commit(1_040_000_000, rates)


def test_pruned_history_remains_causal_and_retains_earliest_checkpoint_state():
    h = RateModelHistory(configuration(), history_s=.25)
    h.reset(0)
    for step in range(30):
        h.commit(step*20_000_000, [1., 0., 0.])
    assert h.earliest_stamp_ns > 0 and h.diagnostics()['checkpoints'] < 16
    with pytest.raises(ValueError): h.state_at(0)
    assert h.output_at(h.latest_stamp_ns)[0] == pytest.approx(1., abs=.005)
    h.reset(600_000_000)
    np.testing.assert_array_equal(h.output_at(600_000_000), np.zeros(3))


def test_axis_coordinates_are_body_flu_and_no_gyro_assimilation_is_implied():
    h = RateModelHistory(configuration()); h.reset(0)
    h.commit(0, [0., 1., 0.])
    out = h.output_at(100_000_000)
    assert out[1] > 0 and out[0] == 0 and out[2] == 0
    assert h.diagnostics()['yaw_identified'] is False
    assert 'no rate measurement' in h.diagnostics()['source']
