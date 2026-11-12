from sanity_zenoh.standalone_paths import load_json
"""Prediction epochs and causal selected commands; no network/aircraft."""
from pathlib import Path
import sys,json
import numpy as np
import pytest
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'loco_mjx').is_dir())
sys.path.insert(0,str(ROOT/'loco_mjx/experiments/sanity_ctbr_sim2sim'))
from sanity_zenoh.command_prediction import CommandPrediction
CFG=load_json(ROOT / 'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity15_mpc_indi_rate_model.json')['rate_response']
BASE=10_000_000_000
STATE=np.r_[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]

def test_corrected_hover_and_timestamped_force_history_keep_state_stationary():
    p=CommandPrediction(CFG,.07151,.953);p.commit(BASE,[.07151*9.81/.953,0,0,0])
    state,hidden=p.predict(STATE,BASE+5_000_000,BASE+45_000_000)
    np.testing.assert_allclose(state,STATE,atol=1e-10)
    np.testing.assert_allclose(hidden,0,atol=1e-10)
    assert p.last['projection_s']==.04


def test_future_selected_command_cannot_change_earlier_projection():
    p=CommandPrediction(CFG,.07151,.953);p.commit(BASE,[.7,.1,0,0])
    earlier,_=p.predict(STATE,BASE+5_000_000,BASE+15_000_000)
    p.commit(BASE+20_000_000,[.8,-.6,.5,0])
    repeated,_=p.predict(STATE,BASE+5_000_000,BASE+15_000_000)
    np.testing.assert_array_equal(earlier,repeated)
    np.testing.assert_array_equal(p.held(BASE-1),np.zeros(4))


def test_measurement_feedback_reanchors_position_on_every_solve():
    p=CommandPrediction(CFG,.07151,.953);p.commit(BASE,[.07151*9.81/.953,0,0,0])
    shifted=STATE.copy();shifted[:3]+=[2,3,.1]
    state,_=p.predict(shifted,BASE+5_000_000,BASE+25_000_000)
    np.testing.assert_allclose(state[:3],shifted[:3],atol=1e-10)


@pytest.mark.parametrize('source,target',[(BASE,BASE-1),(BASE,BASE+60_010_000)])
def test_projection_does_not_bridge_stale_input_or_reverse_time(source,target):
    p=CommandPrediction(CFG,.07151,.953)
    with pytest.raises(ValueError):p.predict(STATE,source,target)


def test_rate_states_use_selected_rate_commands_and_proper_axes():
    p=CommandPrediction(CFG,.07151,.953);p.commit(BASE,[.75,.2,0,0])
    state,hidden=p.predict(STATE,BASE,BASE+20_000_000)
    assert state[4]>0 and state[5]==pytest.approx(0,abs=1e-10)
    assert hidden[0]>0 and hidden[1]>0
    assert abs(hidden[2])<1e-12 and abs(hidden[3])<1e-12


def test_explicit_transport_stage_shifts_command_application_without_stale_pose_permission():
    p=CommandPrediction(CFG,.07151,.953,command_delay_s=.015)
    p.commit(BASE,[.74,.3,0,0])
    np.testing.assert_equal(p.held(BASE+14_000_000),np.zeros(4))
    assert p.held(BASE+15_000_000)[1]==.3
    _,hidden=p.predict(STATE,BASE+5_000_000,BASE+35_000_000)
    assert hidden[0]>0
    with pytest.raises(ValueError):p.predict(STATE,BASE,BASE+76_000_000)


def test_effective_drag_is_included_in_projection_without_hidden_measurement_filter():
    p=CommandPrediction(CFG,.07151,.953,drag=[.587,.820,0])
    p.commit(BASE,[.07151*9.81/.953,0,0,0])
    state=STATE.copy();state[7]=1.
    prediction,_=p.predict(state,BASE+5_000_000,BASE+25_000_000)
    assert prediction[7]==pytest.approx(np.exp(-.587*.02),rel=1e-7)
    assert p.last['source_state'][7]==1.
