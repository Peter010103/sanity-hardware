"""Standalone previews and execution gates must work without lab devices."""
import copy
import json
from pathlib import Path
import sys

import pytest
import full_campaign as runner

EXAMPLES = Path(__file__).resolve().parent.parent / 'examples'


def no_device(*args, **kwargs):
    raise AssertionError('Preview or rejected execution attempted device access')


@pytest.fixture
def guarded(monkeypatch):
    monkeypatch.setattr(runner.lib, 'instantiate_tenma_class_from_device_response', no_device)
    monkeypatch.setattr(runner.t.prior, 'snapshot', no_device)
    monkeypatch.setattr(runner.t.Group, 'open', no_device)
    monkeypatch.setattr(runner, 'CONFIG_PREPARE', None)


@pytest.mark.parametrize('name', ['probe', 'direct-stage'])
def test_preview_needs_no_devices_or_local_calibration(monkeypatch, guarded, name):
    monkeypatch.setattr(sys, 'argv', ['full_campaign.py', str(EXAMPLES / (name + '.json'))])
    assert runner.main() == 0


@pytest.mark.parametrize('flags', [[], ['--battery-disconnected'], ['--rear-terminals-confirmed']])
def test_execution_needs_current_connection_declarations(monkeypatch, guarded, flags):
    monkeypatch.setattr(sys, 'argv', ['full_campaign.py', str(EXAMPLES / 'probe.json'), '--execute', *flags])
    with pytest.raises(SystemExit) as error:
        runner.main()
    assert error.value.code == 2


def test_direct_entrypoint_rejects_acro_plan(monkeypatch, guarded):
    monkeypatch.setattr(sys, 'argv', ['full_campaign.py', str(EXAMPLES / 'acro-stage.json')])
    with pytest.raises(SystemExit) as error:
        runner.main()
    assert error.value.code == 2


def test_malformed_plans_cannot_escape_output_root_or_issue_probe_commands():
    plan = json.loads((EXAMPLES / 'direct-stage.json').read_text())
    for change in ({'label': '../elsewhere'}, {'kind': 'probe'}, {'control_mode': 'unknown'}):
        with pytest.raises(ValueError):
            runner.verify_plan(dict(plan, **change))
    for value in (float('nan'), float('inf'), -1):
        invalid = copy.deepcopy(plan)
        invalid['phases'][0]['duration_s'] = value
        with pytest.raises(ValueError):
            runner.verify_plan(invalid)
