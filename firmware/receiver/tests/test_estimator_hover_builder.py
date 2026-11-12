"""Builder CLI safety constraints and fail-closed resolved configuration checks."""
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/build_comms_firmware.py'
spec = importlib.util.spec_from_file_location('comms_builder', SCRIPT)
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


@pytest.fixture
def build_case(tmp_path, monkeypatch):
    base = tmp_path / 'base.config'
    base.write_text('CONFIG_IDF_TARGET="esp32s3"\nCONFIG_SANITY_UART_TX=4\nCONFIG_SANITY_UART_RX=3\n'
                    'CONFIG_SANITY_MSP_UNIFIED=y\nCONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240\n'
                    'CONFIG_COMPILER_OPTIMIZATION_PERF=y\n')
    out = tmp_path / 'build'
    common = ['--backend', 'zenoh', '--profile', 'tcp-shared', '--base-config', str(base),
              '--idf-path', str(tmp_path / 'idf'), '--pico-source', str(tmp_path / 'pico'),
              '--output', str(out)]
    calls = []
    overrides = {}

    def run(command, **kwargs):
        calls.append(command)
        if command[0] == 'bash':
            resolved = {}
            for line in (out / 'sdkconfig').read_text().splitlines():
                match = re.fullmatch(r'CONFIG_(\w+)=(.*)', line)
                if match:
                    key, value = match.groups()
                    resolved[key] = True if value == 'y' else json.loads(value)
                unset = re.fullmatch(r'# CONFIG_(\w+) is not set', line)
                if unset:
                    resolved[unset[1]] = False
            resolved.update(overrides)
            (out / 'config').mkdir()
            (out / 'config/sdkconfig.json').write_text(json.dumps(resolved))
            (out / 'sanity_zenoh_receiver.bin').write_bytes(b'test-only synthetic application')
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(builder.subprocess, 'run', run)

    def invoke(flags):
        monkeypatch.setattr(sys, 'argv', [str(SCRIPT)] + common + flags)
        builder.main()

    return invoke, out, calls, overrides


@pytest.mark.parametrize('flags', [
    ['--estimator-indi-hover'],
    ['--estimator-indi-hover', '--flight'],
    ['--estimator-indi-hover', '--estimator', '--mocap'],
    ['--estimator-indi-hover', '--flight', '--estimator'],
    ['--estimator-indi-hover', '--flight', '--estimator', '--mocap', '--enable-tuning'],
    ['--estimator-indi-hover', '--flight', '--estimator', '--mocap', '--hx711', '--hx711-profile', 'flight'],
    ['--estimator-indi-hover', '--flight', '--estimator', '--mocap', '--backend', 'udp-unicast'],
    ['--estimator-indi-hover', '--flight', '--estimator', '--mocap', '--profile', 'tcp-split'],
    ['--estimator-indi-hover', '--flight', '--estimator', '--mocap', '--profile', 'udp-split'],
    ['--estimator-indi-hover', '--estimator-pid-hover', '--flight', '--estimator', '--mocap'],
    ['--estimator', '--mocap', '--flight'],
    ['--estimator-pid-hover', '--flight', '--estimator', '--mocap', '--enable-tuning'],
])
def test_invalid_estimator_arguments_fail_before_build_or_output(build_case, flags):
    invoke, out, calls, _ = build_case
    with pytest.raises(SystemExit) as failure:
        invoke(flags)
    assert failure.value.code == 2
    assert not calls
    assert not out.exists()


@pytest.mark.parametrize('hover', [None, 'pid', 'indi'])
@pytest.mark.parametrize('transport', ['tcp-shared', 'udp-shared'])
def test_each_estimator_profile_sets_explicit_exclusive_flags_and_receipt(build_case, hover, transport):
    invoke, out, calls, _ = build_case
    flags = ['--estimator', '--mocap', '--profile', transport]
    if hover:
        flags += ['--flight', '--estimator-' + hover + '-hover']
    invoke(flags)
    receipt = json.loads((out / 'receipt.json').read_text())
    assert receipt['test_only'] == (hover is None)
    assert receipt['estimator_pid_hover'] == (hover == 'pid')
    assert receipt['estimator_indi_hover'] == (hover == 'indi')
    assert receipt['estimator_experiment'] and receipt['mocap_stream']
    assert receipt['freertos_hz'] == 1000 and receipt['effective_uart_baud'] == 921600
    assert not receipt['estimator_controls_aircraft'] and not receipt['flashed']
    assert len(calls) == 2
    assert (out / 'sdkconfig').stat().st_mode & 0o777 == 0o600


@pytest.mark.parametrize('key,value', [
    ('SANITY_ESTIMATOR_EXPERIMENT', False), ('SANITY_ESTIMATOR_INDI_HOVER', False),
    ('SANITY_ESTIMATOR_PID_HOVER', True), ('SANITY_COMMS_TEST_ONLY', True),
    ('SANITY_MOCAP_STREAM', False), ('SANITY_MSP_UNIFIED', False),
    ('FREERTOS_HZ', 100), ('SANITY_RAW_UDP', True), ('SANITY_SPLIT_TELEMETRY', True),
    ('SANITY_MSP_TUNING', True), ('SANITY_HX711', True),
])
def test_indi_hover_rejects_unexpected_resolved_configuration(build_case, key, value):
    invoke, out, _, overrides = build_case
    overrides[key] = value
    with pytest.raises(SystemExit, match='Requested estimator configuration did not resolve'):
        invoke(['--flight', '--estimator', '--mocap', '--estimator-indi-hover'])
    assert not (out / 'receipt.json').exists()


def test_paper_native_profile_sets_exclusive_flags_and_receipt(build_case):
    invoke,out,_,_=build_case
    invoke(['--paper-native','--flight','--estimator','--mocap'])
    receipt=json.loads((out/'receipt.json').read_text())
    assert receipt['paper_native'] and receipt['estimator_experiment']
    assert not receipt['estimator_pid_hover'] and not receipt['estimator_indi_hover']
    assert not receipt['test_only'] and receipt['effective_uart_baud']==921600


@pytest.mark.parametrize('legacy',['SANITY_ESTIMATOR_PID_HOVER','SANITY_ESTIMATOR_INDI_HOVER'])
def test_paper_native_rejects_resolved_legacy_hover_selection(build_case,legacy):
    invoke,out,_,overrides=build_case
    overrides[legacy]=True
    with pytest.raises(SystemExit,match='Requested estimator configuration did not resolve'):
        invoke(['--paper-native','--flight','--estimator','--mocap'])
    assert not (out/'receipt.json').exists()


@pytest.mark.parametrize('flags',[
    ['--paper-native'],['--paper-native','--flight'],
    ['--paper-native','--flight','--estimator','--mocap','--estimator-indi-hover'],
    ['--paper-native','--flight','--estimator','--mocap','--profile','udp-shared'],
])
def test_paper_native_rejects_incompatible_flags(build_case,flags):
    invoke,out,calls,_=build_case
    with pytest.raises(SystemExit) as error:invoke(flags)
    assert error.value.code==2 and not calls and not out.exists()
