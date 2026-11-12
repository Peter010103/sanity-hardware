"""Native acceptance and MSP output for disarmed, PID and INDI logging profiles."""
import ctypes as C
from pathlib import Path
import subprocess
import pytest
from sanity_zenoh.protocol import CTBR, DIRECT, ReceiverStatus, command_packet, neutral, si_command_packet

ROOT = Path(__file__).resolve().parents[1]
TEST_ONLY = 'SANITY_COMMS_TEST_ONLY'
PID = 'SANITY_ESTIMATOR_PID_HOVER'
INDI = 'SANITY_ESTIMATOR_INDI_HOVER'


@pytest.fixture(params=[TEST_ONLY, PID, INDI])
def profile(request, tmp_path):
    output = tmp_path / 'profile.so'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all', '-shared', '-fPIC',
                    '-DCONFIG_' + request.param, '-I', str(ROOT / 'main'),
                    str(ROOT / 'main/control.c'), str(ROOT / 'main/bench.c'),
                    str(ROOT / 'tests/wrapper.c'), '-o', str(output)], check=True)
    lib = C.CDLL(str(output))
    lib.test_reset.argtypes = [C.c_uint, C.c_ulonglong]
    lib.test_receive_msp.argtypes = [C.c_char_p, C.c_uint, C.c_uint, C.c_int]
    lib.test_receive.argtypes = [C.c_char_p, C.c_uint, C.c_uint]
    lib.test_reset(CTBR, 123)
    return request.param, lib


def packet(seq=0, enabled=False, request=False, selection=True, lease=1000):
    return si_command_packet(1, enabled, 123, 9, seq, lease, 100, (0, 0, 0, 0),
                             0xf04fc735, controller_selection=selection,
                             indi_requested=request)


def send(lib, p, now=1000):
    return lib.test_receive_msp(p, len(p), now, 1)


def channels(lib, now=1000):
    out = (C.c_ubyte * 22)()
    assert lib.test_output_msp(now, out) == 22
    return [int.from_bytes(bytes(out)[5 + 2*i:7 + 2*i], 'little') for i in range(8)]


def test_profiles_require_disabled_claim_and_expire_active_output(profile):
    name, lib = profile
    assert not send(lib, packet(0, True))
    assert send(lib, packet())
    assert bool(send(lib, packet(1, True))) == (name != TEST_ONLY)
    values = (C.c_ushort * 4)()
    lib.test_output(1100, values)
    assert not lib.test_enabled()
    assert list(values) == [1500, 1500, 1500, 1000]
    assert not send(lib, packet(2, True, lease=1100), 1100)
    assert send(lib, packet(3, lease=1100), 1100)
    assert bool(send(lib, packet(4, True, lease=1100), 1100)) == (name != TEST_ONLY)


def test_only_indi_profile_accepts_indi_request(profile):
    name, lib = profile
    assert send(lib, packet())
    assert bool(send(lib, packet(1, True, True))) == (name == INDI)
    assert bool(lib.test_enabled()) == (name == INDI)
    output = channels(lib)
    assert output[4] == (2000 if name == INDI else 1000)
    assert output[6] == (2000 if name == INDI else 1000)


@pytest.mark.parametrize('receive', ['test_receive', 'test_receive_msp'])
@pytest.mark.parametrize('legacy', ['szc1', 'szc2', 'szc3', 'direct'])
@pytest.mark.parametrize('profile', [PID, INDI], indirect=True)
def test_hover_profiles_reject_legacy_and_direct_before_mode_mutation(profile, receive, legacy):
    _, lib = profile
    if legacy == 'direct':
        p = command_packet(1, DIRECT, False, 123, 9, 0, 1000, 100, neutral(DIRECT))
    elif legacy == 'szc1':
        p = command_packet(1, CTBR, False, 123, 9, 0, 1000, 100, neutral(CTBR))
    elif legacy == 'szc3':
        p = si_command_packet(1, False, 123, 9, 0, 1000, 100, (0, 0, 0, 0),
                              0xf04fc735, high_rate=True)
    else:
        p = packet(selection=False)
    args = (p, len(p), 1000) + ((1,) if receive == 'test_receive_msp' else ())
    assert not getattr(lib, receive)(*args)
    out = (C.c_ubyte * 48)()
    lib.test_status(1000, out)
    status = ReceiverStatus.decode(bytes(out))
    assert status.mode == CTBR and status.epoch == 123
    assert send(lib, packet())


def test_native_accept_enforces_enabled_guard_without_network_wrapper(profile):
    name, lib = profile
    p = packet()
    assert lib.test_receive(p, len(p), 1000)
    p = packet(1, True, True)
    assert bool(lib.test_receive(p, len(p), 1000)) == (name == INDI)


@pytest.mark.parametrize('stop', ['disable', 'expiry'])
@pytest.mark.parametrize('profile', [INDI], indirect=True)
def test_indi_request_and_arm_go_low_on_disable_or_watchdog(profile, stop):
    _, lib = profile
    assert send(lib, packet())
    assert send(lib, packet(1, True, True))
    assert channels(lib)[6] == 2000
    if stop == 'disable':
        assert send(lib, packet(2))
        output = channels(lib)
    else:
        output = channels(lib, 1100)
    assert output[4] == output[6] == 1000
    assert not lib.test_enabled()


@pytest.mark.parametrize('mode', [CTBR, DIRECT])
@pytest.mark.parametrize('profile', [TEST_ONLY], indirect=True)
def test_disarmed_profile_rejects_enabled_legacy_and_direct_claims(profile, mode):
    _, lib = profile
    lib.test_reset(mode, 123)
    claim = command_packet(1, mode, False, 123, 9, 0, 1000, 100, neutral(mode))
    assert send(lib, claim)
    enabled = command_packet(1, mode, True, 123, 9, 1, 1000, 100, neutral(mode))
    assert not send(lib, enabled)
    assert not lib.test_enabled()


@pytest.mark.parametrize('profile', [PID, INDI], indirect=True)
def test_hover_lease_replay_rejection_and_active_direct_rejection(profile):
    name, lib = profile
    assert send(lib, packet())
    assert send(lib, packet(1, True, name == INDI))
    assert not send(lib, packet(1, True, name == INDI))
    stale = packet(2, True, name == INDI, lease=900)
    assert not send(lib, stale)
    direct = command_packet(1, DIRECT, False, 123, 9, 2, 1000, 100, neutral(DIRECT))
    assert not send(lib, direct)
    out = (C.c_ubyte * 48)()
    lib.test_status(1000, out)
    status = ReceiverStatus.decode(bytes(out))
    assert status.mode == CTBR and status.epoch == 123
    assert lib.test_enabled()
    assert channels(lib)[6] == (2000 if name == INDI else 1000)
