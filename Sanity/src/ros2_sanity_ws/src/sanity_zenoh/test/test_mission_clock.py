from sanity_zenoh.standalone_paths import load_json
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from sanity_zenoh.mission_clock import record_clock_preflight

ROOT = Path(__file__).resolve().parents[6]
SCRIPTS = ROOT/'Sanity/src/ros2_sanity_ws/scripts'
spec = importlib.util.spec_from_file_location('sanity_clock_helper', SCRIPTS/'sync_sanity_clock.py')
clock = importlib.util.module_from_spec(spec); spec.loader.exec_module(clock)


def sample(bound=.3):
    return {'server':clock.SERVER,'stratum':2,'remote_minus_PC_ms':.1,
        'roundtrip_delay_ms':.2,'conservative_absolute_offset_ms':bound}


def receipt():
    return {'passed':True,'server':clock.SERVER,'limit_ms':1.,'samples':[sample() for _ in range(3)],
        'verified_unix_ns':time.time_ns(),'verified_monotonic_ns':time.monotonic_ns()}


class QueryTests(unittest.TestCase):
    def query(self, mutate=lambda raw:None, address=(clock.SERVER,123), time_jump=False):
        sent=1_700_000_000.;elapsed=.0004
        def stamp(t):
            sec=int(t);return struct.pack('>II',sec+2208988800,int((t-sec)*2**32))
        connection=Mock();connection.__enter__=Mock(return_value=connection);connection.__exit__=Mock(return_value=False)
        captured={}
        connection.sendto.side_effect=lambda data, addr:captured.update(request=bytes(data))
        def response(*_):
            raw=bytearray(48);raw[0]=0x24;raw[1]=2
            raw[24:32]=captured['request'][40:48]
            raw[32:40]=stamp(sent+.0003);raw[40:48]=stamp(sent+.0003)
            mutate(raw);return bytes(raw),address
        connection.recvfrom.side_effect=response
        with patch.object(clock.socket,'socket',return_value=connection),patch.object(clock.time,'time',side_effect=[sent,sent+elapsed+(.02 if time_jump else 0.)]),patch.object(clock.time,'monotonic',side_effect=[10.,10.+elapsed]):
            return clock.query()

    def test_valid_ntp_response_measures_small_offset(self):
        r=self.query();self.assertLess(r['conservative_absolute_offset_ms'],1.)
        self.assertAlmostEqual(r['remote_minus_PC_ms'],.1,places=3)

    def test_rejects_wrong_origin_server_unsynchronized_or_bad_stratum(self):
        for mutate,address in [(lambda r:r.__setitem__(24,255),(clock.SERVER,123)),
                               (lambda r:r.__setitem__(0,0xE4),(clock.SERVER,123)),
                               (lambda r:r.__setitem__(1,0),(clock.SERVER,123)),
                               (lambda r:None,('10.0.0.1',123))]:
            with self.subTest(address=address),self.assertRaises(ValueError):self.query(mutate,address)

    def test_rejects_clock_jump_during_measurement(self):
        with self.assertRaisesRegex(ValueError,'clock moved'):self.query(time_jump=True)


class VerificationTests(unittest.TestCase):
    def test_healthy_fast_check_still_requires_three_new_samples_with_same_bound(self):
        now=[0.]
        with patch.object(clock,'query',return_value=sample()) as query,patch.object(clock.time,'monotonic',side_effect=lambda:now[0]),patch.object(clock.time,'sleep',side_effect=lambda t:now.__setitem__(0,now[0]+t)):
            result=clock.verify(2.)
        self.assertEqual(query.call_count,3)
        self.assertAlmostEqual(result['verification_duration_s'],.2)
        self.assertEqual(result['limit_ms'],1.)

    def test_requires_three_consecutive_good_comparisons(self):
        now=[0.]
        with patch.object(clock,'query',side_effect=[sample(),sample(2.),sample(),sample(),sample()]) as query,patch.object(clock.time,'monotonic',side_effect=lambda:now[0]),patch.object(clock.time,'sleep',side_effect=lambda t:now.__setitem__(0,now[0]+t)):
            result=clock.verify(10.)
        self.assertEqual(query.call_count,5);self.assertEqual(len(result['samples']),3)

    def test_timeout_does_not_pass(self):
        with patch.object(clock,'query',return_value=sample(2.)):
            self.assertIsNone(clock.verify(0))

    def test_ensure_already_synced_never_changes_service_or_invokes_sudo(self):
        with patch.object(clock,'verify',return_value=receipt()),patch.object(clock.subprocess,'run') as run:
            result=clock.main(['--ensure'])
        self.assertEqual(result['action'],'verified_existing_sync');run.assert_not_called()

    def test_failed_verification_calls_privileged_repair_only_after_idle_check(self):
        with patch.object(clock,'verify',side_effect=[None,receipt()]) as verify,patch.object(clock,'require_idle_flight') as idle,patch.object(clock.os,'geteuid',return_value=1000),patch.object(clock.subprocess,'run') as run:
            result=clock.main(['--ensure','--wait','4'])
        idle.assert_called_once();command=run.call_args.args[0]
        self.assertEqual(command[0],'sudo');self.assertIn('--apply',command)
        self.assertNotIn('--fly',command);self.assertNotIn('--receipt',command)
        self.assertEqual(verify.call_count,2)
        self.assertEqual(result['action'],'verified_after_repair')

    def test_repair_keeps_private_receipt_owned_and_written_by_caller(self):
        with tempfile.NamedTemporaryFile(prefix='sanity_clock_preflight_',suffix='.json') as temporary:
            path=Path(temporary.name);before=path.stat();fresh=receipt()
            with patch.object(clock,'verify',side_effect=[None,fresh]),patch.object(clock,'require_idle_flight'),patch.object(clock.os,'geteuid',return_value=1000),patch.object(clock.subprocess,'run') as run:
                result=clock.main(['--ensure','--receipt',str(path)])
            command=run.call_args.args[0]
            self.assertNotIn('--receipt',command);self.assertNotIn(str(path),command)
            self.assertEqual(path.stat().st_uid,before.st_uid)
            self.assertEqual(path.stat().st_ino,before.st_ino)
            self.assertEqual(path.stat().st_mode & 0o777,0o600)
            self.assertEqual(load_json(path),result)
            self.assertEqual(result['verified_monotonic_ns'],fresh['verified_monotonic_ns'])
            with tempfile.TemporaryDirectory() as directory,patch.dict(os.environ,{'SANITY_CLOCK_PREFLIGHT_JSON':path.read_text()}):
                self.assertTrue(record_clock_preflight(Path(directory),required=True)['passed'])

    def test_successful_repair_without_fresh_verification_cannot_write_receipt(self):
        with tempfile.NamedTemporaryFile() as temporary:
            path=Path(temporary.name)
            with patch.object(clock,'verify',side_effect=[None,None]),patch.object(clock,'require_idle_flight'),patch.object(clock.os,'geteuid',return_value=1000),patch.object(clock.subprocess,'run'),self.assertRaises(SystemExit):
                clock.main(['--ensure','--receipt',str(path)])
            self.assertEqual(path.read_bytes(),b'')

    def test_failed_privileged_repair_never_saves_a_success_receipt(self):
        with tempfile.NamedTemporaryFile() as temporary:
            path=Path(temporary.name)
            with patch.object(clock,'verify',return_value=None) as verify,patch.object(clock,'require_idle_flight'),patch.object(clock.os,'geteuid',return_value=1000),patch.object(clock.subprocess,'run',side_effect=subprocess.CalledProcessError(1,['sudo'])),self.assertRaises(subprocess.CalledProcessError):
                clock.main(['--ensure','--receipt',str(path)])
            self.assertEqual(verify.call_count,1)
            self.assertEqual(path.read_bytes(),b'')

    def test_running_fleet_blocks_clock_repair(self):
        for name in ('fleet','experiment','identification_experiment','pitch_experiment','fleet_pitch','fleet_discovery','fleet_home_capture','plate_sequence','plate_commissioning','cable_sequence'):
            with tempfile.TemporaryDirectory() as directory:
                pid=Path(directory,'123');pid.mkdir();(pid/'cmdline').write_bytes(b'python\0-m\0sanity_zenoh.'+name.encode()+b'\0')
                with self.assertRaisesRegex(RuntimeError,'running Sanity mission'):
                    clock.require_idle_flight(Path(directory))

    def test_console_plate_and_cable_controllers_block_clock_repair(self):
        for name in ('sanity_plate','sanity_plate_hover','sanity_cable'):
            with tempfile.TemporaryDirectory() as directory:
                pid=Path(directory,'123');pid.mkdir();(pid/'cmdline').write_bytes(b'python\0/usr/bin/'+name.encode()+b'\0')
                with self.assertRaisesRegex(RuntimeError,'running Sanity mission'):clock.require_idle_flight(Path(directory))


class ServiceTests(unittest.TestCase):
    def test_chrony_repair_does_not_start_timesyncd(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(clock,'require_idle_flight'),patch.object(clock,'query',return_value=sample()),patch.object(clock,'service_active',side_effect=lambda name:name=='chrony'),patch.object(clock,'chrony_sources',side_effect=[[],[['^','*',clock.SERVER]]]),patch.object(clock,'CHRONY_MARKER',Path(directory,'marker.json')),patch.object(clock.subprocess,'run') as run:
            self.assertEqual(clock.apply_reference(),'chrony')
            commands=[c.args[0] for c in run.call_args_list]
        self.assertIn(['chronyc','add','server',clock.SERVER,'iburst','prefer','minpoll','4','maxpoll','5'],commands)
        self.assertFalse(any('systemctl' in c for c in commands))
        self.assertFalse(any('makestep' in c for c in commands))

    def test_step_is_immediate_idle_only_and_uses_selected_mocap_source(self):
        with patch.object(clock,'require_idle_flight') as idle,patch.object(clock,'query',side_effect=[sample(),sample(2.)]),patch.object(clock,'service_active',return_value=True),patch.object(clock,'chrony_sources',return_value=[['^','*',clock.SERVER]]),patch.object(clock.subprocess,'run') as run:
            clock.apply_reference()
        self.assertEqual(idle.call_count,2)
        self.assertIn(unittest.mock.call(['chronyc','makestep'],check=True),run.call_args_list)

    def test_no_step_to_unselected_source(self):
        with patch.object(clock,'require_idle_flight'),patch.object(clock,'query',return_value=sample()),patch.object(clock,'service_active',return_value=True),patch.object(clock,'chrony_sources',return_value=[['^','-',clock.SERVER],['^','*','1.2.3.4']]),patch.object(clock.time,'sleep'),patch.object(clock.subprocess,'run') as run:
            clock.apply_reference()
        self.assertFalse(any('makestep' in call.args[0] for call in run.call_args_list))

    def test_timesyncd_refuses_unmanaged_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            config=Path(directory,'custom.conf');config.write_text('unmanaged')
            with patch.object(clock,'require_idle_flight'),patch.object(clock,'query',return_value=sample()),patch.object(clock,'service_active',side_effect=lambda n:n=='systemd-timesyncd'),patch.object(clock,'CONFIG',config),self.assertRaisesRegex(RuntimeError,'unmanaged'):
                clock.apply_reference()
            self.assertEqual(config.read_text(),'unmanaged')


class ReceiptTests(unittest.TestCase):
    def test_receipt_saved_for_recording(self):
        with tempfile.TemporaryDirectory() as directory,patch.dict(os.environ,{'SANITY_CLOCK_PREFLIGHT_JSON':json.dumps(receipt())}):
            report=record_clock_preflight(Path(directory),required=True)
            self.assertEqual(load_json(Path(directory, 'clock_sync.json')),report)

    def test_missing_or_stale_receipt_blocks_physical_startup(self):
        with tempfile.TemporaryDirectory() as directory,patch.dict(os.environ,{},clear=True):
            with self.assertRaisesRegex(ValueError,'missing'):record_clock_preflight(Path(directory),required=True)
            self.assertIsNone(record_clock_preflight(Path(directory),required=False))
        for change in [lambda r:r.update(verified_monotonic_ns=time.monotonic_ns()-121_000_000_000),lambda r:r.update(passed=False),lambda r:r.update(samples=[sample(2.)]*3)]:
            r=receipt();change(r)
            with tempfile.TemporaryDirectory() as directory,patch.dict(os.environ,{'SANITY_CLOCK_PREFLIGHT_JSON':json.dumps(r)}),self.assertRaises(ValueError):
                record_clock_preflight(Path(directory),required=True)

    def test_sim_plan_help_and_emergency_control_skip_clock_gate(self):
        for args in ('--sim','--plan-only','--prepare-only','--help','--control /tmp/run --action emergency-stop'):
            command='export SANITY_CLOCK_PREFLIGHT_JSON=stale; source "$1" '+args+'; test -z "${SANITY_CLOCK_PREFLIGHT_JSON:-}"'
            result=subprocess.run(['bash','-c',command,'test',str(SCRIPTS/'sanity_clock_preflight.sh')],capture_output=True,text=True,timeout=2)
            self.assertEqual(result.returncode,0,result.stderr)

    @unittest.skip("Optional fleet launcher is outside this single-aircraft flight package")
    def test_direct_physical_fleet_cannot_open_recording_or_transport_without_gate(self):
        from sanity_zenoh import fleet
        with tempfile.TemporaryDirectory() as directory,patch.dict(os.environ,{},clear=True),patch.object(fleet.subprocess,'Popen') as popen:
            with self.assertRaisesRegex(ValueError,'Clock preflight missing'):
                fleet.main(['--workspace',str(ROOT),'--config',str(ROOT/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/fleet_hover_three.yaml'),'--fly','--frames-confirmed','--output',str(Path(directory,'run'))])
            popen.assert_not_called()
