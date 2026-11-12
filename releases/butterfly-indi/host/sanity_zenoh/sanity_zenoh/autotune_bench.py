"""Disarmed ESP gain write/readback, optionally verified through independent FC USB."""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import struct
import time
from sanity_zenoh.client import SanitySender
from sanity_zenoh.firmware_collective import FirmwareCollective
from sanity_zenoh.recording import EventRecorder
from sanity_zenoh.tuning_rpc import MspTuningRpc, TuneSnapshot


class IndependentFc:
    """Direct USB reads only: writes must traverse Zenoh and the ESP UART."""

    def __init__(self, root, port, expected_uid):
        source = root / 'Sanity/sanity-hardware/tools/acquisition/betaflight_msp.py'
        spec = importlib.util.spec_from_file_location('sanity_bench_readonly_msp', source)
        self.bf = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.bf)
        self.bf.COMMANDS.update({94: 'PID_ADVANCED', 112: 'PID', 150: 'STATUS_EX'})
        self.transport = self.bf.USBSerial(port)
        self.client = self.bf.MSP(self.transport, timeout=.5)
        try:
            self.identity = self.bf.identify(self.client)
            self.uid = self.client.query(160).hex()
            if self.uid != expected_uid.lower():
                raise ValueError('Independent FC UID mismatch')
            if (self.identity['firmware_version'], self.identity['msp_api_version']) != ('4.5.1', '1.46'):
                raise ValueError('Independent FC must be Betaflight4.5.1/API1.46')
            self.guard()
        except BaseException:
            self.close()
            raise

    def guard(self):
        status = self.bf.status(self.client.query(101), self.client.query(119))
        battery = self.bf.battery(self.client.query(130))
        raw = self.client.query(104)
        if len(raw) < 8:
            raise ValueError('Independent FC motor output data missing')
        motors = list(struct.unpack_from('<4H', raw))
        rpm = [m['rpm'] for m in self.bf.motor_telemetry(self.client.query(139))['motors']]
        if status['armed'] is not False or any(rpm) or motors != [1000] * 4:
            raise ValueError('Independent FC must be disarmed with STOP output and zero RPM')
        if battery['voltage_V'] > 2.:
            raise ValueError('Unplug flight battery for the USB-only bench test')
        return dict(status=status, battery=battery, motors=motors, rpm=rpm)

    def snapshot(self):
        self.guard()
        status = self.client.query(150)
        if len(status) < 11:
            raise ValueError('Independent FC profile missing')
        return TuneSnapshot(self.client.query(112), self.client.query(94), status[10])

    def close(self):
        self.transport.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--roundtrip', action='store_true', help='Resend exact original payloads')
    parser.add_argument('--change-check', action='store_true', help='Temporarily decrement roll P and F by one; restore without EEPROM save')
    parser.add_argument('--damping-check', action='store_true', help='Independently verify paired D/D-min and boost setters, then restore')
    parser.add_argument('--fc-port', help='Independent direct FC USB; no setters sent through this port')
    parser.add_argument('--expected-uid', default='2a0039000a47333135373835')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if (args.change_check or args.damping_check) and not args.fc_port:
        parser.error('--change-check requires --fc-port for independent verification')
    root = Path(__file__).resolve().parents[6]
    out = args.output or root / 'outputs' / ('sanity_autotune_bench_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True, exist_ok=False)
    events = EventRecorder(out / 'events.jsonl')
    sender = rpc = fc = original = target = None
    error = restore_error = None
    checks = {}
    stages = []
    changed = False

    def keepalive():
        sender.ctbr(False, 0., (0., 0., 0.), valid_for_s=1.)

    def verify(stage, expected):
        via_esp = rpc.snapshot(keepalive)
        direct = fc.snapshot() if fc else None
        stages.append(dict(stage=stage, expected=expected.json(), esp=via_esp.json(), direct_usb=direct.json() if direct else None))
        checks[stage + '_esp'] = via_esp == expected
        if fc:
            checks[stage + '_independent_usb'] = direct == expected
        if via_esp != expected or (fc and direct != expected):
            raise ValueError('Gain payload readback mismatch: ' + stage)

    try:
        if args.fc_port:
            fc = IndependentFc(root, args.fc_port, args.expected_uid)
        sender = SanitySender(listen='udp/10.0.0.200:7447', telemetry_listen='tcp/10.0.0.200:7448', diagnostics=True, disabled_only=True, physical_units=True,
            ctbr_mapping=FirmwareCollective(root / 'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_esp_collective.json'), on_event=events)
        deadline = time.monotonic() + 30.
        while True:
            keepalive()
            status = sender.receiver_status()
            if status and status.session == sender.session_id and not status.failsafe and status.fc_ready and not status.enabled:
                break
            if time.monotonic() > deadline:
                raise TimeoutError('Fresh disabled FC acknowledgement required; connect FC USB with battery unplugged')
            time.sleep(.02)
        if not status.tuning_supported:
            raise ValueError('Install tuning-capable ESP firmware first')
        rpc = MspTuningRpc(sender)
        cap = rpc.call(0, keepalive=keepalive)
        if len(cap) not in (7, 8) or not cap[6]:
            raise ValueError('FC must be disarmed for bench check')
        original = rpc.snapshot(keepalive)
        (out / 'original_tune.json').write_text(json.dumps(original.json(), indent=2) + '\n')
        verify('original', original)
        if args.roundtrip:
            rpc.verify_setters(original, keepalive)
            verify('unchanged_setters', original)
        if args.damping_check:
            rpc.require_extensions(3,keepalive)
            target=original
            for axis in (0,1):target=target.with_damping(axis,original.gain(axis,'D')+max(1,round(original.gain(axis,'D')*.05)))
            target=target.with_boost(max(0,original.advanced[52]-5))
            fc.guard();changed=True
            rpc.apply(target,original,keepalive=keepalive)
            verify('paired_damping_and_boost',target)
            rpc.apply(original,original,keepalive=keepalive)
            verify('damping_restored',original)
        if args.change_check:
            if original.gain(0, 'P') < 2 or original.gain(0, 'F') < 2:
                raise ValueError('Roll P/F too small for the one-unit decrement test')
            target = original.changed(0, 'P', original.gain(0, 'P') - 1).changed(0, 'F', original.gain(0, 'F') - 1)
            fc.guard()
            changed = True  # A lost ACK must still trigger restoration.
            rpc.apply(target, original, keepalive=keepalive)
            verify('changed_setters', target)
    except BaseException as caught:
        error = str(caught) or type(caught).__name__
    finally:
        if changed:
            try:
                if fc:
                    fc.guard()
                rpc.apply(original, original, keepalive=keepalive)
                verify('restored', original)
            except BaseException as caught:
                restore_error = str(caught) or type(caught).__name__
        if fc:
            try:
                checks['final_disarmed_stop_zero_rpm'] = bool(fc.guard())
            except Exception as caught:
                error = (error + '; ' if error else '') + str(caught)
        if rpc:
            rpc.close()
        if sender:
            sender.close()
        if fc:
            fc.close()
        events.close()
    checks.update(no_error=error is None, restoration_succeeded=restore_error is None, no_recording_error=events.error is None,
                  shutdown_disabled_acknowledged=bool(sender and sender.shutdown_acknowledged))
    report = dict(passed=all(checks.values()), checks=checks, error=error, restore_error=restore_error,
                  original=original.json() if original else None, target=target.json() if target else None, stages=stages,
                  independent_fc=dict(port=args.fc_port, uid=fc.uid, identity=fc.identity) if fc else None,
                  scope='Disabled zero-thrust commands only; gain writes via Zenoh/ESP/UART4, optional independent read-only FC USB. Temporary changes restored; no EEPROM save, arming or motor test.')
    (out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    print('Results:', out)
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
