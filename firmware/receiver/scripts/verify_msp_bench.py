#!/usr/bin/env python3
"""STOP-only Zenoh -> ESP -> UART4/MSP verification with independent FC USB checks.

Never enables output or sends positive thrust. Does not need a flight battery.
It proves UART replies and observed STOP output, not motor rotation or flight support.
"""
import argparse
from dataclasses import asdict
import datetime
import json
from pathlib import Path
import struct
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'sanity-hardware/tools/acquisition'))
import betaflight_msp as bf
from sanity_zenoh.client import SanitySender
from sanity_zenoh.protocol import COMMAND, DIRECT
from uart4_mode import PORT, serial_records


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default=PORT, help='Independent direct FC USB telemetry')
    p.add_argument('--expected-uid', required=True)
    p.add_argument('--listen', default='tcp/0.0.0.0:7447')
    p.add_argument('--duration', type=float, default=20)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args(argv)
    if not 0 < args.duration <= 60:
        p.error('duration must be finite and in (0,60] seconds')
    if args.output.exists():
        p.error('Choose a new output file')
    bf.COMMANDS[54] = 'CF_SERIAL_CONFIG'
    t = bf.USBSerial(args.port)
    c = bf.MSP(t, timeout=.5)
    sender = None; worker = None; stop = threading.Event()
    source_errors = []; fc_rows = []; status_rows = []; packet_count = [0]
    error = None; final = None
    def sample():
        status = bf.status(c.query(101), c.query(119))
        rpm = [m['rpm'] for m in bf.motor_telemetry(c.query(139))['motors']]
        raw = c.query(104)
        if len(raw) < 8:
            raise RuntimeError('Missing FC motor output data')
        motors = list(struct.unpack_from('<4H', raw))
        row = {'time': time.monotonic(), 'armed': status['armed'], 'rpm': rpm,
               'motor_counts': motors, 'battery': bf.battery(c.query(130))}
        fc_rows.append(row)
        if status['armed'] is not False or any(rpm) or motors != [1000]*4:
            raise RuntimeError('FC is not confirmed disarmed, STOP=1000 and zero RPM')
        return row
    try:
        identity = bf.identify(c)
        uid = c.query(160).hex()
        if uid != args.expected_uid.lower():
            raise RuntimeError('FC UID mismatch')
        if identity['firmware_version'] != '4.5.1' or identity['msp_api_version'] != '1.46':
            raise RuntimeError('Expected Betaflight 4.5.1 / API 1.46')
        ports = serial_records(c.query(54))
        uart4 = next((r for r in ports if r[0] == 3), None)
        if uart4 is None or uart4[1:3] != [1, 5]:
            raise RuntimeError('UART4 must be MSP only at 115200; use uart4_mode.py first')
        sample()
        sender = SanitySender(mode=DIRECT, listen=args.listen)
        publisher = sender.publisher
        class StopOnlyPublisher:
            def put(self, packet):
                fields = COMMAND.unpack(packet)
                if fields[2] != DIRECT or fields[4] != 0 or tuple(fields[11:]) != (1000,)*4:
                    raise RuntimeError('STOP-only guard blocked an enabled/nonzero packet')
                packet_count[0] += 1
                publisher.put(packet)
        sender.publisher = StopOnlyPublisher()
        def produce():
            while not stop.is_set():
                try:
                    sender.direct_thrust(False, (0.,)*4)
                except Exception as exc:
                    source_errors.append(str(exc)); return
                stop.wait(.02)
        worker = threading.Thread(target=produce, daemon=True); worker.start()
        deadline = time.monotonic()+20
        while True:
            s = sender.receiver_status()
            if s and s.session == sender.session_id and not s.enabled and not s.failsafe and s.fc_age_ms < 150:
                break
            if source_errors:
                raise RuntimeError(str(source_errors))
            if time.monotonic() >= deadline:
                raise TimeoutError('No fresh disabled acknowledgement plus FC UART4 status reply')
            time.sleep(.02)
        start = time.monotonic(); next_fc = start
        while time.monotonic()-start < args.duration:
            s = sender.receiver_status()
            if s:
                status_rows.append(asdict(s))
                if s.enabled:
                    raise RuntimeError('Unexpected receiver enable')
            if time.monotonic() >= next_fc:
                sample(); next_fc = time.monotonic()+.2
            if source_errors:
                raise RuntimeError(str(source_errors))
            time.sleep(.02)
    except BaseException as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        stop.set()
        if worker:
            worker.join(timeout=2)
        if sender:
            try:
                sender.close()
            except Exception as exc:
                error = (error+'; ' if error else '')+'Sender cleanup: '+str(exc)
        try:
            final = sample()
        except Exception as exc:
            error = (error+'; ' if error else '')+'Final FC check: '+str(exc)
        t.close()
        report = {'timestamp_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  'stop_only': True, 'arming_commands_sent': False, 'positive_thrust_sent': False,
                  'duration_requested_s': args.duration, 'command_packets': packet_count[0],
                  'receiver_status_samples': len(status_rows),
                  'fresh_fc_status_samples': sum(r['fc_age_ms'] < 150 for r in status_rows),
                  'fc_ready_samples': sum(r['flags'] & 4 != 0 for r in status_rows),
                  'receiver_status': status_rows, 'fc_checks': fc_rows,
                  'final_fc': final, 'error': error,
                  'scope': ('Disabled Zenoh ACK, fresh UART4 MSP status and FC STOP output verified.' if error is None else 'Verification incomplete; inspect error.')+' Positive motor output, all readiness interlocks and flight operation are separate checks.'}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('receiver_status','fc_checks')},indent=2))
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
