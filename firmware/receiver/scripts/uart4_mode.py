#!/usr/bin/env python3
"""Configure Sanity's shared MSP UART, RC source and processing rate via direct FC USB.

MSP bench output remains disarmed; this script never sends RC or motor commands.
Quiesce the ESP before applying a change, then install its matching output image.
"""
import argparse
import datetime
import json
import os
import re
from pathlib import Path
import select
import struct
import sys
import time

BAUD = (0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000)
PORT = '/dev/serial/by-id/usb-Betaflight_Betaflight_STM32F405_0x8000000-if00'


def serial_records(payload):
    if not payload or len(payload) % 7:
        raise ValueError('Malformed MSP_CF_SERIAL_CONFIG response')
    records = [list(struct.unpack_from('<BHBBBB', payload, i)) for i in range(0, len(payload), 7)]
    if len({r[0] for r in records}) != len(records):
        raise ValueError('Duplicate serial port identifiers')
    if any(i >= len(BAUD) for r in records for i in r[2:]):
        raise ValueError('Unknown Betaflight baud index')
    return records


def serial_command(record):
    return 'serial ' + ' '.join(map(str, record[:2] + [BAUD[i] for i in record[2:]]))


def plan(records, mode, features=None):
    if mode not in ('msp', 'msp-bench', 'ctbr'):
        raise ValueError('Expected msp, msp-bench or legacy ctbr')
    vcp = next((r for r in records if r[0] == 20), None)
    current = next((r for r in records if r[0] == 3), None)  # BF port IDs are zero based.
    if not vcp or not vcp[1] & 1:
        raise ValueError('Direct USB MSP must remain available')
    if current is None or current[1] not in (0, 1, 64):
        raise ValueError('UART4 has an unsupported/shared function; refusing to overwrite it')
    if mode == 'ctbr' and any(r[0] != 3 and r[1] & 64 for r in records):
        raise ValueError('Another UART already supplies Serial RX; inspect it first')
    target = current.copy()
    target[1] = 64 if mode == 'ctbr' else 1
    if mode != 'ctbr':
        target[2] = 5  # MSP 115200, preserving all other baud settings.
    expected = [target if r[0] == 3 else r.copy() for r in records]
    commands = [serial_command(target)]
    restore = [serial_command(current)]
    result = {'mode': mode, 'before': records, 'expected': expected,
              'commands': commands, 'restore_commands': restore,
              'changes_required': expected != records}
    if mode == 'msp' or mode == 'ctbr' and features is not None:
        if features is None:
            raise ValueError('Unified MSP requires the current feature mask')
        receiver_features = {'RX_PPM': 0, 'RX_SERIAL': 3, 'RX_PARALLEL_PWM': 13, 'RX_MSP': 14, 'RX_SPI': 25}
        mask = sum(1 << i for i in receiver_features.values())
        for name, bit in receiver_features.items():
            commands.append('feature '+('' if name == ('RX_MSP' if mode == 'msp' else 'RX_SERIAL') else '-')+name)
            restore.append('feature '+('' if features & (1 << bit) else '-')+name)
        result.update(expected_features=(features & ~mask) | (1 << (14 if mode == 'msp' else 3)), changes_required=True)
        if mode == 'msp':
            commands.append('set serial_update_rate_hz = 1000')
            result['expected_serial_rate_hz'] = 1000
    commands.append('save'); restore.append('save')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['msp', 'msp-bench', 'ctbr'])
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--expected-uid', required=True)
    parser.add_argument('--apply', action='store_true', help='Save the planned UART4 change and reboot')
    parser.add_argument('--output', type=Path, help='New private directory for backup, plan and receipt')
    args = parser.parse_args(argv)
    if args.apply and not args.output:
        parser.error('--apply requires --output for a complete pre-change backup')
    if Path(args.port).name != Path(PORT).name:
        parser.error('Use the direct Betaflight USB stable path, not ESP passthrough')
    sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'sanity-hardware/tools/acquisition'))
    import betaflight_msp as bf
    bf.COMMANDS[54] = 'CF_SERIAL_CONFIG'
    os.umask(0o077)
    if args.output:
        args.output.mkdir(parents=True, exist_ok=False)

    def snapshot():
        t = bf.USBSerial(args.port)
        try:
            c = bf.MSP(t, timeout=1)
            identity = bf.identify(c)
            if identity['firmware_version'] != '4.5.1' or identity['msp_api_version'] != '1.46':
                raise RuntimeError('This tool is verified only for Betaflight 4.5.1 / API 1.46')
            uid = c.query(160).hex()
            if uid != args.expected_uid.lower():
                raise RuntimeError('FC UID mismatch')
            status = bf.status(c.query(101), c.query(119))
            rpm = [r['rpm'] for r in bf.motor_telemetry(c.query(139))['motors']]
            if status['armed'] is not False or any(rpm):
                raise RuntimeError('FC must be positively disarmed with all motor RPM zero')
            return {'identity': identity, 'uid': uid, 'status': status, 'rpm': rpm,
                    'serial': serial_records(c.query(54)), 'features': struct.unpack_from('<I', c.query(36))[0]}
        finally:
            t.close()

    before = snapshot()
    change = plan(before['serial'], args.mode, before['features'])
    print(json.dumps(change, indent=2), flush=True)
    if args.output:
        (args.output/'before.json').write_text(json.dumps(before, indent=2)+'\n')
        (args.output/'plan.json').write_text(json.dumps(change, indent=2)+'\n')
        (args.output/'restore_uart4.bfc').write_text('\n'.join(change['restore_commands'])+'\n')
    if not args.apply:
        return
    if change['changes_required']:
        t = bf.USBSerial(args.port)
        entered = saved = False
        transcript = bytearray()
        def prompt(timeout=30):
            data = bytearray(); deadline = time.monotonic()+timeout; last = time.monotonic()
            while time.monotonic() < deadline:
                if select.select([t.fd], [], [], .1)[0]:
                    chunk = os.read(t.fd, 4096)
                    if not chunk:
                        raise ConnectionError('FC USB disconnected during CLI')
                    data.extend(chunk); last = time.monotonic()
                elif data.endswith(b'\r\n# ') and time.monotonic()-last > .2:
                    transcript.extend(data)
                    if b'###ERROR' in data or b'Unknown command' in data:
                        raise RuntimeError('FC rejected CLI command')
                    return bytes(data)
            raise TimeoutError('FC CLI prompt timeout')
        try:
            t.write(b'#', time.monotonic()+2); entered = True; prompt(8)
            t.write(b'dump all\r\n', time.monotonic()+2)
            dump = prompt()
            (args.output/'fc_before_dump_all.txt').write_bytes(dump)
            if len(dump) < 15000 or b'\nserial 3 ' not in dump:
                raise RuntimeError('Full FC backup appears truncated; no settings applied')
            if args.mode == 'msp':
                t.write(b'feature list\r\n', time.monotonic()+2)
                features_text = prompt()
                if b'RX_MSP' not in features_text:
                    raise RuntimeError('Installed FC build does not list RX_MSP support')
                t.write(b'get serial_update_rate_hz\r\n', time.monotonic()+2)
                rate_text = prompt()
                match = re.search(rb'serial_update_rate_hz = (\d+)', rate_text)
                if not match:
                    raise RuntimeError('Cannot back up the current MSP task rate')
                change['restore_commands'].insert(-1, 'set serial_update_rate_hz = '+match[1].decode())
                (args.output/'restore_uart4.bfc').write_text('\n'.join(change['restore_commands'])+'\n')
                (args.output/'plan.json').write_text(json.dumps(change, indent=2)+'\n')
            for command in change['commands'][:-1]:
                t.write(command.encode()+b'\r\n', time.monotonic()+2); prompt()
            t.write(b'save\r\n', time.monotonic()+2); saved = True
        finally:
            (args.output/'cli_transcript.txt').write_bytes(transcript)
            if entered and not saved:
                try:
                    t.write(b'exit\r\n', time.monotonic()+2)  # 4.5.1: reboot without saving.
                except (OSError, TimeoutError):
                    pass
            t.close()
        time.sleep(2)
    deadline = time.monotonic()+12
    while True:
        try:
            after = snapshot()
            break
        except (OSError, TimeoutError, bf.MSPError):
            if time.monotonic() >= deadline:
                raise
            time.sleep(.2)
    (args.output/'after.json').write_text(json.dumps(after, indent=2)+'\n')
    if after['serial'] != change['expected']:
        raise RuntimeError('Saved serial settings do not match the plan; inspect backup/receipt')
    if 'expected_features' in change and after['features'] != change['expected_features']:
        raise RuntimeError('RX_MSP feature selection did not persist; inspect firmware support')
    receipt = {'timestamp_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
               'verified': True, 'mode': args.mode, 'uid': after['uid'],
               'commands': change['commands'] if change['changes_required'] else [],
               'arming_or_motor_commands_sent': False, 'other_serial_ports_unchanged': True, 'features_after': after['features'],
               'msp_task_rate_command_hz': 1000 if args.mode == 'msp' else None}
    (args.output/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
