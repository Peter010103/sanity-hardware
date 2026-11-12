#!/usr/bin/env python3
"""Verify the OptiTrack/PC clock offset; repair only while mission processes are idle."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time

SERVER = os.environ.get('SANITY_MOCAP_NTP_SERVER', '10.0.0.251')
CONFIG = Path('/run/systemd/timesyncd.conf.d/90-sanity-mocap.conf')
CHRONY_MARKER = Path('/run/sanity-mocap-chrony-source.json')
SIGNATURE = '# Sanity mocap LAN clock reference; temporary until reboot\n'
LIMIT_MS = 1.
MISSION_MODULES = {b'sanity_zenoh.fleet', b'sanity_zenoh.experiment', b'sanity_zenoh.identification_experiment',
    b'sanity_zenoh.pitch_experiment', b'sanity_zenoh.ros_sequence', b'sanity_zenoh.local_policy', b'sanity_zenoh.fleet_pitch', b'sanity_zenoh.fleet_discovery', b'sanity_zenoh.fleet_home_capture',
    b'sanity_zenoh.plate_sequence',b'sanity_zenoh.plate_commissioning',b'sanity_zenoh.cable_sequence'}


def query():
    sent = time.time(); monotonic_sent = time.monotonic()
    seconds = int(sent)+2208988800; fraction = int((sent-int(sent))*2**32)
    request = bytearray(48); request[0] = 0x23; request[40:48] = struct.pack('>II', seconds, fraction)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
        connection.settimeout(2.); connection.sendto(request, (SERVER, 123))
        raw, address = connection.recvfrom(512); received = time.time()
    elapsed = time.monotonic()-monotonic_sent
    if abs((received-sent)-elapsed) > .001:
        raise ValueError('PC clock moved during comparison; retry after synchronization settles')
    if (address != (SERVER,123) or len(raw)<48 or raw[24:32]!=request[40:48]
            or raw[0]>>6==3 or raw[0]&7!=4 or not 1<=raw[1]<=15):
        raise ValueError('No valid synchronized NTP response from OptiTrack')
    def stamp(offset):
        whole, part = struct.unpack_from('>II', raw, offset)
        return whole-2208988800+part/2**32
    t2, t3 = stamp(32), stamp(40)
    delay = (received-sent)-(t3-t2); offset = ((t2-sent)+(t3-received))/2
    if t3<t2 or delay<-.001:
        raise ValueError('Inconsistent NTP timestamps')
    return {'server':address[0], 'stratum':raw[1], 'remote_minus_PC_ms':offset*1000,
        'roundtrip_delay_ms':delay*1000, 'conservative_absolute_offset_ms':(abs(offset)+max(delay,0.)/2)*1000}


def verify(wait, interval=.1):
    # Three fresh LAN samples still need to satisfy the same 1 ms bound.
    # A healthy clock need not spend a full second between comparisons.
    started = time.monotonic(); deadline = started+wait; consecutive = []
    while True:
        try:
            sample = query(); print(json.dumps(sample), flush=True)
            consecutive = consecutive+[sample] if sample['conservative_absolute_offset_ms']<LIMIT_MS else []
            if len(consecutive)>=3:
                print('PASS: three clock comparisons within1ms; mocap freshness limits unchanged.', flush=True)
                return {'passed':True, 'server':SERVER, 'limit_ms':LIMIT_MS, 'samples':consecutive[-3:],
                    'sample_interval_s':interval, 'verification_duration_s':time.monotonic()-started,
                    'verified_unix_ns':time.time_ns(), 'verified_monotonic_ns':time.monotonic_ns()}
        except (OSError, ValueError) as error:
            print('Clock comparison:', error, flush=True); consecutive = []
        if time.monotonic()>=deadline:
            print('Clock not verified within1ms; mission startup blocked.', flush=True)
            return None
        time.sleep(min(interval, max(0.,deadline-time.monotonic())))


def require_idle_flight(proc_root=Path('/proc')):
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():continue
        try:args = (proc/'cmdline').read_bytes().split(b'\0')
        except (OSError, PermissionError):continue
        if any(arg in MISSION_MODULES or Path(os.fsdecode(arg)).name in {'sanity_sequence','sanity_plate','sanity_plate_hover','sanity_cable'} for arg in args if arg):
            raise RuntimeError('Finish running Sanity mission processes before changing time synchronization')


def service_active(name):
    return subprocess.run(['systemctl','is-active','--quiet',name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode==0


def chrony_sources():
    result = subprocess.run(['chronyc','-n','-c','sources'], check=True, capture_output=True, text=True)
    return [line.split(',') for line in result.stdout.splitlines() if line.strip()]


def apply_reference():
    require_idle_flight()
    print('Checking OptiTrack NTP before repair:', json.dumps(query()), flush=True)
    if service_active('chrony') or service_active('chronyd'):
        sources = chrony_sources()
        present = any(len(row)>2 and row[2]==SERVER for row in sources)
        if not present:
            subprocess.run(['chronyc','add','server',SERVER,'iburst','prefer','minpoll','4','maxpoll','5'], check=True)
            CHRONY_MARKER.write_text(json.dumps({'signature':SIGNATURE, 'server':SERVER, 'added':True})+'\n')
            CHRONY_MARKER.chmod(0o644)
        subprocess.run(['chronyc','burst','4/4',SERVER], check=True)
        # Do not schedule a future step that could happen after arming. Any immediate
        # step below is allowed only while this idle guard holds and this source is selected.
        for _ in range(12):
            if any(len(row)>2 and row[1]=='*' and row[2]==SERVER for row in chrony_sources()):
                if query()['conservative_absolute_offset_ms']>=LIMIT_MS:
                    require_idle_flight(); subprocess.run(['chronyc','makestep'], check=True)
                break
            time.sleep(.5)
        print('Using active chrony; no second clock service or network profile changes.', flush=True)
        return 'chrony'
    if not service_active('systemd-timesyncd'):
        raise RuntimeError('No supported active time service; enable/configure chrony or systemd-timesyncd before flight')
    if CONFIG.exists() and not CONFIG.read_text().startswith(SIGNATURE):
        raise RuntimeError('Existing unmanaged configuration; refusing to overwrite')
    CONFIG.parent.mkdir(parents=True, exist_ok=True)
    CONFIG.write_text(SIGNATURE+'[Time]\nNTP=\nNTP='+SERVER+'\nPollIntervalMinSec=16s\nPollIntervalMaxSec=32s\nConnectionRetrySec=5s\n')
    CONFIG.chmod(0o644)
    subprocess.run(['systemctl','restart','systemd-timesyncd'], check=True)
    return 'systemd-timesyncd'


def restore_reference():
    require_idle_flight()
    if CHRONY_MARKER.exists():
        marker = json.loads(CHRONY_MARKER.read_text())
        if marker.get('signature')!=SIGNATURE or marker.get('server')!=SERVER or marker.get('added') is not True:
            raise RuntimeError('Unmanaged chrony marker; refusing to restore')
        subprocess.run(['chronyc','delete',SERVER], check=True); CHRONY_MARKER.unlink()
    if CONFIG.exists():
        if not CONFIG.read_text().startswith(SIGNATURE):
            raise RuntimeError('Existing unmanaged configuration; refusing to restore')
        CONFIG.unlink()
        if service_active('systemd-timesyncd'):
            subprocess.run(['systemctl','restart','systemd-timesyncd'], check=True)
    print('Restored only Sanity-managed clock sources; network profiles unchanged.', flush=True)


def finish_report(report, action, receipt):
    report = {**report, 'action':action, 'helper_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'scope':'Startup NTP comparison; clock correction is completed before ROS/Zenoh mission processes start. Does not remove processing/transport delay.'}
    if receipt:
        receipt.write_text(json.dumps(report, indent=2)+'\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--apply',action='store_true'); mode.add_argument('--restore',action='store_true')
    mode.add_argument('--ensure',action='store_true',help='Verify first; if needed repair while idle (sudo) and verify before mission startup')
    parser.add_argument('--wait',type=float,default=45.)
    parser.add_argument('--receipt',type=Path)
    args = parser.parse_args(argv)
    if not math.isfinite(args.wait) or not 0<=args.wait<=120:parser.error('--wait must be0..120seconds')
    if args.apply or args.restore:
        if os.geteuid()!=0:parser.error('Use sudo for --apply/--restore; no password is stored')
        if args.restore:restore_reference(); return
        service = apply_reference(); report = verify(args.wait)
        if not report:raise SystemExit(1)
        return finish_report(report, 'repaired_'+service, args.receipt)
    report = verify(min(args.wait,8.) if args.ensure else args.wait)
    if report:return finish_report(report, 'verified_existing_sync', args.receipt)
    if not args.ensure:raise SystemExit(1)
    require_idle_flight()
    print('Clock repair requires administrator privileges before mission startup; sudo may request your password.', flush=True)
    command = [sys.executable, str(Path(__file__).resolve()), '--apply','--wait',str(args.wait)]
    if os.geteuid()!=0:command.insert(0,'sudo')
    subprocess.run(command, check=True)
    print('Verifying repaired clock before saving the startup receipt as the calling user.',flush=True)
    report=verify(args.wait)
    if not report:raise SystemExit(1)
    return finish_report(report,'verified_after_repair',args.receipt)


if __name__=='__main__':
    try:main()
    except (OSError,RuntimeError,ValueError,subprocess.CalledProcessError) as error:
        print(str(error),file=sys.stderr); raise SystemExit(1)
