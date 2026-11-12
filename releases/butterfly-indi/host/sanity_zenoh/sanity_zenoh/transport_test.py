"""Disabled-only transport qualification and ESP timing analysis."""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path
import hashlib
import secrets
import signal
import struct
import time
import numpy as np

TEST_CONFIG=struct.Struct('>4sBBBBII')
PING=struct.Struct('>4sBBHIQ')
PONG=struct.Struct('>4sBBHIQQQQbBBB')
TRACE_HEADER=struct.Struct('>4sBBBBI')
TRACE_RECORD=struct.Struct('>4B3I5Q2I')
TRACE_KEYS=('type','mode','flags','code','session','sequence','tick','epoch','t0','t1','t2','t3','arg0','arg1')


def decode_diagnostic(raw, kind, device):
    if kind=='trace':
        if len(raw)<TRACE_HEADER.size:raise ValueError('Short trace')
        magic,version,drone,size,count,dropped=TRACE_HEADER.unpack_from(raw)
        if (magic,version,drone,size)!=(b'SZT1',1,device,64) or len(raw)!=TRACE_HEADER.size+count*size:raise ValueError('Invalid trace')
        return {'records':[dict(zip(TRACE_KEYS,TRACE_RECORD.unpack_from(raw,TRACE_HEADER.size+i*size))) for i in range(count)],'dropped':dropped}
    if kind=='test_config':
        if len(raw)!=TEST_CONFIG.size:raise ValueError('Invalid telemetry acknowledgement length')
        magic,version,drone,enabled,code,session,nonce=TEST_CONFIG.unpack(raw)
        if (magic,version,drone)!=(b'SZTA',1,device) or enabled>1 or code>1:raise ValueError('Invalid telemetry acknowledgement')
        return {'enabled':bool(enabled),'code':code,'session':session,'nonce':nonce}
    if kind=='pong':
        if len(raw)!=PONG.size:raise ValueError('Invalid timing pong length')
        f=PONG.unpack(raw)
        if f[:4]!=(b'SZR1',1,device,0):raise ValueError('Invalid timing pong')
        return dict(zip(('probe_id','sent_ns','esp_rx_us','esp_tx_us','epoch','rssi_dbm','channel'),f[4:11]))
    raise ValueError('Unknown diagnostic kind')


def summarize_transport(events, gap_limit_ms=50.):
    from sanity_zenoh.protocol import COMMAND, ReceiverStatus
    events=list(events);wire={};groups=defaultdict(list);summary={}
    for e in events:
        if e['kind']=='zenoh_tx_command':
            h=COMMAND.unpack(bytes.fromhex(e['payload_hex']))
            wire[(h[5],h[6],h[7])]=e
        if e.get('phase') in ('telemetry_off','telemetry_on'):groups[e['phase']].append(e)
    for phase,rows in groups.items():
        accepted={};outputs=[];pongs=[];callback=[];poll_lateness=[];publication_stalls=[];ack_overflow={};trace_drops=[];fc_drops=[];health=[];errors=[];statuses=[]
        for e in rows:
            if 'decode_error' in e:
                errors.append(e['decode_error']);continue
            if e['kind']=='zenoh_rx_trace':
                trace_drops.append(e['dropped'])
                for r in e['records']:
                    if r['type']==5:publication_stalls.append((r['t1']-r['t0'])/1000.)
                    if r['type']==6:poll_lateness.append((r['t1']-r['t0'])/1000.)
                    if r['type']==7:ack_overflow[str(r['code'])]=max(ack_overflow.get(str(r['code']),0),r['arg0'])
                    key=(r['epoch'],r['session'],r['sequence']);tx=wire.get(key)
                    if tx is None or tx.get('phase')!=phase:continue
                    if r['type']==1 and r['flags']&1:
                        accepted[key]=r;callback.append((r['t1']-r['t0'])/1000.)
                    if r['type']==2 and r['code']==(200 if r['mode']==1 else 214):outputs.append(r)
            if e['kind']=='zenoh_rx_fc_telemetry' and 'telemetry' in e:
                contexts=[wire.get((r.get('epoch'),r.get('command_session'),r.get('command_sequence'))) for r in e['telemetry']['records'] if r['type']==1]
                if any(context is not None and context.get('phase')==phase for context in contexts):fc_drops.append(e['telemetry']['esp_dropped_records'])
                health.extend(r['esp_health'] for r in e['telemetry']['records'] if 'esp_health' in r)
            if e['kind']=='zenoh_rx_pong' and 'decode_error' not in e:
                rtt=(e['host_ns']-e['sent_ns'])/1e6;res=(e['esp_tx_us']-e['esp_rx_us'])/1000.
                pongs.append((rtt,res,max(0.,rtt-res)))
            if e['kind']=='zenoh_rx_status':
                try:statuses.append(ReceiverStatus.decode(bytes.fromhex(e['payload_hex'])))
                except ValueError as error:errors.append(str(error))
        by_context=defaultdict(list)
        for key,r in accepted.items():by_context[key[:2]].append(r['t0'])
        gaps=[(b-a)/1000. for values in by_context.values() for a,b in zip(sorted(values),sorted(values)[1:])]
        ages=[(r['t1']-accepted[(r['epoch'],r['session'],r['sequence'])]['t0'])/1000. for r in outputs if (r['epoch'],r['session'],r['sequence']) in accepted]
        times=defaultdict(list)
        for r in outputs:times[r['epoch']].append(r['t1'])
        output_gaps=[(b-a)/1000. for values in times.values() for a,b in zip(sorted(values),sorted(values)[1:])]
        stats={}
        for name,values in (('accepted_command_gap_ms',gaps),('received_command_to_UART_age_ms',ages),('callback_duration_ms',callback),('UART_poll_lateness_over_2ms',poll_lateness),('observed_publication_stall_ms',publication_stalls),('RC_UART_gap_ms',output_gaps),('probe_round_trip_ms',[p[0] for p in pongs]),('probe_round_trip_without_ESP_residence_ms',[p[2] for p in pongs])):
            stats[name]={'samples':len(values),'median':float(np.median(values)) if values else None,'p99':float(np.quantile(values,.99)) if values else None,'max':float(max(values)) if values else None}
        txrows=[e for e in rows if e['kind']=='zenoh_tx_command'];tx_times=sorted(e['host_ns'] for e in txrows)
        gaps_host=np.diff(tx_times)/1e6
        stats['host_publication_completion_gap_ms']={'samples':len(gaps_host),'median':float(np.median(gaps_host)) if len(gaps_host) else None,'p99':float(np.quantile(gaps_host,.99)) if len(gaps_host) else None,'max':float(max(gaps_host)) if len(gaps_host) else None}
        all_disabled=bool(txrows) and all(not e['enabled'] and not COMMAND.unpack(bytes.fromhex(e['payload_hex']))[4] for e in txrows)
        new_trace_drops=max(trace_drops)-min(trace_drops) if trace_drops else None
        stats['accepted_command_gap_ms']['valid_without_trace_loss']=new_trace_drops==0
        status_groups=defaultdict(list)
        for status in statuses:status_groups[status.epoch].append(status)
        holds=[]
        for group in status_groups.values():
            previous=None;begin=None
            for status in sorted(group,key=lambda value:value.receiver_ms):
                if status.sequence!=previous:begin=status.receiver_ms;previous=status.sequence
                holds.append((status.receiver_ms-begin)&0xffffffff)
        stats['status_frontier_hold_lower_bound_ms']={'samples':len(holds),'max':max(holds) if holds else None,'scope':'Repeated accepted sequence across receiver status timestamps; lower bound on hold duration, independent of bulk traces.'}
        # New reduced profiles send complete receiver-local timing windows rather
        # than sampled command traces. Keep legacy full-profile interpretation.
        from sanity_zenoh.arrival_metrics import summarize_windows
        arrival=summarize_windows(rows)
        reduced=arrival['received'] and any(level>0 for level in arrival['profiles'])
        coverage=(arrival['accepted_commands']/len(txrows) if txrows else 0.) if reduced else (len(accepted)/len(txrows) if txrows else 0.)
        if reduced:
            stats['accepted_command_gap_ms']={'samples':arrival['accepted_commands'],
                'median':None,'p99':None,'max':arrival['maximum_accepted_gap_ms'],
                'valid_without_trace_loss':arrival['contiguous'] and arrival['contexts']==1,
                'source':'receiver arrival-window maxima, not sampled individual traces'}
        timing_received=arrival['accepted_commands']>=20 if reduced else len(accepted)>=20
        gap_ok=(arrival['contiguous'] and arrival['contexts']==1 and arrival['maximum_accepted_gap_ms'] is not None and arrival['maximum_accepted_gap_ms']<=gap_limit_ms) if reduced else (bool(gaps) and max(gaps)<=gap_limit_ms)
        checks={'commands_disabled':all_disabled,'receiver_disabled':bool(statuses) and all(not s.enabled for s in statuses),'receiver_epoch_stable':bool(statuses) and len({s.epoch for s in statuses})==1,'timing_trace_received':timing_received,'trace_coverage_at_least_90_percent':bool(txrows) and coverage>=.9,'no_trace_loss':new_trace_drops==0,'no_decode_errors':not errors,'command_gaps_within_limit':gap_ok,'bulk_telemetry_matches_request':bool(fc_drops) if phase=='telemetry_on' else not fc_drops}
        summary[phase]={'passed':all(checks.values()),'checks':checks,'metrics':stats,'commands_published':len(txrows),'accepted_commands_observed':len(accepted),'accepted_trace_coverage_ratio':len(accepted)/len(txrows) if txrows else 0.,'arrival_windows':arrival,'timing_coverage_ratio':coverage,'coverage_method':'windowed accepted count (boundary approximation)' if reduced else 'matched per-command traces','trace_records_dropped_during_phase':new_trace_drops,'FC_records_dropped_during_phase':max(fc_drops)-min(fc_drops) if fc_drops else None,'unmatched_ACK_backlog_overflows':ack_overflow,'latest_ESP_health':health[-1] if health else None,'timing_scope':'ESP callback/UART intervals and host probe round-trip; no one-way latency or FC actuation time inferred.'}
    return {'passed':bool(summary) and all(p['passed'] for p in summary.values()),'gap_limit_ms':gap_limit_ms,'phases':summary,'test_scope':'Disarmed-only command traffic; not flight qualification, actuator response or control stability.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',choices=('tcp-shared','tcp-split','udp-split'),default='udp-split')
    parser.add_argument('--host-ip',default='10.0.0.200')
    parser.add_argument('--duration',type=float,default=600.,help='Seconds per measured telemetry phase')
    parser.add_argument('--telemetry',choices=('both','on','off'),default='both')
    parser.add_argument('--mode',choices=('ctbr','direct'),default='ctbr')
    parser.add_argument('--device',type=int,default=9)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--events',type=Path,help='Offline summary only')
    parser.add_argument('--gap-limit-ms',type=float,default=50.)
    args=parser.parse_args()
    if args.events:
        print(json.dumps(summarize_transport((json.loads(line) for line in args.events.open()),args.gap_limit_ms),indent=2));return
    if not 1<=args.duration<=3600 or not 1<=args.device<=255 or not 0<args.gap_limit_ms<=1000:parser.error('duration1..3600s;device1..255;gap limit0..1000ms')
    from sanity_zenoh.client import SanitySender
    from sanity_zenoh.recording import EventRecorder
    from sanity_zenoh.firmware_collective import FirmwareCollective
    from sanity_zenoh.protocol import CTBR,DIRECT,COMMAND,neutral
    out=args.output or Path(__file__).resolve().parents[6]/'outputs'/('sanity_comms_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')+'_'+args.profile)
    out.mkdir(parents=True,exist_ok=False);events=EventRecorder(out/'events.jsonl');phase='startup';interrupted=False;sender=None;error=None
    def record(e):
        if e['kind']=='zenoh_tx_command':
            h=COMMAND.unpack(bytes.fromhex(e['payload_hex']))
            physical=h[0] in (b'SZC2',b'SZC3')
            invalid=h[4] or (any(bytes.fromhex(e['payload_hex'])[32:]) if physical else tuple(h[11:])!=neutral(h[2]))
            if invalid:
                raise ValueError('Disabled-only test guard rejected output')
        events({'phase':phase,**e})
    def stop(signum,frame):
        nonlocal interrupted
        interrupted=True
    previous={s:signal.signal(s,stop) for s in (signal.SIGINT,signal.SIGTERM)}
    control=('udp' if args.profile.startswith('udp') else 'tcp')+'/'+args.host_ip+':7447'
    telemetry=None if args.profile.endswith('shared') else 'tcp/'+args.host_ip+':7448'
    plan={'profile':args.profile,'control_listener':control,'telemetry_listener':telemetry,'duration_per_phase_s':args.duration,'mode':args.mode,'disabled_only':True,'command_lease_s':1.,'source_hz':50,'transmitter_hz':100,'telemetry_requested':args.telemetry}
    plan['source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),Path(__file__).with_name('client.py'))}
    (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');print('Recording:',out,flush=True)
    try:
        mapping=FirmwareCollective(Path(__file__).resolve().parents[1]/'config/sanity09_esp_collective.json')
        sender=SanitySender(device=args.device,mode=CTBR if args.mode=='ctbr' else DIRECT,listen=control,telemetry_listen=telemetry,diagnostics=True,disabled_only=True,ctbr_mapping=mapping,physical_units=args.mode=='ctbr',on_event=record)
        deadline=time.monotonic()+20.
        while True:
            if interrupted:raise KeyboardInterrupt
            if args.mode=='ctbr':sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.)
            else:sender.set_output(False,neutral(DIRECT),1.)
            status=sender.receiver_status()
            if status and status.session==sender.session_id and not status.enabled and not status.failsafe:break
            if time.monotonic()>deadline:raise TimeoutError('No disabled claim; flash matching comms-test image and check endpoint')
            time.sleep(.02)
        selections=(False,True) if args.telemetry=='both' else (args.telemetry=='on',)
        probe_id=0
        for enabled in selections:
            phase='configuration';nonce=secrets.randbits(32);deadline=time.monotonic()+5.
            while True:
                if interrupted:raise KeyboardInterrupt
                sender.set_output(False,neutral(sender.mode),1.)
                sender.configure_test_telemetry(enabled,nonce)
                ack=sender.test_ack
                if ack and ack.get('nonce')==nonce and ack.get('session')==sender.session_id:
                    if ack['code'] or ack['enabled']!=enabled:raise ValueError('Telemetry configuration denied')
                    break
                if time.monotonic()>deadline:raise TimeoutError('No test telemetry acknowledgement; firmware is missing COMMS_TEST_ONLY support')
                time.sleep(.05)
            # Drain queued diagnostics before measuring; keep the disabled source alive.
            drain=time.monotonic()+1.
            while time.monotonic()<drain:
                sender.set_output(False,neutral(sender.mode),1.);time.sleep(.02)
            phase='telemetry_on' if enabled else 'telemetry_off';start=time.monotonic();next_tick=start;next_probe=start;next_print=start+30.
            events({'kind':'comms_phase','phase':phase,'event':'begin'})
            print(phase,flush=True)
            while time.monotonic()-start<args.duration:
                if interrupted:raise KeyboardInterrupt
                if events.error:raise RuntimeError(events.error)
                if args.mode=='ctbr':sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.)
                else:sender.set_output(False,neutral(DIRECT),1.)
                status=sender.receiver_status()
                if not status or status.enabled or status.session!=sender.session_id:raise ValueError('Receiver lost disabled ownership')
                if time.monotonic()>=next_probe:
                    sent=time.monotonic_ns();sender.probe_publisher.put(PING.pack(b'SZP1',1,args.device,0,probe_id,sent));probe_id+=1;next_probe+=.05
                if time.monotonic()>=next_print:print(f'{phase}: {time.monotonic()-start:.0f}s',flush=True);next_print+=30.
                next_tick=max(next_tick+.02,time.monotonic());time.sleep(max(0.,next_tick-time.monotonic()))
            events({'kind':'comms_phase','phase':phase,'event':'end'})
        phase='drain';drain=time.monotonic()+.5
        while time.monotonic()<drain:sender.set_output(False,neutral(sender.mode),1.);time.sleep(.02)
    except BaseException as caught:error=str(caught) or type(caught).__name__
    finally:
        phase='cleanup'
        if sender:sender.close()
        events.close()
        for sig,handler in previous.items():signal.signal(sig,handler)
        report=summarize_transport((json.loads(line) for line in (out/'events.jsonl').open()),args.gap_limit_ms)
        report.update(error=error,recording_error=events.error,shutdown_acknowledged=sender.shutdown_acknowledged if sender else False)
        report['passed']=report['passed'] and error is None and events.error is None and report['shutdown_acknowledged']
        (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));print('Results:',out,flush=True)
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
