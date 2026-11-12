"""Shared-listener, disabled-only diagnostics for normal Sanity flight receivers."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import signal
import threading
import time

import numpy as np

from sanity_zenoh.client import SanitySender
from sanity_zenoh.firmware_collective import FirmwareCollective
from sanity_zenoh.protocol import COMMAND, ReceiverStatus
from sanity_zenoh.recording import EventRecorder
from sanity_zenoh.transport import SanityTransport
from sanity_zenoh.transport_test import PING, summarize_transport


def load_config(path, workspace, devices=None):
    config=json.loads(Path(path).read_text())
    if set(config)!={'version','transport','devices'} or config.get('version')!=1 or not config.get('devices'):
        raise ValueError('Expected version1 and a nonempty devices list')
    if set(config['transport'])!={'listen','telemetry_listen'}:
        raise ValueError('Expected shared control and diagnostic listeners')
    for key,protocol in [('listen','udp'),('telemetry_listen','tcp')]:
        endpoint=config['transport'][key]
        prefix,address=endpoint.split('/',1);host,port=address.rsplit(':',1)
        ip=ipaddress.ip_address(host)
        if prefix!=protocol or ip.version!=4 or ip.is_unspecified or ip.is_multicast or not 1<=int(port)<=65535:
            raise ValueError('Use an explicit IPv4 UDP control/TCP diagnostic interface')
    seen={key:set() for key in ('id','name','ip')}
    for drone in config['devices']:
        if set(drone)!={'id','name','ip','calibration'}:raise ValueError('Unknown/missing device fields')
        if type(drone['id']) is not int or not 1<=drone['id']<=255:
            raise ValueError('Device IDs must be integers1..255')
        if not drone['name'] or not str(drone['name']).replace('_','').isalnum():
            raise ValueError('Device names must be alphanumeric')
        ipaddress.ip_address(drone['ip'])
        for key in seen:
            if drone[key] in seen[key]:raise ValueError('Duplicate '+key)
            seen[key].add(drone[key])
        profile=Path(drone['calibration'])
        if not profile.is_absolute():profile=workspace/profile
        drone['calibration']=str(profile.resolve())
        FirmwareCollective(profile)
    if devices:
        if len(set(devices))!=len(devices) or set(devices)-seen['id']:
            raise ValueError('Select distinct configured device IDs')
        config['devices']=[d for d in config['devices'] if d['id'] in devices]
    return config


class Feedback:
    def __init__(self, drone, recorder, phase):
        self.drone,self.recorder,self.phase=drone,recorder,phase
        self.lock=threading.Lock()
        self.last={}
        self.health=None
        self.guard_error=None
        self.event_counts=Counter()
        self.boot_ids=set()


    def record(self, event):
        now=time.monotonic()
        try:
            if event['kind']=='zenoh_tx_command':
                raw=bytes.fromhex(event['payload_hex']);header=COMMAND.unpack(raw)
                if header[0] not in (b'SZC2',b'SZC3') or header[3]!=self.drone['id'] or header[4] or any(raw[32:]):
                    raise ValueError('Disabled-zero physical CTBR guard failed')
            if event['kind']=='zenoh_rx_status':
                if ReceiverStatus.decode(bytes.fromhex(event['payload_hex'])).device!=self.drone['id']:
                    raise ValueError('Cross-device status routing')
            with self.lock:
                self.event_counts[event['kind']]+=1
                if event['kind']=='zenoh_rx_trace' and 'decode_error' not in event:self.last['trace']=now
                for record in event.get('telemetry',{}).get('records',[]):
                    if 'esp_health' in record:
                        self.health=record['esp_health'];self.last['health']=now;self.boot_ids.add(self.health['boot_id'])
                    if record['type']==2 and record['msp_command'] in (102,104):
                        self.last[str(record['msp_command'])]=now
                    if 'motor_outputs_external_fc' in record and any(v>1000 for v in record['motor_outputs_external_fc'][:4]):
                        raise ValueError('FC reports active motor outputs during disabled test')
                    if any(m['rpm'] for m in record.get('motors',[])):
                        raise ValueError('FC reports nonzero motor RPM during disabled test')
        except (ValueError,KeyError,TypeError) as error:self.guard_error=str(error)
        self.recorder({'phase':self.phase['value'],'source_device':self.drone['id'],**event})


    def snapshot(self, sender):
        now=time.monotonic()
        with sender.lock:
            status=sender.status
            status_age=(now-sender.status_time)*1000 if status is not None else None
            owner=sender.session_id;calibration=sender.ctbr_mapping.calibration_id
        blockers=[]
        if status is None:blockers.append('receiver_status_missing')
        else:
            if status_age>=1000:blockers.append('receiver_status_stale')
            if status.device!=self.drone['id']:blockers.append('wrong_receiver_device')
            if status.session!=owner:blockers.append('disabled_ownership_not_acknowledged')
            if status.enabled:blockers.append('receiver_unexpectedly_enabled')
            if status.failsafe:blockers.append('receiver_failsafe')
            if not status.fc_ready:blockers.append('FC_not_ready')
            if status.fc_age_ms>=150:blockers.append('FC_status_stale')
            if status.calibration_id!=calibration:blockers.append('calibration_mismatch')
        with self.lock:
            ages={key:(now-self.last[key])*1000 if key in self.last else None for key in ('trace','health','102','104')}
            limits={'trace':1000.,'health':2000.,'102':500.,'104':500.}
            labels={'trace':'timing_trace','health':'ESP_health','102':'FC_gyro','104':'FC_motor_feedback'}
            for key,limit in limits.items():
                if ages[key] is None:blockers.append(labels[key]+'_missing')
                elif ages[key]>=limit:blockers.append(labels[key]+'_stale')
            health=dict(self.health) if self.health else None
            counts=dict(self.event_counts);boots=sorted(self.boot_ids)
        return {'feedback_ready':not blockers,'blocking':blockers,'status_flags':status.flags if status else None,
            'status_age_ms':status_age,'expected_sender_session':owner,'latest_receiver_status':status.__dict__ if status else None,
            'feedback_age_ms':ages,'latest_health':health,'observed_boot_ids':boots,'event_counts':counts}


    def ready(self, sender):
        return self.snapshot(sender)['feedback_ready']


def distribution(values):
    if not values:return {'samples':0,'min':None,'median':None,'p95':None,'p99':None,'max':None}
    q=np.percentile(values,[50,95,99,100])
    return dict(samples=len(values),min=float(min(values)),**dict(zip(('median','p95','p99','max'),map(float,q))))


def maximum_gap_ms(times, start, end):
    values=sorted(t for t in times if start<=t<=end)
    if not values:return (end-start)/1e6
    return max(np.diff([start,*values,end]))/1e6


def assess_device(path, drone, bounds, owner, baseline, gap_limit_ms, shutdown_acknowledged):
    # Retain compact timing records, not full raw gyro/motor payloads, in RAM.
    # The complete original payloads remain in the on-disk JSONL.
    events=[];signals=defaultdict(list);health=[];status_rows=[];errors=[];bad_wire=0;counts=Counter();voltages=[]
    start,end=bounds
    with Path(path).open() as stream:
        for line in stream:
            row=json.loads(line);kind=row['kind'];measured=row.get('phase')=='telemetry_on'
            if row.get('source_device')!=drone['id']:errors.append('Incorrect source_device')
            compact={key:row[key] for key in ('host_ns','kind','phase')}
            if 'decode_error' in row:
                compact['decode_error']=row['decode_error']
                if measured:errors.append(row['decode_error'])
            if kind=='zenoh_tx_command':
                compact['payload_hex']=row['payload_hex'];compact['enabled']=row['enabled']
                raw=bytes.fromhex(row['payload_hex']);h=COMMAND.unpack(raw)
                if h[3]!=drone['id'] or h[4] or any(raw[32:]):bad_wire+=1
                if measured:signals['tx'].append(row['host_ns'])
            elif kind=='zenoh_rx_status':
                compact['payload_hex']=row['payload_hex']
                if measured:
                    s=ReceiverStatus.decode(bytes.fromhex(row['payload_hex']));status_rows.append(s)
                    signals['status'].append(row['host_ns'])
                    if s.device!=drone['id']:errors.append('Cross-device receiver status')
            elif kind=='zenoh_rx_trace' and 'decode_error' not in row:
                compact.update(records=row['records'],dropped=row['dropped'])
                if measured:signals['trace'].append(row['host_ns'])
            elif kind=='zenoh_rx_pong' and 'decode_error' not in row:
                compact.update({k:v for k,v in row.items() if k not in ('payload_hex','unix_ns')})
                if measured:signals['pong'].append(row['host_ns'])
            elif kind=='zenoh_rx_fc_telemetry' and 'telemetry' in row:
                records=[]
                for record in row['telemetry']['records']:
                    if measured:
                        counts[str(record['msp_command'])]+=1
                        if 'voltage_V' in record:voltages.append(record['voltage_V'])
                        if 'decode_error' in record:errors.append(record['decode_error'])
                        if record['type']==2:signals[str(record['msp_command'])].append(row['host_ns'])
                        if 'esp_health' in record:health.append(record['esp_health']);signals['health'].append(row['host_ns'])
                        if any(v>1000 for v in record.get('motor_outputs_external_fc',[])[:4]):errors.append('Active FC motor output')
                        if any(m['rpm'] for m in record.get('motors',[])):errors.append('Nonzero motor RPM')
                    if record['type']==1:
                        records.append({k:record[k] for k in ('type','epoch','command_session','command_sequence')})
                    elif 'esp_health' in record:records.append({'type':5,'esp_health':record['esp_health']})
                compact['telemetry']={'records':records,'esp_dropped_records':row['telemetry']['esp_dropped_records']}
            else:continue
            events.append(compact)
    report=summarize_transport(events,gap_limit_ms)
    boots={h['boot_id'] for h in health}|({baseline['boot_id']} if baseline else set())
    epochs={s.epoch for s in status_rows}|({baseline['epoch']} if baseline else set())
    gaps={key:float(maximum_gap_ms(signals[key],start,end)) for key in ('status','trace','health','102','104')}
    rejected=max((s.rejected for s in status_rows),default=0)-(baseline['rejected'] if baseline else 0)
    checks={'disabled_zero_wire':bad_wire==0,'no_decode_or_routing_errors':not errors,
        'own_sender_session':bool(status_rows) and all(s.session==owner for s in status_rows),
        'no_sampled_receiver_failsafe':bool(status_rows) and all(not s.enabled and not s.failsafe for s in status_rows),
        'FC_ready_throughout':bool(status_rows) and all(s.fc_ready and s.fc_age_ms<150 for s in status_rows),
        'no_new_rejections':rejected==0,'no_boot_reset':bool(health) and len(boots)==1,
        'no_control_epoch_change':len(epochs)==1,'status_gaps_within_benchmark':gaps['status']<=gap_limit_ms,
        'continuous_diagnostics':gaps['trace']<=1000 and gaps['health']<=2000,
        'continuous_FC_gyro_motor_feedback':gaps['102']<=500 and gaps['104']<=500,
        'probe_replies_received':bool(signals['pong']),'shutdown_disabled_acknowledged':shutdown_acknowledged}
    report.update(device=drone['id'],name=drone['name'],passed=report['passed'] and all(checks.values()),
        additional_checks=checks,errors=sorted(set(errors)),boot_ids=sorted(boots),control_epochs=sorted(epochs),
        rejection_increment=rejected,measurement_tail_and_head_included_max_gap_ms=gaps,
        FC_query_record_counts=dict(counts),delivered_FC_voltage_V=distribution(voltages),host_tx_interval_ms=distribution((np.diff(signals['tx'])/1e6).tolist()),
        status_arrival_interval_ms=distribution((np.diff(signals['status'])/1e6).tolist()),
        latest_health=health[-1] if health else None,
        evidence_scope='Normal flight firmware; all commands disabled/zero. RTT includes both links and host/ESP scheduling; no one-way Wi-Fi or FC actuation delay is inferred.')
    return report


def run(config, output, duration, startup_timeout=60., warmup=5., gap_limit_ms=50., stop=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    (output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    sources=[Path(__file__),Path(__file__).with_name('client.py'),Path(__file__).with_name('transport.py'),Path(__file__).with_name('transport_test.py'),*[Path(d['calibration']) for d in config['devices']]]
    (output/'manifest.json').write_text(json.dumps({'unix_ns':time.time_ns(),'host_monotonic_ns':time.monotonic_ns(),'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}},indent=2)+'\n')
    phase={'value':'startup'};stop=stop or threading.Event();recorders={};feedback={};senders={};transport=None
    error=None;bounds=None;baseline={};owners={};shutdown={};completed=False;startup_diagnostics={}
    started=time.monotonic()
    try:
        transport=SanityTransport(**config['transport'])
        for drone in config['devices']:
            device=drone['id'];folder=output/drone['name'];folder.mkdir()
            recorders[device]=EventRecorder(folder/'events.jsonl')
            feedback[device]=Feedback(drone,recorders[device],phase)
            senders[device]=SanitySender(device=device,transport=transport,
                ctbr_mapping=FirmwareCollective(drone['calibration']),physical_units=True,
                diagnostics=True,disabled_only=True,on_event=feedback[device].record)
        ready_since=None;next_print=0.;probe_id=0;next_probe=0.
        while not stop.is_set():
            now=time.monotonic()
            for device,sender in senders.items():
                sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.)
                if feedback[device].guard_error:raise ValueError(feedback[device].guard_error)
                if recorders[device].error:raise RuntimeError(recorders[device].error)
            if now>=next_print:
                print(json.dumps({'phase':phase['value'],'elapsed_s':round(now-started,1),
                    'devices':{str(d):{k:v for k,v in feedback[d].snapshot(s).items() if k in ('feedback_ready','blocking','status_flags','status_age_ms')} for d,s in senders.items()}}),flush=True)
                next_print=now+15.
            if bounds is None:
                ready=all(feedback[d].ready(s) for d,s in senders.items())
                ready_since=(now if ready_since is None else ready_since) if ready else None
                if now-started>startup_timeout:raise TimeoutError('Startup requires all receivers: disabled ownership, FC ready, health, traces and gyro/motor telemetry')
                if ready_since is not None and now-ready_since>=warmup:
                    for d,s in senders.items():
                        baseline[d]={'boot_id':feedback[d].health['boot_id'],'epoch':s.status.epoch,'rejected':s.status.rejected}
                        owners[d]=s.session_id
                    bounds=[time.monotonic_ns(),None];phase['value']='telemetry_on';measurement_start=now
                    print('All receivers ready. Simultaneous disabled measurement started.',flush=True)
            else:
                if now-measurement_start>=duration:completed=True;break
                # Read-only timing probes; no telemetry-toggle RPC or gain changes.
                if now>=next_probe:
                    for device,sender in senders.items():
                        sender.probe_publisher.put(PING.pack(b'SZP1',1,device,0,probe_id,time.monotonic_ns()))
                    probe_id=(probe_id+1)&0xffffffff;next_probe=now+.05
            if bounds is None:probe_id=0;next_probe=0.
            time.sleep(.02)
    except BaseException as caught:error=f'{type(caught).__name__}: {caught}'
    finally:
        if bounds is not None:bounds[1]=time.monotonic_ns()
        if bounds is None:startup_diagnostics={str(d):feedback[d].snapshot(s) for d,s in senders.items()}
        phase['value']='cleanup'
        for device,sender in senders.items():
            try:sender.close();shutdown[device]=sender.shutdown_acknowledged
            except Exception as caught:shutdown[device]=False;error=error or str(caught)
        if transport:
            try:transport.close()
            except Exception as caught:error=error or str(caught)
        for recorder in recorders.values():
            try:recorder.close()
            except Exception as caught:error=error or str(caught)
            if recorder.error:error=error or recorder.error
    reports={}
    if bounds is not None:
        for drone in config['devices']:
            d=drone['id']
            try:report=assess_device(output/drone['name']/'events.jsonl',drone,bounds,owners[d],baseline[d],gap_limit_ms,shutdown.get(d,False))
            except Exception as caught:report={'passed':False,'analysis_error':str(caught)}
            reports[str(d)]=report;(output/drone['name']/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    else:
        for drone in config['devices']:
            d=drone['id'];folder=output/drone['name'];folder.mkdir(exist_ok=True)
            report={'passed':False,'stage':'startup','measurement_completed':False,
                'diagnostics':startup_diagnostics.get(str(d),{'blocking':['sender_not_created']}),
                'shutdown_disabled_acknowledged':shutdown.get(d,False)}
            reports[str(d)]=report;(folder/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    result={'passed':completed and error is None and bool(reports) and all(r['passed'] for r in reports.values()),
        'completed':completed,'interrupted':stop.is_set(),'error':error,'devices':reports,'startup_diagnostics':startup_diagnostics,
        'measurement_host_ns':bounds,'measurement_duration_s':(bounds[1]-bounds[0])/1e9 if bounds else 0.,
        'requested_duration_s':duration,'disabled_only':True,'shutdown_acknowledgements':shutdown,
        'benchmark_gap_ms':gap_limit_ms,'command_lease_s':1.,'source_hz':50,'transmitter_hz_per_device':100,
        'probe_hz_per_device':20,'scope':'Shared UDP control/status and TCP diagnostics, normal flight firmware; no arming, motor tests, ROS or FC settings changes.'}
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('devices','startup_diagnostics')},indent=2),flush=True)
    print('Results:',output,flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    root=Path(__file__).resolve().parents[6]
    parser.add_argument('--config',type=Path,default=Path(__file__).resolve().parents[1]/'config/fleet_comms_three.json')
    parser.add_argument('--devices',nargs='+',type=int)
    parser.add_argument('--duration',type=float,default=300.)
    parser.add_argument('--startup-timeout',type=float,default=60.)
    parser.add_argument('--warmup',type=float,default=5.)
    parser.add_argument('--gap-limit-ms',type=float,default=50.)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--plan-only',action='store_true')
    args=parser.parse_args()
    if not all(math.isfinite(x) for x in (args.duration,args.startup_timeout,args.warmup,args.gap_limit_ms)) or not 1<=args.duration<=3600 or not 1<=args.startup_timeout<=120 or not 0<=args.warmup<args.startup_timeout or not 0<args.gap_limit_ms<=1000:
        parser.error('Invalid finite duration/startup/warmup/gap limits')
    config=load_config(args.config,root,args.devices)
    if args.plan_only:print(json.dumps({'configuration':config,'duration_s':args.duration,'disabled_only':True},indent=2));return
    output=args.output or root/'outputs'/('sanity_fleet_comms_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'))
    stop=threading.Event();previous={s:signal.signal(s,lambda *_:stop.set()) for s in (signal.SIGINT,signal.SIGTERM)}
    try:result=run(config,output,args.duration,args.startup_timeout,args.warmup,args.gap_limit_ms,stop)
    finally:
        for s,handler in previous.items():signal.signal(s,handler)
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
