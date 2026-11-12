"""Disabled-only, matched Zenoh / UDP-unicast comparison with bounded diagnostics."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import signal
import threading
import time

from sanity_zenoh.arrival_metrics import summarize_windows
from sanity_zenoh.client import SanitySender
from sanity_zenoh.datagram_transport import SanityDatagramTransport
from sanity_zenoh.firmware_collective import FirmwareCollective
from sanity_zenoh.fleet_comms import load_config, Feedback, distribution
from sanity_zenoh.protocol import COMMAND, ReceiverStatus
from sanity_zenoh.recording import EventRecorder
from sanity_zenoh.transport import SanityTransport
from sanity_zenoh.transport_test import PING


def assess(path, device, session, expected_level, shutdown, duration, require_fc=False, baseline=None):
    with Path(path).open() as stream:rows=[json.loads(line) for line in stream]
    measured=[r for r in rows if r.get('phase')=='measurement']
    status=[];health=[];rtt=[];tx=[];errors=[];last_feedback={};traffic=Counter()
    for row in rows:
        if row['kind']=='zenoh_tx_command':
            raw=bytes.fromhex(row['payload_hex']);h=COMMAND.unpack(raw)
            if h[3]!=device or h[4] or any(raw[32:]):raise ValueError('Disabled-zero guard or routing violation in recording')
    for row in measured:
        traffic[row['kind']]+=len(bytes.fromhex(row.get('payload_hex','')))
        if 'decode_error' in row:errors.append(row['decode_error'])
        if row['kind']=='zenoh_rx_status':
            status.append((row['host_ns'],ReceiverStatus.decode(bytes.fromhex(row['payload_hex']))));last_feedback['status']=row['host_ns']
        if row['kind']=='zenoh_tx_command':tx.append(row['host_ns'])
        if row['kind']=='zenoh_rx_pong' and 'decode_error' not in row:rtt.append((row['host_ns']-row['sent_ns'])/1e6)
        for record in row.get('telemetry',{}).get('records',[]):
            if 'esp_health' in record:health.append(record['esp_health']);last_feedback['health']=row['host_ns']
    windows=summarize_windows(measured,session)
    statuses=[s for _,s in status];boot_ids={h['boot_id'] for h in health};epochs={s.epoch for s in statuses}
    if baseline:
        boot_ids.add(baseline['boot_id']);epochs.add(baseline['epoch'])
    def max_gap(values):return max(((b-a)/1e6 for a,b in zip(values,values[1:])),default=None)
    # Include head/tail so a missing final window/status cannot look like a good maximum.
    begin=min((r['host_ns'] for r in measured),default=0);end=max((r['host_ns'] for r in measured),default=0)
    status_gap=max_gap([begin,*[t for t,_ in status],end]);health_age=(end-last_feedback['health'])/1e9 if 'health' in last_feedback else None
    checks={'disabled_zero_wire':True,'no_decode_errors':not errors,'status_received':bool(status),
        'receiver_disabled':bool(statuses) and all(not s.enabled and not s.failsafe for s in statuses),
        'own_sender_session':bool(statuses) and all(s.session==session for s in statuses),
        'FC_ready_throughout':not require_fc or (bool(statuses) and all(s.fc_ready and s.fc_age_ms<150 for s in statuses)),
        'no_new_rejections':bool(statuses) and max(s.rejected for s in statuses)==(baseline['rejected'] if baseline else statuses[0].rejected),
        'one_receiver_epoch':len(epochs)==1,'one_boot':len(boot_ids)==1,
        'arrival_windows_received':windows['received'],'contiguous_arrival_windows':windows['contiguous'],
        'one_arrival_context':windows['contexts']==1,'diagnostic_level_matches':windows['profiles']==[expected_level],
        'command_gap_benchmark':windows['maximum_accepted_gap_ms'] is not None and windows['maximum_accepted_gap_ms']<=50,
        'status_gap_benchmark':status_gap is not None and status_gap<=50,
        'probe_replies_received':bool(rtt),'health_fresh_at_end':health_age is not None and health_age<=2.,
        'disabled_shutdown_acknowledged':bool(shutdown)}
    arrival_hosts=[r['host_ns'] for r in measured if any(q.get('type')==8 and q['session']==session for q in r.get('records',[]))]
    checks['arrival_reports_cover_measurement']=bool(arrival_hosts) and max_gap([begin,*arrival_hosts,end])<=1000
    return {'passed':all(checks.values()),'checks':checks,'arrival_windows':windows,'probe_RTT_ms':distribution(rtt),
        'host_publication_interval_ms':distribution([(b-a)/1e6 for a,b in zip(tx,tx[1:])]),
        'max_host_status_gap_ms':status_gap,'trace_and_telemetry_payload_B_s':{k:v/duration for k,v in traffic.items()},
        'commands_published':len(tx),'boot_ids':sorted(boot_ids),'control_epochs':sorted(epochs),
        'latest_health':health[-1] if health else None,'FC_ready_observed':any(s.fc_ready for s in statuses),
        'scope':'Disabled-only comparison; RTT is not one-way Wi-Fi or actuation delay. No command-history reconstruction from sampled traces. FC readiness check is waived only when radio-only is explicitly selected.'}


def run(config, output, backend='zenoh', level=1, duration=300., radio_only=False, startup_timeout=60., zenoh_profile='udp-split'):
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    stop=threading.Event();previous={s:signal.signal(s,lambda *_:stop.set()) for s in (signal.SIGINT,signal.SIGTERM)}
    phase={'value':'startup'};transport=None;senders={};logs={};feedback={};completed=False;error=None;shutdown={};owners={};measurement_start=None;measurement_duration=0.;baselines={}
    (out/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    sources=[Path(__file__),Path(__file__).with_name('client.py'),Path(__file__).with_name('datagram_transport.py'),Path(__file__).with_name('arrival_metrics.py')]
    plan={'backend':backend,'diagnostic_level':level,'duration_s':duration,'radio_only':radio_only,'disabled_only':True,
          'source_hz':50,'transmitter_hz':100,'probe_hz':20,'command_lease_s':1.,'benchmark_ms':50,'zenoh_profile':zenoh_profile,
          'unix_ns':time.time_ns(),'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
    if backend=='udp-unicast':
        plan['udp_host_listen']=config['transport']['listen'].split('/',1)[1].rsplit(':',1)[0]+':7450'
        plan['udp_receiver_endpoints']={str(d['id']):d['ip']+':7451' for d in config['devices']}
    (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');print('Recording:',out,flush=True)
    try:
        if backend=='zenoh':
            transport=SanityTransport(config['transport']['listen']) if zenoh_profile=='udp-shared' else SanityTransport(**config['transport'])
        else:
            host=config['transport']['listen'].split('/',1)[1].rsplit(':',1)[0]
            transport=SanityDatagramTransport(host+':7450',{d['id']:d['ip']+':7451' for d in config['devices']})
        for drone in config['devices']:
            device=drone['id'];folder=out/drone['name'];folder.mkdir();logs[device]=EventRecorder(folder/'events.jsonl');feedback[device]=Feedback(drone,logs[device],phase)
            senders[device]=SanitySender(device=device,transport=transport,ctbr_mapping=FirmwareCollective(drone['calibration']),physical_units=True,diagnostics=True,disabled_only=True,on_event=feedback[device].record);owners[device]=senders[device].session_id
        started=time.monotonic();ready_since=None;next_print=started;next_probe=0.;probe=0
        while not stop.is_set():
            now=time.monotonic();ready={}
            for device,sender in senders.items():
                sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.)
                if feedback[device].guard_error:raise ValueError(feedback[device].guard_error)
                if logs[device].error:raise RuntimeError(logs[device].error)
                status=sender.receiver_status()
                snapshot=feedback[device].snapshot(sender)
                ready[device]=bool(status and status.session==sender.session_id and not status.enabled and not status.failsafe and status.calibration_id==sender.ctbr_mapping.calibration_id and (radio_only or (status.fc_ready and status.fc_age_ms<150)) and snapshot['feedback_age_ms']['health'] is not None and snapshot['feedback_age_ms']['health']<2000 and time.monotonic()-sender.diagnostic_times.get('trace',-float('inf'))<1.)
            if now>=next_print:print(json.dumps({'phase':phase['value'],'elapsed_s':round(now-started,1),'ready':ready}),flush=True);next_print=now+15.
            if measurement_start is None:
                if now-started>startup_timeout:raise TimeoutError('Need disabled ownership, matching calibration, health/arrival diagnostics'+('' if radio_only else ' and fresh FC status'))
                ready_since=(now if ready_since is None else ready_since) if all(ready.values()) else None
                if ready_since is not None and now-ready_since>=2.:
                    baselines={d:{'epoch':sender.status.epoch,'rejected':sender.status.rejected,'boot_id':feedback[d].health['boot_id']} for d,sender in senders.items()}
                    (out/'measurement_start.json').write_text(json.dumps({'host_ns':time.monotonic_ns(),'unix_ns':time.time_ns(),'baselines':baselines},indent=2)+'\n')
                    phase['value']='measurement';measurement_start=now;print('All receivers ready; common measurement started.',flush=True)
            else:
                if now-measurement_start>=duration:completed=True;break
                if now>=next_probe:
                    for device,sender in senders.items():sender.probe_publisher.put(PING.pack(b'SZP1',1,device,0,probe,time.monotonic_ns()))
                    probe=(probe+1)&0xffffffff;next_probe=now+.05
            time.sleep(.02)
    except BaseException as caught:error=f'{type(caught).__name__}: {caught}'
    finally:
        if measurement_start is not None:measurement_duration=time.monotonic()-measurement_start
        phase['value']='cleanup'
        for device,sender in senders.items():
            try:sender.close();shutdown[device]=sender.shutdown_acknowledged
            except Exception as caught:shutdown[device]=False;error=error or str(caught)
        if transport:
            try:transport.close()
            except Exception as caught:error=error or str(caught)
        for log in logs.values():
            log.close()
            if log.error:error=error or log.error
        for sig,handler in previous.items():signal.signal(sig,handler)
    reports={}
    if measurement_start is not None:
        for drone in config['devices']:
            device=drone['id']
            try:report=assess(out/drone['name']/'events.jsonl',device,owners[device],level,shutdown.get(device,False),measurement_duration,not radio_only,baselines.get(device))
            except Exception as caught:report={'passed':False,'assessment_error':str(caught)}
            reports[str(device)]=report;(out/drone['name']/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    result={'passed':completed and error is None and bool(reports) and all(r['passed'] for r in reports.values()),'completed':completed,'error':error,'devices':reports,'shutdown_acknowledgements':shutdown,'measurement_duration_s':measurement_duration,'transport_stats':dict(getattr(transport,'stats',{})) if transport else {},**plan}
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ('devices','source_sha256')},indent=2));return result


def main():
    root=Path(__file__).resolve().parents[6];parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zenoh-profile',choices=('udp-split','udp-shared'),default='udp-split');parser.add_argument('--backend',choices=('zenoh','udp-unicast'),default='zenoh');parser.add_argument('--diagnostic-level',choices=('full','minimal','control-only'),default='minimal')
    parser.add_argument('--config',type=Path,default=root/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/fleet_comms_three.json');parser.add_argument('--devices',nargs='+',type=int)
    parser.add_argument('--duration',type=float,default=300.);parser.add_argument('--radio-only',action='store_true');parser.add_argument('--plan-only',action='store_true');parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not 1<=args.duration<=3600:parser.error('Duration must be1..3600 seconds')
    config=load_config(args.config,root,args.devices);level=('full','minimal','control-only').index(args.diagnostic_level)
    if args.plan_only:print(json.dumps({'config':config,'backend':args.backend,'zenoh_profile':args.zenoh_profile,'diagnostic_level':level,'duration_s':args.duration,'disabled_only':True},indent=2));return
    out=args.output or root/'outputs'/('sanity_comms_compare_'+args.backend+'_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'))
    if not run(config,out,args.backend,level,args.duration,args.radio_only,zenoh_profile=args.zenoh_profile)['passed']:raise SystemExit(1)


if __name__=='__main__':main()
