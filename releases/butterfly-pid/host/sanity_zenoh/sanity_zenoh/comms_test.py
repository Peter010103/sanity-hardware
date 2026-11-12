"""Zenoh link recording: STOP-only default; optional props-off armed minimum."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from sanity_zenoh.client import SanitySender
from sanity_zenoh.protocol import COMMAND, ReceiverStatus
from sanity_zenoh.recording import EventRecorder


def analyse(events, warmup_s=3., armed_minimum=False):
    events=list(events)
    tx=[e for e in events if e['kind']=='zenoh_tx_command']
    rx=[e for e in events if e['kind']=='zenoh_rx_status']
    decoded=[(e,ReceiverStatus.decode(bytes.fromhex(e['payload_hex']))) for e in rx]
    origin=min((e['host_ns'] for e in events),default=0)
    steady=[(e,s) for e,s in decoded if (e['host_ns']-origin)/1e9>=warmup_s]
    sent={}
    wire=[]
    for e in tx:
        c=COMMAND.unpack(bytes.fromhex(e['payload_hex']))
        sent[c[7]]=(e,c)
        wire.append(c)
    acknowledgements=[]
    margins=[]
    for e,s in steady:
        pair=sent.get(s.sequence)
        if pair is None or pair[1][6]!=s.session:
            continue
        command,c=pair
        age=(s.receiver_ms-c[8])&0xffffffff
        margins.append((e['host_ns'],c[9]-age))
        acknowledgements.append((e['host_ns']-command['host_ns'])/1e6)
    def stats(values):
        return dict(zip(['min','median','p95','p99','max'],np.percentile(values,[0,50,95,99,100]).tolist())) if len(values) else None
    flags=[bool(s.failsafe) for e,s in steady]
    episodes=sum(v and (i==0 or not flags[i-1]) for i,v in enumerate(flags))
    offsets=np.array([e['host_ns']/1e6-s.receiver_ms for e,s in steady])
    extra=offsets-offsets.min() if len(offsets) else []
    rejections={}
    for e,s in steady:
        rejections[s.epoch]=max(s.rejected,rejections.get(s.epoch,0))
    checks={'only_allowed_neutral_commands':bool(wire) and all(c[2]==1 and (not c[4] or armed_minimum) and c[11:]==(1500,1500,1500,1000) for c in wire),
        'received_status':bool(steady),'no_sampled_failsafe':not any(flags),'no_rejections':not any(rejections.values()),
        'single_receiver_epoch':len({s.epoch for e,s in steady})==1}
    metrics={'tx_packets':len(tx),'rx_statuses':len(rx),'steady_statuses':len(steady),'sampled_failsafe_episodes':episodes,
        'sampled_failsafe_statuses':sum(flags),'max_rejected_by_epoch':{str(k):v for k,v in rejections.items()},
        'tx_interval_ms':stats(np.diff([e['host_ns'] for e in tx])/1e6),
        'status_arrival_interval_ms':stats(np.diff([e['host_ns'] for e,s in steady])/1e6),
        'command_to_echoed_status_ms':stats(acknowledgements),'lease_remaining_at_status_ms':stats([v for t,v in margins]),
        'extra_status_delivery_delay_ms':stats(extra),'fc_status_age_ms':stats([s.fc_age_ms for e,s in steady]),
        'reported_voltage_V':stats([s.voltage_mv/1000 for e,s in steady if s.voltage_mv]),
        'voltage_unavailable_statuses':sum(s.voltage_mv==0 for e,s in steady)}
    return {'sampled_checks_passed':bool(all(checks.values())),'checks':checks,'metrics':metrics,
        'scope':('Props-off armed-minimum command test; receiver enable is observed, actual FC arm/motor state not independently telemetered.' if armed_minimum else 'Disabled-only transport test; no motor activity. Does not qualify armed flight or motor-on EMI/power effects.'),
        'latency_definitions':{'command_to_echoed_status':'Host publication -> host receipt of status naming that accepted command. Includes status cadence, both transport directions, receiver scheduling; not FC actuation latency.',
        'extra_status_delivery':'Host-arrival minus receiver-clock offset relative to the smallest observed offset. Clock drift can contribute over long captures; not absolute one-way delay. Use host echo latency and receiver lease margin for deadline conclusions.',
        'lease_remaining':'TTL minus receiver-clock age of the last accepted command at status publication; no cross-clock subtraction.',
        'sequence_skips':'Status is slower than command publication; skipped acknowledged sequence IDs do not establish packet loss.',
        'failsafe_sampling':'An expiry between status samples may recover on a fresh disabled command. Zero sampled faults does not prove zero armed-path expiries.'}},margins


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--armed-minimum',action='store_true',help='ARM with neutral rates and minimum throttle only; requires props removed')
    parser.add_argument('--props-removed',action='store_true')
    parser.add_argument('--duration',type=float,default=300.)
    parser.add_argument('--listen',default='tcp/10.0.0.200:7447')
    parser.add_argument('--rate',type=float,default=100.)
    a=parser.parse_args(argv)
    if not 5<=a.duration<=3600 or not 1<=a.rate<=100:
        parser.error('duration5..3600seconds; rate1..100Hz')
    if a.armed_minimum and not a.props_removed:
        parser.error('--armed-minimum requires --props-removed')
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'metadata.json').write_text(json.dumps({'started_utc':datetime.now(timezone.utc).isoformat(),
        'arguments':{k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},'arming_commands_allowed':a.armed_minimum,
        'commands':'armed minimum throttle only, props removed' if a.armed_minimum else 'disabled CTBR neutral only','source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(__file__).with_name('client.py'),Path(__file__).with_name('protocol.py')]}},indent=2)+'\n')
    for source in [Path(__file__),Path(__file__).with_name('client.py'),Path(__file__).with_name('protocol.py'),Path(__file__).with_name('recording.py')]:
        (a.output/('source_'+source.name)).write_bytes(source.read_bytes())
    recorder=EventRecorder(a.output/'events.jsonl')
    sender=None
    error=None
    start=time.monotonic()
    progress=start+15
    try:
        sender=SanitySender(listen=a.listen,rate_hz=a.rate,on_event=recorder)
        recorder({'kind':'test_phase','phase':'disabled_claim'})
        if a.armed_minimum:
            deadline=time.monotonic()+10
            while True:
                sender.ctbr(False,0.,(0.,0.,0.))
                status=sender.receiver_status()
                if status and status.session==sender.session_id and not status.enabled and not status.failsafe:
                    break
                if time.monotonic()>deadline:
                    raise TimeoutError('No fresh disabled receiver acknowledgement')
                time.sleep(.01)
            recorder({'kind':'test_phase','phase':'armed_minimum_request'})
            while True:
                sender.ctbr(True,0.,(0.,0.,0.))
                status=sender.receiver_status()
                if status and status.enabled and not status.failsafe:
                    break
                if time.monotonic()>deadline:
                    raise TimeoutError('No armed-minimum receiver acknowledgement')
                time.sleep(.01)
            recorder({'kind':'test_phase','phase':'armed_minimum_active'})
            print('Receiver enabled at minimum throttle; props-off test active. No positive thrust commands.',flush=True)
        while time.monotonic()-start<a.duration:
            if a.armed_minimum:
                status=sender.receiver_status()
                if status is None or status.failsafe or sender.rearm_required or not status.enabled:
                    raise RuntimeError(sender.stop_reason or 'Receiver enabled-state/lease lost')
            sender.ctbr(a.armed_minimum,0.,(0.,0.,0.))
            if recorder.error:
                raise RuntimeError(recorder.error)
            if time.monotonic()>=progress:
                status=sender.receiver_status()
                print(json.dumps({'elapsed_s':round(time.monotonic()-start,1),'status_flags':status.flags if status else None,
                    'rejected':status.rejected if status else None,'voltage_mv':status.voltage_mv if status else None,'output':'ARMED_MINIMUM' if a.armed_minimum else 'DISABLED'}),flush=True)
                progress=time.monotonic()+15
            time.sleep(1/a.rate)
    except BaseException as exc:
        error=f'{type(exc).__name__}: {exc}'
    finally:
        if sender:
            recorder({'kind':'test_phase','phase':'disabled_shutdown'})
            end=time.monotonic()+.3
            while time.monotonic()<end:
                sender.ctbr(False,0.,(0.,0.,0.))
                time.sleep(.01)
            sender.close()
        recorder.close()
    events=[json.loads(line) for line in (a.output/'events.jsonl').read_text().splitlines()]
    summary,margins=analyse(events,armed_minimum=a.armed_minimum)
    summary.update(elapsed_s=time.monotonic()-start,error=error,recording_error=recorder.error,completed=error is None)
    summary['sampled_checks_passed'] &= error is None and recorder.error is None
    statuses=[ReceiverStatus.decode(bytes.fromhex(e['payload_hex'])) for e in events if e['kind']=='zenoh_rx_status']
    summary['final_receiver_enabled']=statuses[-1].enabled if statuses else None
    summary['armed_minimum_requested']=a.armed_minimum
    summary['sampled_checks_passed'] &= bool(statuses and not statuses[-1].enabled)

    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(10,6),layout='constrained',sharex=True)
    t0=events[0]['host_ns'] if events else time.monotonic_ns()
    if margins:
        axes[0].plot([(t-t0)/1e9 for t,m in margins],[m for t,m in margins],lw=.7)
    axes[0].axhline(0,color='red',ls='--');axes[0].set(ylabel='lease remaining [ms]',title='ESP-reported command lease margin')
    rx=[e for e in events if e['kind']=='zenoh_rx_status']
    axes[1].plot([(e['host_ns']-t0)/1e9 for e in rx[1:]],np.diff([e['host_ns'] for e in rx])/1e6,lw=.7)
    axes[1].set(xlabel='host elapsed [s]',ylabel='status arrival interval [ms]')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('Sanity Zenoh link test — '+('props-off armed minimum' if a.armed_minimum else 'disabled commands only'))
    fig.savefig(a.output/'comms.png',dpi=150);fig.savefig(a.output/'comms.pdf')
    print(json.dumps(summary,indent=2),flush=True)
    if not summary['sampled_checks_passed']:
        raise SystemExit(1)


if __name__=='__main__':main()
