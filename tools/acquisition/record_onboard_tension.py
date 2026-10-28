#!/usr/bin/env python3
"""Record raw onboard HX711 telemetry until Ctrl-C, then plot. No calibration."""
import argparse
from collections import Counter
import csv
from datetime import datetime
import json
import os
import select
from pathlib import Path
import signal
import socket
import sys
import time

from calibrate_onboard_tension import decode_packet, positive, masses
from tension_pull_protocol import PullProtocol, CueAudio
from hx711_timing import HX711Health, TIMING_FIELDS


class EnterReader:
    """Poll stdin without blocking acquisition or queuing approvals for later steps."""
    def __init__(self):
        self.fd=sys.stdin.fileno()
        self.buffer=b''

    def clear(self):
        self.buffer=b''
        while select.select([self.fd],[],[],0)[0]:
            if not os.read(self.fd,4096):
                raise EOFError('Input closed while waiting for Enter; partial recording saved')

    def pressed(self):
        if not select.select([self.fd],[],[],0)[0]:
            return False
        chunk=os.read(self.fd,4096)
        if not chunk:
            raise EOFError('Input closed while waiting for Enter; partial recording saved')
        self.buffer+=chunk
        if b'\n' in self.buffer:
            self.buffer=b''
            return True
        return False


def plot_recording(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    with (directory/'samples.csv').open(newline='') as file:
        rows=list(csv.DictReader(file))
    x=[];y=[];faults=[];previous=None
    for row in rows:
        elapsed=float(row['elapsed_s'])
        if int(row['flags']):
            faults.append(elapsed)
        if row['healthy']!='True':
            previous=None
            x.append(float('nan'));y.append(float('nan'))
            continue
        if previous is not None and (elapsed-float(previous['elapsed_s'])>.5 or
                row['boot_id']!=previous['boot_id'] or
                ((int(row['seq'])-int(previous['seq'])) & 0xffffffff)!=1):
            x.append(float('nan'));y.append(float('nan'))
        x.append(elapsed);y.append(int(row['raw_counts']))
        previous=row
    fig,ax=plt.subplots(figsize=(12,4.8))
    event_file=directory/'events.csv'
    cue_events=[]
    if event_file.exists():
        with event_file.open(newline='') as file:
            cue_events=list(csv.DictReader(file))
    end_s=max((float(r['elapsed_s']) for r in rows),default=0.0)
    for i,event in enumerate(cue_events):
        start=float(event['elapsed_s']);end=float(cue_events[i+1]['elapsed_s']) if i+1<len(cue_events) else end_s
        if event['phase'].startswith('setup_'):
            ax.axvspan(start,max(start,end),color='0.5',alpha=.06,lw=0,zorder=0)
            continue
        if event['phase'] not in ('pull','release') or event['missed']=='True':
            continue
        colour='#e1ae2c' if event['phase']=='pull' else '#38959a'
        ax.axvspan(start,max(start,end),color=colour,alpha=.15,lw=0,zorder=0)
        ax.axvline(start,color=colour,lw=.7,alpha=.7)
        weight=event.get('added_weight_g','')
        known=weight not in ('','None')
        label=(('Load %g g' % float(weight)) if event['phase']=='pull' else 'Unload') if known else (('Pull ' if event['phase']=='pull' else 'Release ')+event['cycle'])
        ax.text(start+.08,.97,label,
                transform=ax.get_xaxis_transform(),va='top',fontsize=8,color='#444c56')
    ax.plot(x,y,lw=.8,color='#087f9b')
    if faults:
        ax.scatter(faults,[.98]*len(faults),transform=ax.get_xaxis_transform(),marker='x',s=16,color='#bc4a35')
        ax.legend(handles=[Line2D([0],[0],color='#087f9b',label='Healthy raw readings'),
                           Line2D([0],[0],color='#bc4a35',marker='x',ls='',label='ADC fault / settling status')],frameon=False)
    if not any(row['healthy']=='True' for row in rows):
        ax.text(.5,.5,'No healthy HX711 samples received',transform=ax.transAxes,ha='center')
    ax.set(xlabel='Time since recording started (s, host arrival clock)',ylabel='Raw ADC counts',title='Strain-gauge response — raw recording')
    ax.ticklabel_format(axis='y',style='plain',useOffset=False)
    ax.spines[['top','right']].set_visible(False)
    ax.grid(alpha=.2)
    fig.text(.08,.015,'No smoothing or force conversion. Gaps are not joined; grey intervals indicate weight setup when present.',fontsize=9,color='#606872')
    fig.tight_layout(rect=(0,.04,1,1))
    fig.savefig(directory/'time_series.png',dpi=180)
    fig.savefig(directory/'time_series.pdf')
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=positive,help='Optional recording limit in seconds; otherwise Ctrl-C or end of guided routine')
    parser.add_argument('--guided-pulls',action='store_true',help='Prompt a timed manual pull/release routine, then stop and plot')
    parser.add_argument('--cycles',type=int,default=3,help='Number of repeats of the complete weight list, or hand-pull repetitions')
    parser.add_argument('--weights-g','--weight-g',nargs='+',metavar='G',help='Known ADDED masses in grams, e.g. 10 or 5,10,20; includes any added hanger; guided mode waits for Enter after each setup')
    parser.add_argument('--base-load-g',type=float,default=0.0,help='Measured mass that stays hanging throughout, e.g. 77.03 for the drone; default0')
    parser.add_argument('--pull-s',type=positive,default=3.0)
    parser.add_argument('--release-s',type=positive,default=5.0)
    parser.add_argument('--baseline-s',type=positive,default=5.0)
    parser.add_argument('--get-ready-s',type=float,default=5.0)
    parser.add_argument('--auto-advance',action='store_true',help='Known-weight mode: use fixed timers without waiting for Enter')
    parser.add_argument('--mute',action='store_true',help='Visual cues only')
    parser.add_argument('--output',type=Path,help='New run directory; default under outputs/strain_gauge/')
    parser.add_argument('--bind',default='0.0.0.0')
    parser.add_argument('--port',type=int,default=7460)
    parser.add_argument('--device-id',type=int,default=9)
    parser.add_argument('--source-ip',default='10.0.0.109')
    args=parser.parse_args()
    try:
        known_weights=masses(','.join(args.weights_g)) if args.weights_g else None
        protocol=PullProtocol(args.cycles,args.pull_s,args.release_s,args.baseline_s,args.get_ready_s,
                              weights_g=known_weights,base_load_g=args.base_load_g,
                              require_enter=bool(known_weights) and not args.auto_advance) if args.guided_pulls or known_weights else None
    except (ValueError,argparse.ArgumentTypeError) as error:
        parser.error(str(error))
    root=Path(__file__).resolve().parents[4]
    directory=(args.output or root/'outputs'/'strain_gauge'/(('tension_pull_' if protocol else 'tension_touch_')+datetime.now().strftime('%Y%m%d_%H%M%S_')+str(time.time_ns()%1000000000).zfill(9))).resolve()
    try:
        directory.mkdir(parents=True,exist_ok=False)
    except OSError as error:
        parser.exit(1,str(error)+'\n')
    stop=False
    def request_stop(signum, frame):
        nonlocal stop
        stop=True
    old_handlers={sig:signal.signal(sig,request_stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    counters=Counter();last_by_boot={};rates={};flag_counts=Counter();health=HX711Health()
    sock=None;audio=None;exit_code=0;error_text=None
    started_ns=time.monotonic_ns();started_unix_ns=time.time_ns()
    next_status=0.0;last_value=None;last_health_ok=None
    columns=['elapsed_s','host_monotonic_ns','ready_us','read_us','raw_counts','flags','healthy',
             'boot_id','seq','resets','max_high_us','send_drops','source_ip','sequence_gap','cue_phase','cue_cycle','cue_added_weight_g','cue_total_mass_g','cue_reference_force_N','cue_repeat','timing_v','receiver_boot_id',*TIMING_FIELDS,
             'source_interval_us','local_queue_age_us','baseline_generation','baseline_invalidated','health_reasons']
    try:
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.bind((args.bind,args.port));sock.settimeout(.02 if protocol else .2)
        audio=CueAudio(directory,enabled=bool(protocol) and not args.mute)
        enter_reader=EnterReader() if protocol and protocol.require_enter else None
        if protocol:
            (directory/'protocol.json').write_text(json.dumps(dict(stages=protocol.stages,duration_s=protocol.duration_s,
                weights_g=known_weights,base_load_g=args.base_load_g,repeats=args.cycles,wait_for_enter=protocol.require_enter,
                note='Cue times and nominal quasistatic reference masses/forces, not measured force. Keep the mount and load point fixed. Added mass includes its hanger; remove all added mass at each release.'),indent=2)+'\n')
        with (directory/'raw.jsonl').open('x',buffering=1) as raw, (directory/'samples.csv').open('x',newline='',buffering=1) as csv_file, (directory/'events.csv').open('x',newline='',buffering=1) as event_file:
            writer=csv.DictWriter(csv_file,fieldnames=columns);writer.writeheader()
            event_columns=['elapsed_s','host_monotonic_ns','planned_elapsed_s','late_by_s','phase','cycle','duration_s','instruction','missed','audio_requested','repeat','added_weight_g','removed_weight_g','base_load_g','total_reference_mass_g','reference_force_N','requires_enter','confirmed','confirmation_wait_s','target_phase','target_added_weight_g']
            event_writer=csv.DictWriter(event_file,fieldnames=event_columns);event_writer.writeheader()
            print('Recording Sanity%02d from %s over Wi-Fi UDP%d.' % (args.device_id,args.source_ip,args.port),flush=True)
            if protocol:
                if protocol.require_enter:
                    print('Known weights: prepare each loaded/unloaded state, then press Enter to start its timer. Raw data continue during setup.',flush=True)
                print('Guided routine: wait for live data, then follow '+('LOAD/UNLOAD' if known_weights else 'PULL/HOLD and LET GO')+' cues. Ctrl-C stops early.',flush=True)
                print('Keep the mount fixed. High tone = '+('attach the specified mass; low tone = remove all added mass.' if known_weights else 'pull gently; low tone = release.'),flush=True)
            else:
                print('Touch/release the sensor as you like. Ctrl-C stops and generates the plot.',flush=True)
            print('Output: '+str(directory),flush=True)
            while not stop:
                now_ns=time.monotonic_ns();elapsed=(now_ns-started_ns)/1e9
                current_health=health.check(now_ns)
                if current_health['health_ok']!=last_health_ok:
                    raw.write(json.dumps(dict(event='acquisition_health',host_monotonic_ns=now_ns,
                        elapsed_s=elapsed,**current_health))+'\n')
                    last_health_ok=current_health['health_ok']
                if protocol:
                    transitions=protocol.update(elapsed)
                    setup_started=any(e['phase'].startswith('setup_') for e in transitions)
                    if enter_reader and setup_started:
                        enter_reader.clear()
                    elif enter_reader and protocol.pending is not None and enter_reader.pressed():
                        now_ns=time.monotonic_ns();elapsed=(now_ns-started_ns)/1e9
                        transitions+=protocol.confirm(elapsed)
                    for event in transitions:
                        event['host_monotonic_ns']=now_ns
                        sound_phase=event['phase'].removeprefix('setup_')
                        if event['confirmed']:
                            sound_phase='baseline'
                        event['audio_requested']=audio.play(sound_phase) if not event['missed'] else False
                        if not event['missed']:
                            suffix=(' — %.1f seconds' % event['duration_s']) if event['duration_s'] else ''
                            cycle=(' [%d/%d]' % (event['cycle'],protocol.cycles)) if event['cycle'] else ''
                            print('\n>>> '+event['instruction']+suffix+cycle,flush=True)
                        event_writer.writerow({key:event[key] for key in event_columns})
                    if protocol.complete:
                        break
                if args.duration is not None and elapsed>=args.duration:
                    break
                if elapsed>=next_status:
                    print('%.1fs | %d healthy samples | %d flagged | last healthy raw: %s' %
                          (elapsed,counters['healthy'],counters['flagged'],last_value if last_value is not None else 'waiting for data'),flush=True)
                    raw.flush();csv_file.flush();next_status=elapsed+2
                try:
                    data,address=sock.recvfrom(4096)
                except socket.timeout:
                    continue
                if args.source_ip and address[0]!=args.source_ip:
                    counters['other_source']+=1
                    continue
                host_ns=time.monotonic_ns();elapsed=(host_ns-started_ns)/1e9
                try:
                    packet=decode_packet(data,args.device_id)
                except (ValueError,KeyError,TypeError) as error:
                    counters['malformed']+=1
                    health.invalidate('malformed_packet')
                    raw.write(json.dumps(dict(event='malformed_packet',elapsed_s=elapsed,source_ip=address[0],error=str(error),payload_hex=data.hex()))+'\n')
                    continue
                packet.update(host_monotonic_ns=host_ns,elapsed_s=elapsed,source_ip=address[0],
                              cue_phase=protocol.phase if protocol else 'free',cue_cycle=protocol.cycle if protocol else 0,
                              cue_added_weight_g=protocol.current['added_weight_g'] if protocol and protocol.current else None,
                              cue_total_mass_g=protocol.current['total_reference_mass_g'] if protocol and protocol.current else None,
                              cue_reference_force_N=protocol.current['reference_force_N'] if protocol and protocol.current else None,
                              cue_repeat=protocol.current['repeat'] if protocol and protocol.current else None)
                observation=health.observe(packet,host_ns)
                packet['acquisition_health']=observation
                raw.write(json.dumps(packet,allow_nan=False)+'\n')
                boot=packet['boot_id'];previous=last_by_boot.get(boot);gap=0;ordered=True
                if previous is not None:
                    delta=(packet['seq']-previous['seq']) & 0xffffffff
                    ordered=0<delta<0x80000000
                    if ordered:
                        gap=delta-1;counters['sequence_gaps']+=gap
                    else:
                        counters['duplicate_or_old']+=1
                if ordered:
                    last_by_boot[boot]=packet
                healthy=observation['sample_usable']
                for reason in observation['reasons']:counters['health_'+reason]+=1
                if healthy:
                    if protocol and protocol.origin_s is None:
                        protocol.start(elapsed)
                    counters['healthy']+=1;last_value=packet['raw']
                    if boot not in rates:
                        rates[boot]=dict(first_ready_us=packet['ready_us'],last_ready_us=packet['ready_us'],samples=0)
                    rates[boot]['last_ready_us']=packet['ready_us'];rates[boot]['samples']+=1
                if packet['flags']:
                    counters['flagged']+=1;flag_counts[str(packet['flags'])]+=1
                counters['received']+=1
                writer.writerow(dict(elapsed_s=elapsed,host_monotonic_ns=host_ns,ready_us=packet['ready_us'],
                    read_us=packet['read_us'],raw_counts=packet['raw'],flags=packet['flags'],healthy=healthy,
                    boot_id=boot,seq=packet['seq'],resets=packet['resets'],max_high_us=packet['max_high_us'],
                    send_drops=packet['send_drops'],source_ip=address[0],sequence_gap=gap,
                    cue_phase=packet['cue_phase'],cue_cycle=packet['cue_cycle'],
                    cue_added_weight_g=packet['cue_added_weight_g'],cue_total_mass_g=packet['cue_total_mass_g'],
                    cue_reference_force_N=packet['cue_reference_force_N'],cue_repeat=packet['cue_repeat'],
                    timing_v=packet.get('timing_v'),receiver_boot_id=packet.get('receiver_boot_id'),
                    **{k:packet.get(k) for k in TIMING_FIELDS},
                    **{k:observation.get(k) for k in ('source_interval_us','local_queue_age_us','baseline_generation','baseline_invalidated')},
                    health_reasons=';'.join(observation['reasons'])))
    except (OSError,EOFError) as error:
        exit_code=1;error_text=str(error)
        print('Recording error: '+error_text,file=sys.stderr)
    finally:
        if sock is not None:
            sock.close()
        if audio is not None:
            audio.close()
        for sig,handler in old_handlers.items():
            signal.signal(sig,handler)
        for rate in rates.values():
            span=(rate['last_ready_us']-rate['first_ready_us'])/1e6
            rate['observed_samples_per_s']=(rate['samples']-1)/span if span>0 else None
        summary=dict(schema='sanity_hx711_touch_recording_v1',created_unix_ns=started_unix_ns,
            duration_host_s=(time.monotonic_ns()-started_ns)/1e9,device_id=args.device_id,source_ip=args.source_ip,
            port=args.port,counts=dict(counters),flag_counts=dict(flag_counts),boot_sample_rates=rates,
            acquisition_health_at_exit=health.check(time.monotonic_ns()),
            health_scope='Raw ADC/continuity checks only, no cable state, verified baseline or force/mocap alignment',
            error=error_text,force_calibrated=False,plot_timebase='host monotonic arrival time',smoothing=False,
            guided_protocol=None if protocol is None else dict(started=protocol.origin_s is not None,
                completed=protocol.complete,origin_s=protocol.origin_s,planned_duration_s=protocol.duration_s,
                cycles=protocol.cycles,repeats=protocol.repeats,weights_g=protocol.weights_g,base_load_g=protocol.base_load_g,final_phase=protocol.phase,wait_for_enter=protocol.require_enter,confirmations=protocol.confirmations,setup_wait_s=protocol.setup_wait_s),
            audio_backend=None if audio is None or not audio.enabled else audio.player,
            audio_failure=False if audio is None else audio.failed)
        (directory/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
        if (directory/'samples.csv').exists():
            print('Saving plot...',flush=True)
            try:
                plot_recording(directory)
            except (ImportError,OSError,ValueError) as error:
                print('Data saved, but plotting failed: '+str(error),file=sys.stderr);exit_code=1
        print('Saved: '+str(directory),flush=True)
    return exit_code


if __name__=='__main__':
    raise SystemExit(main())
