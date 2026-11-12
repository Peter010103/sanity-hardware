"""Export receive-only shadow-test data and raw plots; no force calibration."""
import bisect
import csv
import json
from pathlib import Path


def plot_recording(directory, started_ns):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory)
    def read(name):
        return [json.loads(line) for line in (directory/name).read_text().splitlines() if line.strip()]
    events=read('bench_test_events.jsonl');decisions=read('cable_engagement.jsonl')
    times=[e['host_monotonic_ns'] for e in events]
    samples=[]
    for row in read('strain_raw.jsonl'):
        if not row.get('accepted'):continue
        p=json.loads(row['datagram_utf8']);index=bisect.bisect_right(times,row['host_monotonic_ns'])-1
        e=events[index] if index>=0 else {}
        samples.append(dict(elapsed_s=(row['host_monotonic_ns']-started_ns)/1e9,host_monotonic_ns=row['host_monotonic_ns'],raw_counts=p['raw'],ready_us=p['ready_us'],flags=p['flags'],healthy=row['health']['sample_usable'],boot_id=p['boot_id'],seq=p['seq'],resets=p['resets'],phase=e.get('phase','waiting_data'),trial=e.get('trial',0)))
    columns=['elapsed_s','host_monotonic_ns','raw_counts','ready_us','flags','healthy','boot_id','seq','resets','phase','trial']
    with (directory/'samples.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=columns);w.writeheader();w.writerows(samples)
    plt.rcParams.update({'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True,gridspec_kw={'height_ratios':[2,1,1]})
    x=[];y=[];previous=None
    for s in samples:
        if not s['healthy']:
            x.append(float('nan'));y.append(float('nan'));previous=None;continue
        if previous and (s['boot_id']!=previous['boot_id'] or ((s['seq']-previous['seq'])&0xffffffff)!=1 or s['elapsed_s']-previous['elapsed_s']>.5):
            x.append(float('nan'));y.append(float('nan'))
        x.append(s['elapsed_s']);y.append(s['raw_counts']);previous=s
    axes[0].plot(x,y,lw=.7,color='#19859c',label='Healthy raw ADC counts')
    diagnostic=[d for d in decisions if d.get('kind')=='shadow_decision']
    dx=[(d['host_monotonic_ns']-started_ns)/1e9 for d in diagnostic]
    axes[0].plot(dx,[d['baseline']['counts'] if d.get('baseline') else float('nan') for d in diagnostic],ls='--',lw=1,color='.3',label='Captured unloaded reference')
    axes[0].set(ylabel='Raw ADC counts',title='Sanity09 guided payload loading — receive-only supervisor')
    axes[0].ticklabel_format(axis='y',style='plain',useOffset=False);axes[0].legend(frameon=False,fontsize=8)
    axes[1].plot(dx,[1000*d.get('geometry',{}).get('hook_distance_m',float('nan')) for d in diagnostic],color='#19859c',lw=.8)
    axes[1].set(ylabel='Raw hook distance (mm)')
    states={'SLACK':0,'TAUT':1,'UNKNOWN':2}
    axes[2].step(dx,[states[d['state']] for d in diagnostic],where='post',color='#8866ac',lw=.9)
    axes[2].set(yticks=[0,1,2],yticklabels=['SLACK','TAUT','UNKNOWN'],ylim=(-.3,2.3),xlabel='Recording time (s, host monotonic clock)',ylabel='Shadow state')
    end=max([s['elapsed_s'] for s in samples]+dx+[0.])
    for i,e in enumerate(events):
        if e['phase'] not in ('loaded','released'):continue
        start=(e['host_monotonic_ns']-started_ns)/1e9
        stop=(events[i+1]['host_monotonic_ns']-started_ns)/1e9 if i+1<len(events) else end
        for ax in axes:ax.axvspan(start,max(start,stop),color='#e4b334' if e['phase']=='loaded' else '#42a79c',alpha=.14,lw=0)
        axes[0].text(start,.98,('Load ' if e['phase']=='loaded' else 'Unload ')+str(e['trial']),transform=axes[0].get_xaxis_transform(),va='top',fontsize=8)
    for ax in axes:ax.grid(alpha=.18)
    if not samples:axes[0].text(.5,.5,'No decoded samples received',transform=axes[0].transAxes,ha='center')
    fig.text(.08,.015,'Raw counts; no drift correction or force conversion. Enter labels are operator confirmations, not measured force onset.',fontsize=9)
    fig.tight_layout(rect=(0,.035,1,1));fig.savefig(directory/'time_series.png',dpi=180);fig.savefig(directory/'time_series.pdf');plt.close(fig)
