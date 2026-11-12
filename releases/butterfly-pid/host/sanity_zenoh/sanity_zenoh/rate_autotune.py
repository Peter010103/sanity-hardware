"""Rate-pulse quality checks and bounded coordinate search; no aircraft I/O."""
from collections import deque
import math
import struct
import threading
import time
import numpy as np


class RatePulseLog:
    def __init__(self):
        self.lock=threading.Lock();self.wire={};self.outputs=deque(maxlen=4000);self.gyro=deque(maxlen=2000);self.marks=[]

    def observe(self,event):
        with self.lock:
            if event['kind']=='zenoh_tx_command':
                raw=bytes.fromhex(event['payload_hex'])
                if raw[:4]!=b'SZC2':return
                epoch,session,seq=struct.unpack_from('>QII',raw,8);v=struct.unpack_from('>3hH',raw,32)
                self.wire[(epoch,session,seq)]={'host_ns':event.get('host_ns',time.monotonic_ns()),'enabled':event['enabled'],'rates':np.array(v[:3])/10000.,'thrust':v[3]/10000.}
                if len(self.wire)>4000:self.wire.pop(next(iter(self.wire)))
            elif event['kind']=='zenoh_rx_fc_telemetry':
                for r in event.get('telemetry',{}).get('records',[]):
                    if 'transmitted_rc_channels' in r:self.outputs.append(r)
                    if 'gyro_rad_s_fc' in r:self.gyro.append(r)

    def mark(self,event,started):
        if event['kind'] not in ('probe_begin','probe_end'):return
        with self.lock:self.marks.append({**event,'host_ns':int((started+event['elapsed_s'])*1e9)})

    def clear_marks(self):
        with self.lock:self.marks=[]

    def snapshot(self):
        with self.lock:marks=list(self.marks);outputs=list(self.outputs);gyro=list(self.gyro);wire=dict(self.wire)
        if marks:
            lower=min(m['host_ns'] for m in marks)-300_000_000
            upper=max(m['host_ns'] for m in marks)+500_000_000
            wire={k:v for k,v in wire.items() if lower<=v['host_ns']<=upper}
            outputs=[r for r in outputs if (r['epoch'],r['command_session'],r['command_sequence']) in wire]
            if outputs:
                lower_us=min(r['uart_write_begin_esp_us'] for r in outputs)-300_000
                upper_us=max(r['uart_write_begin_esp_us'] for r in outputs)+500_000
                epochs={r['epoch'] for r in outputs}
                gyro=[r for r in gyro if r['epoch'] in epochs and lower_us<=r['response_parsed_esp_us']<=upper_us]
        return marks,outputs,gyro,wire

    def evaluate(self):
        return evaluate_rate_pulses(self.snapshot())


def evaluate_rate_pulses(snapshot):
    marks,outputs,gyro,wire=snapshot
    result=[]
    for begin in (e for e in marks if e['kind']=='probe_begin'):
        end=next((e for e in marks if e['kind']=='probe_end' and e['trial_index']==begin['trial_index']),None)
        metric={'trial_index':begin['trial_index'],'valid':False,'reasons':[],'delta_rate_rad_s':begin['delta_rate_rad_s'],'rate_axis':begin['rate_axis']}
        if end is None:metric['reasons'].append('missing pulse end');result.append(metric);continue
        duration=float(begin['duration_s'])
        if not math.isfinite(duration) or not 0.<duration<=1.:
            metric['reasons'].append('invalid step duration');result.append(metric);continue
        axis=begin['rate_axis']-1;desired=np.array(begin['baseline_ctbr']);desired[axis+1]+=begin['delta_rate_rad_s']
        candidates=[]
        for r in outputs:
            key=(r['epoch'],r['command_session'],r['command_sequence']);w=wire.get(key)
            if w and w['enabled'] and begin['host_ns']-1_000_000<=w['host_ns']<end['host_ns'] and abs(w['rates'][axis]-desired[axis+1])<.00011 and abs(w['thrust']-desired[0])<.00011:candidates.append(r)
        if not candidates:metric['reasons'].append('no matched UART pulse');result.append(metric);continue
        first=min(candidates,key=lambda r:r['uart_write_begin_esp_us']);epoch=first['epoch'];t0=first['uart_write_begin_esp_us']/1e6;delta=begin['delta_rate_rad_s'];initial=first['transmitted_rc_channels'][axis]
        local=sorted((r for r in outputs if r['epoch']==epoch and -.2<=r['uart_write_begin_esp_us']/1e6-t0<=max(.5,duration+.25)),key=lambda r:r['uart_write_begin_esp_us'])
        after=[r for r in local if r['uart_write_begin_esp_us']/1e6>t0 and r['transmitted_rc_channels'][axis]!=initial]
        width=after[0]['uart_write_begin_esp_us']/1e6-t0 if after else None
        ages=[(r['uart_write_begin_esp_us']-r['command_received_esp_us'])/1000 for r in local if r['uart_write_begin_esp_us']/1e6-t0<=max(.25,duration+.09)]
        if width is None or abs(width-begin['duration_s'])>.045:metric['reasons'].append('UART pulse duration mismatch')
        if not ages or max(ages)>40.:metric['reasons'].append('received command held over40ms')
        g=sorted((r for r in gyro if r['epoch']==epoch and -.18<=r['response_parsed_esp_us']/1e6-t0<=(.3 if duration<=.2 else duration+.2)),key=lambda r:r['response_parsed_esp_us'])
        gt=np.array([r['response_parsed_esp_us']/1e6-t0 for r in g]);gv=np.array([r['gyro_rad_s_fc'][axis] for r in g]);before=(gt>=-.14)&(gt<0);active=(gt>=0)&(gt<=min(width or duration,duration));late_start=.1 if duration<=.2 else .75*duration;late=(gt>=late_start)&active
        minimum_active=max(6,int(duration/.02)-2);minimum_late=2 if duration<=.2 else 3
        if before.sum()<4 or active.sum()<minimum_active or late.sum()<minimum_late or len(gt)<2 or max(np.diff(gt))>.035:metric['reasons'].append('insufficient/gapped gyro samples')
        if metric['reasons']:result.append(metric);continue
        baseline=float(np.median(gv[before]));noise=float(np.std(gv[before]));gain=(gv-baseline)/delta
        peak=float(max(gain[active]));late_gain=float(np.mean(gain[late]));cross=gt[active & (gain>=.9)];rise=float(cross[0]) if len(cross) else duration+.14
        score=4*max(0.,peak-1.)**2+2*float(np.mean((gain[active]-1.)**2))+2*(late_gain-1.)**2+.5*(rise/.16)**2
        metric.update(valid=True,score=score,peak_gain=peak,late_gain=late_gain,rise_time_s=rise,baseline_noise_rad_s=noise,UART_duration_s=width,maximum_command_age_ms=max(ages),uart_start_esp_us=first['uart_write_begin_esp_us'])
        active_t=gt[active];active_gain=gain[active];within=np.abs(active_gain-1.)<=.1
        settling=None
        for index in range(len(active_t)):
            if active_t[-1]-active_t[index]>=.06-1e-9 and np.all(within[index:]):
                settling=float(active_t[index]);break
        metric.update(analysis_hold_s=duration,active_samples=int(active.sum()),late_window_start_s=late_start,
                      peak_time_s=float(active_t[np.argmax(active_gain)]),settling_time_s=settling,
                      settled_during_hold=settling is not None,settling_band_fraction=.1,settling_minimum_dwell_s=.06,
                      late_error_rad_s=float(delta*(late_gain-1.)),response_filter='FC-reported gyro; no extra host low-pass')
        # Recovery is controlled by PD, so compare against the actual RC stream;
        # do not label this a pure open-loop step settling time.
        post=(gt>=width)&(gt<=width+.14)
        if post.sum()>=3 and local:
            rt=np.array([r['uart_write_begin_esp_us']/1e6-t0 for r in local])
            rc=np.array([r['transmitted_rc_channels'][axis] for r in local])
            selected=np.clip(np.searchsorted(rt,gt[post],side='right')-1,0,len(rt)-1)
            commanded=(rc[selected]-1500.)*(600.*np.pi/180.)/500.
            error=(gv[post]-commanded)/abs(delta)
            metric.update(recovery_tracking_rmse_rad_s=float(np.sqrt(np.mean((gv[post]-commanded)**2))),
                          recovery_tracking_peak_rad_s=float(np.max(np.abs(gv[post]-commanded))),
                          recovery_score=float(np.mean(error**2)),recovery_samples=int(post.sum()))
        if noise>.15:metric['valid']=False;metric['reasons'].append('high pre-pulse gyro variation')
        result.append(metric)
    return result

def batch_score(metrics):
    valid=[m for m in metrics if m.get('valid')]
    if any(not all(math.isfinite(m[k]) for k in ('score','peak_gain','late_gain')) for m in valid):raise ValueError('Nonfinite rate-response metrics')
    if len(valid)<3 or len({round(abs(m['delta_rate_rad_s']),2) for m in valid})<2 or len({np.sign(m['delta_rate_rad_s']) for m in valid})<2:
        raise ValueError('Need at least3clean pulses covering both amplitudes and signs')
    if max(m['peak_gain'] for m in valid)>2.:raise ValueError('Rate peak exceeded twice requested increment')
    return {'score':float(np.median([m['score'] for m in valid])),'median_peak_gain':float(np.median([m['peak_gain'] for m in valid])),'median_late_gain':float(np.median([m['late_gain'] for m in valid])),'valid_pulses':len(valid),'total_pulses':len(metrics)}


class RateGainSearch:
    def __init__(self,baseline,axes=('roll','pitch'),evaluations=4,tune_d=False):
        if not axes or any(a not in ('roll','pitch') for a in axes) or len(set(axes))!=len(axes) or not 2<=evaluations<=6:raise ValueError('Distinct roll/pitch axes and2..6evaluations required')
        self.baseline=baseline;self.best=baseline;self.current=baseline;self.axes=tuple(axes);self.evaluations=evaluations;self.tune_d=tune_d
        self.axis_index=0;self.evaluation=0;self.validation=False;self.best_score=None;self.complete=False;self.history=[]

    @property
    def axis(self):return ('roll','pitch').index(self.axes[self.axis_index])

    def accept(self,metrics):
        score=batch_score(metrics);entry={'axis':self.axes[self.axis_index],'evaluation':self.evaluation,'validation':self.validation,'gains':self.current.json(),'metrics':metrics,**score}
        self.history.append(entry)
        if self.validation:
            if score['score']>self.best_score*1.35+.03 or not .75<=score['median_late_gain']<=1.25:raise ValueError('Best-gain validation failed; returning to original gains')
            self.axis_index+=1;self.evaluation=0;self.validation=False;self.best_score=None
            if self.axis_index==len(self.axes):self.complete=True;return None
            self.current=self.best;return self.current
        if self.best_score is None or (score['score']<self.best_score*.95 and .8<=score['median_late_gain']<=1.2):self.best_score=score['score'];self.best=self.current;entry['accepted']=True
        else:entry['accepted']=False
        self.evaluation+=1
        if self.evaluation>=self.evaluations:self.validation=True;self.current=self.best;return self.current
        directions=[('F',-.10),('P',-.05),('D',.05) if self.tune_d else ('F',-.10),('P',-.05),('F',.05)]
        term,fraction=directions[(self.evaluation-1)%len(directions)];original=self.baseline.gain(self.axis,term);previous=self.best.gain(self.axis,term)
        low,high=((1.,1.1) if term=='D' else (.8,1.1) if term=='P' else (.5,1.1))
        value=int(np.clip(previous+int(math.copysign(max(1,round(original*abs(fraction))),fraction)),math.ceil(original*low),math.floor(original*high)))
        self.current=self.best.with_damping(self.axis,value) if term=='D' else self.best.changed(self.axis,term,value);return self.current
