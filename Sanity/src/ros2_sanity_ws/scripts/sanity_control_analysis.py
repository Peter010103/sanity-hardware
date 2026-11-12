"""Offline FC/host diagnostic analysis. Imports no aircraft or ROS interfaces."""
from pathlib import Path
import datetime
import csv
import fcntl
import hashlib
import json
import os
import socket
import subprocess
import time
import numpy as np
from scipy.spatial.transform import Rotation

TESTS=('pd-hover','roll-pulse','pitch-pulse','yaw-pulse','roll-sine','pitch-sine')
EXPECTED_UID='56003f001451333231343731'
AXIS={'roll':0,'pitch':1,'yaw':2}
SIGNS=np.array([1.,-1.,-1.])

def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()

def write_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.tmp')
    temp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');temp.replace(path)

def rms(x):return float(np.sqrt(np.mean(np.asarray(x)**2)))

def counts(x):
    values,n=np.unique(x,return_counts=True)
    return {str(int(v)):int(c) for v,c in zip(values,n)}

def discover(root):
    inventory=[]
    for path in sorted(Path(root).glob('*/run_summary.json')):
        try:r=json.loads(path.read_text())
        except (OSError,ValueError):continue  # A live launcher may be writing its summary.
        if r.get('test') not in TESTS or r.get('inner_controller') not in ('pid','indi'):continue
        reason=r.get('reason')
        if not reason:
            try:reason=json.loads((path.parent/'flight/result.json').read_text()).get('reason')
            except (OSError,ValueError):pass
        row=dict(run=path.parent.name,path=str(path.parent.resolve()),test=r['test'],inner=r['inner_controller'],
                 success=r.get('success') is True,reason=reason or r.get('error'),
                 started_unix=r.get('started_unix',0),state='awaiting_archive')
        archive=path.parent/'blackbox';receipt=archive/'receipt.json'
        if (archive/'read1.bbl').is_file() and receipt.is_file():
            try:
                a=json.loads(receipt.read_text())
                if a.get('success') and a.get('USB_closed'):row['state']='archived'
                else:row['state']='archive_in_progress_or_unconfirmed'
            except (OSError,ValueError):row['state']='archive_in_progress_or_unconfirmed'
        elif not row['success']:row['state']='failed_or_in_progress'
        inventory.append(row)
    return inventory

def host_data(run):
    rows=[]
    with (run/'flight/events.jsonl').open() as stream:
        for line in stream:
            r=json.loads(line)
            if r['kind']=='hover_control' and r.get('state') and r.get('controller'):rows.append(r)
    active=[r for r in rows if r['command']['phase']=='diagnostic_pd'
            and r.get('outer_controller',{}).get('diagnostic',{}).get('excitation',{}).get('elapsed_s') is not None]
    if not active:raise ValueError('No recorded diagnostic phase; retain as incomplete flight')
    start=float(np.median([r['control_monotonic_s']-r['outer_controller']['diagnostic']['excitation']['elapsed_s'] for r in active]))
    # Use one instance of each FC status snapshot, avoiding weighting held samples.
    unique={}
    for r in rows:
        if r.get('controller_sample_age_ms') is not None:
            c=r['controller'];unique.setdefault((c['sequence'],c['fc_time_us']),r)
    samples=list(unique.values())
    offsets=np.array([r['control_monotonic_s']-r['controller_sample_age_ms']*.001-r['controller']['fc_time_us']*1e-6 for r in samples])
    return dict(rows=rows,active=active,snapshots=samples,start=start,offset=float(np.median(offsets)),
                offset_spread_ms=(np.percentile(offsets-np.median(offsets),[5,50,95])*1000).tolist())

def read_csv(path):
    # Preserve appended diagnostics in newer traces; the original 126-field
    # prefix retains its positions for historical comparisons.
    with Path(path).open() as stream:
        names=[n.strip() for n in next(csv.reader(stream))]
    count=0
    while count+2<len(names) and names[count+2]==f'p[{count}]':count+=1
    versions={126:1,152:2,174:3,190:4}
    if count not in versions:raise ValueError('Unsupported FC trace field count')
    # Decoder-generated enum/string columns follow the numeric trace.
    a=np.loadtxt(path,delimiter=',',skiprows=1,usecols=range(count+2),dtype=np.int64)
    if a.ndim!=2 or a.shape[1]<128 or not np.all(np.diff(a[:,1])>0):raise ValueError('Invalid FC trace timestamps/schema')
    if not np.all(a[:,2]==versions[count]):raise ValueError('Unsupported FC trace version/schema')
    if not np.array_equal(a[:,1],a[:,4]):raise ValueError('FC trace p2 time does not match recording time')
    return a


def timing_transients(p):
    """Decode event-latched timing evidence, retained between 250 Hz records.

    Counters are boot-scoped. Event latches reset on arming and survive disarm;
    their timestamps and costs belong to the original event, not the row in
    which the observer reads them. The v1-v3 trace prefixes remain unchanged.
    """
    if p.shape[1]<190 or not np.all(p[:,0]>=4):
        return None
    flags={1:'short_interval',2:'long_interval',4:'previous_execution',8:'hard_limit',
           16:'repeat',32:'grace',64:'unready'}
    result=dict(counter_scope='boot',event_scope='first anomaly/rejection per arm; retained after disarm',
                counters={name:dict(first=int(p[0,col]),last=int(p[-1,col]),delta=int(p[-1,col]-p[0,col]))
                          for name,col in [('anomalies',174),('grace',175),('rejected',176),('observer_reseeds',189)]})
    for name,start in [('first_anomaly',177),('first_rejection',183)]:
        events={}
        for row in p[:,start:start+6]:
            if row[0]==0:continue
            events[(int(row[0]),int(row[1]))]=dict(sequence=int(row[0]),fc_time_us=int(row[1]),interval_us=int(row[2]),
                               previous_pid_execution_us=int(row[3]),flags=int(row[4]),
                               reasons=[label for bit,label in flags.items() if int(row[4])&bit],
                               adapter_execution_us=int(row[5]))
        result[name]=sorted(events.values(),key=lambda r:r['fc_time_us'])
    return result

def log_match(data,snapshots):
    """Match FC status time AND controller sequence AND mode, not time alone."""
    p=data[:,2:];t=data[:,1]
    times=np.array([r['controller']['fc_time_us'] for r in snapshots],dtype=np.int64)
    sequence=np.array([r['controller']['sequence'] for r in snapshots],dtype=np.int64)
    modes=np.array([{'pid':0,'to_indi':1,'indi':2,'to_pid':3}[r['controller']['mode']] for r in snapshots])
    inside=(times>=t[0])&(times<=t[-1])
    index=np.searchsorted(t,times);index=np.clip(index,0,len(t)-1)
    previous=np.maximum(index-1,0)
    index=np.where(abs(t[previous]-times)<abs(t[index]-times),previous,index)
    seqerr=abs(p[index,1]-sequence);terr=abs(t[index]-times)
    good=inside&(seqerr<=6)&(terr<=8000)&(p[index,6]==modes)
    return dict(snapshots=len(times),coverage_fraction=float(np.mean(inside)),
                matching_fraction=float(np.mean(good)),median_sequence_error=float(np.median(seqerr)),
                p95_sequence_error=float(np.percentile(seqerr,95)),median_time_error_us=float(np.median(terr)))

def select_log(candidates,snapshots):
    scored=[(path,data,log_match(data,snapshots)) for path,data in candidates]
    eligible=[x for x in scored if x[2]['matching_fraction']>=.90 and x[2]['coverage_fraction']>=.95]
    if len(eligible)!=1:
        raise ValueError('FC recording match is missing or ambiguous: '+json.dumps({p.name:s for p,_,s in scored}))
    return eligible[0],{path.name:score for path,_,score in scored}

def decode(raw,decoder,cache):
    raw_hash=sha(raw);decoder_hash=sha(decoder)
    folder=cache/(raw_hash[:16]+'_'+decoder_hash[:8]);folder.mkdir(parents=True,exist_ok=True)
    key=dict(raw_sha256=raw_hash,decoder_sha256=decoder_hash)
    with (folder/'.cache.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        return _decode_locked(raw,decoder,folder,key)

def _decode_locked(raw,decoder,folder,key):
    manifest=folder/'decoded.json'
    if manifest.exists():
        saved=json.loads(manifest.read_text())
        if all(saved.get(k)==v for k,v in key.items()):
            files=[folder/name for name in saved['csv_files']]
            if files and all(p.is_file() and sha(p)==saved['csv_sha256'][p.name] for p in files):return folder,files
    with (folder/'decoder.log').open('w') as log:
        child=subprocess.Popen([str(decoder),'--output-dir',str(folder),'--save-headers',str(raw)],stdout=log,stderr=subprocess.STDOUT)
        owner=dict(host=socket.gethostname(),pid=child.pid,pgid=os.getpgid(child.pid),started_unix=time.time())
        write_json(folder/'decoder_process.json',owner)
        try:
            code=child.wait(timeout=90)
        finally:
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.kill();child.wait()
            owner.update(returncode=child.returncode,exited=child.poll() is not None,ended_unix=time.time())
            write_json(folder/'decoder_process.json',owner)
        if code:raise ValueError(f'Decoder failed; see {folder}/decoder.log')
    files=sorted(p for p in folder.glob('read1.*.csv') if not p.name.endswith('.headers.csv'))
    if not files:raise ValueError('No FC recordings decoded')
    write_json(manifest,dict(**key,csv_files=[p.name for p in files],csv_sha256={p.name:sha(p) for p in files}))
    return folder,files

def correlation_lag(demand,rate,dt):
    if np.std(demand)<.005:return None
    values=[]
    for k in range(-int(.1/dt),int(.3/dt)+1):
        u=demand[max(-k,0):len(demand)-max(k,0)];v=rate[max(k,0):len(rate)-max(-k,0)]
        if len(u)>20 and np.std(u)>0 and np.std(v)>0:values.append((float(np.corrcoef(u,v)[0,1]),k))
    if not values:return None
    c,k=max(values)
    return dict(shift_ms=k*dt*1000,correlation=c,search_ms=[-100,300],interpretation='Closed-loop descriptive correlation; not causal delay')

def harmonic_fit(t,y,frequency):
    t=np.asarray(t,dtype=float);y=np.asarray(y,dtype=float)
    if len(t)<20 or not np.isfinite(t).all() or not np.isfinite(y).all():raise ValueError('Finite harmonic samples required')
    local=t-t[0];center=local-np.mean(local);omega=2*np.pi*frequency
    basis=np.column_stack([np.ones(len(t)),center,np.sin(omega*local),np.cos(omega*local)])
    beta,_,rank,_=np.linalg.lstsq(basis,y,rcond=None)
    if rank!=4:raise ValueError('Harmonic basis not identifiable')
    residual=y-basis@beta
    phasor=complex(beta[3],-beta[2])
    return dict(amplitude=float(abs(phasor)),phasor=phasor,residual_RMS=rms(residual),samples=len(t))

def harmonic_response(t,demand,rate,frequency):
    u=harmonic_fit(t,demand,frequency);y=harmonic_fit(t,rate,frequency)
    ratio=y['phasor']/u['phasor'] if u['amplitude']>=.005 else None
    return dict(frequency_hz=frequency,demand_amplitude_rad_s=u['amplitude'],gyro_amplitude_rad_s=y['amplitude'],
                gain=None if ratio is None else float(abs(ratio)),phase_deg=None if ratio is None else float(np.angle(ratio,deg=True)),
                demand_residual_RMS_rad_s=u['residual_RMS'],gyro_residual_RMS_rad_s=y['residual_RMS'],samples=y['samples'],
                weak_demand=bool(u['amplitude']<.02),response_harmonic_below_residual=bool(y['residual_RMS']>y['amplitude']/np.sqrt(2)))

def analyse(row,decoder,cache):
    run=Path(row['path']);raw=run/'blackbox/read1.bbl';archive=json.loads((run/'blackbox/receipt.json').read_text())
    if archive.get('uid')!=EXPECTED_UID:raise ValueError('Archive UID does not match Sanity15')
    raw_hash=sha(raw)
    if raw.stat().st_size!=archive['bytes'] or raw_hash!=archive['sha256']:raise ValueError('Archive hash/length does not match its receipt')
    host=host_data(run)
    # Match on diagnostic snapshots, not the unrecorded pre-arm interval.
    wanted={(r['controller']['sequence'],r['controller']['fc_time_us']) for r in host['active']}
    snapshots=[r for r in host['snapshots'] if (r['controller']['sequence'],r['controller']['fc_time_us']) in wanted]
    folder,files=decode(raw,decoder,cache)
    selected,evidence=select_log([(p,read_csv(p)) for p in files],snapshots)
    csv,data,_=selected;p=data[:,2:]
    origin=host['start']-host['offset'];t=data[:,1]*1e-6-origin;dt=float(np.median(np.diff(t)))
    active=host['active'];test=row['test'];mode=row['inner'];axis=AXIS.get(test.split('-')[0])
    duration=float(active[0]['outer_controller']['diagnostic']['duration_s'])
    observed_end=min(duration,max(float(r['outer_controller']['diagnostic']['excitation']['elapsed_s']) for r in active)+.02)
    mask=(t>=.15)&(t<observed_end-.15)
    if mask.sum()<100:raise ValueError('Insufficient diagnostic FC coverage')
    baseline=(t>=.15)&(t<2.8) if test!='pd-hover' else mask
    x=p[mask];demand=x[:,46:49]*1e-6;gyro=x[:,28:31]*1e-6;filtered=x[:,31:34]*1e-6
    ht=np.array([r['control_monotonic_s']-host['start'] for r in active]);state=np.array([r['state'] for r in active])
    ref=np.array([r['reference']['position'] for r in active]);position_error=np.linalg.norm(state[:,:3]-ref,axis=1)
    rot=Rotation.from_quat(state[:,[4,5,6,3]]);tilt=np.degrees(np.arccos(np.clip(rot.as_matrix()[:,2,2],-1,1)))
    hm=(ht>=.15)&(ht<observed_end-.15)
    e=[r['outer_controller']['diagnostic']['excitation'] for r in active]
    injected=np.array([z['requested_injection_rad_s'] for z in e])*SIGNS
    applied=np.array([z['applied_injection_rad_s'] for z in e])*SIGNS
    command=np.array([r['command']['rates_rad_s'] for r in active])*SIGNS
    base=np.array([z['base_ctbr'][1:] for z in e])*SIGNS
    m=dict(run=row['run'],test=test,inner=mode,flight_success=row['success'],flight_reason=row['reason'],
           raw_sha256=raw_hash,raw_bytes=archive['bytes'],csv=str(csv),csv_sha256=sha(csv),selected_recording=csv.name,
           matching_evidence=evidence,diagnostic_duration_s=duration,observed_diagnostic_end_s=observed_end,FC_origin_s=origin,
           alignment_offset_spread_ms_p5_p50_p95=host['offset_spread_ms'],sample_period_us=dt*1e6,sample_rate_hz=1/dt,
           recording_frames=len(data),diagnostic_frames=len(x),loop_iteration_steps=counts(np.diff(data[:,0])),
           target_axis=axis,rate_error_RMS_rad_s=np.sqrt(np.mean((demand-gyro)**2,axis=0)).tolist(),
           rate_minus_common_H_RMS_rad_s=np.sqrt(np.mean((demand-filtered)**2,axis=0)).tolist(),
           pre_excitation_rate_error_RMS_rad_s=np.sqrt(np.mean(((p[baseline,46:49]-p[baseline,28:31])*1e-6)**2,axis=0)).tolist(),
           rate_max_abs_rad_s=np.max(abs(gyro),axis=0).tolist(),
           position_RMS_m=rms(position_error[hm]),position_max_m=float(position_error[hm].max()),tilt_max_deg=float(tilt[hm].max()),
           injected_peak_rad_s=np.max(abs(injected),axis=0).tolist(),applied_injection_peak_rad_s=np.max(abs(applied),axis=0).tolist(),
           host_clipped_ticks=sum(any(z.get('rate_clipped',[])) for z in e),
           mode_counts=counts(x[:,6]),reason_counts=counts(x[:,7]),rpm_valid_masks=counts(x[:,10]),
           rotor_future_masks=counts(x[:,122]),rotor_missing_masks=counts(x[:,123]),
           voltage_min_median_max_V=np.percentile(x[:,15]*.001,[0,50,100]).tolist(),
           recording_counter_deltas={str(j):int(p[-1,j]-p[0,j]) for j in [112,113,114]},
           diagnostic_counter_deltas={str(j):int(x[-1,j]-x[0,j]) for j in [112,113,114]},
           sampled_adapter_us_p50_p95_max=np.percentile(x[:,119],[50,95,100]).tolist(),
           sampled_previous_CONTROL_tick_us_p50_p95_max=np.percentile(x[:,124],[50,95,100]).tolist(),
           actual_motor_count_min_max=[float(x[:,105:109].min()*.001),float(x[:,105:109].max()*.001)],
           rotor_age_us_p50_p95_max=np.percentile(x[:,57:61],[50,95,100]).tolist(),harmonics=[])
    if p.shape[1]>=152 and np.all(p[:,0]>=2):
        good=x[(x[:,5]&4)!=0]
        m['motor_model_v2']=dict(aggregate_reasons=counts(x[:,16]),servo_failures=counts(x[:,148]),
            transfer_policy=counts(x[:,149]),mask_counts={name:counts((x[:,136]>>shift)&15) for name,shift in
            [('valid',0),('candidate',4),('degraded',8),('held',12),('recovering',16),('seeded',20),('integral_saturated',24)]},
            degraded_entries_delta=int(p[-1,145]-p[0,145]),recovery_completions_delta=int(p[-1,146]-p[0,146]),
            integral_saturation_ticks_delta=int(p[-1,147]-p[0,147]),successful_servo_frames=len(good))
        if len(good):
            m['motor_model_v2'].update(bounds_min_max_count=[np.percentile(good[:,j]*.001,[0,100]).tolist() for j in (126,127)],
                speed_error_RMS_rad_s=np.sqrt(np.mean((good[:,85:89]*.001)**2,axis=0)).tolist(),
                effective_feedforward_min_max_count=[np.percentile(good[:,97+i]*.001,[0,100]).tolist() for i in range(4)])
    if p.shape[1]>=174 and np.all(p[:,0]>=3):
        m['rate_feedforward_v3']=dict(
            feedback_alpha_RMS_rad_s2=np.sqrt(np.mean((x[:,152:155]*.001)**2,axis=0)).tolist(),
            reference_derivative_RMS_rad_s2=np.sqrt(np.mean((x[:,158:161]*.001)**2,axis=0)).tolist(),
            feedforward_alpha_RMS_rad_s2=np.sqrt(np.mean((x[:,161:164]*.001)**2,axis=0)).tolist(),
            gain_milli_per_axis=[counts(x[:,164+i]) for i in range(3)],
            cutoff_hz=counts(x[:,167]),clipped_masks=counts(x[:,172]),
            resets_recording_delta=int(p[-1,173]-p[0,173]),
            status_bits=counts(x[:,171]))
    transients=timing_transients(p)
    if transients is not None:m['timing_transients_v4']=transients
    if axis is not None and test.endswith('-pulse'):
        m['correlation']=correlation_lag(demand[:,axis],gyro[:,axis],dt)
        signs=[];previous=0
        for v in injected[:,axis]*SIGNS[axis]:
            sign=int(np.sign(v)) if abs(v)>1e-10 else 0
            if sign and sign!=previous:signs.append(sign)
            previous=sign
        m['host_pulse_signs_FLU']=signs
    if test.endswith('-sine'):
        recipe=active[0]['outer_controller']['diagnostic']
        excitation=recipe.get('excitation',{})
        # Older records carry the waveform in metadata; runtime excitation is
        # separately logged. Prefer the exact saved configuration when present.
        config_path=run/'configuration.json'
        if config_path.is_file():
            saved=json.loads(config_path.read_text()).get('diagnostic',{})
            if saved:recipe=saved;excitation=recipe.get('excitation',{})
        frequencies=excitation.get('frequencies_hz',[.5,1.,2.])
        segment=excitation.get('segment_s',6.);recovery=excitation.get('recovery_s',2.)
        pre=recipe.get('pre_s',3.);m['sine_recipe']=recipe
        for index,frequency in enumerate(frequencies):
            start=pre+index*(segment+recovery)
            fit=(t>=start+1)&(t<start+segment-1)&(t<observed_end)
            complete=observed_end>=start+segment-1 and fit.sum()>100
            if complete:
                entry=harmonic_response(t[fit],p[fit,46+axis]*1e-6,p[fit,28+axis]*1e-6,frequency)
                entry['complete']=True
            else:
                entry=dict(frequency_hz=frequency,complete=False,reason='excitation_incomplete',samples=int(fit.sum()),
                    gain=None,phase_deg=None,demand_amplitude_rad_s=None,gyro_amplitude_rad_s=None,
                    demand_residual_RMS_rad_s=None,gyro_residual_RMS_rad_s=None,weak_demand=True,response_harmonic_below_residual=False)
            entry['FC_phase_window_s']=[start+1,start+segment-1];m['harmonics'].append(entry)
    if mode=='indi':
        m['active_rotor_error_p95_percent']=np.percentile(abs(x[:,65:69]-x[:,73:77])/np.maximum(x[:,73:77],1)*100,95,axis=0).tolist()
        m['active_allocator_saturation_masks']=counts(x[:,13])
        gains=[]
        for a in range(3):
            err=demand[:,a]-gyro[:,a];valid=abs(err)>.02
            feedback_column=152+a if p.shape[1]>=174 else 25+a
            gains.append(float(np.median(x[valid,feedback_column]*.001/err[valid])) if valid.any() else None)
        m['recorded_rate_gains_per_s']=gains
        m['rate_gain_identification_source']='feedback alpha p152..154' if p.shape[1]>=174 else 'total alpha (legacy adapter without rate-reference FF) p25..27'
    notes=run/'collection_notes.json'
    if notes.is_file():m['collection_notes']=json.loads(notes.read_text())
    return dict(metrics=m,p=p,t=t,host_t=ht,base=base,injection=injected,applied_injection=applied,
                host_command=command,position_error=position_error,tilt=tilt,axis=axis,duration=duration)
