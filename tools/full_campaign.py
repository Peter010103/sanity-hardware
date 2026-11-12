"""Finite regulated-source identification, extending the verified three-cell bench."""
import fcntl,re,math
import argparse,json,time,datetime,sys,threading,queue,signal,hashlib,os,struct,ctypes as C,statistics
from pathlib import Path
from stand_config import ROOT, HERE, FC_PORT, PSU_PORT, CALIBRATION, FC_UID, SLOW_LIBRARY

import campaign as c
import serial
from tenma import tenmaDcLib as lib
b=c.b;t=c.t
real_serial=serial.Serial

def bounded_serial(*args,**kwargs):
 kwargs.update(timeout=.5,write_timeout=.5,exclusive=True);return real_serial(*args,**kwargs)
serial.Serial=bounded_serial

WORKER=None
CONFIG_PREPARE=CONFIG_RESTORE=None
SUPPLY_FIELDS=['request_start_ns','read_complete_ns','unix_time_ns','phase_begin','phase_end','supply_voltage_V','supply_current_A','mode','output','priority']

class SupplyWorker(threading.Thread):
 def __init__(self,supply):
  super().__init__(daemon=True);self.supply=supply;self.cancel=threading.Event();self.samples=queue.Queue(maxsize=2048);self.latest=None;self.error=None;self.phase='setup'
 def run(self):
  try:
   os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
   while not self.cancel.is_set():
    start=time.monotonic_ns();phase=self.phase;v=self.supply.runningVoltage(1);i=self.supply.runningCurrent(1);st={k.strip():v for k,v in self.supply.getStatus().items()};end=time.monotonic_ns()
    row=dict(request_start_ns=start,read_complete_ns=end,unix_time_ns=time.time_ns(),phase_begin=phase,phase_end=self.phase,supply_voltage_V=v,supply_current_A=i,mode=st['channelMode'],output=st['output'],priority=st['V/C priority'])
    self.latest=row;self.samples.put_nowait(row)
  except BaseException as exc:self.error=repr(exc)
 def close(self):
  self.cancel.set();self.join(timeout=3)
  if self.is_alive():raise RuntimeError('Supply worker did not terminate')

class RegulatedReader(t.Reader):
 def __init__(self,group,allow_torque_gaps=False):
  super().__init__(group);self.allow_torque_gaps=allow_torque_gaps;self.faults=queue.Queue();self.fault_history=[];self.sensors=[t.Sensor(group,i) for i in range(3)]
 def sample(self,index):
  if t.CHANNELS[index]['name'] in self.group.disabled_torque:return None
  try:return self.sensors[index].read()
  except BaseException as exc:
   if index==0 or not self.allow_torque_gaps:raise
   name=t.CHANNELS[index]['name'];self.group.disabled_torque.add(name);fault=dict(channel=name,timestamp_ns=time.monotonic_ns(),error=repr(exc),action='disabled until next stopped block; no synthetic torque rows')
   self.fault_history.append(fault);self.faults.put_nowait(fault);return None
 def run(self):
  try:
   os.sched_setaffinity(0,{4});os.sched_setscheduler(0,os.SCHED_FIFO,os.sched_param(70))
   for _ in range(10):
    for i in range(3):self.sample(i)
   while not self.cancel.is_set():
    for i,reader in enumerate(self.group.readers):
     if self.cancel.is_set():break
     sample=self.sample(i)
     if sample is not None:reader.samples.put_nowait(sample)
  except BaseException as exc:
   self.error=repr(exc)
   for reader in self.group.readers[1:]:reader.error=self.error

def check_spinup(bench):
 if bench.command_ack_ns and b.mono()-bench.command_ack_ns>700_000_000:
  low=[i for i,(u,m) in enumerate(zip(bench.vector,bench.latest_rpm['motors']),1) if u>=1400 and m['rpm']<3000]
  if low:raise RuntimeError('Commanded motor failed to leave near-idle speed: '+repr(low))

class RegulatedBench(c.CampaignBench):
 def poll_rpm(self):
  c.ENGINE_BENCH.poll_rpm(self)
  if getattr(self.args,'record_direct_outputs',False):
   a,z,payload=self.query(104);v=list(struct.unpack_from('<4H',payload));self.log.row('fc_outputs',dict(request_start_ns=a,reply_ns=z,phase=self.state,**dict(zip(['M1','M2','M3','M4'],v))))
  check_spinup(self)
 def force_guard(self):
  c.ENGINE_BENCH.force_guard(self)
  if self.state=='initializing':
   end=b.mono()+1_000_000_000
   while any(n not in self.tlatest for n in self.thistory if n not in t.GROUP.disabled_torque) and b.mono()<end:time.sleep(.002);self.drain_force()
  if any(n not in self.tlatest or b.mono()-self.tlatest[n]>150_000_000 for n in self.thistory if n not in t.GROUP.disabled_torque):raise RuntimeError('Required torque ADC stale/missing')
  while True:
   try:self.log.event('optional_torque_channel_fault',**t.GROUP.readers[0].faults.get_nowait())
   except queue.Empty:break
  WORKER.phase=self.state
  while True:
   try:self.log.row('supply',WORKER.samples.get_nowait())
   except queue.Empty:break
  if WORKER.error:raise RuntimeError('Supply acquisition failure '+WORKER.error)
  latest=WORKER.latest
  if not latest or time.monotonic_ns()-latest['read_complete_ns']>2_000_000_000:raise RuntimeError('Supply telemetry stale')
  if latest['output']!='ON' or latest['mode']!='C.V':raise RuntimeError('Supply is not ON in CV mode')
  if abs(latest['supply_voltage_V']-self.args.supply_voltage_V)>.08:raise RuntimeError('Supply voltage not regulating setpoint')
  if latest['supply_current_A']>self.args.actual_current_limit_A:raise RuntimeError('Measured supply current exceeds rear-terminal experiment limit')
 def stopped_bias(self):
  if not self.args.allow_torque_gaps:return super().stopped_bias()
  self.force_guard();mean,sd=self.recent_force_mean();now=b.mono();values=[v for stamp,v in self.force_history if now-1_000_000_000<=stamp<=now]
  if len(values)<40 or sd>.030 or abs(mean-statistics.median(values))>.015:raise RuntimeError('Unstable or contaminated stopped thrust bias')
  self.baseline,self.baseline_sd=mean,sd
  for name,history in self.thistory.items():
   if name in t.GROUP.disabled_torque:self.tbias[name]=None;continue
   vv=[v for stamp,v in history if now-1_000_000_000<=stamp<=now]
   if len(vv)<40 or statistics.stdev(vv)>500:
    t.GROUP.disabled_torque.add(name);self.tbias[name]=None;self.log.event('optional_torque_bias_invalid',channel=name);continue
   self.tbias[name]=statistics.mean(vv)
  self.log.event('all_channels_stopped_bias',thrust_mean_N=mean,thrust_sd_N=sd,torque_bias=self.tbias,disabled_torque_channels=sorted(t.GROUP.disabled_torque))
 def poll_battery(self):
  a,z,payload=self.query(130);v=b.battery(payload);self.latest_battery,self.battery_time=v,z
  self.log.row('battery',dict(request_start_ns=a,reply_ns=z,**{k:v[k] for k in b.FIELDS['battery'] if k not in ('request_start_ns','reply_ns')}));self.next_battery=z+int(b.HEALTH_PERIOD_S*1e9)
  # No flight battery is connected: warning is logged, critical remains a stop.
  # Existing FC settings are 3.50 V warning and 3.30 V critical per cell.
  if v['detected_cells']!=2 or v['battery_state_code'] not in (0,1) or not 6.6<=v['voltage_V']<=8.4:raise RuntimeError('FC bus outside configured 2S critical-voltage experiment bounds')
  if v['battery_state_code']!=getattr(self,'last_battery_state_code',None):self.log.event('regulated_FC_voltage_state',state=v['battery_state_code'],voltage_V=v['voltage_V'],note='PSU only; FC warning retained as data, critical stops')
  self.last_battery_state_code=v['battery_state_code']

 def send(self,vector,phase):
  if sum(x>1000 for x in vector)>1 and max(vector)>self.args.multi_motor_max:raise ValueError('Multi-motor command exceeds explicit plan ceiling')
  return super().send(vector,phase)

def allows_torque_gaps(plan):
 return plan.get('control_mode')=='acro' or plan.get('allow_torque_gaps',False)

def verify_plan(plan):
 if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',plan.get('label','')):raise ValueError('Invalid plan label')
 if plan.get('control_mode','direct') not in ('direct','acro'):raise ValueError('Invalid control mode')
 if plan.get('rear_terminals_confirmed') is not True or plan.get('terminal_rating_A')!=30:raise ValueError('Rear 30 A terminal confirmation required')
 if type(plan.get('command_ceiling')) is not int or not 1000<=plan['command_ceiling']<=2000:raise ValueError('Invalid explicit command ceiling')
 if type(plan.get('allow_torque_gaps',False)) is not bool:raise ValueError('Invalid optional torque flag')
 if plan['kind']=='balanced' and allows_torque_gaps(plan):raise ValueError('Yaw identification requires both torque channels')
 if plan['kind'] not in ['probe','pilot','individual','collective','dynamics','balanced','validation']:raise ValueError('Unknown direct test kind')
 if plan['voltage_V'] not in [7.6,7.8,8.0,8.2]:raise ValueError('Unverified supply voltage')
 if not 0<plan.get('force_limit_N',0)<=8.0:raise ValueError('Unverified force envelope')
 if type(plan.get('multi_motor_max')) is not int or not 1000<=plan['multi_motor_max']<=plan['command_ceiling']:raise ValueError('Unverified collective envelope')
 if not math.isfinite(plan.get('probe_seconds',8)) or not 0<plan.get('probe_seconds',8)<=30:raise ValueError('Invalid probe duration')
 if plan['kind']=='probe' and plan['phases']:raise ValueError('Probe must have no motor phases')
 if any(not math.isfinite(x['duration_s']) for x in plan['phases']):raise ValueError('Nonfinite phase duration')
 if sum(x['duration_s'] for x in plan['phases'])>90:raise ValueError('Plan exceeds finite block duration')
 for x in plan['phases']:
  v=x['commands']
  if len(v)!=4 or any(type(u)!=int or not 1000<=u<=plan['command_ceiling'] for u in v):raise ValueError('Invalid direct motor command')
  if sum(u>1000 for u in v)>1 and max(v)>plan.get('multi_motor_max',1350):raise ValueError('Invalid multi-motor range')
  if not (.05 if plan['kind']=='validation' else .15)<=x['duration_s']<=8:raise ValueError('Invalid phase duration')

def main():
 global WORKER
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('plan',type=Path)
 p.add_argument('--execute',action='store_true')
 p.add_argument('--bench-ready',action='store_true')
 p.add_argument('--battery-disconnected',action='store_true')
 p.add_argument('--rear-terminals-confirmed',action='store_true')
 p.add_argument('--off-axis-checked',action='store_true')
 args=p.parse_args();plan=json.loads(args.plan.read_text());verify_plan(plan)
 if plan.get('control_mode')=='acro' and CONFIG_PREPARE is None:p.error('Use full_acro.py for Acro plans')
 if not args.execute:print(json.dumps(plan,indent=2));return 0
 if not args.battery_disconnected or not args.rear_terminals_confirmed:p.error('PSU execution requires --battery-disconnected --rear-terminals-confirmed')
 if plan['kind']!='probe' and not args.bench_ready:p.error('Powered execution requires --bench-ready')
 if plan['kind'] in ('pilot','individual','dynamics') and not args.off_axis_checked:p.error('Single-motor execution requires --off-axis-checked')
 if not {1,4}.issubset(os.sched_getaffinity(0)):p.error('Validated acquisition requires CPUs 1 and 4')
 from hx711_native import supported_board, LIBRARY
 if not supported_board() or not LIBRARY.is_file() or not SLOW_LIBRARY.is_file():p.error('Build native libraries on the P3768/Tegra234 Jetson first')
 b.load_calibration(CALIBRATION)
 # Own the shared stand for supply setup, capture and all cleanup.
 session_lock=open('/tmp/sanity_bench_experiments.lock','a')
 try:fcntl.flock(session_lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BaseException:session_lock.close();raise
 ROOT.mkdir(parents=True,exist_ok=True)
 os.sched_setaffinity(0,{1})
 stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%S_%f');name='fullrange_'+plan['label']+'_'+stamp;output=ROOT/name;support=ROOT/(name+'_inputs');support.mkdir()
 receipt=dict(plan=plan,output_directory=str(output),support_directory=str(support));supply=group=None;original=None
 def finite(signum,frame):raise KeyboardInterrupt
 old_handlers={sig:signal.signal(sig,finite) for sig in (signal.SIGALRM,signal.SIGTERM,signal.SIGHUP)};signal.alarm(120)
 try:
  supply=lib.instantiate_tenma_class_from_device_response(PSU_PORT)
  if not isinstance(supply,lib.SingleChannelAdapter) or type(supply.device) is not lib.Tenma72_13350:raise RuntimeError('Unexpected supply identity')
  original=dict(voltage_V=supply.readVoltage(1),current_A=supply.readCurrent(1),status={k.strip():v for k,v in supply.getStatus().items()});receipt['original_supply']=original
  if original['status']['output']!='OFF' or original['current_A']!=30:raise RuntimeError('Supply must start OFF with user-requested 30 A limit')
  supply.setVoltage(1,round(plan['voltage_V']*1000));supply.ON();time.sleep(2)
  ready=[]
  for _ in range(12):
   before=t.prior.snapshot();ready.append(before['battery'])
   if before['battery']['detected_cells']==2 and before['battery']['battery_state_code']==0 and abs(before['battery']['voltage_V']-plan['voltage_V'])<.08:break
   time.sleep(.3)
  else:raise RuntimeError('FC voltage/cell detection did not settle after power-on')
  receipt['power_ready_samples']=ready;receipt['before']=before
  if before['status']['armed'] or before['outputs']!=[1000]*4 or any(m['rpm'] for m in before['motors']['motors']):raise RuntimeError('Initial FC not stopped')
  if CONFIG_PREPARE:receipt['configuration_trial']=CONFIG_PREPARE(plan)
  profile=support/'fc_profile_live.json';profile.write_text(json.dumps(before['controller'],indent=2));(support/'plan.json').write_text(json.dumps(plan,indent=2))
  sources=['full_campaign.py','campaign.py','torque_check.py','previous_low_check.py','hx711_slow10.c']+(['full_acro.py'] if plan.get('control_mode')=='acro' else [])
  for source in sources:(support/source).write_bytes((HERE/source).read_bytes())
  (support/'experiment.json').write_text(json.dumps(dict(power_source='Multicomp Pro 72-13350 regulated CV',supply_limit_A=30,measured_current_experiment_stop_A=30,terminal_rating_A=30,rear_terminals_confirmed_by_user=True,command_ceiling=plan['command_ceiling'],FC_bus_bounds_V=[6.6,8.4],FC_voltage_warning_action='logged, PSU only; critical stops',force_limit_N=plan.get('force_limit_N',.8),torque_amplitude_limit=None,optional_torque_channels=allows_torque_gaps(plan),thrust_ADC_required=True,torque_spacing_m=.06,assumed_perpendicular_arms_m=[.03,.03],shared_gain_counts_per_N=c.GAIN,source_voltage_setpoint_V=plan['voltage_V'],command_path='armed Acro RX_MSP; original PID/mixer/filtering retained' if plan.get('control_mode')=='acro' else 'disarmed MSP_SET_MOTOR; rate PID and mixer bypassed',operator_declarations=dict(bench_ready=args.bench_ready,battery_disconnected=args.battery_disconnected,rear_terminals_confirmed=args.rear_terminals_confirmed,off_axis_checked=args.off_axis_checked),ADC_library_path=str(SLOW_LIBRARY),ADC_library_sha256=hashlib.sha256(SLOW_LIBRARY.read_bytes()).hexdigest(),ADC_nominal_phase_delay_us=10,ADC_high_bound_us=50,ADC_worker_CPU=4,ADC_worker_FIFO_priority=70,control_and_supply_CPU=1,limits_note='Force median3 guard only; original raw data preserved. No torque amplitude cutoff. Digital MSP counts are not physical PWM widths.'),indent=2))
  b.FIELDS['fc_outputs']=['request_start_ns','reply_ns','phase','M1','M2','M3','M4']
  b.FIELDS['force']+=['guard_filtered_load_N','guard_filtered_delta_N'];b.FIELDS['supply']=SUPPLY_FIELDS
  b.FIELDS['imu']=['request_start_ns','reply_ns','unix_time_ns','phase','acc_X','acc_Y','acc_Z','gyro_X_dps','gyro_Y_dps','gyro_Z_dps','mag_X','mag_Y','mag_Z']
  b.FIELDS['torque']=['channel','phase','read_complete_ns','ready_ns','unix_time_ns','raw_counts','baseline_counts','delta_counts','force_equivalent_N','max_clock_high_bound_us']
  WORKER=SupplyWorker(supply);WORKER.start();t.GROUP=group=t.Group();group.open();slow=C.CDLL(str(SLOW_LIBRARY),use_errno=True);slow.hx_read.argtypes=group.lib.hx_read.argtypes;slow.hx_read.restype=group.lib.hx_read.restype;group.lib.hx_read=slow.hx_read;group.disabled_torque=set();group.readers[0]=RegulatedReader(group,allows_torque_gaps(plan));b.Bench=RegulatedBench;b.ForceReader=lambda:group.readers[0];b.CEILING=plan['command_ceiling'];b.STALE_FORCE_S=.15;b.HEALTH_PERIOD_S=.10
  options=b.parser(preset='probe' if plan['kind']=='probe' else 'reference').parse_args(['--execute','--bench-ready','--port',c.PORT,'--profile',str(profile),'--calibration',str(CALIBRATION),'--output',str(output),'--force-limit-n',str(plan.get('force_limit_N',.8)),'--rpm-hz','50','--seconds',str(plan.get('probe_seconds',8)),'--battery-id','regulated_PSU_'+str(plan['voltage_V'])+'V','--note','Regulated CV; 3 high-rate HX711s; MSP IMU; raw observations primary; nominal 30 mm torque arms; shared force gain provisional.'])
  options.task_lock_held=True;options.off_axis_checked=args.off_axis_checked;options.battery_disconnected=args.battery_disconnected
  options.command_ceiling=plan['command_ceiling'];options.record_direct_outputs=True;options.supply_voltage_V=plan['voltage_V'];options.actual_current_limit_A=30;options.allow_torque_gaps=allows_torque_gaps(plan);options.multi_motor_max=plan.get('multi_motor_max',1350);receipt['exit_code']=b.run(options,plan['phases'])
 except BaseException as exc:receipt['error']=repr(exc);receipt['exit_code']=1
 finally:
  signal.alarm(0)
  try:
   if group:
    group.close();receipt['restoration']=group.receipt;receipt['disabled_torque_channels']=sorted(group.disabled_torque);receipt['optional_torque_faults']=group.readers[0].fault_history
  except BaseException as exc:receipt['restoration_error']=repr(exc)
  try:
   for _ in range(15):
    after=t.prior.snapshot();receipt['after']=after;receipt['fc_stopped_observed']=not after['status']['armed'] and after['outputs']==[1000]*4 and all(m['rpm']==0 for m in after['motors']['motors'])
    if receipt['fc_stopped_observed']:break
    time.sleep(.1)
  except BaseException as exc:receipt['stop_verification_error']=repr(exc)
  try:
   if WORKER:WORKER.close()
  except BaseException as exc:receipt['supply_worker_cleanup_error']=repr(exc)
  if supply:
   if CONFIG_RESTORE:
    try:
     if not receipt.get('fc_stopped_observed'):raise RuntimeError('Configuration restore deferred because FC stop was not verified')
     receipt['configuration_restored']=CONFIG_RESTORE()
    except BaseException as exc:receipt['configuration_restore_error']=repr(exc)
   try:
    supply.OFF()
    if original:supply.setVoltage(1,round(original['voltage_V']*1000))
    receipt['final_supply']=dict(voltage_setpoint_V=supply.readVoltage(1),current_limit_A=supply.readCurrent(1),status={k.strip():v for k,v in supply.getStatus().items()})
   except BaseException as exc:receipt['supply_cleanup_error']=repr(exc)
   supply.close()
  if (output/'metadata.json').exists():
   meta=json.loads((output/'metadata.json').read_text());meta['regulated_source']=json.loads((support/'experiment.json').read_text());meta['control_mode']=plan.get('control_mode','direct');meta['units']['commands']='RC throttle count repeated in command M1-M4 columns; observed mixer outputs in fc_outputs.csv' if plan.get('control_mode')=='acro' else 'MSP external motor counts';meta['compensation_trial']=plan.get('compensation');meta['limitations']=[s for s in meta['limitations'] if not s.startswith('HX711 nominal')]+['HX711 high-rate continuous sampling; force includes ADC/filter/fixture response.','Supply V/I/status read sequentially around 1.6 complete triplets/s; settled power only.'];(output/'metadata.json').write_text(json.dumps(meta,indent=2))
  try:
   (support/'receipt.json').write_text(json.dumps(receipt,indent=2));print('FINAL_RECEIPT '+json.dumps(receipt),flush=True)
  finally:
   session_lock.close()
   for sig,handler in old_handlers.items():signal.signal(sig,handler)
 return 0 if receipt.get('exit_code')==0 and receipt.get('fc_stopped_observed') and receipt.get('final_supply',{}).get('status',{}).get('output')=='OFF' and not any(k.endswith('_error') for k in receipt) else 1
if __name__=='__main__':sys.exit(main())
