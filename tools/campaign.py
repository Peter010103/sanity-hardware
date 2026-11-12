"""Finite battery-powered repetition / balanced-yaw campaign. No device access by default."""
import argparse,datetime,hashlib,json,math,random,signal,statistics,struct,sys,time,queue
from collections import deque
from pathlib import Path
from stand_config import ROOT, HERE

import torque_check as t
b=t.b
ENGINE_BENCH=b.Bench
PORT=t.prior.PORT
STOP=[1000]*4
SIGNS=[1,-1,-1,1]
GAIN=t.GAIN
t.prior.bf.COMMANDS.update({102:"RAW_IMU",80:"BLACKBOX_CONFIG",70:"DATAFLASH_SUMMARY",79:"SDCARD_SUMMARY"})
MIN_V,MAX_V=7.52,8.40
class CampaignBench(t.TorqueBench):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs);self.next_imu=0;self.guard_force_window=deque(maxlen=3)
 def drain_force(self):
  # Raw samples remain in CSV; only the guard/bias path has a causal median.
  while True:
   try:raw=self.reader.samples.get_nowait()
   except queue.Empty:break
   counts=raw['raw_counts'];force=b.force_from_counts(counts,self.calibration)
   good=counts not in b.RAILS and math.isfinite(force)
   if good:self.guard_force_window.append(force)
   filtered=statistics.median(self.guard_force_window) if self.guard_force_window else force
   delta=None if self.baseline is None else force-self.baseline
   filtered_delta=None if self.baseline is None else filtered-self.baseline
   stamp=raw['read_complete_monotonic_ns']
   self.log.row('force',dict(sequence=self.log.counts['force'],read_begin_ns=raw['read_begin_ns'],ready_observed_ns=raw['ready_observed_monotonic_ns'],read_complete_ns=stamp,unix_time_ns=raw['unix_time_ns'],raw_counts=counts,load_N=force,baseline_N=self.baseline,force_delta_N=delta,thrust_axis_force_N=None if delta is None else self.args.thrust_sign*delta,max_clock_high_bound_us=raw['max_clock_high_bound_us'],valid=good,guard_filtered_load_N=filtered,guard_filtered_delta_N=filtered_delta))
   if not good:raise RuntimeError('HX711 at ADC rail or nonfinite')
   self.latest_force=filtered;self.force_time=stamp;self.force_history.append((stamp,filtered))
   if filtered_delta is not None and abs(filtered_delta)>self.args.force_limit_n:raise RuntimeError('Median3 force delta exceeds retained force cutoff')
  if self.reader.error:raise RuntimeError('HX711 worker failed: '+self.reader.error)
  for config,r in zip(t.CHANNELS[1:],t.GROUP.readers[1:]):
   name=config['name']
   while True:
    try:x=r.samples.get_nowait()
    except queue.Empty:break
    raw=x['raw_counts'];stamp=x['read_complete_monotonic_ns'];base=self.tbias.get(name)
    self.log.row('torque',dict(channel=name,phase=self.state,read_complete_ns=stamp,ready_ns=x['ready_observed_monotonic_ns'],unix_time_ns=x['unix_time_ns'],raw_counts=raw,baseline_counts=base,delta_counts=None if base is None else raw-base,force_equivalent_N=None if base is None else (raw-base)/GAIN,max_clock_high_bound_us=x['max_clock_high_bound_us']))
    if raw in b.RAILS:raise RuntimeError(name+' at ADC rail')
    self.thistory[name].append((stamp,raw));self.tlatest[name]=stamp
   if r.error:raise RuntimeError(name+' worker failed: '+r.error)
 def pump(self,display=True):
  super().pump(display=display)
  if b.mono()>=self.next_imu:
   a,z,payload=self.query(102)
   if len(payload)!=18:raise RuntimeError('Unexpected IMU payload size')
   values=struct.unpack('<9h',payload)
   self.log.row('imu',dict(request_start_ns=a,reply_ns=z,unix_time_ns=time.time_ns(),phase=self.state,**dict(zip(['acc_X','acc_Y','acc_Z','gyro_X_dps','gyro_Y_dps','gyro_Z_dps','mag_X','mag_Y','mag_Z'],values))))
   self.next_imu=a+10000000
   self.force_guard()
 def poll_battery(self):
  ENGINE_BENCH.poll_battery(self)
  v=self.latest_battery
  if v['detected_cells']!=2 or not MIN_V<=v['voltage_V']<=MAX_V:raise RuntimeError('2S battery outside campaign voltage bounds')
  if v['current_A']>8.:raise RuntimeError('Current exceeds retained 8 A cutoff')
 def stopped_bias(self):
  self.force_guard();mean,sd=self.recent_force_mean()
  now=b.mono();values=[v for stamp,v in self.force_history if now-1000000000<=stamp<=now]
  if len(values)<40 or sd>.030 or abs(mean-statistics.median(values))>.015:raise RuntimeError('Unstable or contaminated stopped thrust bias')
  for name,history in self.thistory.items():
   vv=[v for stamp,v in history if now-1000000000<=stamp<=now]
   if len(vv)<40 or statistics.stdev(vv)>500:raise RuntimeError('Unstable stopped torque bias '+name)
  self.baseline,self.baseline_sd=mean,sd;self.baseline_torque()
  self.log.event('all_channels_stopped_bias',phase=self.state,thrust_mean_N=mean,thrust_sd_N=sd)
 def run_plan(self,plan):
  self.hold(dict(name='stopped_baseline',commands=STOP,duration_s=3.,role='stopped'))
  self.stopped_bias()
  for spec in plan:
   if tuple(spec['commands'])!=b.STOP and self.vector==b.STOP:
    self.poll_rpm()
    if any(m['rpm'] for m in self.latest_rpm['motors']):raise RuntimeError('RPM not zero before new powered block')
    self.stopped_bias()
   self.hold(spec)
  self.hold(dict(name='stopped_after',commands=STOP,duration_s=3.,role='stopped'))
  self.final_baseline,_=self.recent_force_mean();self.stopped_bias()

def add_stop(plan,name):plan.append(dict(name='stop_after_'+name,commands=STOP,duration_s=2.,role='stopped',cycle=0))
