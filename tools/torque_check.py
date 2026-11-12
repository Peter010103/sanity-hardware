"""Bounded, disarmed whole-drone torque response experiment. Preview by default."""
import argparse, ctypes as C, csv, datetime, hashlib, json, mmap, os
from pathlib import Path
from stand_config import ROOT, HERE
import queue, signal, statistics, struct, sys, threading, time


import bench_experiments as b
import previous_low_check as prior
from hx711_native import Sample

GAIN = 21036.44535271289
CHANNELS = [dict(name='thrust', dout=105,sck=106,pad=0x02430070,expected=0x58,dpad=0x02430068),
 dict(name='torque_1',dout=144,sck=85,pad=0x02440020,expected=0x55,dpad=0x02448030),
 dict(name='torque_2',dout=41,sck=43,pad=0x02434040,expected=0x54,dpad=0x02434080)]

class Sensor:
 def __init__(self, group, index): self.g,self.i=group,index
 def read(self):
  began=time.monotonic_ns();s=Sample()
  code=self.g.lib.hx_read(self.g.handles[self.i],250000000,C.byref(s))
  if code: raise RuntimeError('HX711 %s error %s, HIGH bound %.2f us'%(CHANNELS[self.i]['name'],code,s.max_high_bound_ns/1000)+'; ready_ns=%d complete_ns=%d partial_raw=%d'%(s.ready_ns,s.complete_ns,s.raw_counts))
  return dict(raw_counts=s.raw_counts,read_begin_ns=began,ready_observed_monotonic_ns=s.ready_ns,
   read_complete_monotonic_ns=s.complete_ns,unix_time_ns=s.unix_ns,max_clock_high_bound_us=s.max_high_bound_ns/1000)
 def close(self): pass

class PassiveReader:
 def __init__(self):
  self.samples=queue.Queue(maxsize=1024);self.error=None;self.cancel=threading.Event();self.ident=None
 def start(self):pass
 def is_alive(self):return False

class Reader(b.ForceReader):
 def __init__(self,g):
  self.group=g
  super().__init__()
 def run(self):
  try:
   cpus=sorted(os.sched_getaffinity(0));os.sched_setaffinity(0,{cpus[-1]})
   os.sched_setscheduler(0,os.SCHED_FIFO,os.sched_param(30))
   sensors=[Sensor(self.group,i) for i in range(3)]
   # One worker owns all three transfers: GPIO clocks never overlap.
   for _ in range(10):
    for sensor in sensors:sensor.read()
   while not self.cancel.is_set():
    for sensor,reader in zip(sensors,self.group.readers):
     if self.cancel.is_set():break
     reader.samples.put_nowait(sensor.read())
  except BaseException as e:
   self.error=repr(e)
   for reader in self.group.readers[1:]:reader.error=self.error

class Group:
 def __init__(self):
  self.handles=[];self.maps={};self.changed=[];self.fd=None;self.readers=[];self.receipt=[];self.latency_fd=None;self.governor=None;self.governor_path=None
 def access(self,address,value=None):
  page=address&~4095
  if page not in self.maps:self.maps[page]=mmap.mmap(self.fd,4096,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ|mmap.PROT_WRITE,offset=page)
  mem=self.maps[page]
  if value is not None:struct.pack_into('<I',mem,address-page,value)
  return struct.unpack_from('<I',mem,address-page)[0]
 def open(self):
  compatible=Path('/proc/device-tree/compatible').read_bytes().split(b'\0')
  if b'nvidia,tegra234' not in compatible or not any(x.startswith(b'nvidia,p3768-') for x in compatible):raise RuntimeError('Unverified board')
  self.latency_fd=os.open('/dev/cpu_dma_latency',os.O_RDWR)
  os.write(self.latency_fd,struct.pack('i',0))
  self.governor_path=Path('/sys/devices/system/cpu/cpu7/cpufreq/scaling_governor')
  self.governor=self.governor_path.read_text().strip()
  self.governor_path.write_text('performance')
  self.lib=C.CDLL(str(Path(__file__).with_name("libhx711.so")),use_errno=True)
  for name,arguments,result in [('hx_open',[C.c_char_p,C.c_uint,C.c_uint],C.c_void_p),('hx_close',[C.c_void_p],None),('hx_reset',[C.c_void_p],C.c_int),('hx_read',[C.c_void_p,C.c_uint64,C.POINTER(Sample)],C.c_int),('hx_chip_matches',[C.c_char_p],C.c_int)]:
   f=getattr(self.lib,name);f.argtypes=arguments;f.restype=result
  chips=[p for p in Path('/dev').glob('gpiochip*') if self.lib.hx_chip_matches(str(p).encode())]
  if len(chips)!=1:raise RuntimeError('GPIO chip ambiguous')
  self.fd=os.open('/dev/mem',os.O_RDWR|os.O_SYNC)
  originals=[]
  for config in CHANNELS:
   old=self.access(config['pad']);data=self.access(config['dpad'])
   if old not in (config['expected'],config['expected']&~0x50) or data&0x400 or data&0x50!=0x50:raise RuntimeError('Unexpected pinmux')
   originals.append(old)
  for config in CHANNELS:
   handle=self.lib.hx_open(str(chips[0]).encode(),config['dout'],config['sck'])
   if not handle:raise OSError(C.get_errno(),'GPIO acquisition failed')
   self.handles.append(handle)
  for config,old in zip(CHANNELS,originals):
   record=dict(channel=config['name'],address=hex(config['pad']),original=hex(old),temporary=hex(old&~0x50))
   self.changed.append((config['pad'],old,record));self.receipt.append(record)
   if self.access(config['pad'],old&~0x50)!=old&~0x50:raise RuntimeError('Pad verification failed')
  for h in self.handles:
   if self.lib.hx_reset(h):raise RuntimeError('ADC reset failed')
  self.readers=[Reader(self),PassiveReader(),PassiveReader()]
 def close(self):
  for r in self.readers:r.cancel.set()
  for r in self.readers:
   if r.ident:r.join(timeout=.5)
   if r.is_alive():raise RuntimeError('ADC thread did not stop; retaining handles')
  for h in self.handles:self.lib.hx_close(h)
  for address,old,record in reversed(self.changed):record['restored']=hex(self.access(address,old));record['restore_ok']=self.access(address)==old
  for mem in self.maps.values():mem.close()
  if self.fd is not None:os.close(self.fd)
  if self.governor is not None:
   self.governor_path.write_text(self.governor)
   self.receipt.append(dict(cpu_governor_restored=self.governor_path.read_text().strip(),original=self.governor))
  if self.latency_fd is not None:os.close(self.latency_fd);self.receipt.append(dict(cpu_latency_request_released=True))
  if any(record.get('restore_ok') is False for record in self.receipt):raise RuntimeError('A GPIO pad did not restore; inspect restoration receipt')

GROUP=None
class TorqueBench(prior.RelativeBaselineBench):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  self.thistory={name:[] for name in ('torque_1','torque_2')};self.tbias={};self.tlatest={}
 def drain_force(self):
  super().drain_force()
  for config,r in zip(CHANNELS[1:],GROUP.readers[1:]):
   name=config['name']
   while True:
    try:x=r.samples.get_nowait()
    except queue.Empty:break
    raw=x['raw_counts'];stamp=x['read_complete_monotonic_ns'];base=self.tbias.get(name)
    self.log.row('torque',dict(channel=name,phase=self.state,read_complete_ns=stamp,
     ready_ns=x['ready_observed_monotonic_ns'],unix_time_ns=x['unix_time_ns'],raw_counts=raw,
     baseline_counts=base,delta_counts=None if base is None else raw-base,
     force_equivalent_N=None if base is None else (raw-base)/GAIN,
     max_clock_high_bound_us=x['max_clock_high_bound_us']))
    if raw in b.RAILS:raise RuntimeError(name+' at ADC rail')
    self.thistory[name].append((stamp,raw));self.tlatest[name]=stamp
   if r.error:raise RuntimeError(name+' worker failed: '+r.error)
 def force_guard(self):
  super().force_guard()
  # Startup only: allow the serial worker to finish its first three-channel cycle.
  if self.state=='initializing':
   deadline=b.mono()+1000000000
   while any(n not in self.tlatest for n in self.thistory) and b.mono()<deadline:
    time.sleep(.002);self.drain_force()
  if any(n not in self.tlatest or b.mono()-self.tlatest[n]>150000000 for n in self.thistory):raise RuntimeError('Torque ADC stale/missing')
 def baseline_torque(self):
  now=b.mono();out={}
  for name,history in self.thistory.items():
   values=[v for t,v in history if now-1000000000<=t<=now]
   if len(values)<30:raise RuntimeError('Too few torque baseline samples')
   mean=statistics.mean(values);sd=statistics.stdev(values)
   if sd>1000:raise RuntimeError('Torque baseline too noisy')
   self.tbias[name]=mean;out[name]=dict(mean_counts=mean,sd_counts=sd,n=len(values))
  self.log.event('torque_bias',phase=self.state,channels=out)
  print('TORQUE_BIAS '+json.dumps(out),flush=True)
 def run_plan(self,plan):
  self.hold(dict(name='stopped_baseline',commands=b.STOP,duration_s=2.5,role='stopped'))
  self.baseline,self.baseline_sd=self.recent_force_mean();self.baseline_torque()
  for spec in plan:
   if tuple(spec['commands'])!=b.STOP:
    self.poll_rpm()
    if self.vector!=b.STOP or any(m['rpm'] for m in self.latest_rpm['motors']):raise RuntimeError('Expected zero RPM between tests')
    self.baseline_torque()
   self.hold(spec)
  self.hold(dict(name='stopped_after',commands=b.STOP,duration_s=2.5,role='stopped'))
  self.final_baseline,_=self.recent_force_mean();self.baseline_torque()
