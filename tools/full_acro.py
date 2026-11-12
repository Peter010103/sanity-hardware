"""Finite RX_MSP Acro identification; force-monitored, always disarms before cleanup."""
import sys,time,json,struct,functools,os,select,re
from pathlib import Path

import full_campaign as reg
b=reg.b;c=reg.c;t=reg.t
t.prior.bf.COMMANDS.update({34:'MODE_RANGES',64:'RX_MAP',105:'RC'})
ORIGINAL_COMP=100
CONFIG_STATE=None

def frame(values,mapping=None):
 if len(values)!=8 or any(not 1000<=v<=2000 for v in values):raise ValueError('RC vector outside approved bench range')
 if values[:3]!=[1500]*3 or values[3]>2000:raise ValueError('Acro bench requires zero body rates and bounded throttle')
 mapping=list(range(8)) if mapping is None else list(mapping)
 if sorted(mapping)!=list(range(8)):raise ValueError('Invalid RX channel map')
 wire=[0]*8
 for logical,channel in enumerate(mapping):wire[channel]=values[logical]
 body=bytes([16,200])+struct.pack('<8H',*wire)
 return b'$M<'+body+bytes([functools.reduce(int.__xor__,body,0)])

def cli_comp(value=None,capture=False):
 """Read or change only active-profile vbat_sag_compensation; no other writes."""
 transport=t.prior.bf.USBSerial(c.PORT);entered=False;log=[]
 def prompt(timeout=10):
  data=bytearray();deadline=time.monotonic()+timeout;last=time.monotonic()
  while time.monotonic()<deadline:
   if select.select([transport.fd],[],[],.08)[0]:data.extend(os.read(transport.fd,2048));last=time.monotonic()
   elif data.endswith(b'\r\n# ') and time.monotonic()-last>.15:return bytes(data)
  raise TimeoutError('CLI compensation prompt timeout')
 def command(text):
  transport.write(text.encode()+b'\r\n',time.monotonic()+1);data=prompt();log.append(data.decode(errors='replace'))
  if b'###ERROR' in data:raise RuntimeError('CLI rejected compensation operation')
  return data.decode(errors='replace')
 try:
  transport.write(b'#',time.monotonic()+1);entered=True;prompt()
  text=command('get vbat_sag_compensation');match=re.search(r'^vbat_sag_compensation = (\d+)',text,re.M)
  if not match:raise RuntimeError('Cannot read original compensation value')
  before=int(match.group(1))
  if capture:
   for text in ['profile','rateprofile','map','get small_angle','get rc_smoothing','get p_roll','get i_roll','get d_roll','get p_pitch','get i_pitch','get d_pitch','get p_yaw','get i_yaw','get d_yaw','get dshot_idle_value','get dyn_idle_min_rpm','get motor_output_limit','get anti_gravity_gain','get thrust_linear','get throttle_limit','get thr_mid','get thr_expo','get rates_type','get roll_rc_rate','get pitch_rc_rate','get yaw_rc_rate','get roll_srate','get pitch_srate','get yaw_srate','get roll_expo','get pitch_expo','get yaw_expo']:
    command(text)
  if CONFIG_STATE is not None and CONFIG_STATE['original'] is None:CONFIG_STATE['original']=before
  if value is not None and value!=before:
   if value not in [0,100]:raise ValueError('Unapproved compensation trial value')
   command('set vbat_sag_compensation = '+str(value));transport.write(b'save\r\n',time.monotonic()+1);entered=False;time.sleep(4);transport.close();transport=t.prior.bf.USBSerial(c.PORT)
   transport.write(b'#',time.monotonic()+1);entered=True;prompt();text=command('get vbat_sag_compensation');match=re.search(r'^vbat_sag_compensation = (\d+)',text,re.M)
   if not match or int(match.group(1))!=value:raise RuntimeError('Compensation readback mismatch')
  return dict(before=before,after=before if value is None else value,cli=log)
 finally:
  if entered:transport.write(b'exit\r\n',time.monotonic()+1);time.sleep(3)
  transport.close()

def prepare(plan):
 global CONFIG_STATE
 desired=plan['compensation'];CONFIG_STATE=dict(original=None,trial=desired)
 result=cli_comp(desired,plan.get('capture_config',False))
 snap=t.prior.snapshot()
 if snap['status']['armed'] or snap['outputs']!=[1000]*4:raise RuntimeError('FC not disarmed after configuration')
 return result

def restore():
 if CONFIG_STATE is None or CONFIG_STATE['original'] is None:return None
 return cli_comp(CONFIG_STATE['original'])

class AcroBench(reg.RegulatedBench):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  if not self.live['features']['mask']&(1<<14):raise RuntimeError('RX_MSP feature not enabled')
  ranges=self.client.query(34);rules=[tuple(ranges[i:i+4]) for i in range(0,len(ranges),4)]
  active=[r for r in rules if len(r)==4 and r[3]>r[2]]
  if [r for r in active if r[0]==0]!=[(0,0,32,48)]:raise RuntimeError('Unverified arming AUX mapping '+repr(active))
  if any(r[0] not in [0,1] or r[0]==1 and not (r[1]==1 and r[2]<=0 and r[3]<=16) for r in active):raise RuntimeError('Unverified stabilisation AUX mapping '+repr(active))
  self.rc_mapping=list(self.client.query(64))
  if sorted(self.rc_mapping)!=list(range(8)):raise RuntimeError('Unsupported RX map')
  self.log.event('acro_mode_ranges_verified',raw_hex=ranges.hex(),active_rules=active,logical_to_wire=self.rc_mapping)
  self.rc=[1500,1500,1500,1000,1000,1500,1000,1000];self.requested_armed=False;self.armed_expected=False;self.arm_deadline=None;self.disarm_deadline=None;self.last_rc_ns=None;self.next_rc_ns=0;self.next_rc_readback_ns=0
 def rc_send(self):
  a=b.mono();self.may_have_sent=True;self.client._exchange(frame(self.rc,self.rc_mapping),200);z=b.mono();self.last_rc_ns=z;self.next_rc_ns=z+20_000_000
  self.log.row('rc_commands',dict(request_start_ns=a,ack_ns=z,phase=self.state,arm_requested=self.armed_expected,**dict(zip(['roll','pitch','yaw','throttle','aux1','aux2','aux3','aux4'],self.rc))))
 def pump(self,display=True):
  if self.last_rc_ns is not None and b.mono()-self.last_rc_ns>150_000_000:raise RuntimeError('RC refresh exceeded 150 ms deadline')
  if b.mono()>=self.next_rc_ns:self.rc_send()
  super().pump(display=display)
 def poll_status(self):
  a,z,payload=self.query(101);data=t.prior.bf.status(payload,self.box_ids);self.latest_status,self.status_time=data,z;self.next_status=z+100_000_000
  self.log.row('status',dict(request_start_ns=a,reply_ns=z,armed=data['armed'],raw_payload_hex=payload.hex()))
  if data['armed']:
   if not self.armed_expected and not (self.disarm_deadline and z<self.disarm_deadline):raise RuntimeError('Unexpected arming')
   self.arm_deadline=None
   for mode in [1,2,31]:
    if mode in self.box_ids and data['mode_flags_first32']&(1<<self.box_ids.index(mode)):raise RuntimeError('Non-Acro stabilisation/flip mode active')
  elif self.armed_expected and (not self.arm_deadline or z>=self.arm_deadline):raise RuntimeError('Arming refused or unexpected disarm: '+payload.hex())
 def poll_rpm(self):
  c.ENGINE_BENCH.poll_rpm(self)
  a,z,payload=self.query(104);v=list(struct.unpack_from('<4H',payload));self.log.row('fc_outputs',dict(request_start_ns=a,reply_ns=z,phase=self.state,**dict(zip(['M1','M2','M3','M4'],v))))
  reg.check_spinup(self)
  if any(not 1000<=u<=2000 for u in v):raise RuntimeError('FC mixer output outside identified 1000-2000 range')
  if z>=self.next_rc_readback_ns:
   a,z,payload=self.query(105);rc=list(struct.unpack_from('<8H',payload));self.log.row('rc_readback',dict(request_start_ns=a,reply_ns=z,phase=self.state,**dict(zip(['roll','pitch','yaw','throttle','aux1','aux2','aux3','aux4'],rc))))
   if self.command_ack_ns and z-self.command_ack_ns>100_000_000 and rc!=self.rc:raise RuntimeError('RX_MSP channel readback mismatch')
   self.next_rc_readback_ns=z+100_000_000
 def send(self,vector,phase):
  if len(set(vector))!=1 or not 1000<=vector[0]<=self.args.command_ceiling:raise ValueError('Only collective Acro throttle is permitted')
  if self.requested_armed and self.baseline is None:raise RuntimeError('Arming requires stopped force baseline')
  if vector[0]>1000:
   self.healthy_before_command()
   if not self.armed_expected or not self.latest_status['armed']:raise RuntimeError('Positive throttle requires confirmed armed Acro')
  self.command_event+=1;a=b.mono();self.vector=tuple(vector)
  changing=self.requested_armed!=self.armed_expected;self.armed_expected=self.requested_armed
  if changing:
   self.arm_deadline=a+3_000_000_000 if self.armed_expected else None;self.disarm_deadline=None if self.armed_expected else a+300_000_000
  self.rc=[1500,1500,1500,vector[0],2000 if self.armed_expected else 1000,1500,1000,1000]
  self.rc_send();z=self.last_rc_ns;self.command_ack_ns=z
  self.log.row('commands',dict(event=self.command_event,phase=phase,request_start_ns=a,ack_ns=z,acknowledged=True,**dict(zip(['M1','M2','M3','M4'],vector))))
  return self.command_event,a,z
 def hold(self,spec):
  self.requested_armed=spec.get('armed',False)
  return super().hold(spec)
 def stop(self):
  if not self.may_have_sent:return True
  self.rc=[1500,1500,1500,1000,1000,1500,1000,1000];self.armed_expected=self.requested_armed=False;good=False
  for _ in range(5):
   self.stop_attempts+=1
   try:self.rc_send();self.stop_acks+=1;good=True
   except BaseException:pass
   time.sleep(.03)
  try:
   for _ in range(10):
    if not t.prior.bf.status(self.client.query(101),self.box_ids)['armed']:break
    self.rc_send();time.sleep(.05)
   else:return False
   self.client._exchange(b.motor_packet([1000]*4,4,1000,2000,1550),214);self.vector=b.STOP
  except BaseException:return False
  return good
 def run_plan(self,plan):
  self.hold(dict(name='disarmed_baseline',commands=[1000]*4,duration_s=3,role='stopped',armed=False));self.stopped_bias()
  self.hold(dict(name='arm_idle',commands=[1000]*4,duration_s=3.2,role='armed_idle',armed=True))
  self.poll_status()
  if not self.latest_status['armed']:raise RuntimeError('Arm not confirmed before sweep')
  for spec in plan:self.hold(spec)
  self.hold(dict(name='disarmed_after',commands=[1000]*4,duration_s=3.2,role='stopped',armed=False));self.final_baseline,_=self.recent_force_mean();self.stopped_bias()

original_verify=reg.verify_plan

def verify(plan):
 original_verify(plan)
 if plan.get('control_mode')!='acro' or plan['compensation'] not in [0,100]:raise ValueError('Explicit Acro compensation condition required')
 if any(len(set(p['commands']))!=1 or p['commands'][0]>plan['command_ceiling'] or not p.get('armed') for p in plan['phases']):raise ValueError('Invalid armed plan')

if __name__=='__main__':
 b.FIELDS['rc_commands']=['request_start_ns','ack_ns','phase','arm_requested','roll','pitch','yaw','throttle','aux1','aux2','aux3','aux4'];b.FIELDS['fc_outputs']=['request_start_ns','reply_ns','phase','M1','M2','M3','M4'];b.FIELDS['rc_readback']=['request_start_ns','reply_ns','phase','roll','pitch','yaw','throttle','aux1','aux2','aux3','aux4']
 reg.RegulatedBench=AcroBench;reg.CONFIG_PREPARE=prepare;reg.CONFIG_RESTORE=restore;reg.verify_plan=verify;sys.exit(reg.main())
