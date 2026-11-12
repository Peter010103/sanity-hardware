import struct
from types import SimpleNamespace
import pytest
import full_acro as a

class Log:
 def row(self,*args):pass
 def event(self,*args,**kwargs):pass
class Client:
 def __init__(self,armed=False):self.packets=[];self.armed=armed
 def _exchange(self,packet,command):self.packets.append((command,packet));return b''
 def query(self,command):
  assert command==101
  return struct.pack('<HHHIB',250,0,33,int(self.armed),0)

def shell(armed=False):
 x=a.AcroBench.__new__(a.AcroBench);x.client=Client(armed);x.log=Log();x.box_ids=[0];x.state='sensor_fault';x.stop_attempts=x.stop_acks=0;x.may_have_sent=True;x.vector=(1300,)*4;x.armed_expected=armed;x.requested_armed=armed;x.rc=[1500,1500,1500,1300,2000,1500,1000,1000];x.last_rc_ns=None;x.baseline=0.0;x.args=SimpleNamespace(command_ceiling=2000);x.rc_mapping=list(range(8))
 return x

def test_frame_rejects_nonzero_rate_or_excess_throttle():
 for v in [[1600,1500,1500,1200,2000,1500,1000,1000],[1500,1500,1500,2001,2000,1500,1000,1000]]:
  with pytest.raises(ValueError):a.frame(v)

def test_positive_throttle_rejected_until_arming_confirmed():
 x=shell(False);x.latest_status={'armed':False};x.healthy_before_command=lambda:None
 with pytest.raises(RuntimeError,match='confirmed armed'):x.send([1200]*4,'positive')
 assert not x.client.packets

def test_disarm_cleanup_does_not_depend_on_load_cell_health(monkeypatch):
 monkeypatch.setattr(a.time,'sleep',lambda seconds:None)
 x=shell(False);x.reader=SimpleNamespace(error='ADC failed')
 assert x.stop()
 assert x.client.packets[-1][0]==214
 for command,packet in x.client.packets:
  if command==200:
   rc=struct.unpack('<8H',packet[5:-1]);assert rc[3:5]==(1000,1000)

def test_no_direct_stop_until_disarm_observed(monkeypatch):
 monkeypatch.setattr(a.time,'sleep',lambda seconds:None)
 x=shell(True)
 assert x.stop() is False
 assert all(cmd==200 for cmd,_ in x.client.packets)

def test_stale_refresh_aborts_before_new_positive_output(monkeypatch):
 monkeypatch.setattr(a.b,'mono',lambda:200_000_000)
 x=shell(False);x.last_rc_ns=1
 with pytest.raises(RuntimeError,match='refresh'):x.pump()
 assert not x.client.packets


def test_aetr_wire_order_preserves_logical_throttle_and_zero_rates():
 logical=[1500,1500,1500,1000,1000,1500,1000,1000];mapping=[0,1,3,2,4,5,6,7]
 wire=struct.unpack("<8H",a.frame(logical,mapping)[5:-1])
 assert wire[2:4]==(1000,1500)
 assert [wire[i] for i in mapping]==logical


def test_invalid_rx_map_rejected():
 with pytest.raises(ValueError):a.frame([1500,1500,1500,1000,1000,1500,1000,1000],[0]*8)

def test_optional_torque_failure_is_a_gap_but_thrust_failure_remains_fatal():
 from full_campaign import RegulatedReader
 class Broken:
  def read(self):raise RuntimeError('ADC failed')
 import queue
 x=RegulatedReader.__new__(RegulatedReader);x.group=SimpleNamespace(disabled_torque=set());x.sensors=[Broken(),Broken(),Broken()];x.allow_torque_gaps=True;x.fault_history=[];x.faults=queue.Queue()
 assert x.sample(1) is None
 assert x.group.disabled_torque=={'torque_1'}
 assert x.fault_history[0]['action'].startswith('disabled')
 with pytest.raises(RuntimeError):x.sample(0)
 x.allow_torque_gaps=False
 with pytest.raises(RuntimeError):x.sample(2)

def test_yaw_identification_cannot_run_with_optional_torque_channels():
 from full_campaign import verify_plan
 with pytest.raises(ValueError,match='requires both torque'):
  verify_plan(dict(label='balanced',kind='balanced',allow_torque_gaps=True,voltage_V=8.0,phases=[],rear_terminals_confirmed=True,terminal_rating_A=30,command_ceiling=2000))


def test_full_endpoint_encodes_but_out_of_range_does_not():
 rc=[1500,1500,1500,2000,2000,1500,1000,1000]
 assert struct.unpack('<8H',a.frame(rc)[5:-1])[3]==2000
 with pytest.raises(ValueError):a.frame([1500,1500,1500,2001,2000,1500,1000,1000])

def test_explicit_stage_and_terminal_confirmation_are_required():
 import json
 from pathlib import Path
 from full_campaign import verify_plan
 p=json.loads((Path(__file__).parent.parent/'examples/direct-stage.json').read_text());verify_plan(p)
 p['command_ceiling']=1200;p['multi_motor_max']=1200
 with pytest.raises(ValueError):verify_plan(p)
 p=json.loads((Path(__file__).parent.parent/'examples/direct-stage.json').read_text());p['rear_terminals_confirmed']=False
 with pytest.raises(ValueError):verify_plan(p)


def test_near_idle_failure_stops_high_command_after_startup_allowance(monkeypatch):
 import full_campaign as f
 monkeypatch.setattr(f.b,'mono',lambda:1_000_000_000)
 x=SimpleNamespace(command_ack_ns=1,vector=[2000,1000,1000,1000],latest_rpm={'motors':[{'rpm':1300},{'rpm':0},{'rpm':0},{'rpm':0}]})
 with pytest.raises(RuntimeError,match='near-idle'):f.check_spinup(x)
 x.command_ack_ns=900_000_000;f.check_spinup(x)
 x.command_ack_ns=1;x.latest_rpm['motors'][0]['rpm']=30000;f.check_spinup(x)


def test_psu_warning_is_recorded_but_critical_voltage_stops(monkeypatch):
 import full_campaign as f
 import json
 from pathlib import Path
 v=dict(detected_cells=2,battery_state_code=1,voltage_V=6.8,current_A=0.0,consumed_mAh=0)
 monkeypatch.setattr(f.b,'battery',lambda payload:v)
 x=f.RegulatedBench.__new__(f.RegulatedBench);x.query=lambda cmd:(0,100,b'');x.log=Log()
 x.poll_battery();assert x.latest_battery['battery_state_code']==1
 v['battery_state_code']=2
 with pytest.raises(RuntimeError,match='critical'):x.poll_battery()
 v.update(battery_state_code=1,voltage_V=6.59)
 with pytest.raises(RuntimeError,match='critical'):x.poll_battery()
