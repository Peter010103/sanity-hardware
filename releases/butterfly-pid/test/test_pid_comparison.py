"""Offline contracts for matched PID branch; no room services or hardware."""
from pathlib import Path
import copy,hashlib,importlib.util,json,re,subprocess,sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from sanity_zenoh.ideal_command_prediction import IdealCommandPrediction
from sanity_zenoh.mpc_benchmark_config import build_config
from sanity_zenoh.indi_hover import make_mission
from sanity_zenoh.protocol import ctbr_si_values,si_command_packet
from sanity_zenoh.firmware_collective import FirmwareCollective
REL=Path(__file__).resolve().parents[1];ROOT=next(p for p in Path(__file__).resolve().parents if (p/'loco_mjx').is_dir());BASE=REL.parent/'butterfly-indi';PKG=REL/'host/sanity_zenoh';SOURCE=PKG/'sanity_zenoh'
base=json.loads((PKG/'config/sanity15_indi_mpc_constant_arclength.json').read_text())
def config(speed=5.,**kw):
 return build_config(base,pattern='lemniscate',inner='pid',speed=speed,schedule='indi_shaped_angular',radius_m=2.,width_m=3.,butterfly_height_m=.3,butterfly_cycles=3,ground_m=kw.pop('ground_m',0.),**kw)

@pytest.mark.parametrize('speed',[3.,4.,5.])
def test_reference_and_shared_controller_parameters_match_indi(speed):
 c=config(speed);m=make_mission(c,ROOT/'loco_mjx');r=m._make_curve([0.,0.,1.])
 p=SOURCE/'indi_lemniscate.py';q=BASE/'host/sanity_zenoh/sanity_zenoh/indi_lemniscate.py'
 assert p.read_bytes()==q.read_bytes()
 spec=importlib.util.spec_from_file_location('archived_butterfly_geometry',q);module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
 old=module.ButterflyReference(center=np.array([0.,0.,1.]),radius=2.,width_m=3.,peak_speed=speed,height_amplitude_m=.3,cycles=3,ramp_s=2.,maximum_acceleration_m_s2=c['trajectory']['maximum_acceleration_m_s2'],maximum_jerk_m_s3=c['trajectory']['maximum_jerk_m_s3'])
 assert r.duration==old.duration
 for t in np.r_[np.linspace(0,r.duration,1101),np.arange(7)*r.segment_duration]:
  a,b=r.sample(float(t)),old.sample(float(t))
  for key in ['position','velocity','acceleration','jerk']:np.testing.assert_array_equal(a[key],b[key])
  assert a['plane_sign']==b['plane_sign'] and a['lap_progress']==b['lap_progress']
 pid=json.loads((REL/f'mpc_{speed:g}mps.json').read_text());indi=json.loads((BASE/f'mpc_{speed:g}mps.json').read_text())
 assert pid.pop('rate_response')=={'enabled':False};indi.pop('rate_response');assert pid==indi
 assert (REL/f'model_{speed:g}mps.xml').read_bytes()==(BASE/f'model_{speed:g}mps.xml').read_bytes()
 assert m.limits.maximum_rate_rad_s==.6 and m.emergency_braking_rate_rad_s==2.

@pytest.mark.parametrize('speed',[3.,4.,5.])
def test_native_pid_ack_is_required_and_indi_never_requested(speed):
 m=make_mission(config(speed),ROOT/'loco_mjx');good=dict(mode='pid',indi_weight=0.,requested=False,reason='ok',rpm_mask=15,permission=True,supported=True,fault_latched=False)
 assert m._ground_acknowledged(good) and m._ground_entry_eligible(good,True)
 for delta in [dict(mode='indi'),dict(indi_weight=.001),dict(requested=True),dict(reason='idle')]:assert not m._ground_acknowledged(dict(good,**delta))
 assert not m._ground_request() and not m._airborne_request()
 assert m._airborne_fault_reason(good) is None
 assert m._airborne_fault_reason(dict(good,mode='indi'))=='pid_authority_lost'
 assert m._airborne_fault_reason(dict(good,fault_latched=True,fault='allocator'))=='fc_controller_fault_allocator'


def test_fresh_ground_world_ceiling_remains_enforced():
 assert config(ground_m=.19)['limits']['ground_m']==.19
 with pytest.raises(ValueError,match='ceiling'):config(ground_m=.20)


def test_ideal_projection_uses_selected_commands_at_delay_without_indi_states():
 p=IdealCommandPrediction(.07151,.953,(0.,0.,0.),.015);T=.07151*9.81/.953
 s=np.r_[0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
 # A command chosen at1s only takes effect1.015s, prior selected hover is held.
 p.commit(900_000_000,[T,0.,0.,0.]);p.commit(1_000_000_000,[T,1.,0.,0.])
 out,latent=p.predict(s,1_000_000_000,1_035_000_000)
 assert latent is None
 assert Rotation.from_quat(out[[4,5,6,3]]).as_rotvec()[0]==pytest.approx(.02,abs=1e-9)
 assert p.last['projection_s']==pytest.approx(.035) and p.last['selected_history_only']
 before=copy.deepcopy(p.commands);p.predict(s,1_000_000_000,1_035_000_000)
 for a,b in zip(before,p.commands):assert a[0]==b[0];np.testing.assert_array_equal(a[1],b[1])
 with pytest.raises(ValueError):p.commit(1_000_000_000,[T,0.,0.,0.])
 with pytest.raises(ValueError):p.predict(s,1_000_000_000,1_080_000_000)


def test_high_rate_pid_mapping_and_ack_gate_are_explicit():
 c=config();mapping=FirmwareCollective(c['collective_profile'],high_rate=True,selected_high_rate=True)
 values=ctbr_si_values(True,.8,[9.5,-9.5,9.5],True,True)
 assert values[:3]==(9500,-9500,9500)
 packet=si_command_packet(15,True,123,456,1,100,100,values,mapping.calibration_id,high_rate=True,controller_selection=True,indi_requested=False)
 assert packet[:5]==b'SZC6\x06' and packet[7]==1  # enabled, INDI request bit clear
 rc=mapping.convert(15,True,.8,[9.5,-9.5,9.5]);assert (rc.roll,rc.pitch,rc.yaw)==(1954,1046,1046)
 with pytest.raises(ValueError):ctbr_si_values(True,.8,[9.501,0.,0.],True,True)
 text=(SOURCE/'indi_hover.py').read_text();assert "if config.get('high_speed_wire'):good=good and receiver.indi_high_rate_supported" in text


def test_real_archived_supervisor_does_not_latch_unused_allocator(tmp_path):
 fc=ROOT/'firmware/fc/tests/pid_supervisor_source'
 adapter=(fc/'paper_adapter.c').read_text()
 assert re.search(r'uint8_t fault = policy.weight > 0 && reason != SH_OK',adapter)
 (tmp_path/'platform.h').write_text('')
 (tmp_path/'test.c').write_text('''#include <assert.h>
#include "supervisor.h"
int main(void){shState_t s={0};shInput_t in={.armed=true,.enabled=true,.request=false,.eligible=false,.reason=SH_ALLOCATOR,.nowUs=1000,.dtUs=1000,.transitionUs=200000};
for(int k=0;k<1000;k++){/* exact adapter predicate */in.fault=s.weight>0&&in.reason!=SH_OK&&in.reason!=SH_MODE&&in.reason!=SH_IDLE&&in.reason!=SH_DISABLED?in.reason:0;shUpdate(&s,&in);assert(s.weight==0&&s.mode==SH_PID&&s.fault==0&&s.reason==SH_OK);in.nowUs+=1000;}
in.fault=SH_TIMING;shUpdate(&s,&in);assert(s.fault==SH_TIMING);return 0;}
''')
 subprocess.run(['cc','-std=c11','-I'+str(tmp_path),'-I'+str(fc),str(tmp_path/'test.c'),str(fc/'supervisor.c'),'-lm','-o',str(tmp_path/'test')],check=True,capture_output=True)
 subprocess.run([str(tmp_path/'test')],check=True)

@pytest.mark.parametrize('bad',[None,'mode','weight','request'])
def test_real_waiting_ticks_do_not_arm_without_native_pid(bad):
 m=make_mission(config(3.),ROOT/'loco_mjx');s=np.zeros(13);s[3]=1.
 c=dict(fc_uid_msp=m.limits.expected_uid,permission=True,supported=True,armed=False,request_aux=3,
        mode='pid',indi_weight=0.,requested=False,fault_latched=False,reason='ok',rpm_mask=15,
        control_period_us=1000.,gyro_native_rad_s=[0.,0.,0.])
 if bad=='mode':c['mode']='indi'
 elif bad=='weight':c['indi_weight']=.01
 elif bad=='request':c['requested']=True
 outputs=[m.tick(100.+i*.02,s,c,mocap_age_s=.004,controller_age_s=.002,receiver_ready=True) for i in range(61)]
 assert all(o['indi_requested'] is False for o in outputs)
 assert m.phase==('arming' if bad is None else 'waiting')
 assert outputs[-1]['enabled']==(bad is None)

@pytest.mark.parametrize('speed',[3.,4.,5.])
def test_actual_source_snapshot_contains_branch_wrapper_geometry_models_and_predictor(tmp_path,speed):
 spec=importlib.util.spec_from_file_location('pid_source_archiver',REL/'run_high_speed_lemniscate.py');launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)
 c=config(speed);launcher.snapshot_sources(tmp_path,c);manifest=json.loads((tmp_path/'source_manifest.json').read_text())['sha256']
 for p in [ROOT/'scripts/run_mpc.py',SOURCE/'ideal_command_prediction.py',SOURCE/'indi_lemniscate.py',SOURCE/'emergency_landing.py',Path(c['mpc']['config']),Path(c['mpc']['model'])]:
  key=str(p.relative_to(ROOT));assert manifest[key]==hashlib.sha256(p.read_bytes()).hexdigest();assert (tmp_path/'source'/key).read_bytes()==p.read_bytes()
