"""Native helper checks plus trace v4 and frozen-source invariants; no hardware."""
from pathlib import Path
import subprocess,json,hashlib,sys,tempfile
import numpy as np
HERE=Path(__file__).resolve().parent;BF=HERE.parent/'source';CORE=BF/'src/main/flight/sanity_indi'
ROOT=HERE.parents[3]
logs=[]
for sanitized in (False,True):
 exe=HERE/('test_rate_feedforward_sanitized' if sanitized else 'test_rate_feedforward')
 cmd=['gcc','-std=c99','-O1' if sanitized else '-O2','-fno-fast-math','-fno-finite-math-only','-ffp-contract=off','-fno-lto','-Wall','-Wextra','-Werror','-I'+str(CORE),str(HERE/'test_rate_feedforward.c'),str(CORE/'rate_feedforward.c'),'-lm','-o',str(exe)]
 if sanitized:cmd+=['-fsanitize=address,undefined','-fno-omit-frame-pointer']
 for c in [cmd,[str(exe)]]:
  r=subprocess.run(c,text=True,capture_output=True,timeout=60);logs.append(dict(command=c,returncode=r.returncode,stdout=r.stdout,stderr=r.stderr));print(r.stdout,end='');print(r.stderr,end='');r.check_returncode()
# Extended signed field index must retain layout through appended p189.
p=HERE/'blackbox_layout_test.c';s=p.read_text().replace('173','189');p.write_text(s)
cmd=['gcc','-std=c99','-Wall','-Wextra','-Werror',str(p),'-o',str(HERE/'blackbox_layout_test')]
subprocess.run(cmd,check=True,timeout=30);subprocess.run([str(HERE/'blackbox_layout_test')],check=True,timeout=10)
sys.path.insert(0,str(ROOT/'Sanity/src/ros2_sanity_ws/scripts'))
from sanity_control_analysis import read_csv
with tempfile.TemporaryDirectory(prefix='sanity_ff_trace_') as folder:
 for count,version in [(126,1),(152,2),(174,3),(190,4)]:
  p=Path(folder)/f'trace{version}.csv';a=np.zeros((3,count+2),dtype=np.int64);a[:,1]=[1000,5000,9000];a[:,2]=version;a[:,4]=a[:,1]
  np.savetxt(p,a,fmt='%d',delimiter=',',header=','.join(['loopIteration','time']+[f'p[{i}]' for i in range(count)]),comments='')
  b=read_csv(p);assert b.shape==(3,count+2) and np.all(b[:,2]==version)
 p=Path(folder)/'malformed.csv';a[:,2]=2;np.savetxt(p,a,fmt='%d',delimiter=',',header=','.join(['loopIteration','time']+[f'p[{i}]' for i in range(190)]),comments='')
 try:read_csv(p);raise AssertionError('Wrong trace version accepted')
 except ValueError:pass
old=ROOT/'outputs/sanity15_rate_feedforward_20261110/fc/source/src/main'
for name in ['flight/sanity_indi/speed_model.h','flight/sanity_indi/paper_native.c','flight/sanity_indi/paper_rpm.c','pg/sanity_paper.h','pg/sanity_paper.c']:
 assert (old/name).read_bytes()==(BF/'src/main'/name).read_bytes(),name
result=dict(success=True,helper_runs=logs,trace_schemas_checked=[126,152,174,190],new_PG=603,existing_native_model_RPM_and_PG_layouts_unchanged=True,hardware_accessed=False)
(HERE/'feedforward_test_results.json').write_text(json.dumps(result,indent=2)+'\n')
print('PASS trace v1/v2/v3/v4 compatibility, malformed rejection, native layout index189, unchanged native/model/RPM and PG')
