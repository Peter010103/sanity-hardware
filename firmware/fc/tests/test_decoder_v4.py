"""Exercise the existing decoder with a 190-field binary Blackbox fixture."""
from pathlib import Path
import subprocess,json,sys
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];folder=HERE/'decoder_v4';folder.mkdir(exist_ok=True)
raw=ROOT/'outputs/sanity15_four_flights_20261110_1746/blackbox/read1.bbl'
with raw.open('rb') as stream:data=stream.read(12000) # local header only, no FC IO
lines=[];pos=0
while data[pos:pos+2]==b'H ':
 end=data.index(b'\n',pos);lines.append(data[pos:end+1]);pos=end+1
header=[]
for line in lines:
 if line.startswith(b'H Field I name:'):line=line.rstrip()+b','+b','.join(f'p[{i}]'.encode() for i in range(152,190))+b'\n'
 elif any(line.startswith(prefix) for prefix in [b'H Field I signed:',b'H Field P predictor:']):line=line.rstrip()+b',1'*38+b'\n'
 elif any(line.startswith(prefix) for prefix in [b'H Field I predictor:',b'H Field I encoding:',b'H Field P encoding:']):line=line.rstrip()+b',0'*38+b'\n'
 elif line.startswith(b'H sanity_indi_trace_version:'):line=b'H sanity_indi_trace_version:4\n'
 elif line.startswith(b'H sanity_indi_trace_fields:'):line=b'H sanity_indi_trace_fields:190\n'
 header.append(line)
def uv(v):
 out=bytearray()
 while v>=128:out.append((v&127)|128);v>>=7
 out.append(v);return out
def sv(v):return uv((v<<1)^(-1 if v<0 else 0))
frames=[];expected=[]
for n in range(3):
 iteration=n*32;time=1000+n*32000;fields=[4,iteration+1000,time]+[(-1 if i%2 else 1)*(i*1000+n) for i in range(3,190)]
 frames.append(b'I'+uv(iteration)+uv(time)+b''.join(sv(v) for v in fields));expected.append([iteration,time,*fields])
p=folder/'read1.bbl';p.write_bytes(b''.join(header)+b''.join(frames))
decoder=ROOT/'outputs/sanity15_rate_feedforward_20261110/decoder/obj/blackbox_decode'
r=subprocess.run([str(decoder),'--output-dir',str(folder),str(p)],capture_output=True,text=True,timeout=30)
(folder/'decoder.log').write_text(r.stdout+r.stderr);r.check_returncode()
sys.path.insert(0,str(ROOT/'Sanity/src/ros2_sanity_ws/scripts'))
from sanity_control_analysis import read_csv
csv=folder/'read1.01.csv';actual=read_csv(csv);assert np.array_equal(actual,np.array(expected)),(actual.shape,np.array(expected).shape)
(HERE/'decoder_v4_results.json').write_text(json.dumps(dict(success=True,frames=len(frames),trace_fields=190,trace_version=4,binary_signed_values_and_extended_indices_verified=True,decoder=str(decoder)),indent=2)+'\n')
print('PASS existing192-field-capacity decoder:190-field v4 binary fixture, all values exact')
