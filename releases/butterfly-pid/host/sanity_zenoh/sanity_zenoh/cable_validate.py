import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


CASES={
    'slow_aware':('circle_slow','aware','none'),
    'fast_aware':('circle_fast','aware','none'),
    'orbit_aware':('payload_orbit','aware','none'),
    'slow_unaware':('circle_slow','unaware','none'),
    'fast_unaware':('circle_fast','unaware','none'),
    'payload_stale':('circle_slow','aware','payload_stale'),
    'solver_delay':('circle_slow','aware','solver_delay'),
    'vehicle_stale':('circle_slow','aware','vehicle_stale'),
    'pair_skew':('circle_slow','aware','pair_skew'),
}


def verify_case(directory, root, emulator, expected):
    data=json.loads((directory/'assessment.json').read_text())
    if tuple(data.get(key) for key in ('mission','controller','fault'))!=expected:
        raise ValueError('Case mission/controller/fault mismatch: '+str(directory))
    sources=json.loads((directory/'source_manifest.json').read_text())
    if sources['native_emulator_sha256']!=hashlib.sha256(emulator.read_bytes()).hexdigest():
        raise ValueError('Native emulator changed: '+str(directory))
    changed=[name for name,digest in sources['files'].items() if not (root/name).is_file() or hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest]
    if changed:raise ValueError('Sources changed since case: '+str(changed))
    return {'passed':bool(data['passed']),'output':str(directory),'sources_verified':True,
            'tracking':data.get('tracking'),'abort_reason':data.get('controller_receipt',{}).get('abort_reason'),
            'fatal':data.get('controller_receipt',{}).get('fatal'),'landed':data.get('controller_receipt',{}).get('landed')}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='Loopback-only, serial nine-case ROS/native cable mission and fault qualification. Opens no aircraft endpoints.')
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--emulator',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--reuse',action='append',default=[],metavar='CASE=OUTPUT_DIRECTORY')
    parser.add_argument('--keep-going',action='store_true',help='Collect independent remaining cases after a failed case')
    args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID','0') in ('','0'):
        parser.error('Set localhost-only DDS and a dedicated nonzero ROS_DOMAIN_ID')
    root=args.workspace.resolve();emulator=args.emulator.resolve();output=args.output.resolve()
    reused={}
    for item in args.reuse:
        name,value=item.split('=',1)
        if name not in CASES or name in reused:parser.error('Unknown or duplicated reused case: '+name)
        reused[name]=Path(value).resolve()
    output.mkdir(parents=True,exist_ok=False)
    report={'passed':False,'complete_suite':False,'scope':'Software and clocked loopback/native simulation only; no loaded hardware flight qualification',
            'cases':{},'error':None,'validator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    for name,expected in CASES.items():
        try:
            if name in reused:
                report['cases'][name]=verify_case(reused[name],root,emulator,expected)
                report['cases'][name]['reused']=True
            else:
                directory=output/name
                command=[sys.executable,'-m','sanity_zenoh.cable_simulation','--workspace',str(root),'--emulator',str(emulator),'--output',str(directory),
                         '--mission',expected[0],'--controller',expected[1],'--fault',expected[2]]
                print('START '+name,flush=True)
                with (output/(name+'.log')).open('w') as log:
                    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    try:process.wait(timeout=900)
                    except BaseException:
                        try:os.killpg(process.pid,signal.SIGTERM)
                        except ProcessLookupError:pass
                        process.wait(timeout=5)
                        raise
                report['cases'][name]=verify_case(directory,root,emulator,expected)
            print(json.dumps({'case':name,**report['cases'][name]}),flush=True)
        except Exception as error:
            report['error']=type(error).__name__+': '+str(error)
        report['complete_suite']=len(report['cases'])==len(CASES)
        report['passed']=bool(report['complete_suite'] and not report['error'] and all(row['passed'] for row in report['cases'].values()))
        (output/'qualification.json').write_text(json.dumps(report,indent=2)+'\n')
        if report['error'] or (not report['cases'][name]['passed'] and not args.keep_going):break
    print(json.dumps({'passed':report['passed'],'complete_suite':report['complete_suite'],'error':report['error'],'report':str(output/'qualification.json')}),flush=True)
    raise SystemExit(0 if report['passed'] else 1)
