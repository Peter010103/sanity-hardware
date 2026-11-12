import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from sanity_zenoh.cable_configuration import controller_configuration, load_physical_configuration
from sanity_zenoh.cable_process import CableWorker
from sanity_zenoh.firmware_collective import FirmwareCollective


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='Offline configuration, dependency and solver warmup. Opens no ROS or command transport.')
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--profile',type=Path)
    parser.add_argument('--mission',choices=('circle_slow','circle_fast','payload_orbit'),default='circle_slow')
    parser.add_argument('--controller',choices=('aware','unaware'),default='aware')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=args.workspace.resolve();package=root/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh'
    profile=json.loads((args.profile or package/'config/sanity_cable.json').read_text());profile.update(mission=args.mission,controller=args.controller)
    physical=load_physical_configuration(root/'loco_mjx/experiments/sanity_cable_mpc',profile,args.profile or package/'config/sanity_cable.json')
    cfg=controller_configuration(physical,profile,simulation=False,enable_output=True)
    mapping=FirmwareCollective(package/'config/sanity09_esp_collective.json')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    (output/'config.json').write_text(json.dumps(cfg,indent=2)+'\n')
    worker=CableWorker(sys.executable,root/'loco_mjx/experiments/sanity_cable_mpc',output/'config.json',output,profile.get('solver_cpu_affinity',[]))
    try:
        sys.path.insert(0,str(root/'loco_mjx/experiments'))
        from sanity_cable_mpc.model import make_model
        model=make_model(cfg);state,command=model.equilibrium()
        result=worker.infer(state,np.tile(np.r_[state,command],(cfg['nodes']+1,1)))
        if 'error' in result or not result.get('mpc',{}).get('valid'):raise RuntimeError('Offline solver check failed: '+str(result))
        report={'passed':True,'hardware_access':False,'transport_opened':False,'geometry_confirmed':profile['geometry_confirmed'],'frames_confirmed':profile['mocap']['frames_confirmed'],
                'mission':args.mission,'controller':args.controller,'calibration_id':mapping.calibration_id,'solver':worker.info,'warm_result':result,
                'scope':'Configuration and software readiness only; loaded hover/flight response remains to validate',
                'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('cable_*.py')}}
        (output/'preflight.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({'passed':True,'transport_opened':False,'mission':args.mission,'controller':args.controller,'calibration_id':mapping.calibration_id,'result':str(output/'preflight.json')}))
    finally:worker.close()
