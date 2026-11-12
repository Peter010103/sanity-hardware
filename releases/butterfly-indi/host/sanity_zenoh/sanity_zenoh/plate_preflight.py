import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
from .plate_configuration import PlateDeploymentConfig
from .plate_process import PlateWorker
from .firmware_collective import FirmwareCollective


def main():
    parser=argparse.ArgumentParser(description='Offline plate configuration/solver checks; opens no ROS or aircraft transport')
    parser.add_argument('--workspace',required=True);parser.add_argument('--profile');parser.add_argument('--mission-profile');parser.add_argument('--output',required=True)
    args=parser.parse_args();root=Path(args.workspace).resolve();sys.path.insert(0,str(root/'loco_mjx'))
    from experiments.sanity_plate_transport.flight_model import PlateFlightModel
    from experiments.sanity_plate_transport.flight_reference import FlightReference
    package=root/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh'
    config=PlateDeploymentConfig(args.profile or package/'config/sanity_plate_108g.json',root,args.mission_profile)
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    mappings=[FirmwareCollective(d['calibration']) for d in config.drones]
    model=PlateFlightModel(config.physical,config.models)
    from experiments.sanity_plate_transport.codegen import compile_model
    observer=compile_model(model,root/'.cache/sanity_plate_observer',names=('dynamics','continuous_jacobian','geometry','length'))
    reference=FlightReference(model);reference.prepare()
    worker=PlateWorker(config,output,root/'.venv-sanity-passage/bin/python');results=[]
    try:
        for when in [0.,5.,5.+config.physical.path_duration_s/4,5.+config.physical.path_duration_s/2,config.physical.duration_s]:
            state,previous=reference.sample(when)
            request=dict(reset=True,state=state.tolist(),segments=[],previous=previous.tolist(),offset=[0.,0.,0.],yaw=0.,reference_time_s=when)
            started=time.monotonic();future,sequence=worker.submit(request);reply=future.result(timeout=3.)
            results.append(dict(time_s=when,reply=reply,roundtrip_ms=(time.monotonic()-started)*1000))
        passed=all('error' not in r['reply'] and r['reply']['mpc']['accepted'] and r['reply'].get('command_space')=='world_force' and np.asarray(r['reply']['command']).shape==(3*config.physical.drones,) and np.isfinite(r['reply']['command']).all() for r in results)
        report=dict(passed=passed,transport_opened=False,hardware_access=False,hardware_calibration_deferred=True,
            geometry_confirmed=config.data['geometry_confirmed'],frames_confirmed=config.mocap['frames_confirmed'],
            calibration_ids=[m.calibration_id for m in mappings],mission_profile=config.mission_profile,
            solver=worker.info,observer=observer,queries=results,scope='Offline software checks only. Calibration and physical flight commissioning remain operator tasks.')
        (output/'preflight.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
        if not passed:raise SystemExit(1)
    finally:worker.close()


if __name__=='__main__':main()
