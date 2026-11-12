import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from sanity_zenoh.cable_state import CableMocap


def audit(recording):
    recording=Path(recording)
    physical=json.loads((recording/'config.json').read_text())
    profile=json.loads((recording/'profile.json').read_text())
    calibration={**profile['mocap'],'payload_estimation':'full_pose','cable_mode_filter':False}
    estimator=CableMocap(physical,calibration)
    controls=[json.loads(line) for line in (recording/'control.jsonl').read_text().splitlines()]
    stamps=np.array([r['stamp_ns'] for r in controls],dtype=np.int64)
    distances=[];seen=set();rejected=0
    with (recording/'tf.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            index=int(np.searchsorted(stamps,row['stamp_ns'],side='right')-1)
            if index<0:continue
            try:
                estimator.update(row['parent'],row['child'],row['stamp_ns'],row['position'],row['quaternion_xyzw'],row['arrival_ns'])
                packet=estimator.sample(row['arrival_ns'])
            except (ValueError,TypeError):
                rejected+=1;continue
            if packet['stamp_ns'] in seen:continue
            seen.add(packet['stamp_ns'])
            if controls[index]['phase'] not in ('acquire','hover','blend','mpc_hover','mission','settle'):continue
            vehicle=packet['vehicle'];payload=packet['payload']
            rotation=Rotation.from_quat(vehicle[[4,5,6,3]])
            if np.linalg.norm(vehicle[7:10])>.15 or np.linalg.norm(payload[7:10])>.15 or rotation.as_matrix()[2,2]<np.cos(np.radians(10.)):continue
            distances.append(packet['hook_distance_m'])
    residual=np.array(distances)-physical['cable_length_m']
    report={'scope':'Offline geometry consistency from slow airborne samples; no outputs or calibration changes',
            'recording':str(recording.resolve()),'configured_cable_length_m':physical['cable_length_m'],'samples':len(distances),'rejected_or_unpaired_samples':rejected,
            'attachment_body_m':physical['attachment_body_m'],'aircraft_marker_to_body_xyz':calibration['aircraft_marker_to_body_xyz'],'payload_marker_to_body_xyz':calibration['payload_marker_to_body_xyz'],
            'consistent':False,'note':'A single hover cannot identify separate 3D marker-to-hook and marker-to-attachment offsets. Measure those offsets if residuals fail; do not compensate by changing measured cable length.'}
    if len(distances)<100:report['reason']='Need at least100 slow airborne paired samples'
    else:
        report.update(hook_distance_median_m=float(np.median(distances)),length_residual_median_mm=float(1000*np.median(residual)),length_residual_p05_p95_mm=(1000*np.percentile(residual,[5,95])).tolist(),length_residual_abs_p95_mm=float(1000*np.percentile(abs(residual),95)))
        report['consistent']=bool(np.percentile(abs(residual),95)<.012)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='Audit calibrated cable geometry from an existing flight recording; no ROS or aircraft connection.')
    parser.add_argument('--recording',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    result=audit(args.recording)
    text=json.dumps(result,indent=2,allow_nan=False)+'\n'
    if args.output:args.output.write_text(text)
    print(text,end='')
    raise SystemExit(0 if result['consistent'] else 1)
