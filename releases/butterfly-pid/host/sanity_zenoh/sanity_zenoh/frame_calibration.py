"""Offline validation of configured mocap/body transforms against known poses.

This evaluates an existing transform. It does not fit or apply a transform,
query ROS, connect to aircraft, or grant flight authority. Reference poses must
be independently established; matching two mocap-derived quantities is not an
independent calibration. Input and configuration hashes are retained.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from sanity_zenoh.mocap import MocapEstimator


def _vector(value, size, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f'{name} requires {size} finite values')
    return a


def evaluate(config, observations, *, position_tolerance_m=.005,
             attitude_tolerance_deg=.5, minimum_orientation_span_deg=10.):
    """Compare all known poses using exactly the online rigid-transform law.

    Version1 observations: independent_reference description plus samples with
    marker_position_m/marker_quat_xyzw (map frame), and reference_body_position_m/
    reference_body_quat_xyzw (configured world frame). The same configured body
    point must be surveyed; this does not establish the aircraft CoM location.
    """
    limits = (position_tolerance_m, attitude_tolerance_deg, minimum_orientation_span_deg)
    if not all(np.isfinite(v) and v > 0 for v in limits):
        raise ValueError('Positive finite calibration thresholds required')
    if observations.get('version') != 1:
        raise ValueError('Known-pose observations version1 required')
    independent = observations.get('independent_reference')
    if not isinstance(independent, dict) or not all(
            isinstance(independent.get(k), str) and independent[k].strip()
            for k in ('method', 'description')):
        raise ValueError('Describe the independent reference method and description')
    rows = observations.get('samples')
    if not isinstance(rows, list) or len(rows) < 3:
        raise ValueError('At least three independently known poses required')
    mocap = config['mocap']
    world = MocapEstimator.rotation(mocap['world_from_map_xyzw'])
    marker_to_body = MocapEstimator.rotation(mocap['marker_to_base_xyzw'])
    offset = _vector(mocap['marker_to_base_xyz'], 3, 'Marker-to-body offset')
    references = [];results = []
    for i, row in enumerate(rows):
        marker = MocapEstimator.rotation(row['marker_quat_xyzw'])
        raw = _vector(row['marker_position_m'], 3, 'Marker position')
        reference_position = _vector(row['reference_body_position_m'], 3, 'Known body position')
        reference_rotation = MocapEstimator.rotation(row['reference_body_quat_xyzw'])
        predicted_rotation = world * marker * marker_to_body
        predicted_position = world.apply(raw + marker.apply(offset))
        position_error = predicted_position - reference_position
        rotation_error = (reference_rotation.inv() * predicted_rotation).as_rotvec()
        references.append(reference_rotation)
        results.append({'index': i, 'label': row.get('label', str(i)),
                        'position_error_m': position_error.tolist(),
                        'position_error_norm_m': float(np.linalg.norm(position_error)),
                        'attitude_error_rotvec_rad': rotation_error.tolist(),
                        'attitude_error_deg': float(np.rad2deg(np.linalg.norm(rotation_error)))})
    span = max(float(np.rad2deg((a.inv() * b).magnitude()))
               for a in references for b in references)
    enough_span = span >= minimum_orientation_span_deg
    within_limits = all(r['position_error_norm_m'] <= position_tolerance_m and
                        r['attitude_error_deg'] <= attitude_tolerance_deg for r in results)
    return {'version': 1, 'purpose': 'Offline configured-transform assessment',
            'passed': bool(enough_span and within_limits), 'flight_qualification': False,
            'independent_reference': independent,
            'reference_independence_verified_by_software': False,
            'transform_applied_or_fitted': False,
            'convention': 'R_world_body=R_world_map*R_map_marker*R_marker_body; '
                          'p_world_body=R_world_map*(p_map_marker+R_map_marker*r_marker_body)',
            'thresholds': {'position_m': position_tolerance_m, 'attitude_deg': attitude_tolerance_deg,
                           'minimum_orientation_span_deg': minimum_orientation_span_deg},
            'configured_transform': {k: mocap[k] for k in (
                'parent_frame', 'tracked_frame', 'world_from_map_xyzw',
                'marker_to_base_xyz', 'marker_to_base_xyzw')},
            'reference_orientation_span_deg': span, 'sufficient_pose_diversity': enough_span,
            'within_pose_error_limits': within_limits, 'samples': results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--position-tolerance-m', type=float, default=.005)
    parser.add_argument('--attitude-tolerance-deg', type=float, default=.5)
    parser.add_argument('--minimum-orientation-span-deg', type=float, default=10.)
    args = parser.parse_args()
    result = evaluate(json.loads(args.config.read_text()), json.loads(args.poses.read_text()),
                      position_tolerance_m=args.position_tolerance_m,
                      attitude_tolerance_deg=args.attitude_tolerance_deg,
                      minimum_orientation_span_deg=args.minimum_orientation_span_deg)
    result['inputs'] = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in (args.config, args.poses)}
    with args.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False);f.write('\n')
    print(json.dumps({k:result[k] for k in ('passed','sufficient_pose_diversity','within_pose_error_limits')}))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
