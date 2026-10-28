#!/usr/bin/env python3
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import select
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from bifilar_inertia import fit_oscillation

WORKSPACE = Path(__file__).resolve().parents[4]
COLUMNS = ['host_monotonic_s', 'host_unix_s', 'stamp_sec', 'stamp_nanosec', 'phase',
           'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw']


def parameters(args):
    cable_mass = args.cable_mass_g/1000
    payload_mass = args.payload_mass_g/1000
    length = args.length_mm/1000
    side = args.cube_side_mm/1000
    hook_height = args.hook_height_mm/1000
    if not np.isfinite([cable_mass, payload_mass, length, side, hook_height]).all() or min(cable_mass, payload_mass, length, side) <= 0 or hook_height < side/2:
        raise ValueError('Positive finite masses/lengths and a hook above the nominal cube COM required')
    height = hook_height-side/2
    distance = (cable_mass*length/2+payload_mass*(length+height))/(cable_mass+payload_mass)
    cube_inertia = payload_mass*side**2/6
    pivot_inertia = cable_mass*length**2/3+payload_mass*(length+height)**2+cube_inertia
    first_moment = cable_mass*length/2+payload_mass*(length+height)
    return {'tracked_frame': args.body.lstrip('/'), 'parent_frame': args.parent.lstrip('/'), 'topic': args.topic,
            'cable_mass_kg': cable_mass, 'payload_with_markers_mass_kg': payload_mass,
            'total_suspended_mass_kg': cable_mass+payload_mass, 'cable_length_pivot_to_hook_m': length,
            'cube_side_m': side, 'hook_above_cube_base_m': hook_height,
            'nominal_hook_to_cube_center_m': height, 'nominal_pivot_to_payload_com_m': length+height,
            'nominal_pivot_to_assembly_com_m': distance, 'nominal_assembly_pivot_inertia_kg_m2': pivot_inertia,
            'nominal_equivalent_period_s': 2*np.pi*np.sqrt(pivot_inertia/(9.81*first_moment)),
            'origin_to_cube_center_body_m': (np.asarray(args.origin_to_center_mm)/1000).tolist(),
            'origin_offset_status': 'User reports origin near cube center; zero offset is provisional, not measured',
            'payload_com_assumption': 'At geometric cube center; marker/hook mass distribution not independently measured',
            'cube_inertia_assumption': 'Uniform solid-cube proxy, not identified inertia of the printed part',
            'equivalent_model_assumption': 'Straight cable and aligned payload for nominal moments/period; actual cube rotation is recorded',
            'aircraft_attachment_description': 'Underside of battery',
            'aircraft_attachment_body_m': None,
            'aircraft_attachment_status': 'Not measured; do not reuse the old simulated attachment offset',
            'axis': args.axis, 'coordinate_assumption': 'Parent frame Z points upward',
            'initial_rest_s': args.rest_seconds, 'swing_duration_s': args.duration,
            'pose_only': True, 'control_publishers_started': False,
            'recorder_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'fit_dependency_sha256': hashlib.sha256(Path(__file__).with_name('bifilar_inertia.py').read_bytes()).hexdigest()}


def record(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from tf2_msgs.msg import TFMessage

    if not sys.stdin.isatty():
        raise ValueError('Run recording in an interactive terminal for the rest and release prompts')
    if not 10 <= args.duration <= 120 or not 2 <= args.rest_seconds <= 20 or not 1 <= args.countdown <= 30:
        raise ValueError('Duration10..120s, initial rest2..20s, countdown1..30s required')
    cfg = parameters(args)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
    directory = (args.output or WORKSPACE/f'outputs/payload_pendulum_{stamp}_{args.axis}').resolve()
    directory.mkdir(parents=True, exist_ok=False)
    (directory/'parameters.json').write_text(json.dumps(cfg, indent=2)+'\n')
    state = {'count': 0, 'dropped_stamps': 0, 'invalid': 0, 'last_stamp': None,
             'last_receive': -math.inf, 'phase': 'waiting', 'release': None}
    stream = (directory/'poses.csv').open('x', newline='')
    writer = csv.writer(stream)
    writer.writerow(COLUMNS)
    events = (directory/'events.jsonl').open('x')
    rclpy.init()
    node = rclpy.create_node('sanity_payload_pendulum_recorder')

    def event(kind):
        events.write(json.dumps({'kind': kind, 'host_monotonic_s': time.monotonic(), 'host_unix_s': time.time(), 'last_source_stamp_ns': state['last_stamp']})+'\n')
        events.flush()

    def receive(message):
        for transform in message.transforms:
            if transform.header.frame_id.lstrip('/') != cfg['parent_frame'] or transform.child_frame_id.lstrip('/') != cfg['tracked_frame']:
                continue
            source_stamp = transform.header.stamp.sec*1_000_000_000+transform.header.stamp.nanosec
            if state['last_stamp'] is not None and source_stamp <= state['last_stamp']:
                state['dropped_stamps'] += 1
                continue
            q, p = transform.transform.rotation, transform.transform.translation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if not np.isfinite(values).all() or not .9 < np.linalg.norm(values[3:]) < 1.1:
                state['invalid'] += 1
                continue
            now = time.monotonic()
            writer.writerow([now, time.time(), transform.header.stamp.sec, transform.header.stamp.nanosec, state['phase'], *values])
            state.update(count=state['count']+1, last_stamp=source_stamp, last_receive=now)

    subscription = node.create_subscription(TFMessage, cfg['topic'], receive,
                    QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))

    def spin_for(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if time.monotonic()-state['last_receive'] > 2:
                raise RuntimeError('Tracking absent for two seconds; recorded data are retained')

    def wait_for_enter(prompt):
        print(prompt, flush=True)
        while True:
            rclpy.spin_once(node, timeout_sec=.02)
            if time.monotonic()-state['last_receive'] > 2:
                raise RuntimeError('Tracking lost while waiting for the operator')
            if select.select([sys.stdin], [], [], 0.)[0]:
                if sys.stdin.readline() == '':
                    raise RuntimeError('Terminal input closed')
                break

    error, interrupted, complete = None, False, False
    try:
        print(f'Output: {directory}\nWaiting for {cfg["parent_frame"]} -> {cfg["tracked_frame"]} on {cfg["topic"]}...', flush=True)
        deadline = time.monotonic()+15
        while state['count'] < 20 or time.monotonic()-state['last_receive'] > .2:
            if time.monotonic() > deadline:
                raise RuntimeError('No fresh matching TF within15seconds; check body/parent names and OptiTrack/DDS')
            rclpy.spin_once(node, timeout_sec=.05)
        wait_for_enter('Keep the mass hanging STILL. Press Enter to record the initial rest interval.')
        state['phase'] = 'rest'
        event('rest_start')
        spin_for(args.rest_seconds)
        event('rest_end')
        state['phase'] = 'positioning'
        wait_for_enter(f'Displace the mass about5–10degrees along {args.axis.upper()} and hold it. Press Enter when ready for the release countdown.')
        for remaining in range(math.ceil(args.countdown), 0, -1):
            print(f'  {remaining}...', flush=True)
            spin_for(min(1., args.countdown))
        state['phase'] = 'swing'
        state['release'] = time.monotonic()
        event('release_cue')
        print('\aRELEASE NOW without pushing; leave the support and mass untouched.', flush=True)
        next_notice = state['release']+5
        while time.monotonic()-state['release'] < args.duration:
            spin_for(.05)
            if time.monotonic() >= next_notice:
                stream.flush()
                print(f'  {time.monotonic()-state["release"]:.0f}/{args.duration:g}s recorded', flush=True)
                next_notice += 5
        event('recording_complete')
        complete = True
        print('\aCapture complete.', flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
        print('\nStopped; preserving the available recording.', flush=True)
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        stream.flush()
        stream.close()
        events.close()
        node.destroy_node()
        rclpy.try_shutdown()
        receipt = {'generated_utc': datetime.now(timezone.utc).isoformat(), 'complete': complete,
                   'interrupted': interrupted, 'error': error, 'samples': state['count'],
                   'duplicate_or_reversed_stamps_dropped': state['dropped_stamps'], 'invalid_poses_dropped': state['invalid'],
                   'release_cue_monotonic_s': state['release'], 'pose_only': True, 'control_publishers_started': False,
                   'filtering': 'TF values unchanged by this recorder; upstream filtering is not queried',
                   'release_timing': 'Operator audio/terminal cue, not a measured physical release instant'}
        (directory/'capture_receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
        print(f'Saved: {directory}', flush=True)
    if error:
        raise RuntimeError(error)
    if not complete:
        raise RuntimeError('Capture incomplete; inspect capture_receipt.json before analysis')
    print(f'Analyze with: {Path(__file__).with_name("run_payload_pendulum.sh")} analyze {directory}', flush=True)


def analyze(args):
    directory = args.directory.resolve()
    cfg = json.loads((directory/'parameters.json').read_text())
    receipt = json.loads((directory/'capture_receipt.json').read_text())
    if receipt['release_cue_monotonic_s'] is None:
        raise ValueError('No release cue was recorded')
    with (directory/'poses.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 100:
        raise ValueError('Fewer than100 recorded poses')
    host = np.array([float(r['host_monotonic_s']) for r in rows])
    stamps = np.array([int(r['stamp_sec'])*1_000_000_000+int(r['stamp_nanosec']) for r in rows], dtype=np.int64)
    positions = np.array([[float(r[k]) for k in ('x_m', 'y_m', 'z_m')] for r in rows])
    quaternions = np.array([[float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')] for r in rows])
    phases = np.array([r['phase'] for r in rows])
    if np.any(np.diff(stamps) <= 0) or not np.isfinite(positions).all() or not np.isfinite(quaternions).all():
        raise ValueError('Finite raw poses with strictly increasing source timestamps required')
    rest = phases == 'rest'
    if rest.sum() < 20 or (stamps[rest][-1]-stamps[rest][0])*1e-9 < 1.5:
        raise ValueError('Need at least1.5seconds of recorded initial rest to infer a fixed pivot')
    rotations = Rotation.from_quat(quaternions)
    offset = np.asarray(cfg['origin_to_cube_center_body_m']) if args.origin_to_center_mm is None else np.asarray(args.origin_to_center_mm)/1000
    centers = positions+rotations.apply(offset)
    initial_center = centers[rest].mean(axis=0)
    initial_rotation = rotations[rest].mean()
    height = cfg['nominal_hook_to_cube_center_m']
    length = cfg['cable_length_pivot_to_hook_m']
    hook_body = offset+initial_rotation.inv().apply([0., 0., height])
    hooks = positions+rotations.apply(hook_body)
    pivot = initial_center+np.array([0., 0., length+height])
    separation = hooks-pivot
    measured_length = np.linalg.norm(separation, axis=1)
    direction = separation/measured_length[:, None]
    angle_x = np.arctan2(direction[:, 0], -direction[:, 2])
    angle_y = np.arctan2(direction[:, 1], -direction[:, 2])
    elapsed = host-receipt['release_cue_monotonic_s']
    select_fit = (phases == 'swing') & (elapsed >= args.start_after_release)
    if args.end_after_release is not None:
        select_fit &= elapsed <= args.end_after_release
    if select_fit.sum() < 100:
        raise ValueError('Insufficient samples in the requested swing fit window')
    axis = args.axis or cfg['axis']
    if axis == 'auto':
        horizontal = direction[select_fit, :2]
        values, vectors = np.linalg.eigh(np.cov(horizontal.T))
        plane = vectors[:, np.argmax(values)]
        if plane[np.argmax(abs(plane))] < 0:
            plane *= -1
    else:
        plane = np.array([1., 0.]) if axis == 'x' else np.array([0., 1.])
    angle = np.arctan2(direction[:, :2]@plane, -direction[:, 2])
    fit_stamp = (stamps[select_fit]-stamps[select_fit][0])*1e-9
    fit, grid, measured, predicted = fit_oscillation(fit_stamp, angle[select_fit], .2, 2.5)
    omega_n = np.sqrt((2*np.pi*fit['frequency_damped_hz'])**2+fit['decay_per_s']**2)
    cube_position = centers
    cable_com = (pivot+hooks)/2
    mc, mp = cfg['cable_mass_kg'], cfg['payload_with_markers_mass_kg']
    assembly_com = (mc*cable_com+mp*cube_position)/(mc+mp)
    warnings = list(fit['warnings'])
    if not receipt['complete']:
        warnings.append('Capture incomplete; fit uses only available data')
    rest_spread = float(np.linalg.norm(centers[rest]-initial_center, axis=1).max())
    length_rms = float(np.sqrt(np.mean((measured_length[select_fit]-length)**2)))
    if rest_spread > .005:
        warnings.append('Initial rest varies by more than5mm; review the inferred fixed pivot')
    if length_rms > .005:
        warnings.append('Hook-to-pivot length residual exceeds5mm; review origin/hook offsets, support motion and tracking')
    failures = list(fit['failures'])
    natural = float(omega_n)
    result = {'status': 'accepted_single_mode_fit' if not failures else 'review_required', 'fit': fit,
              'natural_frequency_hz': natural/(2*np.pi), 'natural_period_s': 2*np.pi/natural,
              'damping_ratio': fit['decay_per_s']/natural,
              'apparent_single_mode_damping_Nms': 2*cfg['nominal_assembly_pivot_inertia_kg_m2']*fit['decay_per_s'],
              'damping_scope': 'Conditional equivalent viscous damping using provisional rigidly-aligned cube/cable inertia; not an automatic MPC calibration',
              'fixed_pivot_world_m': pivot.tolist(), 'origin_to_hook_body_m': hook_body.tolist(),
              'origin_to_cube_center_body_m': offset.tolist(), 'origin_offset_calibrated': False,
              'rest_max_position_deviation_m': rest_spread, 'hook_to_pivot_length_residual_rms_m': length_rms,
              'fit_plane_horizontal_unit': plane.tolist(), 'fit_window_after_cue_s': [float(elapsed[select_fit][0]), float(elapsed[select_fit][-1])],
              'source_timestamp_sample_rate_hz': float(1e9/np.median(np.diff(stamps[select_fit]))),
              'payload_mocap_frame': cfg['tracked_frame'], 'parameters': cfg, 'warnings': warnings, 'failures': failures,
              'assumptions': ['Fixed support; pivot inferred once from initial rest', 'Cube COM nominally at geometric center',
                              'Hook initially vertically above cube center; rest pose establishes its rigid-body lever arm',
                              'Cable COM estimated as midpoint of a straight cable', 'No flight attachment offset is inferred from this fixed-support test'],
              'raw_poses_sha256': hashlib.sha256((directory/'poses.csv').read_bytes()).hexdigest(),
              'analyzer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    output = (args.output or directory/'analysis').resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    with (output/'derived.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['seconds_after_release_cue', 'cable_angle_x_deg', 'cable_angle_y_deg', 'dominant_angle_deg', 'length_residual_mm',
                         'payload_com_x_m', 'payload_com_y_m', 'payload_com_z_m', 'assembly_com_x_m', 'assembly_com_y_m', 'assembly_com_z_m'])
        for i in range(len(rows)):
            writer.writerow([elapsed[i], np.degrees(angle_x[i]), np.degrees(angle_y[i]), np.degrees(angle[i]), 1000*(measured_length[i]-length), *cube_position[i], *assembly_com[i]])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(12., 10.))
    axes[0, 0].plot(elapsed, np.degrees(angle_x), label='X swing')
    axes[0, 0].plot(elapsed, np.degrees(angle_y), label='Y swing')
    axes[0, 0].set(xlabel='Time after release cue [s]', ylabel='Cable angle [deg]')
    axes[0, 0].legend()
    fit_time = elapsed[select_fit][0]+grid
    axes[0, 1].plot(fit_time, np.degrees(measured), label='Measured')
    axes[0, 1].plot(fit_time, np.degrees(predicted), '--', label='Damped-mode fit')
    axes[0, 1].set(xlabel='Time after release cue [s]', ylabel='Fit angle [deg]')
    axes[0, 1].legend()
    axes[1, 0].plot(cube_position[select_fit, 0]-pivot[0], cube_position[select_fit, 2]-pivot[2])
    axes[1, 0].set(xlabel='Payload COM X relative to pivot [m]', ylabel='Z relative to pivot [m]')
    axes[1, 0].set_aspect('equal', adjustable='datalim')
    axes[1, 1].plot(elapsed, 1000*(measured_length-length))
    axes[1, 1].set(xlabel='Time after release cue [s]', ylabel='Hook-to-pivot length residual [mm]')
    axes[2, 0].plot(fit_time, np.degrees(measured-predicted))
    axes[2, 0].set(xlabel='Time after release cue [s]', ylabel='Fit residual [deg]')
    for k, name in enumerate(('X', 'Y', 'Z')):
        axes[2, 1].plot(elapsed, cube_position[:, k]-initial_center[k], label=name)
    axes[2, 1].set(xlabel='Time after release cue [s]', ylabel='Payload COM displacement [m]')
    axes[2, 1].legend()
    for ax in axes.flat:
        ax.grid(linestyle=':', alpha=.6)
    fig.suptitle(f"{result['status']} | T={fit['period_damped_s']:.3f}s | decay={fit['decay_per_s']:.4f}/s\n{cfg['tracked_frame']}; nominal origin/COM geometry; no controller parameters installed")
    fig.tight_layout()
    fig.savefig(output/'pendulum_review.pdf')
    fig.savefig(output/'pendulum_review.png', dpi=160)
    plt.close(fig)
    print(f"{result['status']}: period={fit['period_damped_s']:.4f}s, decay={fit['decay_per_s']:.5f}/s")
    print(f'Fixed pivot: {pivot.tolist()} m\nOutputs: {output}')
    for warning in failures+warnings:
        print('Review: '+warning)
    return 0 if not failures else 2


def main():
    parser = argparse.ArgumentParser(description='Pose-only fixed-support suspended-payload recording and offline analysis.')
    modes = parser.add_subparsers(dest='mode', required=True)
    capture = modes.add_parser('record')
    capture.add_argument('--body', default='45g_mass')
    capture.add_argument('--parent', default='map')
    capture.add_argument('--topic', default='/tf')
    capture.add_argument('--axis', choices=('x', 'y'), required=True)
    capture.add_argument('--output', type=Path)
    capture.add_argument('--duration', type=float, default=20.)
    capture.add_argument('--rest-seconds', type=float, default=3.)
    capture.add_argument('--countdown', type=float, default=10.)
    capture.add_argument('--length-mm', type=float, default=525.)
    capture.add_argument('--cable-mass-g', type=float, default=1.34)
    capture.add_argument('--payload-mass-g', type=float, default=44.73)
    capture.add_argument('--cube-side-mm', type=float, default=45.)
    capture.add_argument('--hook-height-mm', type=float, default=55.)
    capture.add_argument('--origin-to-center-mm', type=float, nargs=3, default=(0., 0., 0.))
    offline = modes.add_parser('analyze')
    offline.add_argument('directory', type=Path)
    offline.add_argument('--output', type=Path)
    offline.add_argument('--axis', choices=('x', 'y', 'auto'))
    offline.add_argument('--origin-to-center-mm', type=float, nargs=3)
    offline.add_argument('--start-after-release', type=float, default=.5)
    offline.add_argument('--end-after-release', type=float)
    args = parser.parse_args()
    if args.mode == 'record':
        record(args)
        return 0
    return analyze(args)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, OSError) as error:
        print(f'Error: {error}', file=sys.stderr)
        raise SystemExit(1)
