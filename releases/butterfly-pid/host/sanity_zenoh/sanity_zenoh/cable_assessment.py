import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def assess_tracking(output, mission):
    output=Path(output)
    controls=[json.loads(line) for line in (output/'controller/control.jsonl').read_text().splitlines()]
    truth=[json.loads(line) for line in (output/'plant.jsonl').read_text().splitlines()]
    commands=[json.loads(line) for line in (output/'controller/commands.jsonl').read_text().splitlines()]
    if not controls or not truth or not commands:return {'passed':False,'reason':'Missing control, truth or actual-command log'}
    stamps=np.array([r['stamp_ns'] for r in truth],dtype=np.int64)
    vehicle=np.array([r['aircraft_position'] for r in truth]);payload=np.array([r['payload_position'] for r in truth])
    forces=np.array([r['truth']['force'] for r in truth])
    command_stamps=np.array([r['stamp_ns'] for r in commands],dtype=np.int64)
    selected=[];causality_errors=[];force_errors=[]
    for row in controls:
        entry=row.get('mpc_input')
        if entry:
            capture=int(entry.get('history_capture_ns',row.get('unix_ns',row['stamp_ns']) if abs(row.get('unix_ns',row['stamp_ns'])-row['stamp_ns'])<1_000_000_000 else row['stamp_ns']))
            index=int(np.searchsorted(command_stamps,capture,side='right')-1)
            if capture<row['stamp_ns']:causality_errors.append(row['stamp_ns'])
            if index<0 or not np.allclose(entry['history_last_command'],commands[index]['mapped'],rtol=0.,atol=1e-12):
                causality_errors.append(row['stamp_ns'])
            if entry['history_last_apply_ns']>entry['apply_ns']:causality_errors.append(row['stamp_ns'])
        t=row.get('mission_time_s')
        if row['phase']=='mission' and t is not None and 12.<=t<=26. and entry and entry['apply_ns']<=stamps[-1]:selected.append(row)
        if row.get('state') is not None:
            measured=np.interp(row['source_stamp_ns'],stamps,forces)
            force_errors.append(row['state'][13]-measured)
    report={'passed':False,'steady_samples':len(selected),'history_causality_errors':len(set(causality_errors)),
            'actuator_force_estimate_rmse_N':float(np.sqrt(np.mean(np.square(force_errors)))) if force_errors else None,
            'scope':'Independent native plant truth used only after the run; control receives dual TF and actual command history'}
    profile_path=output/'controller/profile.json'
    profile=json.loads(profile_path.read_text()) if profile_path.exists() else {}
    if profile.get('takeoff_land_only',False):
        held=[r for r in controls if r['phase']=='hover']
        if not held:return {**report,'reason':'No PID hover phase recorded'}
        held_stamps=np.array([r['stamp_ns'] for r in held],dtype=np.int64)
        actual=np.column_stack([np.interp(held_stamps,stamps,vehicle[:,i]) for i in range(3)])
        goal=np.array([r['reference']['position'] for r in held])
        error=np.linalg.norm(actual-goal,axis=1)
        duration=(held_stamps[-1]-held_stamps[0])/1e9
        no_mpc=all(r.get('mpc_input') is None and r['mpc_weight']==0. for r in controls)
        report.update(hover_duration_s=duration,hover_position_rmse_m=float(np.sqrt(np.mean(error**2))),hover_position_max_error_m=float(error.max()),pid_only=no_mpc)
        report['passed']=bool(duration>=profile.get('hover_duration_s',5.)-.1 and report['hover_position_rmse_m']<.08 and error.max()<.20 and no_mpc and not causality_errors)
        return report
    if profile.get('mpc_hover_only',False):
        held=[r for r in controls if r['phase']=='mpc_hover']
        if not held:return {**report,'reason':'No MPC hover phase recorded'}
        held_stamps=np.array([r['stamp_ns'] for r in held],dtype=np.int64)
        actual=np.column_stack([np.interp(held_stamps,stamps,vehicle[:,i]) for i in range(3)])
        goal=np.array([r['reference']['position'] for r in held])
        error=np.linalg.norm(actual-goal,axis=1)
        duration=(held_stamps[-1]-held_stamps[0])/1e9
        receipt=json.loads((output/'controller/receipt.json').read_text())
        qualified=any(e['phase']=='mpc_hover_qualified' for e in receipt['events'])
        report.update(hover_duration_s=duration,hover_position_rmse_m=float(np.sqrt(np.mean(error**2))),hover_position_max_error_m=float(error.max()),mpc_hover_qualified=qualified)
        report['passed']=bool(qualified and duration>=4.9 and report['hover_position_rmse_m']<.08 and error.max()<.20 and not causality_errors)
        return report
    if len(selected)<100:return {**report,'reason':'Insufficient steady mission data'}
    target_stamps=np.array([r['mpc_input']['apply_ns'] for r in selected],dtype=np.int64)
    actual=np.column_stack([np.interp(target_stamps,stamps,vehicle[:,i]) for i in range(3)])
    load=np.column_stack([np.interp(target_stamps,stamps,payload[:,i]) for i in range(3)])
    preview=np.array([r['mpc_input']['preview_first'] for r in selected]);goal=preview[:,:3]
    body_error=np.linalg.norm(actual-goal,axis=1)
    report.update(aircraft_position_rmse_m=float(np.sqrt(np.mean(body_error**2))),aircraft_position_max_error_m=float(body_error.max()))
    if mission=='payload_orbit':
        physical=json.loads((output/'physical_config.json').read_text())
        cube_goal=goal+Rotation.from_quat(preview[:,[4,5,6,3]]).apply(physical['attachment_body_m'])+(physical['cable_length_m']+physical['payload_com_offset_below_hook_m'])*preview[:,24:27]
        if preview.shape[1]==41:cube_goal=goal+Rotation.from_quat(preview[:,[4,5,6,3]]).apply(physical['attachment_body_m'])+physical['cable_length_m']*preview[:,24:27]-Rotation.from_quat(preview[:,[31,32,33,30]]).apply([0.,0.,physical['payload_com_offset_below_hook_m']])
        error=np.linalg.norm(load-cube_goal,axis=1)
        center=np.asarray(controls[0]['vehicle'][:2])
        radius=np.linalg.norm(load[:,:2]-center,axis=1)
        body_radius=np.linalg.norm(actual[:,:2]-center,axis=1)
        report.update(payload_position_rmse_m=float(np.sqrt(np.mean(error**2))),payload_world_radius_mean_m=float(radius.mean()),
                      payload_world_radius_std_m=float(radius.std()),aircraft_xy_max_from_home_m=float(body_radius.max()))
        report['thresholds']={'payload_position_rmse_m':.05,'payload_world_radius_mean_m':[.10,.20],'aircraft_xy_max_from_home_m':.05}
        tracking_ok=report['payload_position_rmse_m']<.05 and .10<radius.mean()<.20 and body_radius.max()<.05
    else:
        limit=.05 if mission=='circle_slow' else .08
        report['thresholds']={'aircraft_position_rmse_m':limit,'post_stop_swing_rms_deg':5.}
        stopped=[r['swing_deg'] for r in controls if r['phase'] in ('mission','settle') and r.get('mission_time_s') is not None and 32.<=r['mission_time_s']<=38. and r.get('swing_deg') is not None]
        report['post_stop_swing_rms_deg']=float(np.sqrt(np.mean(np.square(stopped)))) if stopped else None
        tracking_ok=report['aircraft_position_rmse_m']<limit and bool(stopped) and report['post_stop_swing_rms_deg']<5.
        cube_rows=[r for r in truth if r.get('phase') in ('mission','settle') and 32.<=r.get('mission_t',-1.)<=38. and 'payload_xyzw' in r and 'payload_world_omega' in r['truth']]
        if cube_rows:
            axis=Rotation.from_quat([r['payload_xyzw'] for r in cube_rows]).apply([0.,0.,1.])
            delta=np.array([np.asarray(r['payload_position'])-r['aircraft_position'] for r in cube_rows]);direction=delta/np.linalg.norm(delta,axis=1,keepdims=True)
            angles=np.degrees(np.arccos(np.clip(np.sum(axis*(-direction),axis=1),-1.,1.)))
            omega=np.array([r['truth']['payload_world_omega'] for r in cube_rows]);omega-=axis*np.sum(omega*axis,axis=1,keepdims=True)
            report.update(post_stop_cube_internal_tilt_rms_deg=float(np.sqrt(np.mean(angles**2))),post_stop_cube_transverse_omega_rms_rad_s=float(np.sqrt(np.mean(np.sum(omega**2,axis=1)))))
            report['thresholds'].update(post_stop_cube_internal_tilt_rms_deg=10.,post_stop_cube_transverse_omega_rms_rad_s=4.)
            tracking_ok=tracking_ok and report['post_stop_cube_internal_tilt_rms_deg']<10. and report['post_stop_cube_transverse_omega_rms_rad_s']<4.

    report['passed']=bool(tracking_ok and not causality_errors)
    return report
