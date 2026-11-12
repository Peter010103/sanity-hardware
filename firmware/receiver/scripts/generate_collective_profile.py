import hashlib
import json
from pathlib import Path
from collections import Counter

import numpy as np
from scipy.spatial import ConvexHull


if __name__ == '__main__':
    receiver=Path(__file__).resolve().parents[1]
    workspace=receiver.parents[3]
    rows=[];sources={}
    for cohort in ['sanity_regulated_id_20261020','sanity_fullrange_id_20261020']:
        path=workspace/'outputs'/cohort/'results.json'
        sources[str(path.relative_to(workspace))]=hashlib.sha256(path.read_bytes()).hexdigest()
        rows += [dict(cohort=cohort,**r) for r in json.loads(path.read_text())['plateaus']
                 if r['role']=='acro_collective' and r['control_mode']=='acro' and r['compensation']==100 and r['steady_motor_speed']]
    u=np.array([r['input_counts'][0] for r in rows]);voltage=np.array([r['FC_voltage_V'] for r in rows])
    source_voltage=np.array([r['source_setpoint_V'] for r in rows]);force=np.array([r['thrust_N'] for r in rows])
    w2=np.array([r['sum_omega_squared'] for r in rows]);omega=np.sqrt(w2/4)
    count=Counter(zip(source_voltage,u));weights=np.array([count[k]**-.5 for k in zip(source_voltage,u)])
    q=(u-1500)/400;z=(voltage-7.4)/.8
    points=np.column_stack([u,voltage]);hull=ConvexHull(points)
    grid=np.array([[x,V] for x in np.linspace(u.min(),u.max(),161) for V in np.linspace(voltage.min(),voltage.max(),101)])
    inside=np.all(grid@hull.equations[:,:2].T+hull.equations[:,2]<=1e-8,axis=1)
    grid=grid[inside];gq=(grid[:,0]-1500)/400;gz=(grid[:,1]-7.4)/.8
    candidates=[]
    k=np.sum(weights**2*w2*force)/np.sum(weights**2*w2**2)
    for degree in [2,3,4]:
        powers=[(n-j,j) for n in range(degree+1) for j in range(n+1)]
        X=np.array([q**i*z**j for i,j in powers]).T
        coef=np.linalg.lstsq(X*weights[:,None],omega*weights,rcond=None)[0]
        scores={}
        for label,groups in [('command',u),('source_voltage',source_voltage)]:
            errors=[];rpm_errors=[]
            for group in np.unique(groups):
                train=groups!=group
                c=np.linalg.lstsq(X[train]*weights[train,None],omega[train]*weights[train],rcond=None)[0]
                kt=np.sum(weights[train]**2*w2[train]*force[train])/np.sum(weights[train]**2*w2[train]**2)
                predicted=X[~train]@c
                errors.extend((4*kt*predicted**2-force[~train]).tolist())
                rpm_errors.extend(((predicted-omega[~train])*60/(2*np.pi)).tolist())
            scores[label+'_heldout_force_RMSE_N']=float(np.sqrt(np.mean(np.square(errors))))
            scores[label+'_heldout_equivalent_RPM_RMSE']=float(np.sqrt(np.mean(np.square(rpm_errors))))
        values=np.array([gq**i*gz**j for i,j in powers]).T@coef
        slope=np.array([np.zeros_like(gq) if i==0 else i*gq**(i-1)*gz**j/400 for i,j in powers]).T@coef
        candidates.append({'degree':degree,'powers':powers,'coefficients_rad_s':coef.tolist(),**scores,
                           'selection_score_N':float(np.sqrt(np.mean([scores['command_heldout_force_RMSE_N']**2,scores['source_voltage_heldout_force_RMSE_N']**2]))),
                           'minimum_speed_rad_s':float(values.min()),'minimum_command_slope':float(slope.min()),
                           'training_force_RMSE_N':float(np.sqrt(np.mean((4*k*(X@coef)**2-force)**2)))})
    valid=[c for c in candidates if c['minimum_speed_rad_s']>0 and c['minimum_command_slope']>0]
    best=min(c['selection_score_N'] for c in valid)
    selected=next(c for c in valid if c['selection_score_N']<=1.05*best)
    if selected['degree']!=2:
        raise ValueError('C evaluator currently expects the selected total-degree-two speed surface')
    settings=workspace/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_motor_collective.json'
    fc=json.loads(settings.read_text())
    if fc['vbat_sag_compensation']!=100 or fc['rate_full_scale_deg_s']!=[600,600,600]:
        raise ValueError('Recorded FC compensation/rate profile differs from identification')
    model_file=workspace/'Sanity/sanity-hardware/parameters/identified_actuators.json'
    observed=json.loads(model_file.read_text())['models']['observed_step_responses']
    profile={'kind':'esp_ctbr_acro100_FC_voltage_RPM_inverse','voltage_input':'delivered FC bus voltage, V',
             'recorded_fc_profile':fc,'rc_smoothing':True,'rate_signs':[1,1,-1],
             'model_rate_limits_rad_s':[2.3,2.3,2.6],'normalization':{'rc_center':1500.,'rc_scale':400.,'voltage_center_V':7.4,'voltage_scale_V':.8},
             'speed_model':selected,'effective_armed_kT_N_per_rad_s_squared':float(k),
             'force_equation':'T_total = 4*kT*omega_equivalent^2; omega_equivalent=sqrt(sum(omega_i^2)/4)',
             'convex_hull_rc_FC_voltage':points[hull.vertices].tolist(),'voltage_range_V':[float(voltage.min()),float(voltage.max())],
             'calibration_points':len(rows),'candidate_models':candidates,'selection':'Lowest degree within 5% of best combined held-out-command and held-out-source-voltage force RMS, positive speed/RC slope over measured hull',
             'speed_law_training_RMSE_N':float(np.sqrt(np.mean((k*w2-force)**2))),
             'sources':sources,'recorded_fc_profile_sha256':hashlib.sha256(settings.read_bytes()).hexdigest(),
             'observed_RPM_responses':[r for r in observed['fits'] if r['mode']=='acro' and r['signal']=='rpm'],
             'zero_thrust_rc':1000,'positive_rc_range':[int(u.min()),int(u.max())],
             'voltage_policy':{'range_interlock':False,'missing_voltage_V':8.0,'outside_range':'clamp model voltage to fitted range; keep reported telemetry unchanged'},
             'positive_below_support':'clamp to minimum measured RC',
             'above_support':'clamp to maximum measured RC',
             'outside_voltage_support':'continue with bounded model voltage; no voltage interlock',
             'scope':'Voltage-aware static inverse identified from armed bench RPM/force; no added RPM controller or inverse lag filter',
             'limitations':['Rigid zero-rate bench measurements, not free-flight or nonzero-moment mixer validation.',
                            'Effective armed thrust coefficient is specific to this measurement path/cohort; not an update of the isolated-rotor constant.',
                            'Displayed FC voltage is used consistently with fitting; internal Betaflight sag voltage may differ.',
                            'Observed RPM timing includes filtering and telemetry; not motor-only or body-rate identification.',
                            'Runtime inversion uses the full measured RC range; points outside the measured joint hull are extrapolations. Model voltage is clamped and missing telemetry uses8V; no accuracy claim outside measured coverage.',
                            'This replaces the nominal fixed-source-reference inverse; simulation/training must select this calibration for an exact match.']}
    digest=hashlib.sha256(json.dumps(profile,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    profile['calibration_id']=int(digest[:8],16);profile['calibration_sha256']=digest
    header=['#ifndef SANITY_ACTUATOR_PROFILE_H','#define SANITY_ACTUATOR_PROFILE_H','#include <stdint.h>',
            f'#define SZ_CALIBRATION_ID UINT32_C(0x{digest[:8]})',f'#define SZ_RPM_KT {k:.17g}',
            f'#define SZ_MODEL_RC_MIN {int(u.min())}',f'#define SZ_MODEL_RC_MAX {int(u.max())}',
            '#define SZ_MODEL_FALLBACK_VOLTAGE_MV 8000',
            f'#define SZ_MODEL_VOLTAGE_MIN_MV {int(np.ceil(voltage.min()*1000))}',f'#define SZ_MODEL_VOLTAGE_MAX_MV {int(np.floor(voltage.max()*1000))}',
            'static const double sz_speed_coefficients[6] = {'+', '.join(f'{c:.17g}' for c in selected['coefficients_rad_s'])+'};',
            f'#define SZ_MODEL_HULL_SIZE {len(hull.vertices)}','static const double sz_model_hull[SZ_MODEL_HULL_SIZE][2] = {']
    header += ['    {'+f'{x:.17g}, {V:.17g}'+'},' for x,V in points[hull.vertices]]
    header += ['};','#endif','']
    (receiver/'main/actuator_profile.h').write_text('\n'.join(header))
    target=workspace/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_esp_collective.json'
    target.write_text(json.dumps(profile,indent=2)+'\n')
    print(json.dumps({'calibration_id':profile['calibration_id'],'selected_degree':selected['degree'],'kT':k,'validation':selected,
                      'voltage_range_V':profile['voltage_range_V']},indent=2))
