import json
import math
from pathlib import Path

import numpy as np

from sanity_zenoh.cable_state import CableMocap
from sanity_zenoh.cable_qualification import HoverQualification
from sanity_zenoh.cable_length import HoverCableLength
from sanity_zenoh.cable_health import PayloadHealth, VoltageGuard, MissionFailureGuard


def load_physical_configuration(experiment, profile, profile_path, override=''):
    if override:
        path=Path(override)
    elif profile.get('physical_config'):
        path=Path(profile['physical_config'])
        if not path.is_absolute():path=Path(profile_path).resolve().parent/path
    else:
        path=Path(experiment)/'measured_cube.json'
    physical=json.loads(path.read_text())
    if 'attachment_body_m' in profile:physical['attachment_body_m']=profile['attachment_body_m']
    return physical


def controller_configuration(physical, profile, simulation=False, enable_output=False):
    if physical.get('payload_mocap_frame',profile['mocap']['payload_frame'])!=profile['mocap']['payload_frame']:
        raise ValueError('Physical payload model and mocap payload frame do not match')
    if profile['mission'] not in ('circle_slow','circle_fast','payload_orbit') or profile['controller'] not in ('aware','unaware'):
        raise ValueError('Choose a supported cable mission and aware/unaware controller')
    if profile['mission']=='payload_orbit' and profile['controller']!='aware':raise ValueError('Payload orbit requires the aware controller')
    if profile.get('payload_spiral') and profile['mission']!='payload_orbit':raise ValueError('The small spiral profile requires mission=payload_orbit')
    ablation=profile.get('hardware_ablation')
    if ablation is not None:
        if ablation!='rigid_total_mass' or profile['controller']!='unaware' or profile['mission']!='circle_fast' or profile.get('circle_period_s')!=2.:
            raise ValueError('Hardware ablation requires explicit rigid_total_mass,unaware,circle_fast and2.0s period')
        if not profile.get('payload_health') or not profile.get('flight_voltage') or profile.get('landing_transfer_s',0.)<.3 or not profile.get('cable_length_calibration',{}).get('enabled') or not profile.get('failure_guard'):
            raise ValueError('Hardware ablation requires payload health, voltage guard, smooth landing transfer, hover length calibration and failure guard')
    if enable_output and not simulation and ((profile['controller']!='aware' and ablation is None) or (profile['mission']=='payload_orbit' and not profile.get('payload_spiral'))):
        raise ValueError('Hardware output currently supports only aware circles, the dedicated small spiral or the explicit guarded rigid-mass ablation profile')
    if profile.get('start_airborne',False) and not simulation:raise ValueError('Airborne initial fixture is simulation-only')
    if not all(math.isfinite(float(profile[key])) for key in ('floor_height_m','hover_height_m','maximum_height_m','climb_s','output_delay_s','maximum_run_s')):
        raise ValueError('Finite mission parameters required')
    if not (0<profile['climb_s']<=30 and profile['floor_height_m']+.8<=profile['hover_height_m']<profile['maximum_height_m']<=profile['floor_height_m']+2. and 0<=profile['output_delay_s']<=.04 and 90<=profile['maximum_run_s']<=300):
        raise ValueError('Conservative climb/hover/ceiling,90..300s budget and output delay0..40ms required')
    if enable_output and not simulation and not (profile.get('geometry_confirmed') and profile['mocap'].get('frames_confirmed')):
        raise ValueError('Aircraft/payload frame transforms and attachment geometry must be confirmed for hardware output')
    if not np.isfinite(physical['attachment_body_m']).all() or np.asarray(physical['attachment_body_m']).shape!=(3,):
        raise ValueError('Finite three-axis attachment offset required')
    if profile.get('landing_speed_profile','standard') not in ('standard','quick'):raise ValueError('Choose standard or quick landing speed profile')
    if profile.get('smooth_takeoff',False):
        slew=np.asarray(profile.get('takeoff_command_slew_per_s',[1.,1.5,1.5,2.]),dtype=float)
        load_rate=float(profile.get('pickup_load_fraction_rate_per_s',1.))
        if slew.shape!=(4,) or not np.isfinite(slew).all() or np.any(slew<=0) or not math.isfinite(load_rate) or not .25<=load_rate<=2.:raise ValueError('Positive finite takeoff command slew and pickup load rate0.25..2/s required')
    cutoff=float(profile.get('landing_cutoff_height_m',0.));dwell=float(profile.get('landing_cutoff_dwell_s',.10))
    if not math.isfinite(cutoff) or not 0<=cutoff<=.15 or not math.isfinite(dwell) or not .05<=dwell<=2.:raise ValueError('Landing cutoff height0..15cm and dwell0.05..2s required')
    if profile.get('mpc_hover_only',False) and profile.get('takeoff_land_only',False):raise ValueError('Choose PID commissioning or MPC hover, not both')
    if profile.get('payload_attitude_dynamics',False) and profile['mocap'].get('payload_estimation')=='position_pendulum':raise ValueError('Full cube MPC requires full-pose payload estimation')
    recovery=float(profile.get('payload_recovery_s',0.))
    if not math.isfinite(recovery) or not 0<=recovery<=.1:raise ValueError('Payload recovery must be in0..100ms')
    hold=float(profile.get('hover_duration_s',5.))
    if not math.isfinite(hold) or not 1.<=hold<=15.:raise ValueError('Hover duration must be finite and between1 and15s')
    if profile.get('takeoff_land_only',False) and (profile['mission']!='circle_slow' or profile['controller']!='aware'):
        raise ValueError('Takeoff/land commissioning requires circle_slow/aware as its stationary base profile')
    lease=profile.get('command_lease_s',1.)
    damping=profile.get('pickup_swing_damping_gain_m_s',.8)
    clearance=np.asarray(profile.get('floor_landing_clearance_xy_m',[.15,0.]))
    if not math.isfinite(lease) or not .04<lease<=1. or not math.isfinite(damping) or not 0<=damping<=2.:
        raise ValueError('Finite command lease40ms..1s and pickup swing gain0..2m/s required')
    if clearance.shape!=(2,) or not np.isfinite(clearance).all() or not .10<=np.linalg.norm(clearance)<=.25:
        raise ValueError('Landing clearance must be a finite10..25cm XY vector')
    HoverQualification(profile.get('mpc_hover_qualification'))
    PayloadHealth(profile.get('payload_health'))
    VoltageGuard(profile.get('flight_voltage'))
    MissionFailureGuard(profile.get('failure_guard'))
    transfer=float(profile.get('landing_transfer_s',0.))
    if not math.isfinite(transfer) or not 0<=transfer<=1.:raise ValueError('Landing transfer must be in0..1s')
    CableMocap(physical,profile['mocap'])
    cfg={**physical,'scope':'cable_control','simulation':bool(simulation),'maximum_height_m':profile['maximum_height_m'],
         'prediction_mode':'rigid_total_mass' if profile['controller']=='unaware' else 'coupled_cable',
         'tracking_frame':'load_com' if profile['mission']=='payload_orbit' else 'aircraft'}
    if profile['mission']=='payload_orbit':
        cfg.update(orbit_stop_start_s=30.,orbit_stop_duration_s=10.,state_cost=[2000.,2000.,1800.,150.,150.,120.,1000.,1000.,1200.,2.,2.,10.,.1,.1,.1,.4,.4,.4,2.,2.,2.,.1])
        if profile.get('payload_spiral'):
            orbit=profile['payload_spiral']
            required={'radius_m','start_s','ramp_out_s','hold_s','ramp_in_s','settle_s','maximum_aircraft_displacement_m','maximum_payload_offset_m'}
            if not isinstance(orbit,dict) or set(orbit)!=required or not all(math.isfinite(float(v)) for v in orbit.values()):raise ValueError('Complete finite payload spiral parameters required')
            if not .01<=orbit['radius_m']<=.05 or not 1<=orbit['start_s']<=5 or not 10<=orbit['ramp_out_s']<=20 or not 5<=orbit['hold_s']<=20 or not 10<=orbit['ramp_in_s']<=20 or not 5<=orbit['settle_s']<=10 or not .10<=orbit['maximum_aircraft_displacement_m']<=.2 or not 1.5*orbit['radius_m']<=orbit['maximum_payload_offset_m']<=.12:raise ValueError('Small spiral bounds require radius1..5cm and slow10..20s ramps')
            if not profile.get('payload_health') or not profile.get('flight_voltage') or transfer<.3 or not profile.get('cable_length_calibration',{}).get('enabled') or profile.get('payload_attitude_dynamics',True):raise ValueError('Small spiral requires payload health, voltage guard, landing transfer and calibrated equivalent pendulum')
            cfg.update(orbit_radius_m=orbit['radius_m'],orbit_start_s=orbit['start_s'],orbit_ramp_s=orbit['ramp_out_s'],orbit_stop_start_s=orbit['start_s']+orbit['ramp_out_s']+orbit['hold_s'],orbit_stop_duration_s=orbit['ramp_in_s'],orbit_settle_s=orbit['settle_s'],body_tilt_limit_deg=15.,cable_swing_limit_deg=20.,state_cost=[4000.,4000.,4500.,200.,200.,250.,1000.,1000.,1200.,2.,2.,10.,.1,.1,.1,0.,0.,0.,0.,0.,0.,.1])
    else:
        scale=float(profile.get('swing_cost_scale',25.))
        if not math.isfinite(scale) or scale<=0:raise ValueError('Positive finite swing-cost scale required')
        aware=profile['controller']=='aware'
        cfg['state_cost']=[4000.,4000.,4500.,200.,200.,250.,0.,0.,0.,2.,2.,10.,.1,.1,.1,*([.2*scale]*3 if aware else [0.]*3),*([2.*scale]*3 if aware else [0.]*3),.1]
    if 'circle_period_s' in profile:
        period=float(profile['circle_period_s'])
        if profile['mission']!='circle_fast' or not math.isfinite(period) or not 2.<=period<=2.5:raise ValueError('Custom fast-circle period requires circle_fast and2.0..2.5s')
        if enable_output and not simulation and period<2.5 and (not profile.get('payload_health') or not profile.get('flight_voltage') or transfer<.3 or not profile.get('cable_length_calibration',{}).get('enabled')):raise ValueError('Faster circles require payload health, voltage guard, landing transfer and hover length calibration')
        cfg['circle_period_s']=period
    cfg['payload_attitude_dynamics']=bool(profile.get('payload_attitude_dynamics',True) and profile['controller']=='aware')
    length_calibration=HoverCableLength(physical['cable_length_m'],profile.get('cable_length_calibration'))
    if length_calibration.enabled:
        if cfg['payload_attitude_dynamics'] or profile['mocap'].get('payload_estimation')!='position_pendulum':raise ValueError('Hover length calibration requires the equivalent pendulum and position-based payload estimator')
        cfg.update(runtime_cable_length=True,runtime_cable_length_bounds_m=length_calibration.bounds,runtime_payload_mass_bounds_kg=[.005,.1])
    cfg['nodes']=int(profile.get('nodes',14));cfg['prediction_dt_s']=float(profile.get('prediction_dt_s',.06))
    cfg['first_command_change_cost']=profile.get('first_command_change_cost',[1000.,50.,50.,10.])
    cfg['command_slew_per_s']=profile.get('command_slew_per_s',[2.,15.,15.,15.])
    cfg['mpc_input_cost']=profile.get('mpc_input_cost',[3.,.15,.15,.15])
    if np.asarray(cfg['mpc_input_cost']).shape!=(4,) or not np.isfinite(cfg['mpc_input_cost']).all() or min(cfg['mpc_input_cost'])<=0:
        raise ValueError('Four positive finite input weights required')
    for name in ('integration_substeps','native_integrator','cable_collision_model','cable_joint_armature_kg_m2','cable_topology'):
        cfg.pop(name,None)
    return cfg
