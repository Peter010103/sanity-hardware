from dataclasses import replace
import json
import os
from pathlib import Path
import numpy as np


class PlateDeploymentConfig:
    def __init__(self, path, workspace, mission_profile=None):
        self.path=Path(path).resolve();self.workspace=Path(workspace).resolve()
        self.data=json.loads(self.path.read_text())
        if self.data.get('version')!=1:raise ValueError('Plate deployment profile version1 required')
        self.settings=self.data['supervision'];self.mocap=self.data['mocap'];self.drones=self.data['drones']
        self.geometry=self.resolve(mission_profile or self.data['mission_profile'])
        from experiments.sanity_plate_transport.config import load_config
        self.physical=replace(load_config(self.geometry),horizon_nodes=8,node_dt=.08)
        if len(self.drones)!=self.physical.drones:raise ValueError('One drone per plate mount required')
        ids=[];frames=[self.mocap['plate_frame']]
        for drone in self.drones:
            if type(drone['id']) is not int or not 1<=drone['id']<=255:raise ValueError('Unique receiver IDs1..255 required')
            ids.append(drone['id']);frames.append(drone['frame'])
            for key in ['aircraft_model','calibration']:drone[key]=str(self.resolve(drone[key]))
            q=np.asarray(drone['marker_to_body_xyzw']);v=np.asarray(drone['marker_to_body_xyz'])
            if q.shape!=(4,) or v.shape!=(3,) or not np.isfinite(np.r_[q,v]).all() or abs(np.linalg.norm(q)-1.)>1e-6:raise ValueError('Finite aircraft marker transform required')
        if len(ids)!=len(set(ids)) or len(frames)!=len(set(frames)) or any(not f for f in frames):raise ValueError('Receiver IDs and all four mocap frames must be unique')
        q=np.asarray(self.mocap['world_from_map_xyzw'])
        if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-6:raise ValueError('Unit world transform required')
        if not 0<self.mocap['maximum_age_s']<=.04 or not 0<self.mocap['maximum_skew_s']<=.012:raise ValueError('Mocap age/skew may not exceed40/12ms')
        required=['climb_s','hover_dwell_s','acquire_timeout_s','blend_s','mpc_hover_s','level_s','landing_speed_m_s','landing_timeout_s','cutoff_height_m','initial_formation_tolerance_m','maximum_run_s','mpc_loss_s']
        if not np.isfinite([self.settings[k] for k in required]).all() or min(self.settings[k] for k in required)<=0:raise ValueError('Positive finite supervision settings required')
        if not np.isfinite(self.settings['floor_height_m']) or not .01<=self.settings['cutoff_height_m']<=.05 or not .01<=self.settings['mpc_loss_s']<=.10:raise ValueError('Finite floor, cutoff<=5cm and MPC recovery<=100ms required')
        maximum_start_height=self.settings.get('maximum_start_height_m',.08)
        if not np.isfinite(maximum_start_height) or maximum_start_height<.005:raise ValueError('Finite maximum aircraft start height>=5mm required')
        if 'landing_body_height_m' in self.settings:
            height=self.settings['landing_body_height_m']
            if not np.isfinite(height) or not .005<=height<=.08:raise ValueError('Floor landing body height must be finite and within5..80mm')
        gain=self.settings.get('pd_attitude_gain',6.)
        if not np.isfinite(gain) or gain<=0:raise ValueError('Positive finite PD attitude gain required')
        pickup=self.settings.get('coordinated_pickup')
        if pickup is not None:
            required_pickup={'enabled','approach_speed_m_s','takeup_speed_m_s','acceleration_m_s2','xy_tolerance_m','z_tolerance_m','maximum_speed_m_s','maximum_tilt_deg','minimum_separation_m','engagement_margin_m','release_margin_m','maximum_extension_m','engagement_dwell_s','settle_s','phase_timeout_s','abort_speed_m_s'}
            if set(pickup)!=required_pickup or type(pickup['enabled']) is not bool:raise ValueError('Explicit complete coordinated_pickup settings required')
            values=[pickup[k] for k in required_pickup-{'enabled'}]
            if not np.isfinite(values).all() or min(values)<=0:raise ValueError('Positive finite coordinated_pickup settings required')
            if not pickup['engagement_margin_m']<pickup['release_margin_m'] or pickup['minimum_separation_m']<self.physical.minimum_drone_separation_m or pickup['maximum_tilt_deg']>=45 or pickup['takeup_speed_m_s']>pickup['approach_speed_m_s']:
                raise ValueError('Invalid coordinated_pickup engagement hysteresis or flight margins')
        self.timing=self.data['timing']
        self.affinity=self.data.get('cpu_affinity',{})
        for cpus in self.affinity.values():
            if not isinstance(cpus,list) or any(type(cpu) is not int or not 0<=cpu<os.cpu_count() for cpu in cpus):raise ValueError('CPU affinity must contain available logical CPU numbers')
        if self.timing['control_period_s']!=.02 or not 0<=self.timing['output_delay_s']<=.04 or not .01<=self.timing['solver_budget_s']<=.04:raise ValueError('50Hz selection and bounded solver/output timing required')
        self.models=[json.loads(Path(d['aircraft_model']).read_text()) for d in self.drones]
        for i,model in enumerate(self.models):
            if 'attachment_body_m' in self.drones[i]:
                hook=np.asarray(self.drones[i]['attachment_body_m'],dtype=float)
                if hook.shape!=(3,) or not np.isfinite(hook).all():raise ValueError('Aircraft attachment_body_m must be a finite three-dimensional offset')
                model.update(attachment_body_m=hook.tolist(),attachment_source=f'Explicit per-aircraft attachment in plate profile {self.path}')
            if 'aircraft_and_cable_mass_g' in self.drones[i]:
                mass=self.drones[i]['aircraft_and_cable_mass_g']*.001-self.physical.cable_mass_kg
                if not np.isfinite(mass) or mass<=0:raise ValueError('Aircraft-plus-cable AUW must exceed the separately modeled cable mass')
                model.update(airframe_mass_kg=float(mass),mass_kg=float(mass),
                    airframe_mass_source=f"Plate profile {self.path}: measured aircraft+cable AUW {self.drones[i]['aircraft_and_cable_mass_g']}g minus separately modeled cable {self.physical.cable_mass_kg*1000.}g")
            if not np.isclose(model['airframe_mass_kg'],self.physical.masses[i]):raise ValueError('Aircraft mass must match plate geometry configuration')
        self.mission_profile=str(self.geometry)


    def resolve(self, path):
        path=Path(path)
        result=path if path.is_absolute() else self.workspace/path
        if not result.is_file():raise ValueError('Missing explicit plate resource: '+str(result))
        return result.resolve()


    def require_hardware(self):
        if self.settings.get('start_airborne',False):raise ValueError('Airborne reset is an offline simulation fixture')
        if self.data.get('geometry_confirmed') is not True or self.mocap.get('frames_confirmed') is not True:raise ValueError('Hardware calibration deferred: confirm plate geometry and all marker frames before enabling output')
        for drone in self.drones:
            if drone.get('hardware_confirmed') is not True:raise ValueError(f"Receiver/frame/model/calibration assignment for device {drone['id']} is still provisional")
