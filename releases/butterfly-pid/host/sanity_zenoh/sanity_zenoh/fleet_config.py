from copy import deepcopy
import ipaddress
import math
from pathlib import Path

import numpy as np
import yaml


class FleetConfig:
    def __init__(self, path, workspace):
        self.path = Path(path).resolve()
        self.root = Path(workspace).resolve()
        self.data = yaml.safe_load(self.path.read_text())
        data = self.data
        if not isinstance(data, dict) or data.get('version') != 1:
            raise ValueError('Expected fleet configuration version 1')
        required = {'version','transport','mocap','mission','safety','drones'}
        if set(data) != required:
            raise ValueError('Fleet sections must be exactly: ' + ', '.join(sorted(required)))
        self.transport = data['transport']
        if set(self.transport) != {'listen','telemetry_listen'}:
            raise ValueError('Shared UDP control and TCP telemetry listeners required')
        for key, scheme in [('listen','udp/'),('telemetry_listen','tcp/')]:
            value = self.transport[key]
            if not isinstance(value,str) or not value.startswith(scheme):
                raise ValueError('Expected ' + scheme + ' endpoint for ' + key)
            host,port = value[len(scheme):].rsplit(':',1)
            if not ipaddress.ip_address(host).is_loopback and host != '10.0.0.200':
                raise ValueError('Keep the existing explicit robotlab host interface')
            if not 1 <= int(port) <= 65535:
                raise ValueError('Invalid listener port')
        self.mocap = data['mocap']
        if set(self.mocap) != {'topic','parent_frame','world_from_map_xyzw'} or self.mocap['topic'] != '/tf':
            raise ValueError('Fleet mocap uses /tf and one common world transform')
        q = np.asarray(self.mocap['world_from_map_xyzw'],dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q),1.,atol=1e-6):
            raise ValueError('World transform must be a finite unit quaternion')
        self.mission = data['mission']
        if set(self.mission) != {'kind','height_m','hover_s','length_m','speed_m_s','cycles','endpoint_hold_s'}:
            raise ValueError('Unexpected or missing hover/line mission setting')
        if self.mission['kind'] not in ('hover','line'):
            raise ValueError('Initial fleet missions are hover and line')
        for key in ('height_m','hover_s','length_m','speed_m_s','endpoint_hold_s'):
            if not isinstance(self.mission[key],(int,float)) or not math.isfinite(self.mission[key]) or self.mission[key] <= 0:
                raise ValueError('Positive finite mission setting required: ' + key)
        if type(self.mission['cycles']) is not int or not 1 <= self.mission['cycles'] <= 100 or self.mission['speed_m_s'] > 3.:
            raise ValueError('Use 1..100 line cycles and speed <=3m/s')
        self.safety = data['safety']
        safety_keys = {'body_radius_m','tracking_margin_m','timing_uncertainty_s','max_speed_m_s','ready_dwell_s',
                       'start_lead_s','startup_timeout_s','maximum_run_s','individual_failure'}
        if not safety_keys <= set(self.safety) or set(self.safety) - safety_keys - {'separation_mode','takeoff_mode'} or self.safety['individual_failure'] not in ('group_land','isolated_land'):
            raise ValueError('Explicit fleet limits and individual_failure policy required')
        self.takeoff_mode = self.safety.get('takeoff_mode','sequential')
        if self.takeoff_mode not in ('sequential','simultaneous'):
            raise ValueError('takeoff_mode must be sequential or simultaneous')
        self.separation_mode = self.safety.get('separation_mode','braking_regions')
        if self.separation_mode not in ('braking_regions','stationary_hover'):
            raise ValueError('Unknown fleet separation_mode')
        if self.separation_mode == 'stationary_hover' and self.mission['kind'] != 'hover':
            raise ValueError('stationary_hover separation is only for a hover mission')
        for key in safety_keys - {'individual_failure'}:
            value = self.safety[key]
            if not isinstance(value,(float,int)) or not math.isfinite(value) or value <= 0:
                raise ValueError('Positive finite fleet limit required: ' + key)
        if self.safety['max_speed_m_s'] <= self.mission['speed_m_s'] or not .2 <= self.safety['start_lead_s'] <= 5.:
            raise ValueError('Fleet speed limit must exceed reference speed; start lead 0.2..5s')
        if self.safety['timing_uncertainty_s'] < .128 or not 120 <= self.safety['maximum_run_s'] <= 1200:
            raise ValueError('Include >=128ms timing uncertainty and runtime in120..1200s')
        self.drones = data['drones']
        if not isinstance(self.drones,list) or not self.drones:
            raise ValueError('At least one drone is required')
        ids, frames, ips = set(), set(), set()
        base = yaml.safe_load((self.root/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/circle_sequence.yaml').read_text())['sanity_sequence']['ros__parameters']
        self.parameters = {}
        self.calibration_ids = {}
        for drone in self.drones:
            keys = {'id','ip','tracked_frame','hardware_confirmed','calibration','controller','placement','home_xy','region','marker_to_base_xyz','marker_to_base_xyzw','sequence_config'}
            if not isinstance(drone,dict) or set(drone) != keys:
                raise ValueError('Unexpected/missing drone configuration field')
            device, frame, ip = drone['id'], drone['tracked_frame'], drone['ip']
            if type(device) is not int or not 1 <= device <= 255 or device in ids:
                raise ValueError('Duplicate or invalid device ID')
            if not isinstance(frame,str) or not frame or frame == self.mocap['parent_frame'] or frame in frames:
                raise ValueError('Duplicate or invalid tracked frame')
            if ip is not None:
                if not isinstance(ip,str) or ipaddress.ip_address(ip).version != 4 or ip in ips:
                    raise ValueError('Duplicate or invalid receiver IP')
                ips.add(ip)
            if type(drone['hardware_confirmed']) is not bool:
                raise ValueError('hardware_confirmed must be boolean')
            ids.add(device); frames.add(frame)
            for name,size in [('home_xy',2),('marker_to_base_xyz',3),('marker_to_base_xyzw',4)]:
                value = np.asarray(drone[name],dtype=float)
                if value.shape != (size,) or not np.isfinite(value).all():
                    raise ValueError('Invalid '+name)
            if not np.isclose(np.linalg.norm(drone['marker_to_base_xyzw']),1.,atol=1e-6):
                raise ValueError('Marker rotation must be a unit quaternion')
            if set(drone['placement']) != {'center_xy','direction_xy','reverse'}:
                raise ValueError('Expected placement center, direction and reverse flag')
            center, direction = (np.asarray(drone['placement'][key],dtype=float) for key in ('center_xy','direction_xy'))
            if center.shape != (2,) or direction.shape != (2,) or not np.isfinite(np.r_[center,direction]).all() or not np.isclose(np.linalg.norm(direction),1.):
                raise ValueError('Finite world centre and unit line direction required')
            if type(drone['placement']['reverse']) is not bool:
                raise ValueError('reverse must be boolean')
            region = np.asarray(drone['region'],dtype=float)
            if region.shape != (2,3) or not np.isfinite(region).all() or np.any(region[0] >= region[1]):
                raise ValueError('Expected finite ordered3D operating region')
            if not drone['calibration']:raise ValueError('An explicit CTBR calibration is required')
            for key in ('calibration','sequence_config'):
                if drone[key]:
                    drone[key] = str(self.resolve(drone[key]))
            from sanity_zenoh.firmware_collective import FirmwareCollective
            self.calibration_ids[device] = FirmwareCollective(drone['calibration']).calibration_id
            controller = drone['controller']
            if not isinstance(controller,dict) or set(controller) - {'type','config','model','checkpoint','residual_model','residual_source'} or controller.get('type') not in ('mpc','policy'):
                raise ValueError('Explicit policy or MPC configuration required')
            if controller['type'] == 'mpc' and ('config' not in controller or set(controller)-{'type','config','model'}):
                raise ValueError('MPC takes config only, without a policy/residual')
            if controller['type'] == 'policy' and ('config' in controller or 'model' in controller or bool(controller.get('residual_model')) != bool(controller.get('residual_source'))):
                raise ValueError('Policy takes checkpoint and an optional complete residual pair, without MPC config')
            if controller['type'] == 'policy' and not controller.get('checkpoint'):
                raise ValueError('Policy checkpoint required')
            for key in set(controller) - {'type'}:
                controller[key] = str(self.resolve(controller[key]))
            params = deepcopy(base)
            if drone['sequence_config']:
                params.update(yaml.safe_load(Path(drone['sequence_config']).read_text())['sanity_sequence']['ros__parameters'])
            params.update(controller=controller['type'],drone_id=device,tracked_frame=frame,
                parent_frame=self.mocap['parent_frame'],tf_topic='/tf',world_from_map_xyzw=list(map(float,self.mocap['world_from_map_xyzw'])),
                marker_to_base_xyz=list(map(float,drone['marker_to_base_xyz'])),marker_to_base_xyzw=list(map(float,drone['marker_to_base_xyzw'])),
                target_height_m=float(self.mission['height_m']),maximum_run_s=float(self.safety['maximum_run_s']),
                start_at_current_xy=True,origin_xy=list(map(float,drone['home_xy'])),control_cpu=-1,
                esp_collective_profile=drone['calibration'],motor_data_root=str(self.root/'Sanity/sanity-hardware'),motor_profile='',
                listen=self.transport['listen'],telemetry_listen=self.transport['telemetry_listen'],connect='',
                transport_diagnostics=True,loco_root=str(self.root/'loco_mjx'),
                policy_python=str(self.root/'.venv-sanity-train/bin/python'),duration_s=60.,
                simulation=False,use_sim_time=False,ctbr_map='',residual_model='',residual_source='',
                checkpoint='',mpc_config='',mpc_model='',test_delay_tick=-1,test_delay_s=0.)
            if controller['type'] == 'mpc':
                params['mpc_config'] = controller['config']
                if controller.get('model'):params['mpc_model'] = controller['model']
            else:
                params['checkpoint'] = controller['checkpoint']
                for key in ('residual_model','residual_source'):
                    if key in controller:params[key] = controller[key]
            if params.get('residual_model') and self.mission['speed_m_s']>3.:
                raise ValueError('Residual reference exceeds its existing interface')
            if not params['ground_root_height_m'] < self.mission['height_m'] < params['maximum_altitude_m']:
                raise ValueError('Fleet height conflicts with per-drone flight envelope')
            self.parameters[device] = params
        from sanity_zenoh.fleet_reference import FleetMission
        self.plans = {d['id']:FleetMission(self.mission,d,self.parameters[d['id']]) for d in self.drones}
        self.separation = self.check_separation()
        duration = max(p.duration for p in self.plans.values())
        if 40 * len(self.drones) + duration + 50 > self.safety['maximum_run_s']:
            raise ValueError('Allow sequential startup, mission and landing reserve in maximum_run_s')


    def resolve(self, value):
        path = Path(value)
        if not path.is_absolute():path = self.root/path
        if not path.exists():raise ValueError('Missing fleet resource: '+str(path))
        return path.resolve()


    def check_separation(self):
        s = self.safety
        stationary = self.separation_mode == 'stationary_hover'
        brake = 0. if stationary else .5 * s['max_speed_m_s'] * max(2.,1.875*s['max_speed_m_s'])
        margin = s['body_radius_m']+s['tracking_margin_m']+s['max_speed_m_s']*s['timing_uncertainty_s']+brake
        result = {'mode':self.separation_mode,'required_center_separation_m':2*margin,'recovery_margin_m':margin,'pairs':[],
            'scope':('Stationary vertical takeoff/hover/landing at each home, circular horizontal separation buffers; no full-speed braking-space guarantee. Live proximity, operating-region and speed checks remain active.' if stationary else
                'Disjoint horizontal reserved regions; conservative line/home convex hull plus PD braking allowance. Declared bounds, not proven tracking or radio guarantees.')}
        boxes = []
        for drone in self.drones:
            mission = self.plans[drone['id']]
            if stationary and not np.allclose(drone['home_xy'],mission.start_xy,rtol=0.,atol=1e-6):
                raise ValueError('stationary_hover requires hover placement at each home; transfers need braking_regions')
            points = np.vstack([drone['home_xy'],mission.start_xy,mission.end_xy])
            lo,hi = np.min(points,axis=0)-margin,np.max(points,axis=0)+margin
            region = np.asarray(drone['region'])
            if np.any(lo<region[0,:2]) or np.any(hi>region[1,:2]) or not region[0,2] <= 0 < self.mission['height_m'] < region[1,2]:
                raise ValueError(f"Drone {drone['id']} lacks reserved takeoff/line/landing and braking space")
            boxes.append((drone['id'],lo,hi))
        for index,(a,lo,hi) in enumerate(boxes):
            for b,other_lo,other_hi in boxes[index+1:]:
                if stationary:
                    distance = float(np.linalg.norm((lo+hi-other_lo-other_hi)/2.))
                    clearance = distance-2*margin
                    if clearance <= 0:
                        raise ValueError(f'Stationary hover separation buffers overlap: {a}, {b}')
                    result['pairs'].append({'devices':[a,b],'center_distance_m':distance,'reserved_region_gap_m':clearance})
                else:
                    gap = np.maximum(0.,np.maximum(lo-other_hi,other_lo-hi))
                    if np.linalg.norm(gap) <= 0:
                        raise ValueError(f'Reserved flight/recovery regions overlap: {a}, {b}')
                    result['pairs'].append({'devices':[a,b],'reserved_region_gap_m':float(np.linalg.norm(gap))})
        return result


    def require_hardware(self):
        for drone in self.drones:
            if not drone['hardware_confirmed'] or drone['ip'] is None:
                raise ValueError(f"Drone {drone['id']} is an unconfigured hardware placeholder; finish setup before --fly")
