from collections import deque, OrderedDict

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from sanity_zenoh.mocap import MocapEstimator


class CableMocap:
    def __init__(self, physical, calibration):
        self.physical = physical
        self.max_age = calibration.get('max_age_s', .04)
        self.max_skew = calibration.get('maximum_pair_skew_s', .012)
        if not np.isfinite(self.max_skew) or not 0<=self.max_skew<=.02:
            raise ValueError('Pair skew must be finite and in0..20ms')
        self.buffers = {'aircraft': deque(maxlen=40), 'payload': deque(maxlen=40)}
        self.position_pendulum=calibration.get('payload_estimation','full_pose')=='position_pendulum'
        if calibration.get('payload_estimation','full_pose') not in ('full_pose','position_pendulum'):raise ValueError('Unknown payload estimator')
        self.estimators = {}
        for key in self.buffers:
            self.estimators[key] = MocapEstimator(calibration['parent_frame'], calibration[key+'_frame'], calibration['world_from_map_xyzw'],
                calibration[key+'_marker_to_body_xyz'], calibration[key+'_marker_to_body_xyzw'], calibration.get('payload_velocity_filter_tau_s',calibration.get('velocity_filter_tau_s',.01)) if key=='payload' else calibration.get('velocity_filter_tau_s',.01),
                position_filter_tau_s=calibration.get('payload_position_filter_tau_s',0.) if key=='payload' else 0.,
                frames_confirmed=calibration.get('frames_confirmed', False), angular_rate_filter_tau_s=calibration.get('payload_angular_rate_filter_tau_s',calibration.get('angular_rate_filter_tau_s',.01)) if key=='payload' else calibration.get('angular_rate_filter_tau_s',.01), max_age_s=self.max_age,
                max_speed_m_s=calibration.get('payload_max_speed_m_s',10.) if key=='payload' else 10.,position_innovation_m=calibration.get('payload_position_innovation_m') if key=='payload' else None,
                max_acceleration_m_s2=calibration.get('payload_max_acceleration_m_s2',40.),orientation_hold_s=calibration.get('payload_orientation_hold_s',0.) if key=='payload' and self.position_pendulum else 0.)
        if self.estimators['aircraft'].child == self.estimators['payload'].child:
            raise ValueError('Distinct aircraft and payload mocap frames required')
        self.rejected = 0
        self.mode_filter = bool(calibration.get('cable_mode_filter',False))
        self.reset_mode_on_gap = bool(calibration.get('reset_cable_mode_on_gap',False))
        self.mode = None
        self.mode_stamp = None
        self.mode_velocity = None
        self.set_cable_length(physical['cable_length_m'])


    def set_cable_length(self, length):
        if not np.isfinite(length) or length<=0:raise ValueError('Positive finite cable length required')
        self.physical['cable_length_m']=float(length)
        self.mode=self.mode_stamp=self.mode_velocity=None
        physical=self.physical
        mc,mp = physical['cable_mass_kg'],physical['payload_mass_kg']
        distance = length+physical['payload_com_offset_below_hook_m']
        self.mode_mass = mc+mp
        self.mode_distance = (mc*length/2+mp*distance)/(mc+mp)
        self.mode_inertia = mc*length**2/3+mp*distance**2+physical['payload_transverse_inertia_kg_m2']


    def update(self, parent, child, stamp_ns, position, quaternion, now_ns):
        for name, estimator in self.estimators.items():
            if (parent, child) != (estimator.parent, estimator.child):
                continue
            try:
                packet = estimator.update(parent, child, stamp_ns, position, quaternion, now_ns, now_ns)
            except ValueError:
                self.rejected += 1
                raise
            if packet is not None and packet['ready']:
                self.buffers[name].append(packet)
            return packet
        return None


    def sample(self, now_ns, require_payload=True):
        required = ('aircraft', 'payload') if require_payload else ('aircraft',)
        for name in required:
            if not self.buffers[name]:
                raise ValueError('Waiting for '+name+' mocap')
            age = (now_ns-self.buffers[name][-1]['stamp_ns'])/1e9
            if not 0 <= age < self.max_age:
                raise ValueError(name+' mocap stale or future')
        if not require_payload:
            packet = self.buffers['aircraft'][-1]
            state = np.asarray(packet['state']).copy()
            rotation = Rotation.from_quat(state[[4, 5, 6, 3]])
            state[10:13] = rotation.inv().apply(state[10:13])
            return {'stamp_ns':packet['stamp_ns'], 'vehicle':state}
        stamps = [self.buffers[name][-1]['stamp_ns'] for name in required]
        if (max(stamps)-min(stamps))/1e9 > self.max_skew:
            raise ValueError('Aircraft/payload timestamp skew too large')
        aligned = min(stamps)
        if not 0 <= (now_ns-aligned)/1e9 < self.max_age:
            raise ValueError('Synchronized mocap stale')
        states = {};position_delays = {}
        for name in required:
            packets = self.buffers[name]
            before = next((p for p in reversed(packets) if p['stamp_ns']<=aligned), None)
            after = next((p for p in packets if p['stamp_ns']>=aligned), None)
            if before is None or after is None or (after['stamp_ns']-before['stamp_ns'])/1e9 > self.max_age:
                raise ValueError('No mocap samples bracketing common timestamp')
            a, b = np.asarray(before['state']), np.asarray(after['state'])
            fraction = (aligned-before['stamp_ns'])/max(after['stamp_ns']-before['stamp_ns'], 1)
            state = a+(b-a)*fraction
            rotation = Rotation.from_quat(a[[4,5,6,3]]) if before is after else Slerp([0.,1.], Rotation.from_quat([a[[4,5,6,3]],b[[4,5,6,3]]]))([fraction])[0]
            state[3:7] = rotation.as_quat()[[3,0,1,2]]
            states[name] = state
            delay_before=before.get('pose_filter_delay_s',{}).get('position',0.);delay_after=after.get('pose_filter_delay_s',{}).get('position',0.)
            position_delays[name]=delay_before+fraction*(delay_after-delay_before)
        vehicle, payload = states['aircraft'], states['payload']
        body = Rotation.from_quat(vehicle[[4,5,6,3]])
        cube = Rotation.from_quat(payload[[4,5,6,3]])
        r = body.apply(self.physical['attachment_body_m'])
        lever = cube.apply([0.,0.,self.physical['payload_com_offset_below_hook_m']])
        attachment = vehicle[:3]+r
        va = vehicle[7:10]+np.cross(vehicle[10:13],r)
        hook = payload[:3]+lever
        vh = payload[7:10]+np.cross(payload[10:13],lever)
        mc, mp = self.physical['cable_mass_kg'], self.physical['payload_mass_kg']
        com = (mc*(attachment+hook)/2+mp*payload[:3])/(mc+mp)
        velocity = (mc*(va+vh)/2+mp*payload[7:10])/(mc+mp)
        separation = np.linalg.norm(com-attachment)
        hook_distance = np.linalg.norm(hook-attachment)
        payload_orientation_held=bool(self.buffers['payload'][-1].get('orientation_held',False))
        if self.position_pendulum:
            centre_delta=payload[:3]-attachment
            centre_distance=np.linalg.norm(centre_delta)
            centre_direction=centre_delta/max(centre_distance,1e-9)
            hook_distance=max(0.,centre_distance-self.physical['payload_com_offset_below_hook_m'])
            hook=attachment+hook_distance*centre_direction
            relative=payload[7:10]-va
            vh=va+(hook_distance/max(centre_distance,1e-9))*relative
            com=attachment+self.mode_distance*centre_direction
            velocity=va+(self.mode_distance/max(centre_distance,1e-9))*relative
            separation=self.mode_distance
        hook_direction=(hook-attachment)/max(hook_distance,1e-9)
        hook_rate=np.cross(hook_direction,vh-va)/max(hook_distance,1e-9)
        s = (com-attachment)/max(separation, 1e-9)
        rate = np.cross(s, velocity-va)/max(separation, 1e-9)
        half = np.asarray(self.physical['payload_size_m'])/2
        lowest = float(payload[2]-(np.linalg.norm(half) if payload_orientation_held else np.abs(cube.as_matrix()[2])@half))
        taut = bool(abs(hook_distance-self.physical['cable_length_m'])<.012 and separation>.45)
        raw_swing = float(np.degrees(np.arccos(np.clip(-s[2],-1.,1.))))
        internal_tilt=float(np.degrees(np.arccos(np.clip(cube.apply([0.,0.,1.])@(-s),-1.,1.))))
        if self.mode_filter and taut:
            hook_direction = (hook-attachment)/max(hook_distance,1e-9)
            if self.mode is None or (self.reset_mode_on_gap and aligned-self.mode_stamp>40_000_000):
                self.mode = [hook_direction.copy(),np.cross(hook_direction,vh-va)/max(hook_distance,1e-9)]
            elif aligned>self.mode_stamp:
                dt = (aligned-self.mode_stamp)/1e9
                if dt>.04:raise ValueError('Cable-mode observer source gap')
                direction, angular = self.mode
                acceleration = (va-self.mode_velocity)/dt
                projection = np.eye(3)-np.outer(direction,direction)
                derivative = (-self.mode_mass*self.mode_distance*np.cross(direction,acceleration-[0.,0.,-9.81])-self.physical['cable_damping_Nms']*projection@(angular-vehicle[10:13]))/self.mode_inertia
                predicted = Rotation.from_rotvec(angular*dt+.5*derivative*dt**2).apply(direction)
                error = np.cross(predicted,hook_direction)
                self.mode = [Rotation.from_rotvec(4.*error*dt).apply(predicted),angular+derivative*dt+8.*error*dt]
            self.mode_stamp,self.mode_velocity = aligned,va.copy()
            s,rate = (item.copy() for item in self.mode)
            s /= np.linalg.norm(s);rate -= s*(rate@s)
            self.mode = [s.copy(),rate.copy()]
        else:
            self.mode = None;self.mode_stamp=None;self.mode_velocity=None
        vehicle = vehicle.copy()
        vehicle[10:13] = body.inv().apply(vehicle[10:13])
        return {'stamp_ns':int(aligned), 'vehicle':vehicle, 'payload':payload, 'direction':s, 'cable_rate':rate,'hook_direction':hook_direction,'hook_rate':hook_rate,
                'com':com, 'com_velocity':velocity, 'hook_distance_m':float(hook_distance), 'cube_lowest_height_m':lowest,
                'cable_length_residual_m':float(hook_distance-self.physical['cable_length_m']),'cable_length_tolerance_m':.012,
                'payload_position_filter_delay_s':float(position_delays['payload']),
                'taut':taut,'payload_internal_tilt_deg':internal_tilt,'payload_orientation_held':payload_orientation_held,'payload_estimation':'position_pendulum' if self.position_pendulum else 'full_pose',
                'swing_deg':raw_swing, 'pair_skew_s':(max(stamps)-min(stamps))/1e9}


class CommandObserver:
    def __init__(self, model, start_ns):
        self.model = model
        self.records = deque([(int(start_ns), np.zeros(11), np.zeros(4))])
        self.matrices = []
        self.transition_bases = []
        self.transitions = OrderedDict()
        for gain, frequency, damping, zero, delay in model.coefficients:
            wn = 2*np.pi*frequency
            matrix = np.zeros((6,6))
            matrix[0,1] = 1.; matrix[1,:2] = [-wn**2,-2*damping*wn]; matrix[1,4] = wn**2
            matrix[2,2], matrix[2,5] = -3/delay,3/delay
            matrix[3,2:4] = [3/delay,-3/delay]; matrix[4,3:5] = [3/delay,-3/delay]
            self.matrices.append(matrix)
            scale=max(3/delay,wn);roots=np.roots([1.,2*damping*wn,wn**2])/scale;pole=-3/delay/scale
            # Hermite interpolation: zero, two oscillator poles, and the triple delay pole.
            conditions=[(0.,0),(roots[0],0),(roots[1],0),(pole,0),(pole,1),(pole,2)]
            hermite=np.array([[0. if k<order else np.prod(np.arange(k-order+1,k+1))*root**(k-order) for k in range(6)] for root,order in conditions])
            if np.linalg.cond(hermite)>1e9:raise ValueError('Actuator poles too close for the analytic command observer')
            powers=np.stack([np.linalg.matrix_power(matrix/scale,k) for k in range(6)])
            basis=np.einsum('ki,kab->iab',np.linalg.inv(hermite),powers)
            self.transition_bases.append((scale,roots,pole,basis))


    def advance(self, state, command, duration):
        if duration < -1e-9 or duration > 5.:
            raise ValueError('Invalid command-observer interval')
        duration = max(duration,0.)
        result = np.asarray(state).copy()
        target = np.clip(command[0],0.,self.model.config['physical_force_range_N'][1])
        tau = self.model.config['collective_tau_up_s'] if target>=state[0] else self.model.config['collective_tau_down_s']
        result[0] = target+(state[0]-target)*np.exp(-duration/tau)
        if duration not in self.transitions:
            values=[]
            for scale,roots,pole,basis in self.transition_bases:
                h=scale*duration;decay=np.exp(pole*h)
                rhs=np.array([1.,np.exp(roots[0]*h),np.exp(roots[1]*h),decay,h*decay,h*h*decay])
                values.append(np.einsum('i,iab->ab',rhs,basis).real)
            self.transitions[duration]=values
            if len(self.transitions)>256:
                self.transitions.popitem(last=False)
        for axis, transition in enumerate(self.transitions[duration]):
            current = np.r_[state[1+axis*2:3+axis*2],state[5+axis*3:8+axis*3],command[axis+1]]
            value = transition@current
            result[1+axis*2:3+axis*2] = value[:2]
            result[5+axis*3:8+axis*3] = value[2:5]
        return result


    def commit(self, apply_ns, command):
        stamp, state, previous = self.records[-1]
        if apply_ns < stamp or np.asarray(command).shape!=(4,) or not np.isfinite(command).all():
            raise ValueError('Ordered finite applied command required')
        updated = self.advance(state, previous, (apply_ns-stamp)/1e9)
        self.records.append((int(apply_ns),updated,np.asarray(command).copy()))
        while len(self.records)>2 and apply_ns-self.records[1][0]>5_000_000_000:
            self.records.popleft()


    def at(self, stamp_ns):
        row = next((r for r in reversed(self.records) if r[0]<=stamp_ns),None)
        if row is None:
            raise ValueError('Observation predates command history')
        return self.advance(row[1],row[2],(stamp_ns-row[0])/1e9)


    def state(self, packet):
        latent=self.at(packet['stamp_ns'])
        if self.model.nx>30:
            payload=packet['payload']
            return np.r_[packet['vehicle'],latent,packet['hook_direction'],packet['hook_rate'],payload[3:7],payload[10:13]]
        return np.r_[packet['vehicle'],latent,packet['direction'],packet['cable_rate']]


    def forecast(self, packet, target_ns):
        start = packet['stamp_ns']
        if not 0 <= (target_ns-start)/1e9 <= .10:
            raise ValueError('Prediction interval exceeds100ms')
        state = self.state(packet)
        if self.model.vehicle_only:
            state[24:30] = [0.,0.,-1.,0.,0.,0.]
        command = next(r[2] for r in reversed(self.records) if r[0]<=start)
        points = [(r[0],r[2]) for r in self.records if start<r[0]<target_ns]+[(target_ns,None)]
        for stamp, next_command in points:
            duration = (stamp-start)/1e9
            count = max(1,int(np.ceil(duration/.01)))
            h = duration/count
            for _ in range(count):
                half = self.advance(state[13:24],command,h/2)
                end = self.advance(state[13:24],command,h)
                k1 = np.asarray(self.model.dynamics(state,command)).ravel()
                middle = state+h*k1/2; middle[13:24] = half
                k2 = np.asarray(self.model.dynamics(middle,command)).ravel()
                middle = state+h*k2/2; middle[13:24] = half
                k3 = np.asarray(self.model.dynamics(middle,command)).ravel()
                finish = state+h*k3; finish[13:24] = end
                k4 = np.asarray(self.model.dynamics(finish,command)).ravel()
                state += h*(k1+2*k2+2*k3+k4)/6
                state[13:24] = end
                state[3:7] /= np.linalg.norm(state[3:7])
                state[24:27] /= np.linalg.norm(state[24:27])
                state[27:30] -= state[24:27]*(state[24:27]@state[27:30])
                if self.model.nx>30:state[30:34]/=np.linalg.norm(state[30:34])
            start = stamp
            if next_command is not None:
                command = next_command
        return state
