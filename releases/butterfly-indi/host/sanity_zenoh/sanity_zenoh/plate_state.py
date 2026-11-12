from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from .mocap import MocapEstimator
from .cable_state import CommandObserver


class PlateMocap:
    def __init__(self, config, model):
        self.config=config;self.model=model;settings=config.mocap
        base=dict(parent=settings['parent_frame'],world_quat=settings['world_from_map_xyzw'],
            filter_tau_s=.03,angular_rate_filter_tau_s=.05,max_age_s=settings['maximum_age_s'],frames_confirmed=settings['frames_confirmed'])
        self.estimators=[MocapEstimator(child=settings['plate_frame'],marker_to_base_xyz=-model.config.mocap_offset,**base)]
        for drone in config.drones:self.estimators.append(MocapEstimator(child=drone['frame'],marker_to_base_xyz=drone['marker_to_body_xyz'],marker_to_base_quat=drone['marker_to_body_xyzw'],**base))
        self.buffers=[deque(maxlen=128) for _ in self.estimators]
        self.observers=None;self.ekf=None


    def update(self, parent, child, stamp_ns, position, quaternion, now_ns):
        for i,estimator in enumerate(self.estimators):
            if (parent,child)!=(estimator.parent,estimator.child):continue
            packet=estimator.update(parent,child,stamp_ns,position,quaternion,now_ns,now_ns)
            if packet and packet['ready']:self.buffers[i].append(packet)
            return packet
        return None


    def sample(self, now_ns, reset_on_gap=False):
        if any(not b for b in self.buffers):raise ValueError('Waiting for all three aircraft and plate mocap')
        stamps=[b[-1]['stamp_ns'] for b in self.buffers]
        if (max(stamps)-min(stamps))/1e9>self.config.mocap['maximum_skew_s']:raise ValueError('Plate/fleet source timestamp skew')
        aligned=min(stamps)
        if not 0<=(now_ns-aligned)/1e9<self.config.mocap['maximum_age_s']:raise ValueError('Plate/fleet mocap stale or future')
        states=[]
        for packets in self.buffers:
            before=next((p for p in reversed(packets) if p['stamp_ns']<=aligned),None)
            after=next((p for p in packets if p['stamp_ns']>=aligned),None)
            if before is None or after is None or after['stamp_ns']-before['stamp_ns']>40_000_000:raise ValueError('No common-time mocap interpolation bracket')
            a,b=np.asarray(before['state']),np.asarray(after['state'])
            fraction=(aligned-before['stamp_ns'])/max(1,after['stamp_ns']-before['stamp_ns'])
            state=a+(b-a)*fraction
            r=Rotation.from_quat(a[[4,5,6,3]]) if before is after else Slerp([0.,1.],Rotation.from_quat([a[[4,5,6,3]],b[[4,5,6,3]]]))([fraction])[0]
            state[3:7]=r.as_quat()[[3,0,1,2]];state[10:13]=r.inv().apply(state[10:13])
            states.append(state)
        if self.observers is None:self.observers=[CommandObserver(v,aligned) for v in self.model.vehicles]
        parts=[states[0]]
        for i,state in enumerate(states[1:]):parts.extend([state,self.observers[i].at(aligned)])
        raw=np.concatenate(parts)
        from experiments.sanity_plate_transport.flight_observer import FlightPoseEKF
        if self.ekf is None or (reset_on_gap and aligned-self.ekf.stamp>40_000_000):self.ekf=FlightPoseEKF(self.model,raw,aligned)
        if aligned>self.ekf.stamp:
            lengths=np.asarray(self.model.length(raw)).ravel()
            taut=bool(raw[2]>self.config.settings['floor_height_m']+.05 and np.all(np.abs(lengths-self.model.config.lengths)<.012))
            raw=self.ekf.observe(raw,aligned,self.forecast_segments(self.ekf.stamp,aligned),taut)
        else:raw=self.ekf.state.copy()
        return dict(stamp_ns=int(aligned),state=raw,source_age_s=(now_ns-aligned)/1e9,pair_skew_s=(max(stamps)-min(stamps))/1e9,observer=dict(self.ekf.last))


    def commit(self, apply_ns, selected):
        if self.observers is None:return
        selected=np.asarray(selected).reshape(-1,4)
        for i,observer in enumerate(self.observers):observer.commit(apply_ns,selected[i])


    def forecast_segments(self, start_ns, end_ns):
        if self.observers is None or not 0<=end_ns-start_ns<=100_000_000:raise ValueError('Bounded forecast with initialized command history required')
        points=sorted({start_ns,end_ns,*[r[0] for observer in self.observers for r in observer.records if start_ns<r[0]<end_ns]})
        result=[]
        for a,b in zip(points[:-1],points[1:]):
            commands=[]
            for observer in self.observers:
                row=next((r for r in reversed(observer.records) if r[0]<=a),None)
                if row is None:raise ValueError('Forecast predates selected-command history')
                commands.extend(row[2])
            result.append([(b-a)/1e9,commands])
        return result
