"""Timestamped, calibrated TF poses to the policy's root-state contract."""

import math
import numpy as np
from scipy.spatial.transform import Rotation


class MocapOutlier(ValueError):
    def __init__(self, stamp_ns, previous_stamp_ns, speed_m_s, angular_rate_rad_s):
        super().__init__("Implausible mocap sample rejected")
        self.diagnostics = {"stamp_ns":int(stamp_ns), "previous_stamp_ns":int(previous_stamp_ns),
            "sample_gap_ms":(stamp_ns-previous_stamp_ns)/1e6,
            "speed_m_s":float(speed_m_s), "angular_rate_rad_s":float(angular_rate_rad_s)}


class MocapEstimator:
    def __init__(self, parent="map", child="sanity09", world_quat=(0., 0., 0., 1.),
                 marker_to_base_xyz=(0., 0., 0.), marker_to_base_quat=(0., 0., 0., 1.),
                 filter_tau_s=.02, max_age_s=.04, frames_confirmed=False, max_future_s=.002, angular_rate_filter_tau_s=None, position_filter_tau_s=0., attitude_filter_tau_s=0., max_speed_m_s=10., position_innovation_m=None, max_acceleration_m_s2=40., orientation_hold_s=0.):
        if not parent or not child or parent == child:
            raise ValueError("Distinct parent and tracked child frames required")
        self.parent, self.child = parent, child
        self.world_rotation = self.rotation(world_quat)
        self.marker_rotation = self.rotation(marker_to_base_quat)
        self.offset = np.asarray(marker_to_base_xyz, dtype=float)
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
            raise ValueError("Marker-to-base translation must be three finite metres")
        angular_tau = filter_tau_s if angular_rate_filter_tau_s is None else angular_rate_filter_tau_s
        if not math.isfinite(angular_tau) or angular_tau < 0 or not math.isfinite(filter_tau_s) or filter_tau_s < 0 or not 0 < max_age_s <= .04 or not math.isfinite(max_future_s) or not 0 <= max_future_s <= .002:
            raise ValueError("Invalid estimator filter or freshness limit")
        if not all(math.isfinite(v) and 0<=v<=.04 for v in (position_filter_tau_s,attitude_filter_tau_s)):
            raise ValueError("Pose filter times must be finite and in0..40ms")
        self.position_tau,self.attitude_tau=position_filter_tau_s,attitude_filter_tau_s
        self.pose_position=None;self.pose_rotation=None;self.pose_delay=np.zeros(2)
        self.tau, self.angular_tau, self.max_age = filter_tau_s, angular_tau, max_age_s
        self.max_future = max_future_s
        self.frames_confirmed = bool(frames_confirmed)
        if not all(math.isfinite(v) and v>=0 for v in (max_speed_m_s,max_acceleration_m_s2,orientation_hold_s)) or max_speed_m_s<=0 or orientation_hold_s>max_age_s or (position_innovation_m is not None and (not math.isfinite(position_innovation_m) or position_innovation_m<=0)):
            raise ValueError('Invalid mocap innovation or orientation hold limits')
        self.max_speed=max_speed_m_s;self.position_innovation=position_innovation_m;self.max_acceleration=max_acceleration_m_s2
        self.orientation_hold_s=orientation_hold_s;self.orientation_stamp=None
        self.previous = None
        self.count = 0
        self.velocity = np.zeros(3)
        self.omega = np.zeros(3)

    @staticmethod
    def rotation(quaternion):
        q = np.asarray(quaternion, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1.) > 1e-3:
            raise ValueError("Expected a unit xyzw quaternion")
        return Rotation.from_quat(q)

    def update(self, parent, child, stamp_ns, position, quaternion, ros_now_ns, receipt_ns):
        if (parent, child) != (self.parent, self.child):
            return None
        age = (ros_now_ns - stamp_ns) / 1e9
        if stamp_ns <= 0 or not -self.max_future <= age < self.max_age:
            raise ValueError(f"Mocap stamp is stale/future (age {age*1000:.3f}ms; future tolerance {self.max_future*1000:.1f}ms); check both computers' clocks")
        # Repeated frozen transforms must not refresh the local receipt clock.
        if self.previous is not None and stamp_ns <= self.previous[0]:
            return None
        marker = self.rotation(quaternion)
        raw_position = np.asarray(position, dtype=float)
        if raw_position.shape != (3,) or not np.isfinite(raw_position).all():
            raise ValueError("Non-finite mocap translation")
        rotation = self.world_rotation * marker * self.marker_rotation
        point = self.world_rotation.apply(raw_position + marker.apply(self.offset))
        dt = (stamp_ns - self.previous[0]) / 1e9 if self.previous is not None else math.inf
        orientation_held=False
        if dt > self.max_age:
            self.orientation_stamp=stamp_ns
            self.count = 1
            self.velocity[:] = 0
            self.omega[:] = 0
        else:
            raw_velocity = (point - self.previous[1]) / dt
            raw_omega = (rotation * self.previous[2].inv()).as_rotvec() / dt
            speed, angular_rate = np.linalg.norm(raw_velocity), np.linalg.norm(raw_omega)
            innovation=np.linalg.norm(point-self.previous[1]-self.velocity*dt)
            if speed>self.max_speed or (self.position_innovation is not None and self.count>=5 and innovation>self.position_innovation+.5*self.max_acceleration*dt**2):
                error=MocapOutlier(stamp_ns,self.previous[0],speed,angular_rate)
                error.diagnostics.update(component='position',innovation_m=float(innovation));raise error
            if angular_rate>30:
                if self.orientation_hold_s and self.orientation_stamp is not None and (stamp_ns-self.orientation_stamp)/1e9<=self.orientation_hold_s and np.linalg.norm(self.offset)<1e-9:
                    rotation=self.previous[2];raw_omega=self.omega.copy();orientation_held=True
                else:
                    error=MocapOutlier(stamp_ns,self.previous[0],speed,angular_rate)
                    error.diagnostics.update(component='orientation');raise error
            else:self.orientation_stamp=stamp_ns
            alpha = 1. if not self.tau or self.count == 1 else -math.expm1(-dt / self.tau)
            angular_alpha = 1. if not self.angular_tau or self.count == 1 else -math.expm1(-dt / self.angular_tau)
            self.velocity += alpha * (raw_velocity - self.velocity)
            self.omega += angular_alpha * (raw_omega - self.omega)
            self.count += 1
        if dt>self.max_age:
            self.pose_position=point.copy();self.pose_rotation=rotation;self.pose_delay[:]=0.
        else:
            position_alpha=-math.expm1(-dt/self.position_tau) if self.position_tau else 1.
            attitude_alpha=-math.expm1(-dt/self.attitude_tau) if self.attitude_tau else 1.
            self.pose_position=(self.pose_position+position_alpha*(point-self.pose_position)) if self.position_tau else point.copy()
            self.pose_rotation=(self.pose_rotation*Rotation.from_rotvec(attitude_alpha*(self.pose_rotation.inv()*rotation).as_rotvec())) if self.attitude_tau else rotation
            self.pose_delay=(1-np.array([position_alpha,attitude_alpha]))*(self.pose_delay+dt)
        self.previous = (stamp_ns, point, rotation)
        return {"version": 1, "stamp_ns": int(stamp_ns), "receipt_ns": int(receipt_ns),
                "orientation_held":orientation_held,"orientation_stamp_ns":int(self.orientation_stamp),"source_age_s": max(age,0.), "raw_source_age_s":age, "future_clock_skew_s":max(-age,0.), "ready": self.count >= 5,
                "frames_confirmed": self.frames_confirmed, "tracked_frame": child,
                "filter_tau_s": {"linear_velocity": self.tau, "angular_rate": self.angular_tau},
                "pose_filter_tau_s":{"position":self.position_tau,"attitude":self.attitude_tau},
                "pose_filter_delay_s":{"position":float(self.pose_delay[0]),"attitude":float(self.pose_delay[1])},
                "state": [*self.pose_position.tolist(), *np.roll(self.pose_rotation.as_quat(), 1).tolist(),
                          *self.velocity.tolist(), *self.omega.tolist()]}


def state_age(packet, now_ns, limit=.04):
    if packet.get("version") != 1 or packet.get("ready") is not True:
        raise ValueError(packet.get("error", "Mocap estimator warming up"))
    age = float(packet["source_age_s"]) + (now_ns - int(packet["receipt_ns"])) / 1e9
    if not math.isfinite(age) or not 0 <= age < limit or now_ns < packet["receipt_ns"]:
        raise ValueError("Mocap state expired")
    state = np.asarray(packet["state"], dtype=float)
    if state.shape != (13,) or not np.isfinite(state).all() or abs(np.linalg.norm(state[3:7]) - 1) > 1e-3:
        raise ValueError("Invalid policy root state")
    return state, age
