from collections import deque
import math

import numpy as np
from scipy.spatial.transform import Rotation


class HoverCableLength:
    def __init__(self, nominal, options=None):
        options = options or {}
        if set(options)-{'enabled', 'window_s', 'timeout_s', 'maximum_adjustment_m', 'maximum_spread_m', 'capture_bounds_m'}:
            raise ValueError('Unknown hover cable-length option')
        self.enabled = options.get('enabled', False)
        if type(self.enabled) is not bool:raise ValueError('Cable-length calibration enabled must be boolean')
        self.nominal = float(nominal)
        self.window = float(options.get('window_s', 2.))
        self.timeout = float(options.get('timeout_s', 12.))
        self.adjustment = float(options.get('maximum_adjustment_m', .03))
        self.spread = float(options.get('maximum_spread_m', .003))
        if not all(math.isfinite(v) for v in (self.nominal, self.window, self.timeout, self.adjustment, self.spread)) or not .1 <= self.nominal <= 2. or not 2. <= self.window <= 5. or not self.window+2 <= self.timeout <= 20. or not .005 <= self.adjustment <= min(.05, .1*self.nominal) or not .001 <= self.spread <= .005:
            raise ValueError('Invalid stable-hover cable calibration limits')
        self.bounds = [round(self.nominal-self.adjustment, 9), round(self.nominal+self.adjustment, 9)]
        if 'capture_bounds_m' in options:
            bounds=options['capture_bounds_m']
            if not isinstance(bounds,(list,tuple)) or len(bounds)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in bounds) or not .1<=bounds[0]<self.nominal<bounds[1]<=2.:
                raise ValueError('Explicit cable capture bounds must be finite 0.1..2m and bracket the nominal length')
            self.bounds=list(map(float,bounds))
        self.samples = deque()
        self.last_source = self.last_now = None
        self.length = None
        self.diagnostic = {'ready': False, 'blockers': ['waiting_for_loaded_hover']}


    def update(self, packet, now_ns, target, floor):
        if self.length is not None:return self.length
        source = int(packet['stamp_ns'])
        blockers = []
        if not 0 <= now_ns-source < 40_000_000:blockers.append('stale_observation')
        if self.last_source is not None and not 0 < source-self.last_source <= 40_000_000:blockers.append('source_gap_or_repetition')
        if self.last_now is not None and not 0 < now_ns-self.last_now <= 40_000_000:blockers.append('control_clock_gap')
        vehicle = np.asarray(packet['vehicle'])
        payload = np.asarray(packet.get('payload', []))
        length = packet.get('hook_distance_m')
        if not np.isfinite(vehicle).all() or payload.shape != (13,) or not np.isfinite(payload).all():blockers.append('missing_or_invalid_pose')
        if length is None or not math.isfinite(length) or not self.bounds[0] <= length <= self.bounds[1]:blockers.append('length_outside_capture_range')
        if np.linalg.norm(vehicle[:3]-target) >= .08 or np.linalg.norm(vehicle[7:10]) >= .08:blockers.append('aircraft_not_settled')
        tilt = math.degrees(math.acos(float(np.clip(Rotation.from_quat(vehicle[[4,5,6,3]]).as_matrix()[2,2], -1, 1))))
        if tilt >= 6.:blockers.append('aircraft_tilt')
        if payload.shape == (13,) and np.linalg.norm(payload[7:10]) >= .08:blockers.append('payload_not_settled')
        if packet.get('swing_deg', 180.) >= 8.:blockers.append('payload_swing')
        if packet.get('cube_lowest_height_m', floor)-floor < .15:blockers.append('payload_not_free_hanging')
        if packet.get('payload_orientation_held', False):blockers.append('payload_orientation_held')
        if blockers:self.samples.clear()
        else:self.samples.append((source, float(length)))
        self.last_source, self.last_now = source, int(now_ns)
        while len(self.samples)>1 and self.samples[1][0] <= source-int(self.window*1e9):self.samples.popleft()
        covered = (self.samples[-1][0]-self.samples[0][0])/1e9 if self.samples else 0.
        values = np.array([value for stamp, value in self.samples])
        variation = float(np.ptp(values)) if len(values) else None
        if covered < self.window or len(values)<80:blockers.append('stable_window_warming')
        if variation is not None and variation > self.spread:blockers.append('length_not_stable')
        if not blockers:self.length = float(np.median(values))
        self.diagnostic = {'ready': self.length is not None, 'blockers': blockers, 'covered_s': covered, 'samples': len(values),
            'spread_m': variation, 'median_m': float(np.median(values)) if len(values) else None, 'capture_bounds_m': self.bounds,
            'nominal_m': self.nominal, 'calibrated_m': self.length, 'method': 'Median equivalent hook length during stable, freely hanging PID hover; frozen for this flight'}
        return self.length
