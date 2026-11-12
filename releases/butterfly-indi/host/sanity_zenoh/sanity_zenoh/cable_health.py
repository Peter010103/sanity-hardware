import math

import numpy as np
from scipy.spatial.transform import Rotation


class PayloadHealth:
    def __init__(self, options=None):
        self.enabled=options is not None
        options=options or {}
        if set(options)-{'maximum_internal_tilt_deg','maximum_transverse_rate_rms_rad_s','rate_filter_tau_s','dwell_s'}:raise ValueError('Unknown payload health setting')
        self.tilt_limit=float(options.get('maximum_internal_tilt_deg',20.))
        self.rate_limit=float(options.get('maximum_transverse_rate_rms_rad_s',3.))
        self.tau=float(options.get('rate_filter_tau_s',.1));self.dwell=float(options.get('dwell_s',.2))
        if not all(math.isfinite(v) for v in [self.tilt_limit,self.rate_limit,self.tau,self.dwell]) or not 10<=self.tilt_limit<=30 or not 1.5<=self.rate_limit<=5. or not .05<=self.tau<=.2 or not .1<=self.dwell<=.3:raise ValueError('Invalid payload rotation health limits')
        self.energy=0.;self.last_source=None;self.bad_since=None;self.diagnostic={'available':False,'healthy':False,'reason':None}


    def update(self, packet, now_ns):
        source=packet.get('stamp_ns',0);payload=np.asarray(packet.get('payload',[]),dtype=float)
        available=payload.shape==(13,) and np.isfinite(payload).all() and 0<=now_ns-source<40_000_000 and not packet.get('payload_orientation_held',False)
        if not available:
            self.last_source=None;self.bad_since=None;self.energy=0.
            self.diagnostic={'available':False,'healthy':False,'reason':None};return self.diagnostic
        if self.last_source is not None and source==self.last_source:
            self.diagnostic={**self.diagnostic,'available':False,'healthy':False,'reason':None};return self.diagnostic
        if self.last_source is not None and not 0<source-self.last_source<=40_000_000:
            self.last_source=None;self.bad_since=None;self.energy=0.
            self.diagnostic={'available':False,'healthy':False,'reason':None};return self.diagnostic
        axis=Rotation.from_quat(payload[[4,5,6,3]]).apply([0.,0.,1.])
        relative=payload[10:13]-np.asarray(packet.get('hook_rate',packet.get('cable_rate',[0.,0.,0.])))
        transverse=float(np.linalg.norm(relative-axis*(axis@relative)))
        dt=.02 if self.last_source is None else (source-self.last_source)/1e9
        self.energy+=-math.expm1(-dt/self.tau)*(transverse**2-self.energy)
        rate=math.sqrt(max(0.,self.energy));tilt=float(packet.get('payload_internal_tilt_deg',180.))
        bad=not math.isfinite(tilt) or tilt>self.tilt_limit or rate>self.rate_limit
        self.bad_since=(now_ns if self.bad_since is None else self.bad_since) if bad else None
        elapsed=0. if self.bad_since is None else (now_ns-self.bad_since)/1e9
        reason=None
        if bad and elapsed>=self.dwell:reason=f'Payload rotation health: internal tilt={tilt:.1f}deg, relative transverse rate RMS={rate:.2f}rad/s for{elapsed:.2f}s'
        self.last_source=source
        self.diagnostic={'available':True,'healthy':not bad,'internal_tilt_deg':tilt,'relative_transverse_rate_rad_s':transverse,'relative_transverse_rate_rms_rad_s':rate,'bad_dwell_s':elapsed,'reason':reason}
        return self.diagnostic


class VoltageGuard:
    def __init__(self, options=None):
        self.enabled=options is not None
        options=options or {}
        if set(options)-{'minimum_start_V','landing_below_V','dwell_s'}:raise ValueError('Unknown flight voltage setting')
        self.minimum_start=float(options.get('minimum_start_V',7.6));self.floor=float(options.get('landing_below_V',7.))
        self.dwell=float(options.get('dwell_s',.5));self.bad_since=None
        if not all(math.isfinite(v) for v in [self.minimum_start,self.floor,self.dwell]) or not 6.8<=self.floor<self.minimum_start<=8.2 or not .2<=self.dwell<=1.:raise ValueError('Invalid2S flight voltage guard')
        self.diagnostic={}


    def start_ready(self, voltage):
        return voltage is not None and math.isfinite(voltage) and self.minimum_start<=voltage<=8.5


    def update(self, voltage, now_ns):
        bad=voltage is None or not math.isfinite(voltage) or voltage<self.floor
        self.bad_since=(now_ns if self.bad_since is None else self.bad_since) if bad else None
        elapsed=0. if self.bad_since is None else (now_ns-self.bad_since)/1e9
        reason=f'FC battery voltage {voltage}V below flight threshold {self.floor:.2f}V for{elapsed:.2f}s' if bad and elapsed>=self.dwell else None
        self.diagnostic={'reported_voltage_V':voltage,'bad_dwell_s':elapsed,'reason':reason}
        return reason


class MissionFailureGuard:
    def __init__(self, options=None):
        self.enabled=options is not None
        options=options or {}
        if set(options)-{'maximum_tracking_error_m','maximum_swing_deg','maximum_body_tilt_deg','dwell_s'}:raise ValueError('Unknown mission failure guard setting')
        self.limits={'tracking_error_m':float(options.get('maximum_tracking_error_m',.20)),
                     'swing_deg':float(options.get('maximum_swing_deg',45.)),
                     'body_tilt_deg':float(options.get('maximum_body_tilt_deg',35.))}
        self.dwell=float(options.get('dwell_s',.20))
        if not all(math.isfinite(value) for value in [*self.limits.values(),self.dwell]) or not .10<=self.limits['tracking_error_m']<=.30 or not 15.<=self.limits['swing_deg']<=45. or not 10.<=self.limits['body_tilt_deg']<=35. or not .05<=self.dwell<=.25:
            raise ValueError('Mission failure limits require tracking10..30cm,swing15..45deg,body tilt10..35deg,dwell50..250ms')
        self.bad_since={key:None for key in self.limits};self.diagnostic={'reason':None}


    def update(self, packet, target_position, now_ns):
        vehicle=np.asarray(packet['vehicle'],dtype=float);target=np.asarray(target_position,dtype=float)
        if vehicle.shape!=(13,) or target.shape!=(3,) or not np.isfinite(vehicle).all() or not np.isfinite(target).all():raise ValueError('Finite vehicle state and timestamp-aligned target required for mission guard')
        rotation=Rotation.from_quat(vehicle[[4,5,6,3]])
        values={'tracking_error_m':float(np.linalg.norm(vehicle[:3]-target)),
                'swing_deg':float(packet.get('swing_deg',0.)),
                'body_tilt_deg':float(np.degrees(np.arccos(np.clip(rotation.as_matrix()[2,2],-1.,1.))))}
        elapsed={};triggered=[]
        for key,limit in self.limits.items():
            bad=not math.isfinite(values[key]) or values[key]>limit
            self.bad_since[key]=(int(now_ns) if self.bad_since[key] is None else self.bad_since[key]) if bad else None
            elapsed[key]=0. if self.bad_since[key] is None else (now_ns-self.bad_since[key])/1e9
            if self.enabled and bad and now_ns-self.bad_since[key]>=round(self.dwell*1e9):triggered.append(key)
        reason=('Mission failure threshold: '+', '.join(f'{key}={values[key]:.3f}>{self.limits[key]:.3f}' for key in triggered)+f' for{self.dwell:.2f}s; PD landing') if triggered else None
        self.diagnostic={**values,'limits':self.limits.copy(),'bad_dwell_s':elapsed,'triggered':triggered,'reason':reason,
                         'target_position':target.tolist(),'source_stamp_ns':int(packet['stamp_ns'])}
        return self.diagnostic
