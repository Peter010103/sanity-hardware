"""Causal, bounded world-XY acceleration-bias estimate from settled-flight residuals."""
from collections import deque
import math

import numpy as np
from scipy.spatial.transform import Rotation

DEFAULTS = {'enabled':False,'time_constant_s':2.,'window_s':.25,'max_bias_m_s2':.6,
    'innovation_limit_m_s2':2.,'max_speed_m_s':.3,'max_body_rate_rad_s':.5,
    'max_tilt_deg':10.,'min_height_m':.3,'max_reference_speed_m_s':.05,
    'max_reference_acceleration_m_s2':.1}


def validate_configuration(configuration):
    if not isinstance(configuration,dict) or set(configuration)-set(DEFAULTS):
        raise ValueError('Invalid disturbance_compensation configuration')
    result={**DEFAULTS,**configuration}
    if type(result['enabled']) is not bool:raise ValueError('Disturbance enabled must be boolean')
    for key,value in result.items():
        if key!='enabled' and (not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0):
            raise ValueError('Positive finite disturbance setting required: '+key)
    if not (.5<=result['time_constant_s']<=10 and .1<=result['window_s']<=1
            and .05<=result['max_bias_m_s2']<=2 and result['max_bias_m_s2']<=result['innovation_limit_m_s2']<=10
            and result['max_tilt_deg']<=30 and result['max_body_rate_rad_s']<=5):
        raise ValueError('Disturbance estimator settings outside supported bounds')
    return result


def bounded_xy(vector,limit):
    value=np.r_[np.asarray(vector,dtype=float)[:2],0.]
    norm=np.linalg.norm(value[:2])
    return value*(min(1.,limit/norm) if norm else 1.)


class HorizontalAccelerationBias:
    def __init__(self,configuration,mass,gravity=9.81,drag=(0.,0.,0.)):
        self.config=validate_configuration(configuration)
        self.mass=float(mass);self.gravity=float(gravity);self.drag=np.asarray(drag,dtype=float)
        if not 0<self.mass<1 or not np.isfinite(self.gravity) or not self.gravity>0 or self.drag.shape!=(3,) or not np.isfinite(self.drag).all() or np.any(self.drag<0):
            raise ValueError('Invalid nominal observer model')
        self.reset()

    def reset(self):
        self.estimate=np.zeros(3);self.previous=None;self.commands=deque(maxlen=512)
        self.residual=np.zeros(3);self.elapsed=0.;self.updates=0
        self.last={'enabled':self.config['enabled'],'learning':False,'reason':'initializing',
            'estimate_world_m_s2':[0.,0.,0.],'updates':0}

    def commit(self,stamp_ns,ctbr,learning_allowed):
        command=np.asarray(ctbr,dtype=float)
        if command.shape!=(4,) or not np.isfinite(command).all():raise ValueError('Finite selected CTBR required by bias observer')
        if stamp_ns is None:return
        stamp_ns=int(stamp_ns)
        if stamp_ns<=0 or (self.commands and stamp_ns<=self.commands[-1][0]):
            raise ValueError('Selected-command clock must increase')
        self.commands.append((stamp_ns,command.copy(),bool(learning_allowed)))
        while len(self.commands)>2 and self.commands[1][0]<stamp_ns-2_000_000_000:self.commands.popleft()

    def gated(self,state,context):
        c=self.config
        if not c['enabled']:return 'disabled'
        if not context or not context.get('allow_learning',False):return 'mission_phase_or_disabled_output'
        if context.get('height_above_ground_m',0.)<c['min_height_m']:return 'near_ground'
        if np.linalg.norm(state[7:10])>c['max_speed_m_s']:return 'moving'
        if np.linalg.norm(state[10:13])>c['max_body_rate_rad_s']:return 'rotating'
        if np.degrees(np.arccos(np.clip(1-2*(state[4]**2+state[5]**2),-1.,1.)))>c['max_tilt_deg']:return 'tilted'
        if np.linalg.norm(context.get('reference_velocity',[0.]*3))>c['max_reference_speed_m_s']:return 'moving_reference'
        if np.linalg.norm(context.get('reference_acceleration',[0.]*3))>c['max_reference_acceleration_m_s2']:return 'accelerating_reference'
        return None

    def predicted_delta_velocity(self,start,end,before,after):
        preceding=[item for item in self.commands if item[0]<=start]
        if not preceding:return None,'missing_selected_command'
        active=preceding[-1]
        changes=[item for item in self.commands if start<item[0]<end]
        points=changes+[(end,None,False)]
        integral=0.;cursor=start
        for point in points:
            _,command,allowed=active
            if not allowed:return None,'selected_command_gated'
            if (not .5*self.mass*self.gravity<command[0]<1.5*self.mass*self.gravity
                    or np.linalg.norm(command[1:])>self.config['max_body_rate_rad_s']):
                return None,'selected_command_transient'
            integral += command[0]*(point[0]-cursor)/1e9
            cursor=point[0];active=point
        a=Rotation.from_quat(before[[4,5,6,3]]).as_matrix()
        b=Rotation.from_quat(after[[4,5,6,3]]).as_matrix()
        dt=(end-start)/1e9
        drag=.5*(a@(self.drag*(a.T@before[7:10]))+b@(self.drag*(b.T@after[7:10])))
        return .5*(a[:,2]+b[:,2])*integral/self.mass-(np.array([0.,0.,self.gravity])+drag)*dt,None

    def observe(self,state,stamp_ns,context=None):
        state=np.asarray(state,dtype=float)
        if state.shape!=(13,) or not np.isfinite(state).all():raise ValueError('Finite13-state input required by bias observer')
        if abs(np.linalg.norm(state[3:7])-1.)>.01:raise ValueError('Unit quaternion required by bias observer')
        if context:
            for key in ('height_above_ground_m','reference_velocity','reference_acceleration'):
                if key in context and not np.isfinite(np.asarray(context[key],dtype=float)).all():
                    raise ValueError('Finite disturbance context required: '+key)
            for key in ('reference_velocity','reference_acceleration'):
                if key in context and np.asarray(context[key]).shape!=(3,):
                    raise ValueError('Three-axis reference required: '+key)
        if stamp_ns is not None and int(stamp_ns)<=0:raise ValueError('Positive source timestamp required')
        reason=self.gated(state,context)
        self.last={'enabled':self.config['enabled'],'learning':False,'reason':reason or 'collecting',
            'estimate_world_m_s2':self.estimate.tolist(),'updates':self.updates}
        if stamp_ns is None:
            self.residual[:]=0.;self.elapsed=0.;self.previous=None
            self.last['reason']='missing_source_timestamp';return self.estimate.copy()
        stamp_ns=int(stamp_ns)
        previous=self.previous
        if previous is not None and stamp_ns==previous[0]:
            self.last['reason']='repeated_source_timestamp';return self.estimate.copy()
        self.previous=(stamp_ns,state.copy(),reason is None)
        if reason or previous is None or not previous[2]:
            self.residual[:]=0.;self.elapsed=0.;return self.estimate.copy()
        dt=(stamp_ns-previous[0])/1e9
        if not 0<dt<=.04:
            self.residual[:]=0.;self.elapsed=0.;self.last['reason']='source_clock_gap';return self.estimate.copy()
        predicted,reason=self.predicted_delta_velocity(previous[0],stamp_ns,previous[1],state)
        if reason:
            self.residual[:]=0.;self.elapsed=0.;self.last['reason']=reason;return self.estimate.copy()
        self.residual += state[7:10]-previous[1][7:10]-predicted
        self.elapsed += dt
        self.last.update(learning=True,reason='collecting',window_elapsed_s=self.elapsed)
        if self.elapsed>=self.config['window_s']:
            raw=bounded_xy(self.residual/self.elapsed,self.config['innovation_limit_m_s2'])
            alpha=-math.expm1(-self.elapsed/self.config['time_constant_s'])
            self.estimate=bounded_xy(self.estimate+alpha*(raw-self.estimate),self.config['max_bias_m_s2'])
            self.updates+=1
            self.last.update(reason='updated',innovation_world_m_s2=raw.tolist(),
                estimate_world_m_s2=self.estimate.tolist(),updates=self.updates)
            self.residual[:]=0.;self.elapsed=0.
        return self.estimate.copy()
