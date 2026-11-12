"""Ideal-rate PID state projection using only already-selected commands.

No INDI poles or latent rate states. The comparison MPC keeps ideal PID rate
tracking; measured-source age,20ms asynchronous selection and15ms nominal
output delay are still handled explicitly, without relaxing freshness.
"""
from bisect import bisect_right
import math
import numpy as np
from scipy.spatial.transform import Rotation

class IdealCommandPrediction:
    def __init__(self,mass,thrust_effectiveness=1.,drag=(0.,0.,0.),command_delay_s=.015):
        self.mass=float(mass);self.eta=float(thrust_effectiveness);self.drag=np.asarray(drag,dtype=float)
        self.command_delay_s=float(command_delay_s)
        if not 0<self.mass<1 or not .8<=self.eta<=1.2 or self.drag.shape!=(3,) or not np.isfinite(self.drag).all() or np.any(self.drag<0) or not 0<=self.command_delay_s<=.03:
            raise ValueError('Finite physical ideal-PID projection settings required')
        self.commands=[];self.last={}
    def reset(self):self.commands=[];self.last={}
    def commit(self,stamp_ns,command):
        u=np.asarray(command,dtype=float)
        if type(stamp_ns) is not int or stamp_ns<=0 or u.shape!=(4,) or not np.isfinite(u).all():raise ValueError('Typed time and finite selected CTBR4 required')
        effective=stamp_ns+round(self.command_delay_s*1e9)
        if self.commands and effective<=self.commands[-1][0]:raise ValueError('Selected timestamps must increase')
        self.commands.append((effective,u.copy()))
        while len(self.commands)>2 and self.commands[1][0]<effective-2_000_000_000:self.commands.pop(0)
    def held(self,stamp):
        index=bisect_right([t for t,_ in self.commands],stamp)-1
        return self.commands[index][1] if index>=0 else np.zeros(4)
    def derivative(self,x,u):
        q=x[3:7];w=u[1:4]
        qdot=.5*np.r_[-np.dot(q[1:],w),q[0]*w+np.cross(q[1:],w)]
        rotation=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        a=rotation[:,2]*(self.eta*u[0]/self.mass)-[0.,0.,9.81]-rotation@(self.drag*(rotation.T@x[7:10]))
        return np.r_[x[7:10],qdot,a]
    def predict(self,state,source_stamp_ns,target_stamp_ns):
        s=np.asarray(state,dtype=float)
        if s.shape!=(13,) or not np.isfinite(s).all() or abs(np.linalg.norm(s[3:7])-1)>.001:raise ValueError('Finite measured state13 required')
        if any(type(t) is not int or t<=0 for t in (source_stamp_ns,target_stamp_ns)):raise ValueError('Typed positive prediction timestamps required')
        duration=(target_stamp_ns-source_stamp_ns)*1e-9
        if not 0<=duration<=.060001+self.command_delay_s:raise ValueError('Projection requires fresh40ms source plus explicit selection/delay window')
        x=s[:10].copy()
        times=[source_stamp_ns]+[t for t,_ in self.commands if source_stamp_ns<t<target_stamp_ns]+[target_stamp_ns]
        for begin,end in zip(times,times[1:]):
            span=(end-begin)*1e-9;steps=max(1,math.ceil(span/.005));h=span/steps;u=self.held(begin)
            for _ in range(steps):
                k1=self.derivative(x,u);k2=self.derivative(x+h*k1/2,u);k3=self.derivative(x+h*k2/2,u);k4=self.derivative(x+h*k3,u)
                x+=h*(k1+2*k2+2*k3+k4)/6;x[3:7]/=np.linalg.norm(x[3:7])
        body_rate=self.held(target_stamp_ns)[1:4];world_rate=Rotation.from_quat(x[[4,5,6,3]]).apply(body_rate)
        predicted=np.r_[x,world_rate]
        self.last=dict(mode='ideal_PID_selected_command_projection',source_stamp_ns=source_stamp_ns,target_stamp_ns=target_stamp_ns,
            projection_s=duration,nominal_command_delay_s=self.command_delay_s,selected_history_only=True,
            rate_model_state=None,source_state=s.tolist(),predicted_state=predicted.tolist())
        return predicted,None
