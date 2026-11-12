# Frozen/adapted from isolated flip preparation simulate_indi_flip.py Plant.
# Change: configurable assumed yaw lag/gain for sensitivity only.
"""Finite offline full-attitude MPC screen with an independent numpy plant.

Measured small-angle rate model, collective lag, motor allocation/headroom and
15ms held-command delay. Nonlinear high-rate INDI/RPM servo remain unvalidated.
"""
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation

class Plant:
    def __init__(self,cfg,reference,*,mass_scale=1.,bandwidth_scale=1.,delay=.015,bias=(0.,0.,0.)):
        self.bias=np.asarray(bias);self.cfg=cfg;self.mass=cfg['mass_kg']*mass_scale;self.I=np.array(cfg['inertia_kg_m2']);self.allocation=np.array(cfg['allocation_per_m'])
        self.state=reference.sample(0.)[:14].copy();self.rate_state=np.zeros((2,2));self.delay=delay
        self.history=[(-1.,reference.sample(0.)[14:18].copy())];self.time=0.;self.saturations=0;self.ff_clips=0;self.max_torque_ratio=0.;self.rate_ff=np.zeros(3);self.last_delayed=np.array([cfg['mass_kg']*9.81,0.,0.,0.])
        profile=json.loads(Path(cfg['flight_rate_profile']).read_text())
        self.filtered_velocity=self.state[7:10].copy();self.filtered_rate=np.zeros(3)
        self.samples=[(-.01,self.state.copy(),self.filtered_velocity.copy(),self.filtered_rate.copy())]
        self.coef=[]
        for axis in ('roll','pitch'):
            a=profile['axes'][axis];self.coef.append((2*np.pi*a['natural_frequency_hz']*bandwidth_scale,a['damping_ratio'],a['zero_time_s']))
    def advance(self,command,duration=.02):
        self.history.append((self.time,np.asarray(command).copy()))
        for _ in range(round(duration/.001)):
            h=.001;t=self.time
            chosen=max(0,np.searchsorted([a[0] for a in self.history],t-self.delay,side='right')-1)
            u=self.history[chosen][1];r=self.state[10:13].copy();rnext=r.copy()
            for axis,(wn,zeta,zero) in enumerate(self.coef):
                a,b=self.rate_state[axis];dd=wn*wn*(u[axis+1]-a)-2*zeta*wn*b
                # Exact output definition matches the second-order + zero model.
                a+=h*b;b+=h*dd;self.rate_state[axis]=[a,b];rnext[axis]=a+zero*b
            rnext[2]+=h*(u[3]*self.cfg.get('yaw_gain',1.)-r[2])/self.cfg.get('yaw_tau_s',.125)
            derivative=(u[1:]-self.last_delayed[1:])/h;self.last_delayed=u.copy()
            self.rate_ff+=(h*2*np.pi*20/(1+h*2*np.pi*20))*(.25*derivative-self.rate_ff)
            self.ff_clips+=int(np.any(abs(self.rate_ff[:2])>20.))
            ff_excess=self.rate_ff-np.clip(self.rate_ff,-20.,20.);ff_excess[2]=0.
            # Bounded-FF proxy: remove the acceleration contribution lost at
            # the exact native PT1/beta/cap, then retain measured closed-loop
            # rate dynamics and physical allocation. This is not the FC core.
            requested_torque=self.I@((rnext-r)/h-ff_excess)+np.cross(r,self.I@r)
            torque=np.clip(requested_torque,-np.array(self.cfg['torque_limits_Nm']),self.cfg['torque_limits_Nm'])
            fmin,fmax=self.cfg['physical_force_range_N'];force=self.state[13]
            capacity=self.cfg['torque_reserve_fraction']*max(0,min(force-fmin,fmax-force))/4
            deviations=self.allocation@torque;ratio=np.max(abs(deviations))/max(capacity,1e-9)
            self.max_torque_ratio=max(self.max_torque_ratio,float(ratio))
            scale=min(1.,1./max(ratio,1.));self.saturations+=int(scale<1.)
            physical_acceleration=np.linalg.solve(self.I,torque*scale-np.cross(r,self.I@r))
            rnext=r+h*physical_acceleration
            # Keep the rate-model observation compatible after allocation clips.
            for axis,(_,_,zero) in enumerate(self.coef):self.rate_state[axis,0]=rnext[axis]-zero*self.rate_state[axis,1]
            rotation=Rotation.from_quat(self.state[[4,5,6,3]])
            rotation=rotation*Rotation.from_rotvec(.5*(r+rnext)*h)
            matrix=rotation.as_matrix()
            acceleration=matrix[:,2]*force*self.cfg.get('thrust_effectiveness',1.)/self.mass-[0,0,9.81]-matrix@(np.asarray(self.cfg.get('drag_per_s',[0.,0.,0.]))*(matrix.T@self.state[7:10]))+self.bias
            self.state[:3]+=self.state[7:10]*h+.5*acceleration*h*h;self.state[7:10]+=acceleration*h
            self.state[3:7]=rotation.as_quat()[[3,0,1,2]];self.state[10:13]=rnext
            target=np.clip(u[0],fmin,fmax);tau=self.cfg['collective_tau_up_s'] if target>=force else self.cfg['collective_tau_down_s']
            self.state[13]+=(-np.expm1(-h/tau))*(target-force)
            self.time+=h
            if round(self.time/.001)%5==0:
                alpha=-np.expm1(-.005/.02)
                self.filtered_velocity+=alpha*(self.state[7:10]-self.filtered_velocity)
                self.filtered_rate+=alpha*(self.state[10:13]-self.filtered_rate)
            self.samples.append((self.time,self.state.copy(),self.filtered_velocity.copy(),self.filtered_rate.copy()))
