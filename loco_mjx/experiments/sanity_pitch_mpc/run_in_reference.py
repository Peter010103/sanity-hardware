"""Dynamically screened, C4 run-in / steady pitch loops / forward exit."""
import json
from pathlib import Path

import numpy as np
from numpy.polynomial import Polynomial


class RunInPitchReference:
    def __init__(self, configuration, laps=1, bottom=.5, direction=1, profile=None, hold_s=2., recovery_s=4.):
        profile=profile or json.loads(Path(__file__).with_name('run_in_profile.json').read_text())
        if laps!=int(laps) or laps<1 or direction not in (-1,1) or not np.isfinite([bottom,hold_s,recovery_s]).all() or min(bottom,hold_s,recovery_s)<=0:
            raise ValueError('Positive finite heights/times, integral laps and direction +/-1 required')
        self.profile=profile;self.a=profile['horizontal_radius_m'];self.b=profile['vertical_radius_m'];self.omega=profile['phase_rate_rad_s']
        self.pull_up_s=profile['entry_s'];self.acceleration_s=profile.get('acceleration_s',0.);self.entry_s=self.pull_up_s+self.acceleration_s;self.distance=profile['run_in_distance_m'];self.bottom=bottom;self.direction=direction;self.laps=int(laps)
        self.hold,self.recovery=hold_s,recovery_s
        self.entry_phase=profile.get('entry_phase_rad',0.)
        self.loop_s=(2*np.pi*self.laps-2*self.entry_phase)/self.omega;self.moving_s=2*self.entry_s+self.loop_s
        self.loop_start=self.hold+self.entry_s;self.loop_end=self.loop_start+self.loop_s
        self.duration=self.hold+self.moving_s+self.recovery
        self.times=np.unique(np.r_[np.linspace(0.,self.duration,int(np.ceil(self.duration/.001))+1),
            self.hold,self.hold+self.acceleration_s,self.loop_start,self.loop_end,self.loop_end+self.pull_up_s,self.hold+self.moving_s])
        t=self.times;derivatives=[np.zeros((len(t),3)) for _ in range(5)]
        derivatives[0][:]=[-self.distance,0.,bottom]
        entry=(t>=self.hold+self.acceleration_s)&(t<self.loop_start);loop=(t>=self.loop_start)&(t<self.loop_end)
        exit=(t>=self.loop_end)&(t<self.loop_end+self.pull_up_s);tail=t>=self.hold+self.moving_s
        polynomials=[Polynomial(c) for c in profile['entry_coefficients_xz']]
        for k in range(5):
            for axis,poly in zip((0,2),polynomials):
                derivatives[k][entry,axis]=poly.deriv(k)((t[entry]-self.hold-self.acceleration_s)/self.pull_up_s)/self.pull_up_s**k
                factor=(-1)**(k+1) if axis==0 else (-1)**k
                derivatives[k][exit,axis]=factor*poly.deriv(k)(1-(t[exit]-self.loop_end)/self.pull_up_s)/self.pull_up_s**k
            theta=self.entry_phase+(t[loop]-self.loop_start)*self.omega
            derivatives[k][loop,0]=self.a*self.omega**k*np.sin(theta+k*np.pi/2)
            derivatives[k][loop,2]=-self.b*self.omega**k*np.cos(theta+k*np.pi/2)
        if self.acceleration_s:
            advance=Polynomial([0.,0.,0.,0.,0.,7.,-14.,10.,-2.5])*profile['run_in_speed_m_s']*self.acceleration_s
            accelerating=(t>=self.hold)&(t<self.hold+self.acceleration_s)
            braking=(t>=self.loop_end+self.pull_up_s)&(t<self.hold+self.moving_s)
            for k in range(5):
                derivatives[k][accelerating,0]=advance.deriv(k)((t[accelerating]-self.hold)/self.acceleration_s)/self.acceleration_s**k
                derivatives[k][braking,0]=(-1)**(k+1)*advance.deriv(k)(1-(t[braking]-self.loop_end-self.pull_up_s)/self.acceleration_s)/self.acceleration_s**k
            derivatives[0][accelerating,0]-=self.distance;derivatives[0][braking,0]+=self.distance
        derivatives[0][entry,2]+=bottom;derivatives[0][loop,2]+=bottom+self.b;derivatives[0][exit,2]+=bottom
        derivatives[0][tail]=[self.distance,0.,bottom]
        for value in derivatives:value[:,0]*=direction
        self.position,self.velocity,self.acceleration,self.jerk,self.snap=derivatives
        specific_force=self.acceleration+[0.,0.,9.81];square=np.sum(specific_force**2,axis=1);magnitude=np.sqrt(square)
        if magnitude.min()<1e-4:raise ValueError('Reference contains a singular zero-force attitude')
        pitch=np.unwrap(np.arctan2(specific_force[:,0],specific_force[:,2]))
        rate=(specific_force[:,2]*self.jerk[:,0]-specific_force[:,0]*self.jerk[:,2])/square
        alpha=(specific_force[:,2]*self.snap[:,0]-specific_force[:,0]*self.snap[:,2])/square-2*rate*np.sum(specific_force*self.jerk,axis=1)/square
        mass=configuration['mass_kg'];force=mass*magnitude
        force_derivative=mass*np.sum(specific_force*self.jerk,axis=1)/magnitude
        tau=np.where(force_derivative>=0,configuration['collective_tau_up_s'],configuration['collective_tau_down_s'])
        rate_tau=configuration['inertia_kg_m2'][1][1]/configuration['rate_kv'][1]
        ff=np.column_stack([force+tau*force_derivative,np.zeros(len(t)),rate+rate_tau*alpha,np.zeros(len(t))])
        q=np.column_stack([np.cos(pitch/2),np.zeros(len(t)),np.sin(pitch/2),np.zeros(len(t))])
        rates=np.column_stack([np.zeros(len(t)),rate,np.zeros(len(t))])
        self.pitch=pitch;self.pitch_acceleration=alpha;self.table=np.c_[self.position,q,self.velocity,rates,force,ff]
        self.info={'kind':'Level forward acceleration, C4 pull-up, constant-phase ellipse, C4 exit and level braking',
            'horizontal_radius_m':self.a,'vertical_radius_m':self.b,'phase_rate_rad_s':self.omega,'bottom_m':bottom,
            'top_m':float(self.position[:,2].max()),'height_min_m':float(self.position[:,2].min()),'laps':self.laps,'direction':direction,
            'hold_s':self.hold,'entry_s':self.entry_s,'pull_up_s':self.pull_up_s,'acceleration_s':self.acceleration_s,'level_run_in_speed_m_s':profile.get('run_in_speed_m_s'),'run_in_distance_m':self.distance,'bottom_tangent_speed_m_s':self.a*self.omega,
            'entry_velocity_m_s':[direction*self.a*self.omega*np.cos(self.entry_phase),0.,self.b*self.omega*np.sin(self.entry_phase)],
            'loop_start_s':self.loop_start,'loop_end_s':self.loop_end,'loop_s':self.loop_s,'moving_s':self.moving_s,'recovery_s':self.recovery,'duration_s':self.duration,
            'initial_position_m':self.position[0].tolist(),'final_position_m':self.position[-1].tolist(),
            'position_bounds_m':[self.position.min(axis=0).tolist(),self.position.max(axis=0).tolist()],
            'force_reference_turns':float(-(pitch[-1]-pitch[0])/(2*np.pi)*direction),
            'peak_reference_pitch_rate_rad_s':float(abs(rate).max()),'peak_feedforward_pitch_command_rad_s':float(abs(ff[:,2]).max()),
            'peak_force_N':float(force.max()),'peak_force_command_N':float(ff[:,0].max()),
            'feasibility_scope':'Sampled nominal calibrated model; high-rate actuator physics remain assumed. Not a robust flight feasibility certificate.'}
        self.validate(configuration)


    def validate(self, configuration):
        force=self.table[:,13];command=self.table[:,14:18]
        fmin,fmax=configuration['physical_force_range_N'];maximum=min(configuration['maximum_thrust_N'],fmax)
        capacity=configuration['torque_reserve_fraction']*np.maximum(np.minimum(force-fmin,fmax-force),0.)/4
        torque=configuration['inertia_kg_m2'][1][1]*self.pitch_acceleration
        deviation=abs(torque)*np.max(abs(np.asarray(configuration['allocation_per_m'])[:,1]))
        margins={'horizontal_envelope_m':float(2.-np.linalg.norm(self.position[:,:2],axis=1).max()),'height_floor_m':float(self.position[:,2].min()-.15),'height_ceiling_m':float(2.-self.position[:,2].max()),
            'thrust_low_N':float(command[:,0].min()),'thrust_high_N':float(maximum-command[:,0].max()),
            'pitch_rate_rad_s':float(configuration.get('pitch_limit_rad_s',9.5)-abs(command[:,2]).max()),
            'pitch_torque_Nm':float(configuration['torque_limits_Nm'][1]-abs(torque).max()),'rotor_headroom_N':float((capacity-deviation).min())}
        self.info['nominal_constraint_margins']=margins
        failures=[k for k,v in margins.items() if v < -1e-5]
        if failures:raise ValueError('Run-in reference infeasible under configured model: '+', '.join(f'{k}={margins[k]:.6g}' for k in failures))
        if abs(self.info['force_reference_turns']-self.laps)>.01:raise ValueError('Entry/exit introduced an unintended inversion')
        return margins


    def phase(self, time_s):
        if time_s<self.hold:return 'mpc_initial_hover'
        if time_s<self.hold+self.acceleration_s:return 'run_in'
        if time_s<self.loop_start:return 'pitch_entry'
        if time_s<self.loop_end:return 'pitch_loops'
        if time_s<self.loop_end+self.pull_up_s:return 'pitch_exit'
        if time_s<self.hold+self.moving_s:return 'run_out'
        return 'mpc_recovery'


    def sample(self, time_s):
        times=np.atleast_1d(time_s)
        values=np.column_stack([np.interp(times,self.times,self.table[:,i]) for i in range(self.table.shape[1])])
        values[:,3:7]/=np.linalg.norm(values[:,3:7],axis=1)[:,None]
        return values[0] if np.ndim(time_s)==0 else values
