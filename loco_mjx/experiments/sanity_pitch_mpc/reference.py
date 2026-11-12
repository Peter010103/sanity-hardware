"""Hover-to-hover vertical ellipses; simulation-only pitch-loop references."""
import numpy as np


class PitchLoopReference:
    def __init__(self, horizontal_radius=.5, vertical_radius=.65, phase_rate=5.719490967129585,
            ramp_s=.65, laps=1, bottom=.5, direction=1, hold_s=2., recovery_s=4.):
        values=[horizontal_radius,vertical_radius,phase_rate,ramp_s,bottom,hold_s,recovery_s]
        if not np.isfinite(values).all() or min(values)<=0 or laps!=int(laps) or laps<1 or direction not in (-1,1):
            raise ValueError('Positive finite geometry/timing, integral laps and direction +/-1 required')
        self.a,self.b,self.omega,self.ramp=horizontal_radius,vertical_radius,phase_rate,ramp_s
        self.laps,self.bottom,self.direction,self.hold,self.recovery=int(laps),bottom,direction,hold_s,recovery_s
        self.moving_s=2*np.pi*laps/phase_rate+ramp_s
        if phase_rate*ramp_s>=2*np.pi*laps:raise ValueError('Insufficient path length for ramps')
        if bottom+2*vertical_radius>2.:raise ValueError('Reference exceeds the requested2m ceiling')
        self.duration=hold_s+self.moving_s+recovery_s
        self.times=np.arange(0.,self.duration+.001,.001)
        time=np.clip(self.times-hold_s,0.,self.moving_s)
        phase=phase_rate*(time-ramp_s/2);speed=np.full_like(time,phase_rate);acc=np.zeros_like(time)
        up=time<ramp_s;down=time>self.moving_s-ramp_s
        u=time[up]/ramp_s
        phase[up]=phase_rate*ramp_s*(2.5*u**4-3*u**5+u**6)
        speed[up]=phase_rate*(10*u**3-15*u**4+6*u**5)
        acc[up]=phase_rate/ramp_s*(30*u**2-60*u**3+30*u**4)
        u=(self.moving_s-time[down])/ramp_s
        phase[down]=2*np.pi*laps-phase_rate*ramp_s*(2.5*u**4-3*u**5+u**6)
        speed[down]=phase_rate*(10*u**3-15*u**4+6*u**5)
        acc[down]=-phase_rate/ramp_s*(30*u**2-60*u**3+30*u**4)
        position=np.c_[direction*self.a*np.sin(phase),np.zeros_like(time),bottom+self.b*(1-np.cos(phase))]
        velocity=np.c_[direction*self.a*np.cos(phase)*speed,np.zeros_like(time),self.b*np.sin(phase)*speed]
        acceleration=np.c_[direction*self.a*(np.cos(phase)*acc-np.sin(phase)*speed**2),np.zeros_like(time),self.b*(np.sin(phase)*acc+np.cos(phase)*speed**2)]
        force=acceleration+[0.,0.,9.81]
        pitch=np.unwrap(np.arctan2(force[:,0],force[:,2]))
        self.pitch=pitch
        rate=np.gradient(pitch,.001);rate_derivative=np.gradient(rate,.001)
        force_magnitude=.06947*np.linalg.norm(force,axis=1)
        force_derivative=np.gradient(force_magnitude,.001)
        q=np.c_[np.cos(pitch/2),np.zeros_like(pitch),np.sin(pitch/2),np.zeros_like(pitch)]
        rates=np.c_[np.zeros_like(time),rate,np.zeros_like(time)]
        ff=np.c_[force_magnitude+np.where(force_derivative>=0,.0438819,.0485511)*force_derivative,
            np.zeros_like(time),rate+.04*rate_derivative,np.zeros_like(time)]
        self.table=np.c_[position,q,velocity,rates,force_magnitude,ff]
        self.info={'kind':'vertical ellipse with quintic speed ramps; positive thrust aligned with acceleration+gravity',
            'horizontal_radius_m':self.a,'vertical_radius_m':self.b,'bottom_m':bottom,'top_m':bottom+2*self.b,
            'phase_rate_rad_s':phase_rate,'ramp_s':ramp_s,'laps':laps,'direction':direction,'hold_s':hold_s,
            'moving_s':self.moving_s,'recovery_s':recovery_s,'duration_s':self.duration,
            'force_reference_turns':float(-(pitch[-1]-pitch[0])/(2*np.pi)*direction),
            'peak_reference_pitch_rate_rad_s':float(abs(rate).max()),'peak_feedforward_pitch_command_rad_s':float(abs(ff[:,2]).max()),
            'peak_force_N':float(force_magnitude.max()),'peak_force_command_N':float(ff[:,0].max())}
        if self.info['force_reference_turns']<laps-.01:raise ValueError('Requested timing does not yield all inverted loops')


    def sample(self, time_s):
        times=np.atleast_1d(time_s)
        values=np.column_stack([np.interp(times,self.times,self.table[:,i]) for i in range(self.table.shape[1])])
        values[:,3:7]/=np.linalg.norm(values[:,3:7],axis=1)[:,None]
        return values[0] if np.ndim(time_s)==0 else values
