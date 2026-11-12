"""Measured-state PD landing reference; no inference or transport dependencies."""

import math
import numpy as np


def smooth(u):
    u = float(np.clip(u,0.,1.))
    return u**3*(10-15*u+6*u*u)


class Landing:
    def __init__(self, start, position, home_xy, ground, reason, policy_weight=1., velocity=(0.,0.,0.), braking_acceleration=1., exit_height_m=.10, exit_dwell_s=2., speed_profile="standard"):
        if speed_profile not in ("standard","quick"):raise ValueError("Choose standard or quick landing speed profile")
        quick=speed_profile=="quick"
        self.flare_speed=.10 if quick else .06
        self.flare_penetration=.04 if quick else .02
        self.start, self.position = start,np.asarray(position,dtype=float).copy()
        self.initial_position = self.position.copy()
        self.velocity = np.asarray(velocity,dtype=float).copy()
        if self.velocity.shape != (3,) or not np.isfinite(self.velocity).all() or not math.isfinite(braking_acceleration) or braking_acceleration <= 0:
            raise ValueError('Finite initial velocity and positive braking acceleration required')
        self.velocity[2] = 0.
        self.brake_s = max(.5 if quick else 2.,1.875*np.linalg.norm(self.velocity)/braking_acceleration)
        self.position += .5*self.brake_s*self.velocity
        self.home = np.r_[home_xy,self.position[2]]
        self.ground, self.reason = ground,reason
        self.initial_weight = policy_weight
        if not all(math.isfinite(v) and v > 0 for v in (exit_height_m,exit_dwell_s)):
            raise ValueError('Positive finite landing exit height/dwell required')
        self.exit_height_m,self.exit_dwell_s=exit_height_m,exit_dwell_s
        self.last_exit_sample_s = None
        self.touch_since = None
        self.touch_time = None
        self.return_s = max(.5 if quick else 4.,np.linalg.norm(self.home[:2]-self.position[:2])/(.20 if quick else .12))
        self.descend_start = self.start+self.brake_s+self.return_s
        self.slow_height = min(self.position[2],ground+.12)
        self.descend_s = max(1.5 if quick else 2.,(self.position[2]-self.slow_height)/(.25 if quick else .10))
        self.timeout = self.descend_start+self.descend_s+12.

    def sample(self, now):
        pos,vel,acc = self.position.copy(),np.zeros(3),np.zeros(3)
        elapsed = now-self.start
        phase = "landing_blend"
        if elapsed < self.brake_s:
            u = float(np.clip(elapsed/self.brake_s,0.,1.))
            distance = u-2.5*u**4+3*u**5-u**6
            pos = self.initial_position+self.velocity*self.brake_s*distance
            vel = self.velocity*(1.-smooth(u))
            acc = -self.velocity*30*u*u*(1-u)**2/self.brake_s
            if np.linalg.norm(self.velocity)>.1:
                phase = "landing_brake"
        elif now<self.descend_start:
            u = np.clip((elapsed-self.brake_s)/self.return_s,0.,1.)
            delta = self.home-self.position
            pos += delta*smooth(u)
            vel = delta*30*u*u*(1-u)**2/self.return_s
            acc = delta*60*u*(1-u)*(1-2*u)/self.return_s**2
            phase = "landing_return"
        elif now>=self.descend_start:
            pos = self.home.copy()
            if now<self.descend_start+self.descend_s:
                u = (now-self.descend_start)/self.descend_s
                delta = self.slow_height-self.position[2]
                pos[2] += delta*smooth(u)
                vel[2] = delta*30*u*u*(1-u)**2/self.descend_s
                acc[2] = delta*60*u*(1-u)*(1-2*u)/self.descend_s**2
                phase = "landing_descend"
            else:
                elapsed = now-self.descend_start-self.descend_s
                u = min(elapsed,1.)
                distance = self.flare_speed*(2.5*u**4-3*u**5+u**6+max(elapsed-1.,0.))
                pos[2] = max(self.ground-self.flare_penetration,self.slow_height-distance)
                if pos[2]>self.ground-self.flare_penetration:
                    vel[2] = -self.flare_speed*smooth(u)
                    acc[2] = -self.flare_speed*30*u*u*(1-u)**2
                phase = "landing_flare"
        if self.touch_time is not None:
            phase = "landing_spool_down"
        return {"position":pos.tolist(),"velocity":vel.tolist(),"acceleration":acc.tolist(),"phase":phase}

    def update_touchdown(self, now, state, tilt):
        state = np.asarray(state)
        if state.shape != (13,) or not np.isfinite(state).all() or not math.isfinite(now):
            raise ValueError('Finite landing state/time required')
        if self.last_exit_sample_s is not None and (now<self.last_exit_sample_s or now-self.last_exit_sample_s>.040000001):
            self.touch_since = None
        self.last_exit_sample_s = now
        ready = now>=self.start and abs(state[2]-self.ground)<=self.exit_height_m
        self.touch_since = (now if self.touch_since is None else self.touch_since) if ready else None
        if self.touch_time is None and self.touch_since is not None and now-self.touch_since>=self.exit_dwell_s-1e-9:
            self.touch_time = now
        return self.touch_time is not None and now-self.touch_time>=.6

    def weight(self, now):
        return self.initial_weight*(1.-smooth((now-self.start)/2.))

    def spool(self, now):
        return 1. if self.touch_time is None else 1.-smooth((now-self.touch_time)/.6)
