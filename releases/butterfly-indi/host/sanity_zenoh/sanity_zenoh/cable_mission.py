from collections import deque
import importlib
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from sanity_zenoh.landing import Landing
from sanity_zenoh.cable_qualification import HoverQualification
from sanity_zenoh.cable_length import HoverCableLength
from sanity_zenoh.cable_health import PayloadHealth, MissionFailureGuard


def smooth(value):
    value=np.clip(value,0.,1.)
    return value**3*(10.-15.*value+6.*value**2)


class CableMission:
    def __init__(self, physical, profile, experiment):
        sys.path.insert(0,str(Path(experiment).resolve().parent))
        sys.path.insert(0,str(Path(experiment).resolve().parent/'sanity_ctbr_sim2sim'))
        from sanity_cable_mpc.circle_reference import CircleReference
        from takeoff_controller import PositionPD
        self.cfg,self.profile=physical,profile
        self.reference=CircleReference(physical,profile['mission'],stationary=bool(profile.get('takeoff_land_only',False) or profile.get('mpc_hover_only',False)))
        self.length_calibration=HoverCableLength(physical['cable_length_m'],profile.get('cable_length_calibration'))
        if self.length_calibration.enabled:self.reference.prepare_length_range(self.length_calibration.bounds)
        self.length_pending=None;self.length_applied=None;self.length_start_ns=None
        self.duration=self.reference.duration
        self.payload_health=PayloadHealth(profile.get('payload_health'))
        self.failure_guard=MissionFailureGuard(profile.get('failure_guard'))
        self.orbit_stage=None;self.orbit_bad_since=None;self.orbit_diagnostic=None
        self.pd=PositionPD(physical['airframe_mass_kg'],position_gains=(2.,2.,4.),velocity_gains=(2.,2.,3.),attitude_gain=4.5,integral_gains=(.5,.5,.8))
        self.bounds=np.array([[0.,physical['maximum_thrust_N']],[-2.3,2.3],[-2.3,2.3],[-2.6,2.6]])
        self.phase='waiting';self.start_ns=None;self.home=None;self.yaw=0.;self.turn=Rotation.identity()
        self.mission_start_ns=self.blend_start_ns=self.stable_since_ns=self.settle_start_ns=None
        self.mpc_hover_start_ns=self.mpc_hover_ready_ns=self.mpc_missing_since_ns=None
        self.hover_gate=HoverQualification(profile.get('mpc_hover_qualification'))
        self.hover_qualification=None;self.hover_status_ns=None
        self.mpc_misses=deque();self.mpc_recovery_weight=1.;self.payload_rotation_bad_since_ns=None
        self.landing=None;self.abort_reason=None;self.last_command=np.zeros(4);self.land_command=np.zeros(4)
        self.place_start_ns=self.place_from=self.place_position=self.reposition_start_ns=self.reposition_from=self.landing_xy=None
        self.finished=False;self.completed_mission=False;self.events=[]
        self.maximum_drone_displacement=0.;self.hover_start_ns=None
        self.payload_floor_height_m=float(profile['floor_height_m'])
        self.payload_drop_m=physical['cable_length_m']+physical['payload_com_offset_below_hook_m']+physical['payload_size_m'][2]/2-physical['attachment_body_m'][2]
        self.quick_landing=profile.get('landing_speed_profile','standard')=='quick'
        self.reposition_s=2.5 if self.quick_landing else 6.
        self.payload_missing_since_ns=None;self.payload_released=False;self.cable_invalid_since_ns=None;self.last_load_fraction=0.;self.cutoff_ready_since_ns=None
        self.pickup_fraction=0.;self.update_dt=.02;self.last_update_ns=None


    def check_ground_start(self, packet):
        floor=float(self.profile['floor_height_m'])
        body_height=float(packet['vehicle'][2]-floor)
        cube_height=float(packet['cube_lowest_height_m']-floor)
        bypass=bool(self.profile.get('skip_ground_height_check',False))
        if not math.isfinite(body_height) or not math.isfinite(cube_height):raise ValueError('Finite measured ground geometry required')
        if not bypass and (not .005<=body_height<=.08 or not -.012<=cube_height<=.03):
            raise ValueError(f'Ground start requires both aircraft and payload near the calibrated floor: floor={floor:.4f}m, aircraft above floor={body_height:.4f}m (allowed0.005..0.080m), cube bottom above floor={cube_height:.4f}m (allowed-0.012..0.030m)')
        return {'floor_height_m':floor,'aircraft_height_above_floor_m':body_height,'cube_bottom_above_floor_m':cube_height,'height_check_bypassed':bypass}


    def initialize(self, packet, now_ns):
        self.home=packet['vehicle'][:3].copy()
        if not self.profile.get('start_airborne',False):
            self.check_ground_start(packet)
            if self.profile.get('skip_ground_height_check',False):
                self.payload_floor_height_m=float(packet['cube_lowest_height_m'])
                self.events.append({'phase':'ground_reference_override','stamp_ns':int(now_ns),'aircraft_start_z_m':float(self.home[2]),'payload_rest_bottom_z_m':self.payload_floor_height_m})
        rotation=Rotation.from_quat(packet['vehicle'][[4,5,6,3]])
        self.yaw=rotation.as_euler('xyz')[2];self.turn=Rotation.from_euler('z',self.yaw)
        if self.profile.get('start_airborne',False):self.home[2]=self.profile['floor_height_m']+.016
        self.start_ns=int(now_ns)
        self.phase=('calibrate_length' if self.length_calibration.enabled else 'acquire') if self.profile.get('start_airborne',False) else 'takeoff'
        if self.phase=='calibrate_length':self.length_start_ns=int(now_ns)
        self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})


    def preview(self, times_ns):
        times_ns=np.asarray(times_ns)
        relative=np.maximum((times_ns-(self.mission_start_ns if self.mission_start_ns is not None else times_ns.max()+1_000_000_000))/1e9,0.)
        if self.profile.get('takeoff_land_only',False) or self.profile.get('mpc_hover_only',False):relative=np.zeros_like(relative)
        preview=self.reference.sample(relative)
        preview[:,:3]=self.turn.apply(preview[:,:3]-[0.,0.,1.2])+np.r_[self.home[:2],self.profile['hover_height_m']]
        preview[:,7:10]=self.turn.apply(preview[:,7:10])
        rotations=self.turn*Rotation.from_quat(preview[:,[4,5,6,3]])
        preview[:,3:7]=rotations.as_quat()[:,[3,0,1,2]]
        preview[:,24:27]=self.turn.apply(preview[:,24:27]);preview[:,27:30]=self.turn.apply(preview[:,27:30])
        if self.cfg.get('prediction_mode')=='rigid_total_mass':
            preview[:,24:30]=[0.,0.,-1.,0.,0.,0.]
            preview[:,31:34]=preview[:,10:13]
            for axis,coefficients in enumerate(self.reference.model.coefficients):
                preview[:,31+axis]/=coefficients[0]
                preview[:,14+axis*2]=preview[:,31+axis];preview[:,15+axis*2]=0.
                preview[:,18+axis*3:21+axis*3]=preview[:,31+axis,None]
        if self.cfg.get('payload_attitude_dynamics',False):
            z=-preview[:,24:27]
            y=np.cross(z,self.turn.apply([1.,0.,0.]));y/=np.linalg.norm(y,axis=1,keepdims=True)
            cube=Rotation.from_matrix(np.stack([np.cross(y,z),y,z],axis=-1)).as_quat()[:,[3,0,1,2]]
            preview=np.column_stack([preview[:,:30],cube,preview[:,27:30],preview[:,30:]])
        return preview


    def apply_cable_length(self, length, now_ns):
        if self.phase!='calibrate_length' or self.length_applied is not None or self.length_pending!=length:
            raise ValueError('Cable calibration may only be applied once in PID hover')
        self.reference.set_cable_length(length)
        self.cfg['cable_length_m']=float(length)
        self.payload_drop_m=length+self.cfg['payload_com_offset_below_hook_m']+self.cfg['payload_size_m'][2]/2-self.cfg['attachment_body_m'][2]
        self.length_applied=float(length);self.length_pending=None;self.cable_invalid_since_ns=None;self.stable_since_ns=None
        self.phase='acquire'
        self.events.append({'phase':'cable_length_calibrated','stamp_ns':int(now_ns),'reason':f'Hover cable length {length*1000:.1f} mm; frozen for this flight',**self.length_calibration.diagnostic})
        self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})
        if self.profile['mission']=='payload_orbit':self.events[-2].update(orbit_period_s=float(self.reference.period),orbit_radius_m=self.reference.radius)


    def request_landing(self, packet, now_ns, reason=None):
        if self.phase in ('place_payload','reposition','landing') or self.finished:return
        self.abort_reason=reason or self.abort_reason
        if reason and self.profile.get('landing_transfer_s',0.)>0. and self.home is not None and packet.get('cube_lowest_height_m',1.)<self.payload_floor_height_m+.015 and abs(packet['vehicle'][2]-self.home[2])<=self.profile.get('landing_cutoff_height_m',.05) and abs(packet['vehicle'][9])<.12:
            self.phase='landed';self.finished=True
            self.events.append({'phase':self.phase,'stamp_ns':int(now_ns),'method':'near_ground_abort','height_above_start_m':float(packet['vehicle'][2]-self.home[2]),'reason':reason});return
        self.place_start_ns=now_ns;self.place_from=packet['vehicle'][:3].copy()
        self.place_position=self.place_from.copy()
        loaded_height=self.payload_drop_m-(.05 if self.quick_landing else .0087)
        self.place_position[2]=self.payload_floor_height_m+(loaded_height if 'payload' in packet else .30)
        if reason and self.profile.get('landing_transfer_s',0.)>0.:self.place_position[2]=min(self.place_from[2],self.place_position[2])
        self.place_duration=max(3. if self.quick_landing else 4.,abs(self.place_from[2]-self.place_position[2])/(.18 if self.quick_landing else .06))
        self.phase='place_payload';self.land_command=self.last_command.copy();self.pd.reset()
        self.stable_since_ns=None
        self.events.append({'phase':self.phase,'reason':reason or 'mission_complete','stamp_ns':int(now_ns)})


    def update(self, packet, now_ns):
        if self.start_ns is None:self.initialize(packet,now_ns)
        vehicle=packet['vehicle'];now=(now_ns-self.start_ns)/1e9
        self.update_dt=.02 if self.last_update_ns is None else min(.04,max(.001,(now_ns-self.last_update_ns)/1e9))
        self.last_update_ns=int(now_ns)
        rotation=Rotation.from_quat(vehicle[[4,5,6,3]])
        tilt=np.degrees(np.arccos(np.clip(rotation.as_matrix()[2,2],-1.,1.)))
        if self.home is not None and np.linalg.norm(vehicle[:2]-self.home[:2])>1.:
            raise ValueError('Aircraft left the1m horizontal mission envelope')
        if tilt>45. or vehicle[2]>self.profile['maximum_height_m']:
            raise ValueError('Aircraft hard tilt/height envelope exceeded')
        if now>self.profile.get('maximum_run_s',180.):raise ValueError('Cable mission runtime exceeded')
        if self.phase in ('calibrate_length','acquire','hover','blend','mpc_hover','mission','settle'):
            bad_cable=not packet.get('taut',False)
            if self.phase=='calibrate_length':
                measured=packet.get('hook_distance_m')
                bad_cable='payload' not in packet or measured is None or not np.isfinite(measured) or not self.length_calibration.bounds[0]<=measured<=self.length_calibration.bounds[1]
            if (self.profile.get('takeoff_land_only',False) and 'payload' in packet) or self.profile.get('payload_recovery_s',0.)>0.:
                self.cable_invalid_since_ns=(now_ns if self.cable_invalid_since_ns is None else self.cable_invalid_since_ns) if bad_cable else None
                bad_cable=self.cable_invalid_since_ns is not None and now_ns-self.cable_invalid_since_ns>=int(self.profile.get('payload_recovery_s',.1)*1e9)
            bad_rotation=not packet.get('payload_orientation_held',False) and packet.get('payload_internal_tilt_deg',0.)>30.
            self.payload_rotation_bad_since_ns=(now_ns if self.payload_rotation_bad_since_ns is None else self.payload_rotation_bad_since_ns) if bad_rotation else None
            if self.payload_rotation_bad_since_ns is not None and now_ns-self.payload_rotation_bad_since_ns>=200_000_000:self.request_landing(packet,now_ns,'Payload internal rotation exceeded30deg for200ms')
            if bad_cable or packet.get('swing_deg',0.)>55.:
                if packet.get('swing_deg',0.)>55.:
                    reason=f"Payload swing envelope exceeded: measured={packet['swing_deg']:.1f}deg, limit=55.0deg"
                elif 'payload' not in packet:
                    reason='Payload observation unavailable: '+packet.get('payload_observation_error','No synchronized payload state')
                elif self.phase=='calibrate_length':
                    reason='Payload outside hover cable-length capture range '+str(self.length_calibration.bounds)+' m'
                elif packet.get('hook_distance_m') is not None:
                    measured=packet['hook_distance_m'];expected=self.cfg['cable_length_m']
                    reason=f"Payload cable geometry inconsistent: estimated hook distance={1000*measured:.1f}mm, configured={1000*expected:.1f}mm, residual={1000*(measured-expected):+.1f}mm, tolerance=+/-{1000*packet.get('cable_length_tolerance_m',.012):.1f}mm; possible slack or attachment/marker-offset mismatch"
                else:
                    reason='Payload cable geometry unavailable; taut state not established'
                self.request_landing(packet,now_ns,reason)
        if self.payload_health.enabled and self.phase in ('mpc_hover','mission','settle'):
            health=self.payload_health.update(packet,now_ns)
            if health['reason']:self.request_landing(packet,now_ns,health['reason'])
        if self.failure_guard.enabled and self.phase in ('blend','mpc_hover','mission','settle'):
            target=self.preview([int(packet['stamp_ns'])])[0,:3]
            diagnostic=self.failure_guard.update(packet,target,now_ns)
            if diagnostic['reason']:
                self.events.append({'phase':'failure_guard_trip','stamp_ns':int(now_ns),'reason':diagnostic['reason'],'diagnostic':diagnostic.copy()})
                self.request_landing(packet,now_ns,diagnostic['reason'])
        if self.profile.get('payload_spiral') and self.phase in ('mission','settle'):
            settings=self.profile['payload_spiral']
            displacement=float(np.linalg.norm(vehicle[:2]-self.home[:2]))
            offset=None
            if 'payload' in packet:
                attachment=vehicle[:3]+rotation.apply(self.cfg['attachment_body_m'])
                offset=float(np.linalg.norm(packet['payload'][:2]-attachment[:2]))
            bad=displacement>settings['maximum_aircraft_displacement_m'] or (offset is not None and offset>settings['maximum_payload_offset_m']) or tilt>15.
            self.orbit_bad_since=(now_ns if self.orbit_bad_since is None else self.orbit_bad_since) if bad else None
            if self.orbit_bad_since is not None and now_ns-self.orbit_bad_since>=200_000_000:self.request_landing(packet,now_ns,f'Spiral flight envelope exceeded: aircraft XY displacement={displacement:.3f}m, payload offset={offset}, aircraft tilt={tilt:.1f}deg')
        if self.phase=='takeoff':
            u=np.clip((now-1.5)/self.profile['climb_s'],0.,1.)
            distance=self.profile['hover_height_m']-self.home[2]
            ref={'position':[*self.home[:2],self.home[2]+distance*smooth(u)],'velocity':[0.,0.,distance*30*u**2*(1-u)**2/self.profile['climb_s']], 'acceleration':[0.,0.,distance*60*u*(1-u)*(1-2*u)/self.profile['climb_s']**2]}
            if u>=1.:
                self.phase='calibrate_length' if self.length_calibration.enabled else 'acquire'
                if self.phase=='calibrate_length':self.length_start_ns=int(now_ns)
                self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})
        else:
            goal=self.preview([now_ns])[0]
            relative=0. if self.mission_start_ns is None or self.profile.get('mpc_hover_only',False) else max(0.,(now_ns-self.mission_start_ns)/1e9)
            acceleration=self.turn.apply(self.reference.targets([relative])[2][0])
            ref={'position':goal[:3].tolist(),'velocity':goal[7:10].tolist(),'acceleration':acceleration.tolist()}
            if self.profile.get('payload_spiral') and self.phase in ('mission','settle'):
                settings=self.profile['payload_spiral']
                out_end=settings['start_s']+settings['ramp_out_s'];in_start=out_end+settings['hold_s'];in_end=in_start+settings['ramp_in_s']
                stage='rest' if relative<settings['start_s'] else 'spiral_out' if relative<out_end else 'hold_orbit' if relative<in_start else 'spiral_in' if relative<in_end else 'stationary_tail'
                if stage!=self.orbit_stage:
                    self.orbit_stage=stage;self.events.append({'phase':'payload_spiral_stage','stamp_ns':int(now_ns),'reason':stage})
                desired_rotation=Rotation.from_quat(goal[[4,5,6,3]])
                desired_payload=goal[:3]+desired_rotation.apply(self.cfg['attachment_body_m'])+self.reference.model.payload_distance*goal[24:27]
                self.orbit_diagnostic={'stage':stage,'target_payload_position':desired_payload.tolist(),'payload_error_m':None if 'payload' not in packet else float(np.linalg.norm(packet['payload'][:3]-desired_payload)),'target_relative_radius_m':float(self.reference.model.payload_distance*np.linalg.norm(goal[24:26])),'period_s':float(self.reference.period),'aircraft_xy_displacement_m':float(np.linalg.norm(vehicle[:2]-self.home[:2]))}

        if self.phase=='calibrate_length':
            self.length_pending=self.length_calibration.update(packet,now_ns,np.r_[self.home[:2],self.profile['hover_height_m']],self.payload_floor_height_m)
            if now_ns-self.length_start_ns>self.length_calibration.timeout*1e9:
                self.request_landing(packet,now_ns,'Cable length calibration timed out: '+', '.join(self.length_calibration.diagnostic['blockers']))
                self.length_pending=None
        if self.phase=='acquire':
            ready=np.linalg.norm(vehicle[:3]-ref['position'])<.08 and np.linalg.norm(vehicle[7:10])<.30 and tilt<10. and packet.get('taut',False) and packet.get('swing_deg',100.)<25.
            self.stable_since_ns=(now_ns if self.stable_since_ns is None else self.stable_since_ns) if ready else None
        if self.profile.get('takeoff_land_only',False):
            if self.phase=='acquire' and self.stable_since_ns is not None and now_ns-self.stable_since_ns>=500_000_000:
                self.phase='hover';self.hover_start_ns=now_ns
                self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})
            if self.phase=='hover' and (now_ns-self.hover_start_ns)/1e9>=self.profile.get('hover_duration_s',5.):
                self.completed_mission=True;self.request_landing(packet,now_ns)
        if self.phase=='blend' and (now_ns-self.blend_start_ns)/1e9>=2.:
            self.phase='mpc_hover';self.mpc_hover_start_ns=now_ns;self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})
        if self.phase=='mpc_hover' and (now_ns-self.mpc_hover_start_ns)/1e9>20.:
            blockers=', '.join((self.hover_qualification or {}).get('blockers',[]))
            self.request_landing(packet,now_ns,'MPC hover qualification timed out'+(': '+blockers if blockers else ''))
        if self.phase=='mission' and (now_ns-self.mission_start_ns)/1e9>=self.duration:
            self.phase='settle';self.settle_start_ns=now_ns;self.stable_since_ns=None;self.completed_mission=True
            self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})
        if self.phase=='settle':
            ready=packet.get('swing_deg',100.)<5. and np.linalg.norm(vehicle[7:10])<.12
            if self.profile.get('payload_spiral'):
                ready=ready and 'payload' in packet and packet.get('swing_deg',100.)<3. and np.linalg.norm(packet['payload'][7:10])<.06 and np.linalg.norm(packet.get('hook_rate',[100.,0.,0.]))<.15 and self.payload_health.diagnostic.get('healthy',False)
            self.stable_since_ns=(now_ns if self.stable_since_ns is None else self.stable_since_ns) if ready else None
            if self.stable_since_ns is not None and (now_ns-self.stable_since_ns)/1e9>2.:
                self.request_landing(packet,now_ns)
            elif (now_ns-self.settle_start_ns)/1e9>15.:
                self.request_landing(packet,now_ns,'Swing did not settle in15s')
        if self.phase=='acquire' and now>self.profile['climb_s']+35.:
            self.request_landing(packet,now_ns,'Loaded hover did not become ready')
        if self.phase=='place_payload':
            missing='payload' not in packet
            self.payload_missing_since_ns=(now_ns if self.payload_missing_since_ns is None else self.payload_missing_since_ns) if missing else None
            lost=missing and (not self.quick_landing or now_ns-self.payload_missing_since_ns>=250_000_000)
            if lost and self.place_position[2]>self.payload_floor_height_m+.31:
                self.place_from=vehicle[:3].copy();self.place_position[2]=self.payload_floor_height_m+.30
                self.place_start_ns=now_ns;self.place_duration=max(3. if self.quick_landing else 4.,abs(vehicle[2]-self.place_position[2])/(.18 if self.quick_landing else .06));self.stable_since_ns=None
            elapsed=(now_ns-self.place_start_ns)/1e9
            u=np.clip(elapsed/self.place_duration,0.,1.)
            delta=self.place_position-self.place_from
            ref={'position':(self.place_from+delta*smooth(u)).tolist(),'velocity':(delta*30*u**2*(1-u)**2/self.place_duration).tolist(),'acceleration':(delta*60*u*(1-u)*(1-2*u)/self.place_duration**2).tolist()}
            lowest=packet.get('cube_lowest_height_m',vehicle[2]-self.payload_drop_m)
            ready=lowest<self.payload_floor_height_m+.012 and np.linalg.norm(vehicle[7:10])<.12 and np.linalg.norm(vehicle[:3]-self.place_position)<.04
            self.stable_since_ns=(now_ns if self.stable_since_ns is None else self.stable_since_ns) if ready else None
            if self.quick_landing and 'payload' in packet:
                ready=ready and packet['hook_distance_m']<self.cfg['cable_length_m']-.015 and abs(packet['payload'][9])<.08
                self.stable_since_ns=(now_ns if self.stable_since_ns is None else self.stable_since_ns) if ready else None
            if u>=1. and self.stable_since_ns is not None and (now_ns-self.stable_since_ns)/1e9>=(.4 if self.quick_landing else .5):
                self.phase='reposition';self.reposition_start_ns=now_ns;self.reposition_from=vehicle[:3].copy()
                self.landing_xy=np.asarray(packet.get('payload',self.place_position))[:2]+self.turn.apply([*self.profile.get('floor_landing_clearance_xy_m',[.15,0.]),0.])[:2]
                self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})
            if elapsed>self.place_duration+15.:raise ValueError('Payload placement timeout')
        if self.phase=='reposition':
            u=np.clip((now_ns-self.reposition_start_ns)/(self.reposition_s*1e9),0.,1.)
            destination=np.r_[self.landing_xy,self.profile['floor_height_m']+.45]
            if self.abort_reason and self.profile.get('landing_transfer_s',0.)>0.:destination[2]=min(destination[2],self.reposition_from[2])
            delta=destination-self.reposition_from
            ref={'position':(self.reposition_from+delta*smooth(u)).tolist(),'velocity':(delta*30*u**2*(1-u)**2/self.reposition_s).tolist(),'acceleration':(delta*60*u*(1-u)*(1-2*u)/self.reposition_s**2).tolist()}
            if u>=1.:
                self.landing=Landing(now,vehicle[:3],self.landing_xy,self.home[2],self.abort_reason or 'mission_complete',0.,velocity=vehicle[7:10],exit_height_m=.025,exit_dwell_s=.4 if self.quick_landing else 1.,speed_profile='quick' if self.quick_landing else 'standard')
                if self.quick_landing:self.pd.reset()
                self.phase='landing';self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})
        if self.phase=='landing':
            ref=self.landing.sample(now)
            if now>self.landing.timeout:raise ValueError('Payload-aware landing timeout')
            cube_down=packet.get('cube_lowest_height_m',self.payload_floor_height_m)<self.payload_floor_height_m+.015
            cutoff=float(self.profile.get('landing_cutoff_height_m',0.))
            if cutoff>0.:
                ready=ref['phase'] in ('landing_descend','landing_flare') and cube_down and not packet.get('taut',False) and abs(vehicle[2]-self.home[2])<=cutoff and abs(vehicle[9])<.12 and tilt<10.
                self.cutoff_ready_since_ns=(now_ns if self.cutoff_ready_since_ns is None else self.cutoff_ready_since_ns) if ready else None
                if self.cutoff_ready_since_ns is not None and (now_ns-self.cutoff_ready_since_ns)/1e9>=self.profile.get('landing_cutoff_dwell_s',.10):
                    self.finished=True;self.phase='landed'
                    self.events.append({'phase':self.phase,'stamp_ns':int(now_ns),'method':'near_ground_cutoff','height_above_start_m':float(vehicle[2]-self.home[2]),'cutoff_height_m':cutoff})
            elif cube_down and abs(vehicle[9])<.12 and tilt<10. and self.landing.update_touchdown(now,vehicle,tilt):
                self.finished=True;self.phase='landed';self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})
        distance=packet.get('hook_distance_m',max(0.,vehicle[2]-self.profile['floor_height_m']-.055))
        fraction=smooth((distance/self.cfg['cable_length_m']-.85)/.15)
        if 'payload' not in packet and self.phase in ('calibrate_length','acquire','hover','blend','mpc_hover','mission','settle'):fraction=self.last_load_fraction
        if self.profile.get('smooth_takeoff',False) and self.phase in ('takeoff','calibrate_length','acquire','hover','blend','mpc_hover','mission','settle'):
            target=fraction if 'payload' in packet else self.pickup_fraction
            change=self.profile.get('pickup_load_fraction_rate_per_s',1.)*self.update_dt
            self.pickup_fraction+=float(np.clip(target-self.pickup_fraction,-change,change))
            fraction=self.pickup_fraction
        if self.phase in ('place_payload','reposition','landing'):
            lowest=packet.get('cube_lowest_height_m',vehicle[2]-self.payload_drop_m)
            fraction*=smooth((lowest-self.payload_floor_height_m)/.015)
        if self.quick_landing and self.phase=='place_payload' and 'payload' not in packet and self.payload_missing_since_ns is not None and now_ns-self.payload_missing_since_ns<250_000_000:
            fraction=self.last_load_fraction
        self.last_load_fraction=float(fraction)
        if self.quick_landing and self.phase in ('place_payload','reposition','landing') and fraction<.05 and not self.payload_released:
            self.pd.reset();self.payload_released=True
            self.events.append({'phase':'payload_released','stamp_ns':int(now_ns)})
        self.pd.mass=self.cfg['airframe_mass_kg']+fraction*(self.cfg['cable_mass_kg']+self.cfg['payload_mass_kg'])
        if packet.get('taut',False) and 'cable_rate' in packet and -packet['direction'][2]>.3 and packet.get('cube_lowest_height_m',self.payload_floor_height_m+1.)>self.payload_floor_height_m+.03:
            damping=self.profile.get('pickup_swing_damping_gain_m_s',.8)*np.cross(packet['cable_rate'],packet['direction'])[:2]
            damping*=min(1.,.8/max(np.linalg.norm(damping),1e-9))
            ref={**ref,'acceleration':(np.asarray(ref['acceleration'])+np.r_[damping,0.]).tolist()}
        pd=self.pd.command(vehicle[:3],vehicle[7:10],rotation,ref,yaw=self.yaw,dt=self.update_dt,integrate=now>(2. if self.profile.get('smooth_takeoff',False) else 1.5))
        if self.phase=='takeoff':
            spool=np.array([self.cfg['airframe_mass_kg']*9.81*smooth((now-.5)/1.),0.,0.,0.])
            if self.profile.get('smooth_takeoff',False):
                blend=smooth((now-1.)/1.)
                pd=(1.-blend)*spool+blend*pd
            elif now<1.5:pd=spool
        if self.landing is not None:pd*=self.landing.spool(now)
        pd=np.clip(pd,self.bounds[:,0],self.bounds[:,1]);self.pd.accept_output(pd)
        if self.finished:pd[:]=0.
        return {'pd':pd,'reference':ref,'phase':self.phase,'need_mpc':not self.profile.get('takeoff_land_only',False) and self.phase in ('acquire','blend','mpc_hover','mission','settle') and packet.get('taut',False),'load_fraction':float(fraction)}


    def select(self, pd, candidate, packet, now_ns):
        if not self.profile.get('takeoff_land_only',False) and self.phase=='acquire' and candidate is not None and self.stable_since_ns is not None and (now_ns-self.stable_since_ns)/1e9>=.5:
            if abs(candidate[0]-pd[0])<.25 and np.linalg.norm(candidate[1:]-pd[1:])<1.:
                self.phase='blend';self.blend_start_ns=now_ns;self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})
        weight=smooth((now_ns-self.blend_start_ns)/2e9) if self.phase=='blend' else 1. if self.phase in ('mpc_hover','mission','settle') else 0.
        if weight and candidate is None:
            self.mpc_hover_ready_ns=None
            if self.mpc_missing_since_ns is None:
                self.mpc_missing_since_ns=now_ns;self.mpc_misses.append(now_ns)
                self.events.append({'phase':'mpc_pid_bridge','stamp_ns':int(now_ns),'reason':'No fresh accepted prediction; recomputing PID from current measurement'})
            while self.mpc_misses and now_ns-self.mpc_misses[0]>1_000_000_000:self.mpc_misses.popleft()
            if now_ns-self.mpc_missing_since_ns>=100_000_000 or len(self.mpc_misses)>=3:
                self.request_landing(packet,now_ns,'Repeated or prolonged MPC unavailability')
            weight=0.;self.mpc_recovery_weight=0.
        elif candidate is not None:
            if self.mpc_missing_since_ns is not None:self.events.append({'phase':'mpc_resumed','stamp_ns':int(now_ns)})
            self.mpc_missing_since_ns=None
            self.mpc_recovery_weight=min(1.,self.mpc_recovery_weight+self.update_dt/.3)
            weight*=self.mpc_recovery_weight
        if self.phase=='mpc_hover':
            vehicle=packet['vehicle'];rotation=Rotation.from_quat(vehicle[[4,5,6,3]])
            tilt=np.degrees(np.arccos(np.clip(rotation.as_matrix()[2,2],-1.,1.)))
            hover=self.cfg['airframe_mass_kg']+self.cfg['cable_mass_kg']+self.cfg['payload_mass_kg']
            checks={
                'fresh_full_mpc_authority':candidate is not None and weight>=.999,
                'position_error':bool(np.linalg.norm(vehicle[:3]-np.r_[self.home[:2],self.profile['hover_height_m']])<.08),
                'aircraft_speed':bool(np.linalg.norm(vehicle[7:10])<.20),
                'aircraft_tilt':bool(tilt<6.),
                'command_rate':bool(candidate is not None and np.linalg.norm(candidate[1:])<self.hover_gate.command_limit),
                'hover_thrust':bool(candidate is not None and abs(candidate[0]-hover*9.81)<.20),
                'payload_internal_tilt':bool(packet.get('payload_internal_tilt_deg',0.)<10.),
                'payload_swing':bool(packet.get('swing_deg',100.)<8.),
                'payload_rate':bool(np.linalg.norm(packet.get('cable_rate',[100.,0.,0.]))<.35),
                'cable_taut':bool(packet.get('taut',False))}
            diagnostic=self.hover_gate.update(now_ns,packet.get('stamp_ns',now_ns),float(np.linalg.norm(vehicle[10:13])),checks)
            diagnostic['minimum_hover_elapsed_s']=(now_ns-self.mpc_hover_start_ns)/1e9
            self.hover_qualification=diagnostic
            self.mpc_hover_ready_ns=self.hover_gate.other_ready_since_ns
            if self.hover_status_ns is None or now_ns-self.hover_status_ns>=2_000_000_000:
                rate=diagnostic['filtered_rate_norm_rad_s']
                rate_text='unavailable' if rate is None else f'{rate:.3f}rad/s'
                reason=f"filtered rate={rate_text}; rate window={diagnostic['window_covered_s']:.2f}/2.00s, good={100*diagnostic['rate_good_fraction']:.1f}%; waiting for "+(', '.join(diagnostic['blockers']) if diagnostic['blockers'] else 'minimum five-second hover' if now_ns-self.mpc_hover_start_ns<5_000_000_000 else 'none')
                self.events.append({'phase':'mpc_hover_status','stamp_ns':int(now_ns),'reason':reason,'qualification':diagnostic.copy()});self.hover_status_ns=now_ns
            if diagnostic['ready'] and now_ns-self.mpc_hover_start_ns>=5_000_000_000:
                self.events.append({'phase':'mpc_hover_qualified','stamp_ns':int(now_ns),'qualification':diagnostic.copy()})
                if self.profile.get('mpc_hover_only',False):
                    self.completed_mission=True;self.request_landing(packet,now_ns);weight=0.
                else:
                    self.phase='mission';self.mission_start_ns=now_ns;self.events.append({'phase':self.phase,'stamp_ns':int(now_ns)})
        command=pd if not weight else (1-weight)*pd+weight*candidate
        if self.phase=='place_payload':
            elapsed=(now_ns-self.place_start_ns)/1e9
            alpha=smooth(elapsed/.5)
            transfer=float(self.profile.get('landing_transfer_s',0.))
            if transfer>0.:
                alpha=smooth(elapsed/transfer)
                command=(1-alpha)*self.land_command+alpha*pd
                maximum=np.asarray(self.cfg.get('command_slew_per_s',[2.,15.,15.,15.]))*self.update_dt
                command=self.last_command+np.clip(command-self.last_command,-maximum,maximum)
            else:command=pd if self.abort_reason else (1-alpha)*self.land_command+alpha*pd
        if self.profile.get('smooth_takeoff',False) and self.phase=='takeoff':
            maximum=np.asarray(self.profile.get('takeoff_command_slew_per_s',[1.,1.5,1.5,2.]))*self.update_dt
            command=self.last_command+np.clip(command-self.last_command,-maximum,maximum)
        self.last_command=np.clip(command,self.bounds[:,0],self.bounds[:,1])
        self.pd.accept_output(self.last_command)
        return self.last_command.copy(),float(weight)
