from pathlib import Path
import sys

import numpy as np

SOURCE = Path(__file__).resolve().parents[6]/'loco_mjx/experiments/sanity_ctbr_sim2sim'
if str(SOURCE) not in sys.path:sys.path.insert(0,str(SOURCE))
from missions import DeploymentMission
from takeoff_controller import TakeoffReference, smooth_fraction


class FleetMission(DeploymentMission):
    def __init__(self, settings, drone, parameters):
        self.settings, self.drone = dict(settings), drone
        super().__init__('circle',settings['speed_m_s'],radius=.5,laps=1,
            command_speed_limit=parameters.get('command_speed_limit_m_s',0.),
            fallback_speed_limit=parameters.get('fallback_speed_limit_m_s',4.),
            ramp_s=parameters.get('mission_ramp_s',2.),auto_radius=True,
            reference_tilt_limit_deg=parameters.get('reference_tilt_limit_deg',20.),controller=parameters['controller'])
        self.pattern = settings['kind']
        self.center = np.asarray(drone['placement']['center_xy'],dtype=float)
        self.direction = np.asarray(drone['placement']['direction_xy'],dtype=float)
        if drone['placement']['reverse']:self.direction = -self.direction
        half = .5*settings['length_m'] if self.pattern == 'line' else 0.
        self.start_xy = self.center-half*self.direction
        self.end_xy = self.center+half*self.direction
        self.leg_s = max(1.875*settings['length_m']/settings['speed_m_s'],
                         np.sqrt(6.*settings['length_m']/self.max_reference_acceleration))
        self.cycle_s = 2*(self.leg_s+settings['endpoint_hold_s'])
        self.duration = settings['hover_s'] if self.pattern == 'hover' else self.cycle_s*settings['cycles']
        self.minimum_mission_time = 25.+self.duration
        self.landing_reserve_s = 45.
        self.transfer_s = max(6.,1.875*np.linalg.norm(self.start_xy-np.asarray(drone['home_xy']))/.5)


    def sample_world(self, elapsed):
        pos = np.r_[self.start_xy,self.settings['height_m']]
        vel,acc = np.zeros(3),np.zeros(3)
        phase = 'fleet_hover'
        if self.pattern == 'line' and 0 <= elapsed < self.duration:
            within = elapsed % self.cycle_s
            half = self.leg_s+self.settings['endpoint_hold_s']
            reverse = within >= half
            leg_time = within-half if reverse else within
            origin = self.end_xy if reverse else self.start_xy
            delta = (self.start_xy-self.end_xy) if reverse else self.end_xy-self.start_xy
            u = np.clip(leg_time/self.leg_s,0.,1.)
            pos[:2] = origin+delta*smooth_fraction(u)
            vel[:2] = delta*30*u*u*(1-u)**2/self.leg_s
            acc[:2] = delta*60*u*(1-u)*(1-2*u)/self.leg_s**2
            phase = 'fleet_line' if leg_time < self.leg_s else 'fleet_endpoint_hold'
        return {'position':pos.tolist(),'velocity':vel.tolist(),'acceleration':acc.tolist(),'phase':phase}


    def metadata(self):
        return {'pattern':self.pattern,'duration_s':self.duration,'world_start_xy':self.start_xy.tolist(),
            'world_end_xy':self.end_xy.tolist(),'height_m':self.settings['height_m'],'leg_duration_s':self.leg_s,
            'cycles':self.settings['cycles'],'reference_clock':'shared host monotonic epoch',
            'line_parameterization':'quintic rest-to-rest legs; requested speed is peak, not cruise speed',
            'peak_line_speed_m_s':1.875*self.settings['length_m']/self.leg_s if self.pattern=='line' else 0.,
            'position_gain_per_s':self.position_gain,'velocity_gain':self.velocity_gain,
            'acceleration_lead_s':self.acceleration_lead_s,'command_norm_limit_m_s':self.command_speed_limit,
            'pd_transfer_s':self.transfer_s,'minimum_nominal_mission_time_s':self.minimum_mission_time,
            'nominal_landing_reserve_s':self.landing_reserve_s}


    def takeoff_reference(self, parameters):
        return FleetReference(self,parameters['ground_root_height_m'],parameters['target_height_m'])


class FleetReference(TakeoffReference):
    def __init__(self, mission, ground_height, target_height):
        super().__init__(ground_height,target_height)
        self.mission = mission
        self.home = np.asarray(mission.drone['home_xy'],dtype=float)
        self.orbit_start = None
        self.bind_home(self.home)


    def bind_home(self, home):
        self.home = np.asarray(home,dtype=float).copy()
        self.transfer_start = self.climb_end+2.
        self.transfer_s = max(6.,1.875*np.linalg.norm(self.mission.start_xy-self.home)/.5)
        self.arrival_time = self.transfer_start+self.transfer_s
        self.handover_ready_time = self.arrival_time+2.


    def sample(self, time_s):
        reference = super().sample(time_s)
        delta = self.mission.start_xy-self.home
        if time_s < self.transfer_start:
            return reference
        if time_s < self.arrival_time:
            u = np.clip((time_s-self.transfer_start)/self.transfer_s,0.,1.)
            reference.update(position=[*(delta*smooth_fraction(u)),self.target_height],
                velocity=[*(delta*30*u*u*(1-u)**2/self.transfer_s),0.],
                acceleration=[*(delta*60*u*(1-u)*(1-2*u)/self.transfer_s**2),0.],phase='reposition')
        elif self.orbit_start is None or time_s < self.orbit_start:
            reference.update(position=[*delta,self.target_height],velocity=[0.]*3,acceleration=[0.]*3,phase='fleet_ready_hold')
        else:
            reference = self.mission.sample_world(time_s-self.orbit_start)
            reference['position'] = (np.asarray(reference['position'])-np.r_[self.home,0.]).tolist()
        return reference
