"""Shared full-attitude pitch missions, fixed group clock and swept-path separation."""
from copy import deepcopy
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from sanity_zenoh.pitch_mission import PitchMission

BODY_RADIUS_M=.15
TRACKING_MARGIN_M=.20
PATH_SAMPLING_ALLOWANCE_M=.02


class PitchFleetClock:
    def __init__(self,missions,lead_s=.5,timeout_s=65.,skip_prestart_readiness_check=False,skip_path_corridor_check=False):
        if not missions or not .2<=lead_s<=5. or not math.isfinite(timeout_s) or timeout_s<20:
            raise ValueError('Explicit pitch fleet, future lead and bounded startup required')
        if type(skip_prestart_readiness_check) is not bool:raise ValueError('skip_prestart_readiness_check must be a boolean')
        if type(skip_path_corridor_check) is not bool:raise ValueError('skip_path_corridor_check must be a boolean')
        self.skip_path_corridor_check=skip_path_corridor_check;self.corridor_bypasses=set()
        self.missions=missions;self.lead=lead_s;self.timeout=timeout_s
        self.skip_prestart_readiness_check=skip_prestart_readiness_check;self.prestart_loss_reported=False
        self.epoch=None;self.started=False;self.abort_reason=None;self.events=[];self.begin_time=None

    def begin(self,now,states):
        if self.begin_time is not None:raise ValueError('Fleet takeoff cannot be restarted')
        if set(states)!=set(self.missions):raise ValueError('State required for every aircraft')
        for device,mission in self.missions.items():mission.begin(now,states[device])
        self.begin_time=now;self.events.append({'kind':'group_takeoff','time_s':now,'devices':list(self.missions)})

    def request_landing(self,reason):
        if self.abort_reason is None:
            self.abort_reason=reason;self.events.append({'kind':'group_landing_requested','reason':reason})
        for mission in self.missions.values():
            if not mission.done:mission.request_landing(reason)

    def check_path_corridor(self,now,device,position,tree,margin):
        distance=float(tree.query(position)[0])
        if distance>margin:
            if not self.skip_path_corridor_check:self.request_landing('measured_path_corridor_exceeded')
            elif device not in self.corridor_bypasses:
                self.corridor_bypasses.add(device)
                self.events.append({'kind':'path_corridor_exceeded_bypassed','time_s':now,'device':device,'distance_m':distance,'limit_m':margin})
        return distance


    def update(self,now):
        if self.begin_time is None:return
        if any(m.landing_requested and m.landing_requested!='mission_complete' for m in self.missions.values()) and self.abort_reason is None:
            self.request_landing('participant_landing_requested')
        if self.abort_reason:return
        if self.epoch is None:
            if now-self.begin_time>self.timeout:self.request_landing('group_readiness_timeout');return
            if all(m.epoch is not None and m.motion_ready and m.landing is None for m in self.missions.values()):
                self.epoch=now+self.lead
                for mission in self.missions.values():mission.schedule_motion(self.epoch,now)
                self.events.append({'kind':'group_motion_scheduled','time_s':now,'motion_epoch_s':self.epoch,'skip_prestart_readiness_check':self.skip_prestart_readiness_check})
        elif now<self.epoch:
            unready=[device for device,m in self.missions.items() if not m.motion_ready]
            if any(m.landing is not None for m in self.missions.values()) or (unready and not self.skip_prestart_readiness_check):
                self.request_landing('readiness_lost_before_common_start')
            elif unready and not self.prestart_loss_reported:
                self.prestart_loss_reported=True
                self.events.append({'kind':'prestart_readiness_loss_bypassed','time_s':now,'motion_epoch_s':self.epoch,
                    'devices':unready,'blocking':{str(d):(self.missions[d].hover_gate or {}).get('blocking',[]) for d in unready}})
        elif not self.started:
            self.started=True;self.events.append({'kind':'group_motion_started','time_s':now,'motion_epoch_s':self.epoch})


def path_points(mission,home):
    reference=mission.reference
    points=reference.table[:,:3]+np.r_[home,0.]
    # Include takeoff, PD transfer, return and descent; sample spacing <=1cm.
    def line(a,b):return np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(np.asarray(b)-a)/.01))+1))
    ground=np.r_[home,0.];hover=np.r_[home,reference.bottom]
    return np.vstack([points,line(ground,hover),line(hover,points[0]),line(points[-1],hover)])


def check_paths(missions,homes,body_radius=BODY_RADIUS_M,tracking_margin=TRACKING_MARGIN_M,minimum_path_separation_m=None,skip_swept_path_check=False):
    if set(missions)!=set(homes) or not np.isfinite([body_radius,tracking_margin]).all() or body_radius<=0 or tracking_margin<=0:
        raise ValueError('Explicit common-world homes and finite positive buffers required')
    if type(skip_swept_path_check) is not bool:raise ValueError('skip_swept_path_check must be a boolean')
    geometric_minimum=2*body_radius+PATH_SAMPLING_ALLOWANCE_M
    default_required=2*(body_radius+tracking_margin)+PATH_SAMPLING_ALLOWANCE_M
    required=default_required if minimum_path_separation_m is None else float(minimum_path_separation_m)
    if not math.isfinite(required) or required<geometric_minimum:
        raise ValueError(f'Minimum path separation must be finite and at least {geometric_minimum:.2f}m (airframe diameter plus sampling allowance)')
    paths={device:path_points(m,homes[device]) for device,m in missions.items()}
    trees={device:cKDTree(p) for device,p in paths.items()}
    full_tubes_disjoint=required>=default_required-1e-12
    pairs=[]
    for index,a in enumerate(missions):
        for b in list(missions)[index+1:]:
            distance=float(trees[a].query(paths[b])[0].min())
            pairs.append({'devices':[a,b],'minimum_swept_center_distance_m':distance,'required_m':required,'clearance_m':distance-required,'meets_clearance':distance>required})
            if distance<=required and not skip_swept_path_check:
                raise ValueError(f'Flip swept paths overlap for {a}/{b}: {distance:.3f}m; require >{required:.3f}m. Parallel separated X-Z lanes are the initial layout.')
    bounds={str(d):[p.min(axis=0).tolist(),p.max(axis=0).tolist()] for d,p in paths.items()}
    all_pairs_meet_clearance=all(p['meets_clearance'] for p in pairs)
    full_tubes_disjoint=full_tubes_disjoint and all_pairs_meet_clearance
    mode='Swept-path overlap gate skipped; distances recorded only' if skip_swept_path_check else 'Disjoint full swept-path tubes' if full_tubes_disjoint else 'Reduced full swept-path clearance; runtime tracking bands can overlap'
    return {'mode':mode+', including takeoff/run-in/flip/recovery/landing',
        'overlap_check_enabled':not skip_swept_path_check,'overlap_check_skipped':skip_swept_path_check,
        'all_pairs_meet_clearance':all_pairs_meet_clearance,
        'pairs':pairs,'world_path_bounds_m':bounds,'body_radius_m':body_radius,'tracking_margin_m':tracking_margin,
        'minimum_path_separation_m':required,'default_minimum_path_separation_m':default_required,
        'minimum_geometric_separation_m':geometric_minimum,'full_tracking_tubes_disjoint':full_tubes_disjoint,
        'planning_margin_per_drone_m':(required-geometric_minimum)/2,
        'continuous_sampling_allowance_m':PATH_SAMPLING_ALLOWANCE_M,
        'scope':('Pairwise swept-path rejection explicitly bypassed; distances are diagnostic only. ' if skip_swept_path_check else '')+'Nominal full sweeps evaluated over any relative path phase. Lowering clearance reduces reserved tracking error; the unchanged runtime corridor can then overlap another aircraft corridor. Not collision avoidance or a guarantee against arbitrary failures/aerodynamic interaction.'},trees


def simulation_homes(frames,spacing=1.2):
    return {frame:[0.,(i-(len(frames)-1)/2)*spacing] for i,frame in enumerate(frames)}


def create_missions(root,configuration,drones,laps=1,bottom=.5,direction=1,skip_horizontal_limit=False):
    return {d['id']:PitchMission(Path(root),deepcopy(configuration),laps,direction,bottom,coordinated=True,skip_horizontal_limit=skip_horizontal_limit) for d in drones}
