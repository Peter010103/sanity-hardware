"""Pure validated paired-benchmark config builder. No worker/network/hardware I/O."""
import argparse
import copy
import json
import math
from pathlib import Path

from sanity_zenoh.indi_hover import make_mission
from sanity_zenoh.high_speed_lemniscate import SPEED_LEVELS, reference_limits, screen_reference


def workspace_root():
    return next(path for path in Path(__file__).resolve().parents if (path/'loco_mjx').is_dir())


def build_config(base_config, *, pattern, inner, speed, schedule=None, radius_m=None, ground_m):
    if pattern not in ('circle','lemniscate') or inner not in ('pid','indi'):
        raise ValueError('Explicit circle/lemniscate and pid/indi required')
    if isinstance(speed,bool) or speed not in (.8,1.,1.5,*SPEED_LEVELS):
        raise ValueError('Explicit speed levels are0.8,1.0,1.5,3,4,5,6m/s')
    if isinstance(ground_m,bool) or not math.isfinite(ground_m):
        raise ValueError('A finite freshly measured ground reference is required')
    schedule=schedule or 'constant_arclength'
    if radius_m is None:radius_m=1.5 if pattern=='circle' else 3.2
    high_speed = speed in SPEED_LEVELS
    if high_speed and (pattern!='lemniscate' or inner!='indi' or schedule!='indi_shaped_angular' or radius_m!=2.):
        raise ValueError('High-speed preparation requires INDI, lemniscate, indi_shaped_angular and explicit radius2m')
    config=copy.deepcopy(base_config)
    if config.get('flight_profile')!='indi_mpc_benchmark':
        raise ValueError('Builder requires the explicit common benchmark base profile')
    config['inner_controller']=inner
    config['mpc']['config']=str(Path(__file__).resolve().parents[1]/'config'/('sanity15_mpc_indi_rate_model.json' if inner=='indi' else 'sanity09_mpc.json'))
    config['ground_indi'].update(thrust_weight=.4,start_climb_on_ack=inner=='indi',launch_boost_weight=1.2 if inner=='indi' else 0.,launch_boost_max_s=.5,launch_slew_up_N_s=3.)
    config['limits']['ground_m']=float(ground_m)
    config['limits']['maximum_run_s']=180.
    soft_tilt,hard_tilt={4.:(60.,75.),5.:(72.,82.),6.:(78.,85.)}.get(speed,(40.,60.))
    config['benchmark_envelope'].update(soft_tilt_deg=soft_tilt,hard_tilt_deg=hard_tilt)
    options=config['trajectory']
    options.update(kind='paired_mpc_circle' if pattern=='circle' else 'paired_mpc_gerono',pattern=pattern,
                   radius_m=radius_m,peak_speed_m_s=float(speed),schedule=schedule,laps=3)
    if high_speed:options.update(reference_limits(speed))
    if speed==3.:
        config['benchmark_envelope'].update(maximum_speed_m_s=4.)
    if high_speed:
        tilt,hard,weight,rate,maximum_speed={3.:(40.,60.,1.5,2.3,4.),4.:(60.,75.,1.8,9.5,5.),5.:(72.,82.,2.2,9.5,6.5),6.:(78.,85.,2.9,9.5,7.5)}[speed]
        config['benchmark_envelope'].update(soft_tilt_deg=tilt,hard_tilt_deg=hard,mpc_rate_limit_rad_s=rate,mpc_thrust_weight=weight,maximum_speed_m_s=maximum_speed)
        config['mpc'].update(config=str(Path(__file__).resolve().parents[3]/f'mpc_{speed:g}mps.json'),model=str(Path(__file__).resolve().parents[3]/f'model_{speed:g}mps.xml'))
        config['high_speed_wire']=speed>3.
    mission=make_mission(config,workspace_root()/'loco_mjx')
    config['limits']['maximum_run_s']=float(mission.minimum_runtime_s)
    make_mission(config,workspace_root()/'loco_mjx')
    config['commissioning']=(f'Explicit matched MPC+{inner.upper()} benchmark; {pattern}, {schedule},radius{radius_m}m,3laps,peak{speed}m/s. '
        'Common measured mass, geometry, command bounds and envelope; INDI uses identified prediction dynamics, PID uses the legacy ideal-rate predictor; takeoff differs: INDI0.4weight ground warmup then immediate climb on guarded ACK, bounded1.2weight launch floor for at most0.5s with3N/s upward slew until confirmed liftoff; PID retains0.4weight and legacy timing with boost disabled. Circle centeredworldXY[0,0] starts[0,-R]; lemniscate startsworldcenter. '
        'Ground supplied by caller: physical launcher must obtain a fresh stationary disabled-bench capture before flight. '
        'limits.pid_only=false selects the common permission-ON guarded protocol; inner_controller and actual native FC ACK determine inner identity.')
    config['benchmark_provenance']=dict(pattern=pattern,inner_controller=inner,speed_m_s=float(speed),schedule=schedule,
        radius_m=float(radius_m),footprint_xy_m=[2.*radius_m,(2. if pattern=='circle' else 1.)*radius_m],
        ground_thrust_weight=config['ground_indi']['thrust_weight'],start_climb_on_ack=config['ground_indi']['start_climb_on_ack'],
        launch_boost_weight=config['ground_indi']['launch_boost_weight'],launch_boost_max_s=config['ground_indi']['launch_boost_max_s'],launch_slew_up_N_s=config['ground_indi']['launch_slew_up_N_s'],
        requested_laps=3,ground_m=float(ground_m),mpc_predictor='identified_INDI' if inner=='indi' else 'legacy_ideal_PID',runtime_rule='max(120,ceil(reference_duration_s+60)), capped180s; ordinary hover cap120s unchanged',
        reference_duration_s=mission.lap_duration_s)
    if schedule=='indi_shaped_angular':
        reference=mission._make_curve([*mission.trajectory_options.center_xy_m,ground_m+mission.limits.height_m])
        design=reference.design_metadata
        config['benchmark_provenance']['reference_design']=dict(design)
        config['commissioning']+=' Explicit reference acceleration/jerk shaping lengthens phase ramps for the selected geometry and peak speed. Mocap coverage is not established by reference shaping.'
    if high_speed:
        screen=screen_reference(speed)
        config['high_speed_preparation']=dict(reference_screen=screen,
            ordinary_live_supported=not screen['ordinary_envelope_violations'],
            required_release='ordinary SZC4' if not screen['ordinary_envelope_violations'] else 'isolated high-speed MPC/envelope plus matched SZC6 receiver/host when rates exceed2.3rad/s',
            scope='Requested peaks and R2 geometry preserved; Measured trajectory tilt guards use the reviewed speed-specific expanded thresholds; PD takeoff/landing command caps and native guards unchanged. The explicit3m/s airborne/braking envelope is40deg soft,60deg hard and4m/s. Offline model study is not flight qualification.')
        if speed==3.:
            config['high_speed_preparation']['airborne_and_braking_envelope']=dict(soft_tilt_deg=40.,hard_tilt_deg=60.,maximum_speed_m_s=4.,
                scope='Latches after MPC admission through fault landing; takeoff/ground retain original guards; MPC tilt remains30deg')
        config['commissioning']+=' HIGH-SPEED PREPARATION: reference bounds explicitly increased to preserve requested peak and two-second ramps; reference screen is not flight qualification.'
    if high_speed:
        config['high_speed_preparation'].update(selected_release_supported=True,selected_wire='SZC6' if speed>3. else 'SZC4',status='experimental host profile; matched receiver capability ACK required before enabling',PD_takeoff_and_landing_thrust_weight=1.5)
        config['high_speed_preparation']['airborne_and_braking_envelope']=dict(config['benchmark_envelope'],scope='Latches after MPC admission through fault landing; pre-admission speed2m/s and ordinary PD/native guards unchanged')
        config['high_speed_preparation']['speed_headroom_rationale']='Caps5/6.5/7.5m/s for requested4/5/6 are above requested peaks and prior nominal model peaks;6cap also exceeds severe model peak6.538m/s. These are hard interruption thresholds, not reference speed changes or room-coverage validation.'
    return config


def main():
    root=workspace_root();parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pattern',choices=['circle','lemniscate'],required=True)
    parser.add_argument('--inner',choices=['pid','indi'],required=True)
    parser.add_argument('--speed',type=float,choices=[.8,1.,1.5,*SPEED_LEVELS],required=True)
    parser.add_argument('--schedule',choices=['constant_arclength','variable_angular','indi_shaped_angular'])
    parser.add_argument('--radius-m',type=float,help='Lemniscate: 2 (4 x 2 m) or 3.2 (historical 6.4 x 3.2 m); circle: 1.5')
    parser.add_argument('--ground-m',type=float,required=True)
    parser.add_argument('--base-config',type=Path,default=root/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity15_indi_mpc_constant_arclength.json')
    parser.add_argument('--config-output',type=Path,required=True)
    args=parser.parse_args()
    config=build_config(json.loads(args.base_config.read_text()),pattern=args.pattern,inner=args.inner,speed=args.speed,schedule=args.schedule,radius_m=args.radius_m,ground_m=args.ground_m)
    args.config_output.parent.mkdir(parents=True,exist_ok=True)
    with args.config_output.open('x') as stream:stream.write(json.dumps(config,indent=2)+'\n')
    print(json.dumps(dict(configuration=str(args.config_output.resolve()),inner_controller=config['inner_controller'],trajectory=config['trajectory'],maximum_run_s=config['limits']['maximum_run_s'],reference_duration_s=config['benchmark_provenance']['reference_duration_s']),indent=2))


if __name__=='__main__':main()
