#!/usr/bin/env python3
"""Named MPC flight recipes. Default action is offline planning."""
import argparse
from datetime import datetime,timezone
import fcntl
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import uuid

from configure_lab import ROOT,PACKAGE,configuration,load_lab,resolve,write_config
from recipes import RECIPES
from session import OwnedSession,inventory

WS=ROOT/'Sanity/src/ros2_sanity_ws'
PYTHON=ROOT/'.venv-sanity-deploy/bin/python'


def release_dir(family):
    return ROOT/'releases'/('high-speed' if family=='high-speed' else family)


def environment(family,lab):
    env=os.environ.copy()
    package=release_dir(family)/'host/sanity_zenoh' if family in ('high-speed','butterfly-indi','butterfly-pid') else PACKAGE
    env['PYTHONPATH']=str(package)+':'+str(WS/'src/udp_broadcaster')+(':'+env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
    env.update(SANITY_MOCAP_NTP_SERVER=lab['mocap_ntp_server'],OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    return env


def command_for(recipe,config,args,lab):
    family=recipe['family'];mode='--prepare-only' if args.prepare_only else '--fly' if args.fly else '--bench-check' if args.bench_check else '--plan-only'
    package=PACKAGE
    if family=='legacy':
        settings=package/'config'/('sanity09_mpc_high_bank.json' if recipe.get('high_bank') else 'sanity09_mpc.json')
        if args.prepare_only:
            return [str(PYTHON),str(ROOT/'scripts/prepare_solver.py'),'--source',str(ROOT/'loco_mjx/experiments/sanity_ctbr_sim2sim/mpc_controller.py'),'--config',str(settings),'--model',str(ROOT/'loco_mjx/loco_mjx/environments/robots/sanity_ctbr/data/plane.xml'),'--output',str(args.output/'solver')]
        result=[str(PYTHON),'-m','sanity_zenoh.experiment','--workspace',str(ROOT),'--controller','mpc',
            '--config',str(config),'--mpc-config',str(settings),'--collective-profile',str(resolve(lab['pid']['collective_profile'])),
            '--mission',recipe['pattern'],'--speed',str(recipe['speed']),'--radius',str(recipe['radius']),'--laps',str(recipe['laps']),
            '--height','1.0','--output',str(args.output/'flight'),mode]
        if recipe.get('speed')==3.:result+=['--max-reference-acceleration','5','--max-reference-tilt','30']
        if recipe.get('high_bank'):result+=['--ramp-time','4','--ramp-profile','smoothstep','--max-reference-acceleration','13','--max-reference-tilt','58']
    elif family=='pitch':
        result=[str(PYTHON),'-m','sanity_zenoh.pitch_experiment','--workspace',str(ROOT),'--config',str(config),
            '--collective-profile',str(resolve(lab['pid']['collective_profile'])),'--laps',str(recipe['laps']),'--bottom','0.5',
            '--output',str(args.output/'flight'),mode]
        if args.hover_only:result+=['--hover-only']
    else:
        if args.prepare_only:
            # Build exactly the chosen mission/config with no ROS, sender or devices.
            sys.path[:0]=[str(PACKAGE),str(WS/'src/udp_broadcaster')]
            if family in ('high-speed','butterfly-indi','butterfly-pid'):sys.path.insert(0,str(release_dir(family)/'host/sanity_zenoh'))
            from sanity_zenoh.mpc_benchmark_config import build_config
            actual=build_config(json.loads(config.read_text()),pattern='lemniscate',inner=recipe['inner'],speed=recipe['speed'],
                schedule=recipe['schedule'],radius_m=recipe['radius'],ground_m=lab['indi']['ground_m'],**(dict(width_m=3.,butterfly_height_m=.3,butterfly_cycles=3) if family.startswith('butterfly') else {}))
            write_config(args.output/'prepared_configuration.json',actual)
            source=release_dir(family)/'mpc_controller.py' if family in ('high-speed','butterfly-indi','butterfly-pid') else ROOT/'loco_mjx/experiments/sanity_ctbr_sim2sim/mpc_controller.py'
            return [str(PYTHON),str(ROOT/'scripts/prepare_solver.py'),'--source',str(source),'--config',actual['mpc']['config'],'--model',actual['mpc']['model'],'--output',str(args.output/'solver')]
        launcher=release_dir(family)/'run_high_speed_lemniscate.py' if family in ('high-speed','butterfly-indi','butterfly-pid') else WS/'scripts/run_sanity_mpc_benchmark.py'
        result=[str(PYTHON),str(launcher),'--inner',recipe['inner'],'--pattern','lemniscate','--schedule',recipe['schedule'],
            '--speed',str(recipe['speed']),'--radius-m',str(recipe['radius']),'--base-config',str(config),'--output',str(args.output/'capture'),mode]
    if family.startswith('butterfly') and not args.prepare_only:result+=['--width-m','3','--butterfly-height-m','.3','--butterfly-cycles','3']
    if args.frames_confirmed:result+=['--frames-confirmed']
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recipe',nargs='?',choices=sorted(RECIPES));p.add_argument('--list',action='store_true')
    p.add_argument('--lab-config',type=Path);p.add_argument('--output',type=Path)
    modes=p.add_mutually_exclusive_group();modes.add_argument('--plan-only',action='store_true');modes.add_argument('--prepare-only',action='store_true')
    modes.add_argument('--bench-check',action='store_true');modes.add_argument('--fly',action='store_true')
    p.add_argument('--frames-confirmed',action='store_true');p.add_argument('--hover-only',action='store_true',help='Pitch recipe: qualify MPC hover without a flip')
    args=p.parse_args()
    if args.list:
        print(json.dumps(RECIPES,indent=2));return 0
    if not args.recipe:p.error('Choose a recipe or --list')
    recipe=RECIPES[args.recipe];live=args.fly or args.bench_check
    if live and (not args.frames_confirmed or not args.lab_config):p.error('Live runs require --frames-confirmed and an explicit --lab-config')
    if args.bench_check and recipe['family'] not in ('benchmark','high-speed','butterfly-indi','butterfly-pid'):p.error('Disabled combined bench applies to native benchmark recipes; use the legacy receiver commissioning procedure for PID/flip profiles')
    if args.hover_only and recipe['family']!='pitch':p.error('--hover-only is a pitch qualification mode')
    lab=load_lab((args.lab_config or ROOT/'config/lab.example.json').resolve())
    os.chdir(ROOT)
    args.output=(args.output or ROOT/'outputs'/f'{args.recipe}_{datetime.now(timezone.utc):%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}').resolve()
    args.output.mkdir(parents=True,exist_ok=False)
    config=args.output/('local_config.yaml' if recipe['family'] in ('legacy','pitch') else 'local_config.json')
    write_config(config,configuration(lab,recipe))
    command=command_for(recipe,config,args,lab);env=environment(recipe['family'],lab)
    plan=dict(recipe=args.recipe,description=recipe,effective_lab=lab,command=command,output=str(args.output),
        action='flight' if args.fly else 'disabled_bench' if args.bench_check else 'solver_preparation' if args.prepare_only else 'offline_plan',
        network_or_hardware_access=live,software_revision='standalone MPC snapshot; exact source hashes recorded in branch')
    (args.output/'launch_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    print('Output:',args.output,flush=True)
    if not live:
        result=subprocess.run(command,cwd=ROOT,env=env)
        return result.returncode
    # One invocation owns the local command listeners/aircraft experiment session.
    with open('/tmp/sanity_mpc_flight_session.lock','a') as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        (args.output/'lab_process_inventory.json').write_text(json.dumps(inventory(),indent=2)+'\n')
        # Source ROS only in live modes. The clock check itself is read-only.
        if recipe['family'] in ('legacy','pitch'):
            wrapper='source '+shlex.quote(str(WS/'scripts/sanity_clock_preflight.sh'))+' --fly; exec '+shlex.join(command)
        else:wrapper='exec '+shlex.join(command)
        shell=['bash','-c','set -e; source /opt/ros/jazzy/setup.bash; '+wrapper]
        session=OwnedSession(args.output/'owned_processes.json')
        try:code=session.run(shell,cwd=ROOT,env=env)
        finally:session.close()
    return code

if __name__=='__main__':raise SystemExit(main())
