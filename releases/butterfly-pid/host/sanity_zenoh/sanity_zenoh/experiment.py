"""One entry point for recording and running the Sanity takeoff/circle/landing."""

import argparse
from datetime import datetime,timezone
import hashlib
import fcntl
import json
import math
import os
from pathlib import Path
from sanity_zenoh.mission_clock import record_clock_preflight
import signal
import subprocess
import sys
import time
import uuid


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sim",action="store_true",help="isolated ROS/Zenoh closed-loop simulation")
    mode.add_argument("--fly",action="store_true",help="enable physical CTBR, including arm/takeoff")
    parser.add_argument("--workspace",type=Path,required=True)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--config",type=Path)
    parser.add_argument("--controller",choices=["policy","mpc"],default="policy",help="Select learned policy or standalone trajectory-to-CTBR MPC")
    parser.add_argument("--mpc-config",type=Path,help="MPC JSON weights/horizon; defaults to config/sanity09_mpc.json")
    parser.add_argument("--checkpoint",type=Path,help="Override the selected201.3M tracking-priority Sanity CTBR checkpoint")
    parser.add_argument("--residual",action="store_true",help="Use the paired8.4M learned outer tracker")
    parser.add_argument("--residual-checkpoint",type=Path,help="Explicit outer checkpoint override; implies --residual")
    parser.add_argument("--plan-only",action="store_true",help="Print mission geometry/limits without ROS recording, model loading or Zenoh")
    parser.add_argument("--command-speed-limit",type=float,help="Policy velocity-norm cap, <=3m/s; zero=automatic")
    parser.add_argument("--fallback-speed-limit",type=float,help="Measured-speed fallback threshold; zero=automatic")
    parser.add_argument("--ramp-time",type=float,help="Reference acceleration/deceleration duration,s")
    parser.add_argument("--ramp-profile",choices=["linear","smoothstep"],help="Reference speed ramp shape; default from mission config")
    parser.add_argument("--auto-radius",action="store_true",default=None,help="Enlarge the radius to meet the reference acceleration budget")
    parser.add_argument("--max-reference-acceleration",type=float,help="Horizontal reference acceleration budget,m/s²")
    parser.add_argument("--max-reference-tilt",type=float,help="Reference planning tilt limit,deg; must not exceed the policy tilt fallback")
    parser.add_argument("--max-runtime",type=float,help="Total controller runtime budget,s")
    parser.add_argument("--frames-confirmed",action="store_true")
    parser.add_argument("--bypass-voltage-check",action="store_true",help="deprecated compatibility flag; ESP mapping has no voltage-range interlock")
    parser.add_argument("--mission",choices=["circle","lemniscate"],default="circle")
    parser.add_argument("--speed",type=float,default=.3,help="Horizontal cruise speed,m/s")
    parser.add_argument("--height","--base-height",dest="height",type=float,help="Starting hover and mission base height in model-world Z,m; default1m or configured height")
    parser.add_argument("--radius",type=float,default=.5)
    parser.add_argument("--laps",type=int,default=2)
    parser.add_argument("--duration",type=float,default=60.,help="Earliest landing time; extends to finish requested laps and final hover")
    parser.add_argument("--dds-profile",type=Path)
    parser.add_argument("--policy-delay-tick",type=int,default=-1)
    parser.add_argument("--freeze-tf-at",type=float,default=-1.)
    parser.add_argument("--interrupt-at",type=float,default=-1.)
    a = parser.parse_args(argv)
    root = a.workspace.resolve()
    pkg = root/"Sanity/src/ros2_sanity_ws/src/sanity_zenoh"
    config = a.config or pkg/"config/circle_sequence.yaml"
    a.mpc_config = (a.mpc_config or pkg/"config/sanity09_mpc.json").resolve()
    if a.controller=="mpc" and (a.residual or a.residual_checkpoint or a.checkpoint or a.sim):
        parser.error("MPC is a standalone CTBR baseline: no checkpoint/residual; legacy --sim is policy-only (use run_takeoff.py --controller mpc)")
    if a.controller=="mpc" and not a.mpc_config.is_file():
        parser.error("Missing MPC configuration: "+str(a.mpc_config))
    import yaml
    residual = None
    a.residual = a.residual or a.residual_checkpoint is not None
    settings = yaml.safe_load(config.read_text())["sanity_sequence"]["ros__parameters"]
    if a.sim and a.fallback_speed_limit is None:
        a.fallback_speed_limit = 0.
    for name,key,default in [("command_speed_limit","command_speed_limit_m_s",0.),
            ("fallback_speed_limit","fallback_speed_limit_m_s",4.),("ramp_time","mission_ramp_s",2.),
            ("ramp_profile","mission_ramp_profile","linear"),("auto_radius","auto_radius",False),("max_reference_acceleration","max_reference_acceleration_m_s2",3.5),
            ("height","target_height_m",1.),("max_reference_tilt","reference_tilt_limit_deg",20.),("max_runtime","maximum_run_s",300.)]:
        if getattr(a,name) is None:
            setattr(a,name,settings.get(key,default))
    if not math.isfinite(a.max_runtime) or not 120<=a.max_runtime<=1200 or not math.isfinite(a.duration) or not 1<=a.duration<=a.max_runtime-30:
        parser.error("Use maximum runtime in[120,1200]s and a duration with at least30s landing reserve")
    if a.residual:
        from sanity_zenoh.residual_runtime import residual_metadata,CANDIDATE_SHA256
        bundle = root/"outputs/sanity_velocity_residual_20261021"
        inner = a.checkpoint or bundle/"input/inner_201326592.model"
        outer = a.residual_checkpoint or bundle/"training/outer_8388608.model"
        try:
            residual = residual_metadata(outer,inner,bundle/"source")
        except (ValueError,OSError) as error:
            parser.error(str(error))
        if a.residual_checkpoint is None and residual["sha256"]!=CANDIDATE_SHA256:
            parser.error("Default residual candidate checksum mismatch")
        if a.speed>3. or a.command_speed_limit not in (0.,3.):
            parser.error("Residual deployment requires reference speed <=3m/s and --command-speed-limit3 (or automatic)")
        if a.sim:
            parser.error("Legacy --sim does not implement the residual; use its deployment validation harness")
        a.checkpoint = inner
        a.command_speed_limit = 3.
    sys.path.insert(0,str(root/"loco_mjx/experiments/sanity_ctbr_sim2sim"))
    from missions import DeploymentMission
    if not math.isfinite(a.max_reference_tilt) or not 0 < a.max_reference_tilt <= settings.get("fallback_tilt_limit_deg",30.):
        parser.error("Reference tilt limit must be positive, finite and no larger than the policy tilt fallback")
    try:
        profile = DeploymentMission(a.mission,a.speed,a.radius,a.laps,a.command_speed_limit,a.fallback_speed_limit,
            a.ramp_time,a.max_reference_acceleration,a.auto_radius,a.max_reference_tilt,controller=a.controller,ramp_profile=a.ramp_profile)
    except ValueError as error:
        parser.error(str(error))
    plan = profile.metadata()
    fallback_position = settings.get("fallback_position_tolerance_m",1.)
    fallback_tilt = settings.get("fallback_tilt_limit_deg",30.)
    hard_position = settings.get("hard_position_error_limit_m",1.5)
    hard_tilt = settings.get("hard_tilt_limit_deg",45.)
    if not (math.isfinite(hard_position) and 0 < settings.get("handover_position_tolerance_m",.06) < fallback_position < hard_position
            and math.isfinite(fallback_tilt) and math.isfinite(hard_tilt) and 5 < fallback_tilt < hard_tilt <= 80
            and (a.controller=="mpc" or hard_tilt<=45)):
        parser.error("Ordered position/tilt limits required:5 < fallback tilt < hard tilt <=80 degrees; policy hard tilt <=45")
    if a.controller=="mpc":
        mpc_tilt = json.loads(a.mpc_config.read_text()).get("tilt_limit_deg",30.)
        if not math.isfinite(mpc_tilt) or not a.max_reference_tilt <= mpc_tilt <= fallback_tilt:
            parser.error("MPC tilt limits must satisfy: reference <= MPC constraint <= fallback < hard")
    hover_height = a.height
    maximum_altitude = settings.get("maximum_altitude_m",1.5)
    if not (math.isfinite(hover_height) and math.isfinite(maximum_altitude) and settings.get("ground_root_height_m",.0152) < hover_height < maximum_altitude):
        parser.error("Hover height must be above ground and below the finite maximum altitude")
    plan.update(fallback_position_tolerance_m=fallback_position,fallback_tilt_limit_deg=fallback_tilt,
        hard_position_error_limit_m=hard_position,hard_tilt_limit_deg=hard_tilt,target_height_m=hover_height,hard_altitude_limit_m=maximum_altitude)
    if max(profile.minimum_mission_time,a.duration)+profile.landing_reserve_s>a.max_runtime:
        parser.error("Mission exceeds runtime budget with landing reserve; increase --max-runtime")
    checkpoint_explicit = a.checkpoint is not None
    a.checkpoint = (a.checkpoint if a.checkpoint is not None else root/"outputs/sanity_velocity_response_20261021/source/experiments/runs/sanity_ctbr/response_tracking_20261021_long/seed0/models/model_201326592.model").resolve()
    plan.update(checkpoint=str(a.checkpoint),checkpoint_explicit=checkpoint_explicit,
        minimum_requested_landing_time_s=a.duration,maximum_run_s=a.max_runtime,
        checkpoint_speed_validation="Planning checks geometry and declared command limits; closed-loop performance is not established by selecting a checkpoint")
    if residual:
        plan.update(outer_controller=residual,vertical_command_limit_m_s=.2,yaw_gain_per_s=1.,yaw_rate_limit_rad_s=.6,reference_point="root",
            residual_reference_training_max_m_s=residual["manifest"]["configuration"]["reference_speed_max_m_s"],
            residual_reference_extrapolation=a.speed>residual["manifest"]["configuration"]["reference_speed_max_m_s"])
    if a.controller=="mpc":
        plan.update(controller="mpc",checkpoint=None,checkpoint_explicit=False,outer_controller=None,
            mpc_config=str(a.mpc_config),mpc_configuration=json.loads(a.mpc_config.read_text()),
            policy_scope="Standalone CTBR MPC replaces both the velocity policy and outer planner; no velocity-goal cap in this control path",
            checkpoint_speed_validation="Reference feasibility and physical CTBR bounds only; no flight-performance claim")
    else:
        plan["controller"] = "policy"
    if a.plan_only:
        print(json.dumps(plan,indent=2))
        return
    if a.controller!="mpc" and not a.checkpoint.is_file():
        parser.error(f"Checkpoint does not exist: {a.checkpoint}")
    if a.sim and (checkpoint_explicit or a.command_speed_limit!=0. or a.fallback_speed_limit!=0. or a.ramp_time!=2. or a.auto_radius or a.max_reference_tilt!=20. or a.height!=1.):
        parser.error("The legacy ROS-clock simulation does not accept checkpoint/profile overrides; use run_takeoff.py for the new policy")
    if a.sim and (a.mission!="circle" or a.speed!=.3 or a.radius!=.5 or a.laps!=2):
        parser.error("The legacy ROS-clock simulation is circle0.3 only; use the direct takeoff runner for other missions")
    if a.fly and not a.frames_confirmed and not a.config:
        parser.error("Use --frames-confirmed after checking marker/model axes, or supply a calibrated --config")
    if a.sim and a.duration!=60:
        parser.error("Functional simulation follows the fixed60s circle reference before landing")
    if not a.sim and (a.policy_delay_tick>=0 or a.freeze_tf_at>=0 or a.interrupt_at>=0):
        parser.error("Fault injection is simulation-only")
    env = os.environ.copy()
    if a.dds_profile:
        env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(a.dds_profile.resolve())
    if a.sim:
        domain_lock = None
        for domain in range(94,180):
            candidate = open(f"/tmp/sanity_ros_sim_domain_{domain}.lock","a")
            try:
                fcntl.flock(candidate,fcntl.LOCK_EX|fcntl.LOCK_NB)
                domain_lock = candidate
                break
            except BlockingIOError:
                candidate.close()
        if domain_lock is None:
            raise RuntimeError("No isolated simulation DDS domain available")
        env.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY="1",RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
        env.pop("FASTRTPS_DEFAULT_PROFILES_FILE",None)
        env.pop("FASTDDS_DEFAULT_PROFILES_FILE",None)
    mode_name = "sim" if a.sim else "flight" if a.fly else "shadow"
    output = (a.output or root/"outputs"/("sanity_experiment_"+datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")+"_"+mode_name+"_"+uuid.uuid4().hex[:6])).absolute()
    output.mkdir(parents=True,exist_ok=False)
    clock_report=None if a.sim else record_clock_preflight(output,required=True)
    print("Recording:",output,flush=True)
    print(f"Mission: {a.mission} {a.speed:g}m/s, radius={profile.radius:.3f}m; command cap={profile.command_speed_limit:g}m/s{' (unused by CTBR MPC)' if a.controller=='mpc' else ''}, fallback speed={profile.fallback_speed_limit:g}m/s; base height={a.height:g}m",flush=True)
    if plan.get("residual_reference_extrapolation"):
        print("Residual reference exceeds its trained2m/s envelope; experimental deployment with the existing3m/s command cap.",flush=True)
    if a.controller=="mpc":
        print("Controller: standalone CTBR MPC | config:",a.mpc_config,"| no learned policy loaded",flush=True)
    else:
        print("Checkpoint:",a.checkpoint,"| outer:","paired learned residual" if residual else "balanced tracker",flush=True)
    print("Reference XY bounds relative to takeoff origin:",plan["world_xy_bounds_relative_to_origin_m"],"m; additional braking/recovery space required.",flush=True)
    if a.controller!="mpc" and plan["nominal_outer_goal_clipped_fraction"]>0:
        print(f"Reference plus acceleration lead can reach {plan['peak_nominal_outer_goal_m_s']:.3f}m/s; command cap remains {profile.command_speed_limit:g}m/s within the declared policy range.",flush=True)
    (output/"mission_plan.json").write_text(json.dumps(plan,indent=2)+"\n")
    print("Physical CTBR output ENABLED; fresh state/receiver checks precede arming." if a.fly else "Physical output disabled." if not a.sim else "Isolated simulation; no hardware output.",flush=True)
    source = output/"source"
    source.mkdir()
    manifest = {}
    paths = list((pkg/"sanity_zenoh").glob("*.py"))+list((pkg/"config").glob("*"))
    paths += list((root/"loco_mjx/experiments/sanity_ctbr_sim2sim").glob("*.py"))
    paths += [root/"loco_mjx/loco_mjx/environments/robots/sanity_ctbr/data/plane.xml",a.mpc_config]
    paths += [root/"Sanity/sanity-hardware/parameters/actuators.json",root/"Sanity/sanity-hardware/data/processed/actuator/static_points.csv",
        root/"Sanity/src/ros2_sanity_ws/scripts/run_sanity_experiment.sh",
        root/"Sanity/src/ros2_sanity_ws/scripts/assess_landing.py"]
    if residual:
        paths += [Path(p) for p in residual["source_sha256"]]
        paths += [Path(residual["checkpoint"]),Path(residual["inner_checkpoint"])]
    if a.config:
        paths += [a.config.resolve()]
    for path in paths:
        if not path.is_file():
            continue
        relative = path.relative_to(root) if path.is_relative_to(root) else Path("external")/path.name
        dest = source/relative
        dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_bytes(path.read_bytes())
        manifest[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    (output/"manifest.json").write_text(json.dumps({"argv":sys.argv,"mode":mode_name,"source_sha256":manifest,
        "environment":{k:env.get(k) for k in ["ROS_DOMAIN_ID","ROS_LOCALHOST_ONLY","RMW_IMPLEMENTATION","FASTRTPS_DEFAULT_PROFILES_FILE","FASTDDS_DEFAULT_PROFILES_FILE"]},
        "clock_anchor":{"unix_ns":time.time_ns(),"monotonic_ns":time.monotonic_ns()},"clock_sync":clock_report,"mission_plan":plan,
        "recording":"MCAP raw ROS TF/static/rosout; async JSONL native Zenoh payloads, raw/estimated pose, control/command events; actor tensors/actions/history"},indent=2)+"\n")
    ready = output/"recorder.ready"
    baglog = (output/"rosbag.log").open("w")
    controllog = (output/"console.log").open("w")
    bag = controller = None
    progress_stream = None
    interrupts = 0
    def interrupt(signum,frame):
        nonlocal interrupts
        interrupts += 1
        if controller and controller.poll() is None:
            controller.send_signal(signal.SIGINT if interrupts==1 else signal.SIGTERM)
            print("Landing requested; recording continues." if interrupts==1 and a.fly else "Termination requested.",flush=True)
    signal.signal(signal.SIGINT,interrupt)
    signal.signal(signal.SIGTERM,interrupt)
    receipt = {"mode":mode_name,"hardware_output_requested":a.fly,"started_unix_ns":time.time_ns()}
    try:
        bag = subprocess.Popen(["ros2","bag","record","--storage","mcap","--output",str(output/"mocap"),
            "/tf","/tf_static","/rosout","/clock"],stdout=baglog,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        deadline = time.monotonic()+15
        while not list((output/"mocap").glob("*.mcap")):
            if bag.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError("ROS bag recorder did not start; see rosbag.log")
            time.sleep(.05)
        if a.bypass_voltage_check:
            print("Voltage range interlock is disabled by default; --bypass-voltage-check is unnecessary.",flush=True)
        ready.touch()
        print("MCAP recording started; loading controller/checkpoint.",flush=True)
        if a.sim:
            command = [sys.executable,"-m","sanity_zenoh.clocked_sequence_sim","--workspace",str(root),"--output",str(output/"simulation"),
                "--emulator",str(root/"outputs/sanity_local_policy_20261016/emulator/receiver_emulator"),"--recorder-ready-file",str(ready),
                "--policy-delay-tick",str(a.policy_delay_tick),"--freeze-tf-at",str(a.freeze_tf_at),"--interrupt-at",str(a.interrupt_at)]
            if a.bypass_voltage_check:
                command += ["--bypass-voltage-check"]
        else:
            command = [sys.executable,"-m","sanity_zenoh.ros_sequence","--ros-args","--params-file",str(a.config or pkg/"config/circle_sequence.yaml"),
                "-p","loco_root:="+str(root/"loco_mjx"),
                "-p","controller:="+a.controller,"-p","mpc_config:="+str(a.mpc_config),
                "-p","policy_python:="+str(root/".venv-sanity-train/bin/python"),"-p","output:="+str(output/"controller"),
                "-p","bypass_voltage_check:="+str(a.bypass_voltage_check).lower(),
                "-p","enable_output:="+str(a.fly).lower(),"-p","duration_s:="+str(a.duration),
                "-p","mission_pattern:="+a.mission,"-p","speed_m_s:="+str(a.speed),
                "-p","radius_m:="+str(a.radius),"-p","laps:="+str(a.laps),
                "-p","target_height_m:="+str(a.height),
                "-p","command_speed_limit_m_s:="+str(a.command_speed_limit),
                "-p","fallback_speed_limit_m_s:="+str(a.fallback_speed_limit),
                "-p","mission_ramp_s:="+str(a.ramp_time),"-p","mission_ramp_profile:="+a.ramp_profile,"-p","auto_radius:="+str(a.auto_radius).lower(),
                "-p","max_reference_acceleration_m_s2:="+str(a.max_reference_acceleration),
                "-p","reference_tilt_limit_deg:="+str(a.max_reference_tilt),
                "-p","maximum_run_s:="+str(a.max_runtime),
                "-p","motor_data_root:="+str(root/"Sanity/sanity-hardware"),"-p","motor_profile:="+str(pkg/"config/sanity09_motor_collective.json"),
                "-p","esp_collective_profile:="+str(pkg/"config/sanity09_esp_collective.json"),
                "-p","recorder_ready_file:="+str(ready)]
            if a.controller=="policy":
                command += ["-p","checkpoint:="+str(a.checkpoint)]
            if residual:
                command += ["-p","residual_model:="+residual["checkpoint"],"-p","residual_source:="+residual["source"]]
            if a.frames_confirmed:
                command += ["-p","frames_confirmed:=true"]
        receipt["command"] = command
        controller = subprocess.Popen(command,stdout=controllog,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        started = time.monotonic()
        run = output/"simulation/controller" if a.sim else output/"controller"
        next_progress = time.monotonic()+3
        while controller.poll() is None:
            if time.monotonic()>=next_progress:
                event_path = run/"events.jsonl"
                if progress_stream is None and event_path.is_file():
                    progress_stream = event_path.open()
                last = None
                if progress_stream:
                    while True:
                        position = progress_stream.tell()
                        line = progress_stream.readline()
                        if not line:
                            break
                        if not line.endswith("\n"):
                            progress_stream.seek(position)
                            break
                        event = json.loads(line)
                        if event["kind"]=="control":
                            last = event["record"]
                if last:
                    print(f"{last['time_s']:6.2f}s  {last['phase']}  z={last['state'][2]:.3f}m  {a.controller}={last['policy_weight']:.2f}",flush=True)
                next_progress = time.monotonic()+3
            if bag.poll() is not None:
                ready.unlink(missing_ok=True)
            if not a.sim and time.monotonic()-started>40 and not (run/"events.jsonl").exists():
                raise TimeoutError("Controller startup failed; see console.log")
            if time.monotonic()-started>(360 if a.sim else a.max_runtime+55):
                controller.send_signal(signal.SIGTERM)
                raise TimeoutError("Experiment maximum elapsed time exceeded")
            time.sleep(.1)
        receipt["controller_returncode"] = controller.returncode
    finally:
        if controller and controller.poll() is None:
            controller.send_signal(signal.SIGTERM)
            controller.wait(timeout=10)
        if bag:
            if bag.poll() is None:
                bag.send_signal(signal.SIGINT)
            bag.wait(timeout=15)
            receipt["rosbag_returncode"] = bag.returncode
        ready.unlink(missing_ok=True)
        receipt["finished_unix_ns"] = time.time_ns()
        (output/"experiment_receipt.json").write_text(json.dumps(receipt,indent=2)+"\n")
        if progress_stream:
            progress_stream.close()
        baglog.close()
        controllog.close()
    if not (run/"receipt.json").is_file():
        print("Controller failed before writing a receipt. See",output/"console.log",flush=True)
        raise SystemExit(1)
    assessor = root/"Sanity/src/ros2_sanity_ws/scripts/assess_landing.py"
    analysis = subprocess.run([sys.executable,str(assessor),str(output)],env=env)
    print("Results:",output/"summary.json",flush=True)
    if receipt.get("controller_returncode") or analysis.returncode:
        raise SystemExit(1)


if __name__=="__main__":
    main()
