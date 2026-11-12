"""Recorded experimental Sanity vertical pitch loops over native Zenoh."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
from sanity_zenoh.mission_clock import record_clock_preflight
import signal
import subprocess
import time
import uuid

import numpy as np
from sanity_zenoh.pitch_mission import PitchMission,PitchWorker,wait_for_pitch_receiver


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace",type=Path,required=True)
    parser.add_argument("--fly",action="store_true")
    parser.add_argument("--frames-confirmed",action="store_true")
    parser.add_argument("--plan-only",action="store_true")
    parser.add_argument("--prepare-only",action="store_true",help="Build/cache the MPC solver without ROS, Zenoh or hardware access")
    parser.add_argument("--output",type=Path)
    parser.add_argument("--collective-profile",type=Path)
    parser.add_argument("--config",type=Path)
    parser.add_argument("--laps",type=int,default=5)
    parser.add_argument("--direction",type=int,choices=(-1,1),default=1)
    parser.add_argument("--bottom",type=float,default=.5)
    parser.add_argument("--hover-only",action="store_true",help="Qualify stable MPC hover and land without running the flip reference")
    parser.add_argument("--command-delay-ms",type=float,default=0.,help="Known command-delay prediction only; not a fitted flight lag")
    args=parser.parse_args();root=args.workspace.resolve();pkg=root/"Sanity/src/ros2_sanity_ws/src/sanity_zenoh"
    import yaml
    settings=yaml.safe_load((args.config or pkg/"config/pitch_loop_sequence.yaml").read_text())["sanity_sequence"]["ros__parameters"]
    configuration=json.loads((pkg/"config/sanity09_pitch_loop.json").read_text())
    lease_s=float(configuration.get("command_lease_s",1.))
    if not np.isfinite(lease_s) or not 0<lease_s<=1.:parser.error("Command lease must be in(0,1]seconds")
    configuration["hover_only"]=args.hover_only
    configuration["angular_rate_filter_tau_s"]=settings.get("angular_rate_filter_tau_s",.05)
    configuration["position_filter_tau_s"]=settings.get("position_filter_tau_s",0.)
    configuration["attitude_filter_tau_s"]=settings.get("attitude_filter_tau_s",0.)
    mission=PitchMission(root,configuration,args.laps,args.direction,args.bottom,args.command_delay_ms/1000.)
    plan={"controller":"experimental_full_attitude_pitch_mpc","reference":mission.reference.info,"command_bounds":mission.bounds.tolist(),
        "rate_prediction":mission.rate_observer.info if mission.rate_observer is not None else "legacy","handover_limits":mission.handover_limits.copy(),"command_lease_s":lease_s,"command_delay_s":mission.delay,"host_selection_delay_s":mission.selection_delay,"host_output_delay_s":mission.host_output_delay,"hover_only":args.hover_only,"prior_reference_joint_stress_successes":"5/8; not evidence for the new run-in reference",
        "mocap_filter_tau_s":{"linear_velocity":settings["velocity_filter_tau_s"],"angular_rate":settings.get("angular_rate_filter_tau_s",.05),"position":configuration["position_filter_tau_s"],"attitude":configuration["attitude_filter_tau_s"]},
        "mocap_frame":settings["tracked_frame"],"marker_to_base_xyzw":settings["marker_to_base_xyzw"],"mocap_config":str(args.config or pkg/"config/pitch_loop_sequence.yaml"),"transport":"Zenoh SZC3:rate1e-3rad/s,thrust1e-4N","flight_requested":args.fly,
        "interrupt":"First Ctrl-C finishes the planned loop sequence then lands; second Ctrl-C or SIGTERM disarms immediately. No inverted PD recovery."}
    if mission.reference.duration+75.>120.:parser.error("Too many loops for the120s takeoff/recovery/landing runtime budget")
    if args.plan_only:print(json.dumps(plan,indent=2));return
    if args.prepare_only:
        if args.fly:parser.error("--prepare-only cannot be combined with --fly")
        output=(args.output or root/"outputs"/("sanity_mpc_prepare_"+datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")+"_"+uuid.uuid4().hex[:6])).resolve()
        output.mkdir(parents=True,exist_ok=False)
        path=output/"mpc_config.json";path.write_text(json.dumps(configuration,indent=2)+"\n")
        print("Preparing cached MPC solver; no ROS/Zenoh/hardware connection.",flush=True)
        started=time.monotonic();worker=PitchWorker(root,path,output)
        try:
            info={**worker.info,"startup_s":time.monotonic()-started,"hardware_access":False}
            (output/"solver_info.json").write_text(json.dumps(info,indent=2)+"\n")
            print(f"MPC ready in {info['startup_s']:.2f}s; cache {'hit' if info.get('cache_hit') else 'built'}: {info['cache']}",flush=True)
        finally:worker.close()
        return
    if args.fly and not args.frames_confirmed:parser.error("--frames-confirmed required for physical output")
    output=(args.output or root/"outputs"/("sanity_pitch_"+datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")+("_flight_" if args.fly else "_shadow_")+uuid.uuid4().hex[:6])).resolve()
    output.mkdir(parents=True,exist_ok=False)
    clock_report=record_clock_preflight(output,required=True)
    (output/"mission_plan.json").write_text(json.dumps(plan,indent=2)+"\n")
    sources=list((pkg/"sanity_zenoh").glob("*.py"))+list((root/"loco_mjx/experiments/sanity_pitch_mpc").glob("*.py"))
    sources += [root/"loco_mjx/experiments/sanity_pitch_mpc/run_in_profile.json",pkg/"config/sanity09_pitch_loop.json",pkg/"config/sanity09_esp_collective.json",args.config or pkg/"config/pitch_loop_sequence.yaml",
        root/"Sanity/src/ros2_sanity_ws/scripts/run_sanity_pitch_loop.sh",root/"loco_mjx/experiments/sanity_ctbr_sim2sim/takeoff_controller.py"]
    if mission.rate_observer is not None:sources.append(Path(mission.rate_observer.info["profile"]))
    hashes={}
    for source in sources:
        relative=source.relative_to(root) if source.is_relative_to(root) else Path("external")/source.name
        target=output/"source"/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(source.read_bytes())
        hashes[str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
    (output/"manifest.json").write_text(json.dumps({"source_sha256":hashes,"unix_ns":time.time_ns(),"monotonic_ns":time.monotonic_ns(),"clock_sync":clock_report,"plan":plan},indent=2)+"\n")
    print("Recording:",output,flush=True)
    print("PD takeoff -> move behind loop -> stable MPC hover -> PD landing" if args.hover_only else f"PD takeoff -> move behind loop -> stable MPC hover -> forward run-in -> {args.laps} pitch loops -> forward exit/hover -> PD landing",flush=True)
    print("Physical CTBR ENABLED; SZC3 capability must be acknowledged before arming." if args.fly else "SHADOW: output remains disarmed.",flush=True)
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    from sanity_zenoh.client import SanitySender
    from sanity_zenoh.firmware_collective import FirmwareCollective
    from sanity_zenoh.mocap import MocapEstimator,MocapOutlier,state_age
    from sanity_zenoh.recording import EventRecorder

    class PitchNode(Node):
        def __init__(self):
            super().__init__("sanity_pitch_loop")
            self.worker=None;self.sender=None;self.events=EventRecorder(output/"events.jsonl")
            self.packet=None;self.slot=None;self.error=None;self.finished=False;self.stage="waiting";self.started=time.monotonic();self.last_print=-1.
            self.mapping=FirmwareCollective(args.collective_profile or pkg/"config/sanity09_esp_collective.json",high_rate=True)
            self.estimator=MocapEstimator(settings["parent_frame"],settings["tracked_frame"],settings["world_from_map_xyzw"],
                settings["marker_to_base_xyz"],settings["marker_to_base_xyzw"],settings["velocity_filter_tau_s"],frames_confirmed=args.frames_confirmed,
                angular_rate_filter_tau_s=settings.get("angular_rate_filter_tau_s",.05),
                position_filter_tau_s=configuration["position_filter_tau_s"],attitude_filter_tau_s=configuration["attitude_filter_tau_s"])
            self.subscription=self.create_subscription(TFMessage,settings["tf_topic"],self.receive_tf,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.sender=SanitySender(device=settings["drone_id"],listen=settings["listen"],connect=settings.get("connect") or None,
                telemetry_listen=settings.get("telemetry_listen") or None,diagnostics=settings.get("transport_diagnostics",False),
                ctbr_mapping=self.mapping,physical_units=True,on_event=self.events)
            try:
                status=wait_for_pitch_receiver(self.sender,lease_s,on_event=self.events)
                print(f"Zenoh receiver acknowledged disabled CTBR; FC ready, voltage={status.voltage_mv/1000.:.2f}V.",flush=True)
                self.mapping.set_voltage(status.voltage_mv/1000.)
                _,force=self.mapping.curve()
                configuration["physical_force_range_N"]=[float(force[0]),float(force[-1])]
                configuration["maximum_thrust_N"]=min(configuration["maximum_thrust_N"],float(force[-1]))
                mission.bounds[0,1]=configuration["maximum_thrust_N"]
                mission.reference.validate(configuration)
                path=output/"mpc_config.json";path.write_text(json.dumps(configuration,indent=2)+"\n")
                print("Loading MPC solver; output remains disabled.",flush=True)
                loading_started=time.monotonic();self.worker=PitchWorker(root,path,output)
                print(f"MPC ready in {time.monotonic()-loading_started:.2f}s ({'cache hit' if self.worker.info.get('cache_hit') else 'new cache build'}); waiting for fresh mocap.",flush=True)
                self.events({"kind":"configuration","plan":plan,"controller":self.worker.info,"mapping":self.mapping.provenance,"initial_receiver_status":vars(status)})
            except BaseException:
                self.sender.close();self.events.close();raise
            self.started=time.monotonic()
            self.command_timer=self.create_timer(.01,self.transmit)
            self.control_timer=self.create_timer(.02,self.control)


        def receive_tf(self,message):
            for tf in message.transforms:
                if (tf.header.frame_id,tf.child_frame_id)!=(self.estimator.parent,self.estimator.child):continue
                t,q=tf.transform.translation,tf.transform.rotation;stamp=tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
                self.events({"kind":"raw_tf","stamp_ns":stamp,"translation":[t.x,t.y,t.z],"quaternion_xyzw":[q.x,q.y,q.z,q.w]})
                try:
                    packet=self.estimator.update(tf.header.frame_id,tf.child_frame_id,stamp,[t.x,t.y,t.z],[q.x,q.y,q.z,q.w],self.get_clock().now().nanoseconds,time.monotonic_ns())
                    if packet is not None:self.packet=packet;self.events({"kind":"tf_estimate","packet":packet})
                except MocapOutlier as error:self.events({"kind":"mocap_rejected",**error.diagnostics})
                except (ValueError,TypeError) as error:self.packet={"error":str(error)}


        def state(self):
            return state_age(self.packet or {},time.monotonic_ns())


        def stop(self,reason=None):
            if self.finished:return
            self.error=reason;self.finished=True;self.stage="stopped";self.slot=None
            self.sender.invalidate(reason or "Mission completed")
            self.events({"kind":"stopped","reason":reason,"mpc_rejection":mission.mpc_rejection,"last_mpc_result":self.worker.latest() if self.worker else None})


        def transmit(self):
            try:
                now=time.monotonic()
                if self.finished:self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=lease_s);return
                if bag.poll() is not None:raise ValueError("MCAP recorder exited")
                if self.events.error:raise ValueError("Event recording failed: "+self.events.error)
                status=self.sender.receiver_status()
                if self.stage=="waiting":
                    self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=lease_s)
                    if now-self.started>30.:raise ValueError("Startup timeout waiting for fresh mocap/MCAP/receiver")
                    try:state,age=self.state()
                    except ValueError:return
                    if args.fly and not mission.pd.liftoff.preflight(state,now):
                        self.events({"kind":"liftoff_preflight","record":mission.pd.liftoff.preflight_status})
                        return
                    if not any((output/"mocap").glob("*.mcap")):return
                    if not (status and status.pitch_loop_supported and status.session==self.sender.session_id and not status.failsafe and status.fc_ready and status.fc_age_ms<150):return
                    if args.fly:self.stage="enabling"
                    else:mission.begin(now,state);self.stage="running"
                elif self.stage=="enabling":
                    state,age=self.state()
                    if args.fly and not mission.pd.liftoff.preflight(state,now):
                        self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=lease_s);self.stage="waiting";return
                    self.sender.ctbr(True,0.,(0.,0.,0.),valid_for_s=lease_s)
                    if status and status.enabled and status.session==self.sender.session_id and not status.failsafe:mission.begin(now,state);self.stage="running"
                elif self.stage=="running":
                    self.state()
                    if not status or not status.pitch_loop_supported or not status.fc_ready or status.fc_age_ms>=150:raise ValueError("Receiver/FC status stale or capability lost")
                    if args.fly and (not status.enabled or status.failsafe or self.sender.rearm_required):raise ValueError("Receiver interlock/lease lost")
                    if self.slot is None:
                        if now-mission.started>.04:raise ValueError("No fresh pitch controller command")
                        return
                    if now-self.slot[0]>=.04:raise ValueError("Pitch controller output expired")
                    self.mapping.set_voltage(status.voltage_mv/1000.)
                    command=self.slot[1]
                    self.sender.ctbr(args.fly,float(command[0]),tuple(command[1:]),valid_for_s=lease_s)
            except Exception as error:self.stop("Command: "+str(error))


        def control(self):
            if self.stage!="running" or self.finished:return
            try:
                now=time.monotonic();state,age=self.state()
                if self.worker.error and mission.epoch is not None and mission.landing is None:raise ValueError("Pitch worker: "+self.worker.error)
                self.mapping.set_voltage(self.sender.receiver_status().voltage_mv/1000.)
                force_limit=self.mapping.limit(3.2)
                result=self.worker.latest()
                command,ref=mission.choose(now,state,result,force_limit)
                outgoing=command if args.fly else np.zeros(4)
                mission.commit(now,outgoing)
                self.slot=(now,outgoing)
                self.events({"kind":"control","tick":mission.tick,"elapsed_s":now-mission.started,"phase":mission.phase,"state":state.tolist(),"state_age_s":age,
                    "reference":ref,"handover_gate":mission.gate,"mpc_hover_gate":mission.hover_gate,"motion_epoch":mission.motion_epoch,"command":outgoing.tolist(),"requested_command":command.tolist(),"force_estimate_N":mission.force,"policy_weight":1. if mission.epoch is not None and mission.landing is None else 0.,
                    "result":result,"pd":mission.pd.last,"mapping":self.mapping.last})
                if mission.done:self.stop();return
                if not self.worker.error and mission.landing is None:self.worker.submit(mission.inference_request(now,state,age,self.packet.get("pose_filter_delay_s")))
                if now-self.last_print>=1.:
                    gate=mission.gate if mission.phase.startswith("pd_") else mission.hover_gate if mission.phase=="mpc_initial_hover" else None
                    blocking="; waiting for "+",".join(gate["blocking"]) if gate else ""
                    print(f"{now-mission.started:6.2f}s {mission.phase} z={state[2]:.3f}m{blocking}",flush=True);self.last_print=now
            except Exception as error:self.stop("Controller: "+str(error))


        def close(self):
            self.stop(self.error)
            if self.sender:self.sender.close()
            if self.worker:self.worker.close()
            self.events.close()
            (output/"receipt.json").write_text(json.dumps({"error":self.error,"mpc_rejection":mission.mpc_rejection,"ticks":mission.tick,"landed":mission.done,"phase":mission.phase,
                "mpc_start_monotonic_s":mission.epoch,"motion_start_monotonic_s":mission.motion_epoch,"mission_start_monotonic_s":mission.started,"origin_xy":mission.origin.tolist() if mission.origin is not None else None,"ground_height_m":mission.ground,"reference":mission.reference.info,"time_anchor":{"unix_ns":time.time_ns(),"monotonic_ns":time.monotonic_ns()},"landing_reason":mission.landing.reason if mission.landing else None,
                "sender_shutdown_acknowledged":self.sender.shutdown_acknowledged,"sender_shutdown_error":self.sender.shutdown_error,
                "recording_error":self.events.error,"finished_unix_ns":time.time_ns()},indent=2)+"\n")

    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node=None;bag=None;baglog=(output/"rosbag.log").open("w");executor=SingleThreadedExecutor();interrupts=0
    def interrupt(signum,frame):
        nonlocal interrupts
        interrupts+=1
        if node:
            if signum==signal.SIGINT and interrupts==1 and node.stage=="running":
                mission.request_landing("operator_interrupt")
                print("Landing requested; an active pitch sequence finishes before upright PD landing. Second Ctrl-C disarms immediately.",flush=True)
            else:node.stop("Emergency termination")
        else:raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    error=None
    try:
        bag=subprocess.Popen(["ros2","bag","record","--storage","mcap","--output",str(output/"mocap"),settings["tf_topic"],"/tf_static","/rosout"],stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
        node=PitchNode();executor.add_node(node)
        while rclpy.ok() and not node.finished:executor.spin_once(timeout_sec=.002)
    except BaseException as caught:
        error=str(caught) or type(caught).__name__
        (output/"startup_error.json").write_text(json.dumps({"error":error,"unix_ns":time.time_ns()},indent=2)+"\n")
        if node:node.stop(error)
    finally:
        executor.shutdown(timeout_sec=2)
        if node:node.close();node.destroy_node()
        if bag and bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            try:bag.wait(timeout=10)
            except subprocess.TimeoutExpired:bag.terminate();bag.wait(timeout=5)
        baglog.close()
        if rclpy.ok():rclpy.shutdown()
        print("Results:",output,flush=True)
    if error:raise SystemExit(error)
    if node and node.error:raise SystemExit(node.error)


if __name__=="__main__":main()
