"""Recorded PD takeoff, flight-identification pulses and landing over native Zenoh."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
from sanity_zenoh.mission_clock import record_clock_preflight
import signal
import subprocess
import sys
import threading
import time
import uuid

import numpy as np
from sanity_zenoh.identification_mission import IdentificationMission


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace",type=Path,required=True)
    parser.add_argument("--mission",choices=("hover","heights","collective","roll","pitch","yaw","rate_sine"),default="heights")
    parser.add_argument("--autotune",action="store_true")
    parser.add_argument("--axes",nargs="+",choices=("roll","pitch"),default=["roll","pitch"])
    parser.add_argument("--evaluations-per-axis",type=int,default=4)
    parser.add_argument("--tune-d",action="store_true",help="Also test bounded D increases;50Hz telemetry does not qualify high-frequency motor noise")
    parser.add_argument("--tuning-profile",choices=("legacy","damping","boost"),default="legacy",help="Isolated paired D/D-min or boost comparisons with repeated validation")
    parser.add_argument("--step-duration",type=float,default=.4,help="Autotune rate hold in seconds (default0.4)")
    parser.add_argument("--step-amplitudes",type=float,nargs=2,default=[.2,.3],metavar=("SMALL","LARGE"),help="Autotune positive rate amplitudes inrad/s; both signs are tested")
    parser.add_argument("--save-best",action="store_true",help="Save validated best gains only after completed landing and FC disarm; otherwise restore original")
    parser.add_argument("--sine-frequencies",type=float,nargs="+",default=[.5,1.,2.],help="Rate-sine frequencies in0.5..4Hz")
    parser.add_argument("--sine-amplitude",type=float,default=.3,help="Steady rate injection amplitude in0.1..0.3rad/s")
    parser.add_argument("--sine-repeats",type=int,default=2,help="One or two repeats of each axis/frequency")
    parser.add_argument("--fly",action="store_true")
    parser.add_argument("--frames-confirmed",action="store_true")
    parser.add_argument("--plan-only",action="store_true")
    parser.add_argument("--height",type=float,default=1.)
    parser.add_argument("--heights",type=float,nargs="+",default=[.5,.8,1.1,.8,.5])
    parser.add_argument("--hold",type=float,default=10.)
    parser.add_argument("--mass",type=float,default=.06985)
    parser.add_argument("--floor-z",type=float,help="Measured floor datum in mocap world; otherwise use initial tracked ground height")
    parser.add_argument("--seed",type=int,default=0)
    parser.add_argument("--config",type=Path)
    parser.add_argument("--output",type=Path)
    args=parser.parse_args();root=args.workspace.resolve();pkg=root/"Sanity/src/ros2_sanity_ws/src/sanity_zenoh"
    try:
        if args.autotune and args.mission=="rate_sine":raise ValueError("rate_sine is fixed-gain; do not combine with --autotune")
        if args.mission=="rate_sine" and args.save_best:raise ValueError("rate_sine never saves or changes gains")
        if args.autotune:
            from sanity_zenoh.autotune_mission import AutotuneMission
            mission=AutotuneMission(root,args.height,args.mass,args.axes,args.evaluations_per_axis,args.tune_d,args.save_best,args.floor_z,args.tuning_profile,args.step_duration,args.step_amplitudes);args.mission="autotune"
        elif args.mission=="rate_sine":
            from sanity_zenoh.sine_mission import SineIdentificationMission
            mission=SineIdentificationMission(root,height=args.height,mass=args.mass,axes=args.axes,frequencies=args.sine_frequencies,
                amplitude=args.sine_amplitude,repeats=args.sine_repeats,floor_z=args.floor_z)
        else:mission=IdentificationMission(root,args.mission,args.height,args.heights,args.hold,args.mass,args.seed,args.floor_z)
    except ValueError as error:parser.error(str(error))
    import yaml
    config=args.config or pkg/"config/pitch_loop_sequence.yaml"
    settings=yaml.safe_load(config.read_text())["sanity_sequence"]["ros__parameters"]
    plan={**mission.info,"flight_requested":args.fly,"mocap_config":str(config),"mocap_frame":settings["tracked_frame"],
        "marker_to_base_xyzw":settings["marker_to_base_xyzw"],"recording":"MCAP raw TF and JSONL selected CTBR/probe/PD/native Zenoh/FC/ESP telemetry; no Blackbox",
        "mocap_filter_tau_s":{"linear_velocity":settings["velocity_filter_tau_s"],"angular_rate":settings.get("angular_rate_filter_tau_s",.05)},
        "startup_telemetry":"Fresh gyro and FC motor-output responses required before arming; RPM optional. Missing telemetry pauses entry to later probes."}
    if args.plan_only or not args.fly:print(json.dumps(plan,indent=2));return
    if not args.frames_confirmed:parser.error("--frames-confirmed required for physical output")
    output=(args.output or root/"outputs"/("sanity_identification_"+datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")+"_"+args.mission+"_flight_"+uuid.uuid4().hex[:6])).resolve()
    output.mkdir(parents=True,exist_ok=False)
    clock_report=record_clock_preflight(output,required=True)
    (output/"mission_plan.json").write_text(json.dumps(plan,indent=2)+"\n")
    sources=list((pkg/"sanity_zenoh").glob("*.py"))+[config,pkg/"config/sanity09_esp_collective.json",
        root/"loco_mjx/experiments/sanity_ctbr_sim2sim/takeoff_controller.py",root/"Sanity/src/ros2_sanity_ws/scripts/run_sanity_identification.sh"]
    if args.mission=="rate_sine":sources.append(root/"Sanity/src/ros2_sanity_ws/scripts/run_sanity_rate_sine.sh")
    hashes={}
    for source in sources:
        relative=source.relative_to(root) if source.is_relative_to(root) else Path("external")/source.name
        target=output/"source"/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(source.read_bytes())
        hashes[str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
    (output/"manifest.json").write_text(json.dumps({"argv":sys.argv,"source_sha256":hashes,"unix_ns":time.time_ns(),"monotonic_ns":time.monotonic_ns(),"clock_sync":clock_report,"plan":plan},indent=2)+"\n")
    print("Recording:",output,flush=True)
    print("PD takeoff ->",args.mission,"identification -> PD landing; Blackbox skipped",flush=True)
    print("Physical CTBR ENABLED; fresh mocap, FC and gyro/motor telemetry precede arming.",flush=True)
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    from sanity_zenoh.client import SanitySender
    from sanity_zenoh.firmware_collective import FirmwareCollective
    from sanity_zenoh.mocap import MocapEstimator,MocapOutlier,state_age
    from sanity_zenoh.recording import EventRecorder
    from sanity_zenoh.telemetry_capture import summarize_telemetry

    class IdentificationNode(Node):
        def __init__(self):
            super().__init__("sanity_identification")
            self.sender=None;self.events=EventRecorder(output/"events.jsonl")
            self.packet=None;self.slot=None;self.error=None;self.finished=False;self.stage="waiting";self.started=time.monotonic();self.last_print=-1.
            self.telemetry_lock=threading.Lock();self.telemetry_seen={};self.gyro_rad_s_fc=None
            self.mapping=FirmwareCollective(pkg/"config/sanity09_esp_collective.json")
            self.estimator=MocapEstimator(settings["parent_frame"],settings["tracked_frame"],settings["world_from_map_xyzw"],
                settings["marker_to_base_xyz"],settings["marker_to_base_xyzw"],settings["velocity_filter_tau_s"],frames_confirmed=args.frames_confirmed,
                angular_rate_filter_tau_s=settings.get("angular_rate_filter_tau_s",.05))
            self.subscription=self.create_subscription(TFMessage,settings["tf_topic"],self.receive_tf,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
            try:
                self.sender=SanitySender(device=settings["drone_id"],listen=settings["listen"],connect=settings.get("connect") or None,
                    telemetry_listen=settings.get("telemetry_listen") or None,diagnostics=settings.get("transport_diagnostics",False),
                    ctbr_mapping=self.mapping,physical_units=True,on_event=self.receive_event)
                deadline=time.monotonic()+12.
                while True:
                    self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.)
                    status=self.sender.receiver_status()
                    if status and status.session==self.sender.session_id and not status.failsafe and status.fc_ready and status.fc_age_ms<150 and self.telemetry_ready(status):break
                    if time.monotonic()>deadline:
                        blocked=[]
                        if status is None:blocked.append("receiver status")
                        elif status.session!=self.sender.session_id or status.failsafe:blocked.append("disabled ownership acknowledgement")
                        if status is None or not status.fc_ready or status.fc_age_ms>=150:blocked.append("FC UART readiness")
                        if not self.telemetry_ready(status):blocked.append("fresh gyro/motor telemetry on "+(settings.get("telemetry_listen") or settings["listen"]))
                        self.events({"kind":"startup_blocked","blocking":blocked,"receiver_status":vars(status) if status else None,"telemetry_commands_seen":list(self.telemetry_seen)})
                        raise ValueError("Startup blocked by: "+", ".join(blocked)+". Output remains disabled.")
                    time.sleep(.01)
                if args.autotune:
                    from sanity_zenoh.tuning_rpc import MspTuningRpc
                    if not status.tuning_supported:raise ValueError("ESP firmware lacks MSP tuning capability; update before flight")
                    rpc=MspTuningRpc(self.sender)
                    try:mission.attach(rpc,output,lambda:self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.))
                    except BaseException:rpc.close();raise
                if args.mission=="rate_sine":
                    from sanity_zenoh.tuning_rpc import MspTuningRpc
                    rpc=MspTuningRpc(self.sender)
                    try:
                        snapshot=rpc.snapshot(lambda:self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.))
                        (output/"original_tune.json").write_text(json.dumps(snapshot.json(),indent=2)+"\n")
                        self.events({"kind":"fixed_gain_snapshot","gains":snapshot.json(),"writes":False})
                    finally:rpc.close()
                self.events({"kind":"configuration","plan":plan,"mapping":self.mapping.provenance,"initial_receiver_status":vars(status)})
            except BaseException:
                if self.sender:self.sender.close()
                self.events.close();raise
            self.started=time.monotonic()
            self.command_timer=self.create_timer(.01,self.transmit)
            self.control_timer=self.create_timer(.02,self.control)


        def receive_event(self,event):
            if args.autotune:mission.observe_event(event)
            if event.get("kind")=="zenoh_rx_fc_telemetry":
                with self.telemetry_lock:
                    for record in event.get("telemetry",{}).get("records",[]):
                        if record["type"]==2 and record["msp_command"] in (102,104) and "decode_error" not in record:
                            self.telemetry_seen[record["msp_command"]]=(time.monotonic(),record["epoch"],record["response_parsed_esp_us"])
                            if record["msp_command"]==102:self.gyro_rad_s_fc=record.get("gyro_rad_s_fc")
            self.events(event)


        def telemetry_ready(self,status):
            if status is None:return False
            with self.telemetry_lock:
                values=[self.telemetry_seen.get(command) for command in (102,104)]
                gyro_ok=self.gyro_rad_s_fc is not None
            return gyro_ok and all(value is not None and time.monotonic()-value[0]<.5 and value[1]==status.epoch and
                -.1<((status.receiver_ms*1000-value[2])/1e6)<.5 for value in values)


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
            if self.sender:self.sender.invalidate(reason or "Identification completed")
            self.events({"kind":"stopped","reason":reason})


        def transmit(self):
            try:
                now=time.monotonic()
                if self.finished:self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.);return
                if bag.poll() is not None:raise ValueError("MCAP recorder exited")
                if self.events.error:raise ValueError("Event recording failed: "+self.events.error)
                status=self.sender.receiver_status()
                if self.stage=="waiting":
                    self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.)
                    if now-self.started>30.:raise ValueError("Startup timeout waiting for fresh mocap/MCAP/FC telemetry")
                    try:state,age=self.state()
                    except ValueError:return
                    if args.fly and not mission.pd.liftoff.preflight(state,now):
                        self.events({"kind":"liftoff_preflight","record":mission.pd.liftoff.preflight_status})
                        return
                    if not any((output/"mocap").glob("*.mcap")):return
                    if status and status.session==self.sender.session_id and not status.failsafe and status.fc_ready and status.fc_age_ms<150 and self.telemetry_ready(status):self.stage="enabling"
                elif self.stage=="enabling":
                    state,age=self.state()
                    if args.fly and not mission.pd.liftoff.preflight(state,now):
                        self.sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.);self.stage="waiting";return
                    if not status or not self.telemetry_ready(status):raise ValueError("Telemetry lost before arming")
                    self.sender.ctbr(True,0.,(0.,0.,0.),valid_for_s=1.)
                    if status.enabled and status.session==self.sender.session_id and not status.failsafe:
                        mission.begin(now,state);self.stage="running"
                elif self.stage=="running":
                    self.state()
                    if not status or not status.fc_ready or status.fc_age_ms>=150:raise ValueError("Receiver/FC status stale")
                    if not status.enabled or status.failsafe or self.sender.rearm_required:raise ValueError("Receiver interlock/lease lost")
                    if self.slot is None:
                        if now-mission.started>.04:raise ValueError("No fresh PD controller command")
                        return
                    if now-self.slot[0]>=.04:
                        # ROS callback ordering can run TX before the next PD tick.
                        # Fresh state/FC were checked above: compute an abort landing,
                        # rather than sending disarm because one callback was late.
                        mission.request_landing('controller_deadline_missed')
                        self.control()
                        if self.finished:return
                        if self.slot is None or time.monotonic()-self.slot[0]>=.04:
                            raise ValueError('Unable to generate fresh PD landing command')
                    self.mapping.set_voltage(status.voltage_mv/1000.)
                    command=self.slot[1]
                    self.sender.ctbr(True,float(command[0]),tuple(command[1:]),valid_for_s=1.)
            except Exception as error:self.stop("Command: "+str(error))


        def control(self):
            if self.stage!="running" or self.finished:return
            try:
                now=time.monotonic()
                if mission.last_now is not None and now-mission.last_now<.005:return
                state,age=self.state();status=self.sender.receiver_status()
                if status is None or not status.enabled or status.failsafe or not status.fc_ready or status.fc_age_ms>=150:
                    raise ValueError("Fresh enabled receiver/FC required for PD control")
                self.mapping.set_voltage(status.voltage_mv/1000.)
                telemetry_ok=self.telemetry_ready(status)
                with self.telemetry_lock:gyro=self.gyro_rad_s_fc if telemetry_ok else None
                command,ref,probe=mission.choose(now,state,self.mapping.limit(2.85),telemetry_ok,gyro_rad_s_fc=gyro)
                mission.commit(now,command);self.slot=(now,command)
                self.events({"kind":"control","tick":mission.tick,"elapsed_s":now-mission.started,"phase":mission.phase,"state":state.tolist(),"state_age_s":age,
                    "reference":ref,"command":command.tolist(),"selected_history_command":mission.history[-1][1].tolist(),
                    "probe":probe,"settled_gate":mission.gate,"telemetry_ready":telemetry_ok,"pd":mission.pd.last,"mapping":self.mapping.last})
                for event in mission.pending_events:
                    event={"elapsed_s":now-mission.started,**event}
                    if args.autotune:mission.probe_event(event)
                    self.events({"elapsed_s":now-mission.started,**event})
                    if event["kind"] in ("probe_begin","probe_end"):
                        print(f"{now-mission.started:6.2f}s {event['kind']} trial={event['trial_index']+1}/{len(mission.trials)}",flush=True)
                mission.pending_events=[]
                if mission.done:self.stop();return
                if now-self.last_print>=1.:
                    index=probe["trial_index"]
                    gate=mission.gate if mission.phase in ("settle","recovery") else None
                    waiting=(f" waiting={','.join(gate['blocking']) or 'dwell'} gyro={gate['rate_filtered_norm_rad_s']:.3f}/{gate['rate_limit_rad_s']:.1f}rad/s" if gate else "")
                    print(f"{now-mission.started:6.2f}s {mission.phase} height={state[2]-mission.datum:.3f}m"+(f" trial={index+1}/{len(mission.trials)}" if index is not None else "")+waiting,flush=True)
                    self.last_print=now
            except Exception as error:self.stop("Controller: "+str(error))


        def close(self):
            self.stop(self.error)
            if args.autotune and mission.rpc:mission.finalize(self.sender,self.events)
            if self.sender:self.sender.close()
            self.events.close()
            receipt={"error":self.error,"mission":args.mission,"ticks":mission.tick,"landed":mission.done,"phase":mission.phase,
                "mission_start_monotonic_s":mission.started,"origin_xy":mission.origin.tolist() if mission.origin is not None else None,
                "initial_tracked_ground_height_m":mission.ground,"height_datum_m":mission.datum,"trials_completed":len(mission.trial_results),"trials_planned":len(mission.trials),
                "trial_results":mission.trial_results,"landing_reason":mission.landing.reason if mission.landing else None,
                "profile_completed":bool(mission.done and mission.landing and mission.landing.reason==("autotune_complete" if args.autotune else "identification_complete")),
                "sender_shutdown_acknowledged":self.sender.shutdown_acknowledged if self.sender else False,
                "sender_shutdown_error":self.sender.shutdown_error if self.sender else None,"recording_error":self.events.error,"finished_unix_ns":time.time_ns(),"native_FC_logging":False}
            if args.autotune:receipt["autotune"]=mission.tuning_receipt
            (output/"receipt.json").write_text(json.dumps(receipt,indent=2)+"\n")
            print(f"Identification completed={receipt['profile_completed']}; probes={len(mission.trial_results)}/{len(mission.trials)}; landing={receipt['landing_reason']}",flush=True)
            report=summarize_telemetry(json.loads(line) for line in (output/"events.jsonl").open())
            (output/"telemetry_summary.json").write_text(json.dumps(report,indent=2)+"\n")

    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node=None;bag=None;baglog=(output/"rosbag.log").open("w");executor=SingleThreadedExecutor();interrupts=0
    def interrupt(signum,frame):
        nonlocal interrupts
        interrupts+=1
        if node:
            if signum==signal.SIGINT and interrupts==1 and node.stage=="running":
                mission.request_landing("operator_interrupt");print("Excitation cancelled; PD landing requested. Second Ctrl-C disarms immediately.",flush=True)
            else:node.stop("Emergency termination")
        else:raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    error=None
    try:
        bag=subprocess.Popen(["ros2","bag","record","--storage","mcap","--output",str(output/"mocap"),settings["tf_topic"],"/tf_static","/rosout"],stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
        node=IdentificationNode();executor.add_node(node)
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
    if args.mission=="rate_sine" and (output/"events.jsonl").exists():
        try:
            from sanity_zenoh.sine_analysis import assess_sine_run
            report=assess_sine_run(output)
            print(f"Sine analysis: {sum(row['valid'] for row in report['blocks'])}/{len(report['blocks'])} usable completed blocks; see sine_response.pdf and sine_summary.json",flush=True)
        except Exception as caught:
            (output/"sine_analysis_error.json").write_text(json.dumps({"error":str(caught)},indent=2)+"\n")
            print("Offline sine analysis failed; raw flight recording retained:",caught,flush=True)
    if error:raise SystemExit(error)
    if node and node.error:raise SystemExit(node.error)
    if args.autotune and node and (not mission.tuning_receipt.get("success") or mission.tuning_receipt.get("cleanup_error")):raise SystemExit("Autotune did not complete and finalize successfully; inspect tuning_result.json")


if __name__=="__main__":main()
