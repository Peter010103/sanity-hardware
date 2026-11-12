"""Deterministic ROS-clock functional validation with native Zenoh/MSP in loop."""

import argparse
import gc
import json
import os
from pathlib import Path
import socket
import signal
import struct
import subprocess
import sys
import threading
import time
import yaml

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from sanity_zenoh.local_policy import ActorProcess
from sanity_zenoh.ros_sequence import SequenceNode
from sanity_zenoh.simulation_wire import decode_msp
from sanity_zenoh.collective import MotorCollective


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--emulator",type=Path,required=True)
    parser.add_argument("--policy-delay-tick",type=int,default=-1)
    parser.add_argument("--policy-delay",type=float,default=.12)
    parser.add_argument("--freeze-tf-at",type=float,default=-1.)
    parser.add_argument("--bypass-voltage-check",action="store_true")
    parser.add_argument("--recorder-ready-file",default="")
    parser.add_argument("--synthetic-thrust",action="store_true")
    parser.add_argument("--interrupt-at",type=float,default=-1.)
    parser.add_argument("--real-time-factor",type=float,default=.5)
    a = parser.parse_args(argv)
    if os.environ.get("ROS_LOCALHOST_ONLY")!="1" or os.environ.get("ROS_DOMAIN_ID","0") in ("","0"):
        parser.error("Use localhost-only DDS and an isolated nonzero domain")
    if not 0<a.real_time_factor<=.5:
        parser.error("Use a factor in (0,.5] so the nominal child has inference time")
    root = a.workspace.resolve()
    a.output.mkdir(parents=True,exist_ok=False)
    with socket.socket() as port:
        port.bind(("127.0.0.1",0))
        endpoint = f"tcp/127.0.0.1:{port.getsockname()[1]}"
    checkpoint = root/"outputs/sanity_ctbr_finetune_20261015/runs/sanity_ctbr/r9_finetune/seed0/models/model_1073741824.model"
    python = root/".venv-sanity-train/bin/python"
    source = root/"loco_mjx/experiments/sanity_ctbr_sim2sim/policy_worker.py"
    mapping = a.output/"SYNTHETIC_NOT_FOR_HARDWARE.json"
    mapping.write_text(json.dumps({"thrust_newtons":[0.,3.2],"throttle_rc":[1000.,1800.],"rate_full_scale_deg_s":[600.,600.,600.],"rate_signs":[1.,1.,-1.],"model_rate_limits_rad_s":[2.3,2.3,2.6],"max_thrust_newtons":3.2}))
    params = {"drone_id":1,"use_sim_time":True,"simulation":True,"enable_output":True,"frames_confirmed":True,
        "loco_root":str(root/"loco_mjx"),"checkpoint":str(checkpoint),"policy_python":str(python),
        "output":str(a.output/"controller"),"listen":endpoint,"ctbr_map":str(mapping),"control_cpu":-1,
        "test_delay_tick":a.policy_delay_tick,"test_delay_s":float(a.policy_delay)}
    profile_parameters = yaml.safe_load((root/"Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/circle_sequence.yaml").read_text())["sanity_sequence"]["ros__parameters"]
    for key in ("handover_position_tolerance_m", "fallback_position_tolerance_m", "fallback_speed_limit_m_s"):
        params[key] = profile_parameters[key]
    collective = None
    if not a.synthetic_thrust:
        profile = root/"Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_motor_collective.json"
        params.update(motor_data_root=str(root/"Sanity/sanity-hardware"),motor_profile=str(profile))
        params["bypass_voltage_check"] = a.bypass_voltage_check
        collective = MotorCollective(params["motor_data_root"],json.loads(profile.read_text()),a.bypass_voltage_check)
        collective.set_voltage(7.8)
    params["recorder_ready_file"] = a.recorder_ready_file
    config = a.output/"simulation.yaml"
    config.write_text(json.dumps({"sanity_sequence":{"ros__parameters":params}},indent=2))
    rclpy.init(args=["--ros-args","--params-file",str(config)],signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    controller = driver = plant = emulator = reader = None
    latest = {"ctbr":[0.]*4,"enabled":False}
    lock = threading.Lock()
    log = (a.output/"receiver.jsonl").open("w",buffering=1)
    try:
        plant = ActorProcess(python,source,checkpoint,a.output/"plant_worker.log",realtime=True)
        fixture = plant.call("ground_fixture",deadline=time.monotonic()+10)
        state = [0.,0.,fixture["ground_root_height_m"]+.001,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        controller = SequenceNode()
        interruptions = 0
        def interrupt(signum,frame):
            nonlocal interruptions
            interruptions += 1
            if signum==signal.SIGINT and interruptions==1 and controller.stage=="running":
                controller.request_landing("operator_interrupt")
            else:
                controller.stop("Emergency termination")
        signal.signal(signal.SIGINT,interrupt)
        signal.signal(signal.SIGTERM,interrupt)
        driver = rclpy.create_node("clocked_simulated_mocap")
        tf_pub = driver.create_publisher(TFMessage,"/tf",10)
        clock_pub = driver.create_publisher(Clock,"/clock",10)
        executor.add_node(controller)
        executor.add_node(driver)
        emulator = subprocess.Popen([str(a.emulator),"3",endpoint,"300"],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        def receive():
            for line in emulator.stdout:
                log.write(line)
                try:
                    decoded = decode_msp(json.loads(line),collective)
                    with lock:
                        latest.update(**decoded)
                except (ValueError,KeyError,TypeError,struct.error):
                    pass
        reader = threading.Thread(target=receive,daemon=True)
        reader.start()
        gc.collect()
        gc.disable()
        clock_ns = 1_000_000_000
        start_wall = time.monotonic()
        next_tick = start_wall
        step = 0
        frozen_stamp = None
        with (a.output/"plant.jsonl").open("w",buffering=1) as physical_log:
            while not controller.finished.is_set() and step<25000:
                if a.interrupt_at>=0 and controller.started_ns is not None and (clock_ns-controller.started_ns)/1e9>=a.interrupt_at:
                    os.kill(os.getpid(),signal.SIGINT)
                    a.interrupt_at = -1.
                with lock:
                    command = list(latest["ctbr"])
                    enabled = latest["enabled"]
                advanced = plant.call("advance",deadline=time.monotonic()+1,state=state,ctbr=command,substeps=1)
                state = advanced["state"]
                clock_ns += 5_000_000
                clock = Clock()
                clock.clock.sec,clock.clock.nanosec = divmod(clock_ns,10**9)
                clock_pub.publish(clock)
                deadline = time.monotonic()+2
                while controller.get_clock().now().nanoseconds<clock_ns:
                    executor.spin_once(timeout_sec=.001)
                    if time.monotonic()>deadline:
                        raise TimeoutError("ROS clock discovery/update failed")
                # Publish physical sample time; never refresh a frozen source stamp.
                if step%2==0:
                    tf = TransformStamped()
                    tf.header.frame_id,tf.child_frame_id = "map","sanity09"
                    sample = clock_ns
                    if a.freeze_tf_at>=0 and step*.005>=a.freeze_tf_at:
                        if frozen_stamp is None:
                            frozen_stamp = clock_ns
                        sample = frozen_stamp
                    tf.header.stamp.sec,tf.header.stamp.nanosec = divmod(sample,10**9)
                    tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = state[:3]
                    tf.transform.rotation.w,tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z = state[3:7]
                    tf_pub.publish(TFMessage(transforms=[tf]))
                    physical_log.write(json.dumps({"stamp_ns":clock_ns,"host_ns":time.monotonic_ns(),"state":state,
                        "applied":command,"enabled":enabled,"contacts":advanced["contact_count"],"sim_time_s":(step+1)*.005})+"\n")
                # Drain due ROS callbacks, but do not wait for policy inference.
                for _ in range(12):
                    executor.spin_once(timeout_sec=0.)
                step += 1
                next_tick = time.monotonic() + .005/a.real_time_factor
                while time.monotonic()<next_tick and not controller.finished.is_set():
                    executor.spin_once(timeout_sec=max(0.,min(.001,next_tick-time.monotonic())))
        if not controller.finished.is_set():
            raise TimeoutError("Sequence did not finish in125 simulation seconds")
        controller.close()
        time.sleep(.15)
        receipt = {"backend":"mujoco","clock":"ROS simulation clock","hardware_access":False,
            "transport":"native Zenoh -> Pico -> decoded MSP RC -> model CTBR","real_time_factor":a.real_time_factor,
            "scope":"Functional simulation, not live-clock scheduling qualification",
            "controller_returncode":1 if controller.error else 0,"physics_steps":step,"physics_dt_s":.005,
            "fixture":fixture,"model":plant.info,"arguments":{k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},
            "final_receiver":dict(latest),"wall_s":time.monotonic()-start_wall}
        (a.output/"simulation_receipt.json").write_text(json.dumps(receipt,indent=2)+"\n")
        print(json.dumps({"output":str(a.output),"error":controller.error,"steps":step}),flush=True)
    finally:
        executor.shutdown(timeout_sec=2)
        if controller:
            if not (a.output/"controller/receipt.json").exists():
                controller.close()
            controller.destroy_node()
        if driver:
            driver.destroy_node()
        if emulator:
            emulator.terminate()
            emulator.wait(timeout=3)
        if reader:
            reader.join(timeout=2)
        if plant:
            plant.close()
        log.close()
        if rclpy.ok():
            rclpy.shutdown()
        gc.enable()


if __name__=="__main__":
    main()
