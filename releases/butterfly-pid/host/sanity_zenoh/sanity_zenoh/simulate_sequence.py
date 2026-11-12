"""Real-time ROS TF + native Zenoh/Pico closed loop; no physical device access."""

import argparse
import gc
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from sanity_zenoh.local_policy import ActorProcess
from sanity_zenoh.simulation_wire import decode_msp


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--emulator",type=Path,required=True)
    parser.add_argument("--control-cpu",type=int,default=-1)
    parser.add_argument("--duration",type=float,default=60.)
    parser.add_argument("--policy-delay-tick",type=int,default=-1)
    parser.add_argument("--policy-delay",type=float,default=.12)
    parser.add_argument("--freeze-tf-at",type=float,default=-1.)
    a = parser.parse_args(argv)
    if os.environ.get("ROS_LOCALHOST_ONLY")!="1" or os.environ.get("ROS_DOMAIN_ID","0") in ("","0"):
        parser.error("Synthetic TF requires localhost-only DDS and an isolated nonzero domain")
    a.workspace = a.workspace.resolve()
    a.output.mkdir(parents=True,exist_ok=False)
    checkpoint = a.workspace/"outputs/sanity_ctbr_finetune_20261015/runs/sanity_ctbr/r9_finetune/seed0/models/model_1073741824.model"
    python = a.workspace/".venv-sanity-train/bin/python"
    source = a.workspace/"loco_mjx/experiments/sanity_ctbr_sim2sim/policy_worker.py"
    plant = controller = emulator = reader = None
    rclpy.init()
    node = rclpy.create_node("sanity_simulated_mocap")
    pub = node.create_publisher(TFMessage,"/tf",10)
    latest = {"ctbr":[0.]*4,"enabled":False}
    lock = threading.Lock()
    error = None
    native_log = (a.output/"receiver.jsonl").open("w",buffering=1)
    controller_log = (a.output/"ros_node.log").open("w")
    try:
        plant = ActorProcess(python,source,checkpoint,a.output/"plant_worker.log",realtime=True)
        fixture = plant.call("ground_fixture",deadline=time.monotonic()+10)
        state = [0.,0.,fixture["ground_root_height_m"]+.001,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        with socket.socket() as port:
            port.bind(("127.0.0.1",0))
            endpoint = f"tcp/127.0.0.1:{port.getsockname()[1]}"
        mapping = a.output/"SYNTHETIC_NOT_FOR_HARDWARE.json"
        mapping.write_text(json.dumps({"thrust_newtons":[0.,3.2],"throttle_rc":[1000.,1800.],
                                       "rate_full_scale_deg_s":[600.,600.,600.],"rate_signs":[1.,1.,-1.],
                                       "model_rate_limits_rad_s":[2.3,2.3,2.6],"max_thrust_newtons":3.2}))
        params = {"drone_id":1,"loco_root":str(a.workspace/"loco_mjx"),"checkpoint":str(checkpoint),"policy_python":str(python),
            "output":str(a.output/"controller"),"enable_output":True,"simulation":True,"frames_confirmed":True,
            "listen":endpoint,"ctbr_map":str(mapping),"duration_s":float(a.duration),
            "control_cpu":a.control_cpu,"test_delay_tick":a.policy_delay_tick,"test_delay_s":float(a.policy_delay)}
        config = a.output/"simulation.yaml"
        config.write_text(json.dumps({"sanity_sequence":{"ros__parameters":params}},indent=2))
        controller = subprocess.Popen([sys.executable,"-m","sanity_zenoh.ros_sequence","--ros-args","--params-file",str(config)],stdout=controller_log,stderr=subprocess.STDOUT)
        deadline = time.monotonic()+20
        while True:
            with socket.socket() as probe:
                probe.settimeout(.02)
                listening = probe.connect_ex(("127.0.0.1",int(endpoint.rsplit(":",1)[1])))==0
            if listening:
                break
            if controller.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError("Controller did not open its loopback listener; see ros_node.log")
            time.sleep(.02)
        emulator = subprocess.Popen([str(a.emulator),"3",endpoint,str(a.duration+30)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        def read_receiver():
            for line in emulator.stdout:
                native_log.write(line)
                try:
                    packet = json.loads(line)
                    command = decode_msp(packet)
                    with lock:
                        latest.update(**command,host_ns=time.monotonic_ns())
                except (ValueError,KeyError,TypeError,struct.error):
                    pass
        reader = threading.Thread(target=read_receiver,daemon=True)
        reader.start()
        gc.collect()
        gc.disable()
        started = time.monotonic()
        epoch_ros_ns = node.get_clock().now().nanoseconds
        next_tick = started + .005
        step = 0
        frozen_stamp = None
        with (a.output/"plant.jsonl").open("w",buffering=1) as log:
            while controller.poll() is None and time.monotonic()-started<a.duration+15:
                time.sleep(max(0.,next_tick-time.monotonic()))
                with lock:
                    command = list(latest["ctbr"])
                    enabled = latest["enabled"]
                advanced = plant.call("advance",deadline=time.monotonic()+1,state=state,ctbr=command,substeps=1)
                state = advanced["state"]
                now = node.get_clock().now()
                sample_ns = epoch_ros_ns + (step+1)*5_000_000
                # Real source poses at 100 Hz, received through the normal TF estimator.
                if step%2==0:
                    tf = TransformStamped()
                    tf.header.frame_id = "map"
                    tf.child_frame_id = "sanity09"
                    control_receipt = a.output/"controller/receipt.json"
                    elapsed = time.monotonic()-started
                    freeze = a.freeze_tf_at>=0 and elapsed>=a.freeze_tf_at
                    if freeze:
                        if frozen_stamp is None:
                            frozen_stamp = now.to_msg()
                        tf.header.stamp = frozen_stamp
                    else:
                        tf.header.stamp.sec = sample_ns//10**9
                        tf.header.stamp.nanosec = sample_ns%10**9
                    tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = state[:3]
                    tf.transform.rotation.w,tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z = state[3:7]
                    pub.publish(TFMessage(transforms=[tf]))
                    log.write(json.dumps({"stamp_ns":sample_ns,"published_ros_ns":now.nanoseconds,"host_ns":time.monotonic_ns(),"sim_time_s":(step+1)*.005,
                        "wall_time_s":elapsed,"state":state,"applied":command,"enabled":enabled,"contacts":advanced["contact_count"],
                        "scheduler_lag_s":time.monotonic()-next_tick})+"\n")
                step += 1
                next_tick += .005
        if controller.poll() is None:
            raise TimeoutError("ROS controller did not finish")
        # Capture the node's final explicit neutral/disarmed output.
        time.sleep(.15)
        receipt = {"backend":"mujoco", "hardware_access":False,"transport":"native Zenoh -> Pico -> decoded MSP RC -> model CTBR",
            "ros_input":"/tf map -> sanity09, finite-difference/filter velocity","controller_returncode":controller.returncode,
            "physics_steps":step,"physics_dt_s":.005,"fixture":fixture,"model":plant.info,
            "arguments":{k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},"final_receiver":dict(latest)}
        (a.output/"simulation_receipt.json").write_text(json.dumps(receipt,indent=2)+"\n")
        print(json.dumps({"output":str(a.output),"controller_returncode":controller.returncode,"physics_steps":step}),flush=True)
    except BaseException as exc:
        error = f"{type(exc).__name__}: {exc}"
        (a.output/"simulation_error.txt").write_text(error+"\n")
        raise
    finally:
        if controller and controller.poll() is None:
            controller.send_signal(signal.SIGINT)
            try:
                controller.wait(timeout=5)
            except subprocess.TimeoutExpired:
                controller.kill()
                controller.wait(timeout=3)
        if emulator:
            if emulator.poll() is None:
                emulator.terminate()
            emulator.wait(timeout=3)
        if reader:
            reader.join(timeout=2)
        if plant:
            plant.close()
        native_log.close()
        controller_log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
