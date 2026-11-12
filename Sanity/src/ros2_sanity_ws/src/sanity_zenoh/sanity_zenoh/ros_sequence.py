"""TF in over DDS, timed PD/policy selection, physical CTBR out over native Zenoh."""

from dataclasses import asdict
import json
import gc
import math
import os
from pathlib import Path
import sys
import signal
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from udp_broadcaster.ctbr_protocol import CtbrMapping
from sanity_zenoh.async_policy import AsyncPolicy
from sanity_zenoh.client import SanitySender
from sanity_zenoh.mocap import MocapEstimator, MocapOutlier, state_age
from sanity_zenoh.collective import MotorCollective
from sanity_zenoh.firmware_collective import FirmwareCollective
from sanity_zenoh.landing import Landing
from sanity_zenoh.recording import EventRecorder
from sanity_zenoh.residual_runtime import residual_metadata, trajectory_preview, CALIBRATION_SHA256


class SequenceNode(Node):
    def __init__(self, *, node_name="sanity_sequence", parameter_overrides=None,
                 transport=None, mission_factory=None, subscribe_tf=True, isolated_io=False):
        super().__init__(node_name, parameter_overrides=parameter_overrides or [])
        self.fleet_mode = mission_factory is not None
        self.isolated_io = isolated_io
        self.external_mocap = None
        self.fleet_admitted = not self.fleet_mode
        self.started_monotonic_ns = None
        defaults = {"loco_root": "", "checkpoint": "", "policy_python": "", "output": "",
            "enable_output": False, "simulation": False, "drone_id": 9, "residual_model": "", "residual_source": "",
            "controller": "policy", "mpc_config": "", "mpc_model": "", "start_at_current_xy": True, "landing_enabled": True, "land_on_policy_fallback": True,"landing_exit_height_m": .10,"landing_exit_dwell_s": 2.,
            "bypass_voltage_check": False, "esp_collective_profile": "", "motor_data_root": "", "motor_profile": "", "recorder_ready_file": "",
            "listen": "tcp/10.0.0.200:7447", "connect": "", "telemetry_listen": "", "transport_diagnostics": False, "ctbr_map": "",
            "tf_topic": "/tf", "parent_frame": "map", "tracked_frame": "sanity09",
            "world_from_map_xyzw": [0.,0.,0.,1.], "marker_to_base_xyz": [0.,0.,0.],
            "marker_to_base_xyzw": [0.,0.,0.,1.], "frames_confirmed": False,
            "velocity_filter_tau_s": .02, "angular_rate_filter_tau_s": .05, "duration_s": 60., "target_height_m": 1., "maximum_altitude_m": 1.5,
            "pd_integral_gains": [.8,.8,1.2], "pd_integral_limits_m_s2": [.45,.45,.8],
            "handover_thrust_tolerance_n": .20, "handover_position_tolerance_m": .06, "fallback_position_tolerance_m": 1., "fallback_speed_limit_m_s": 4.,
            "fallback_tilt_limit_deg": 30., "hard_position_error_limit_m": 1.5, "hard_tilt_limit_deg": 45.,
            "ground_root_height_m": .0152, "origin_xy": [0.,0.], "radius_m": .5,
            "mission_pattern": "circle", "speed_m_s": .3, "laps": 2,
            "command_speed_limit_m_s": 0., "mission_ramp_s": 2., "mission_ramp_profile": "linear", "auto_radius": False,
            "max_reference_acceleration_m_s2": 3.5, "reference_tilt_limit_deg": 20., "maximum_run_s": 300., "control_cpu": -1, "test_delay_tick": -1, "test_delay_s": 0.}
        for name,value in defaults.items():
            self.declare_parameter(name,value)
        v = self.values = {name:self.get_parameter(name).value for name in defaults}
        self.sim_clock = bool(self.get_parameter("use_sim_time").value)
        if self.sim_clock and not v["simulation"]:
            raise ValueError("Physical output requires the live ROS clock")
        if not math.isfinite(v["maximum_run_s"]) or not 120 <= v["maximum_run_s"] <= 1200 or not (55 if v["enable_output"] else 1) <= v["duration_s"] <= v["maximum_run_s"]-30 or not np.isfinite(v["origin_xy"]).all() or len(v["origin_xy"]) != 2:
            raise ValueError("Use a finite mission duration with at least30s of runtime reserve, maximum_run_s in[120,1200], and finite XY origin")
        if not (math.isfinite(v["target_height_m"]) and math.isfinite(v["maximum_altitude_m"]) and v["ground_root_height_m"] < v["target_height_m"] < v["maximum_altitude_m"]):
            raise ValueError("Hover height must be above ground and below the finite maximum altitude")
        if not all(math.isfinite(v[key]) and v[key]>0 for key in ("landing_exit_height_m","landing_exit_dwell_s")):
            raise ValueError("Positive finite landing exit height/dwell required")
        if not (math.isfinite(v["hard_position_error_limit_m"]) and 0 < v["handover_position_tolerance_m"] < v["fallback_position_tolerance_m"] < v["hard_position_error_limit_m"]
                and math.isfinite(v["fallback_tilt_limit_deg"]) and math.isfinite(v["hard_tilt_limit_deg"])
                and 5 < v["fallback_tilt_limit_deg"] < v["hard_tilt_limit_deg"] <= 80
                and (v["controller"]=="mpc" or v["hard_tilt_limit_deg"]<=45)):
            raise ValueError("Ordered position/tilt limits required:5 < fallback tilt < hard tilt <=80; policy hard tilt <=45")
        if not math.isfinite(v["reference_tilt_limit_deg"]) or not 0 < v["reference_tilt_limit_deg"] <= v["fallback_tilt_limit_deg"]:
            raise ValueError("Reference planning tilt must be positive, finite and no larger than the policy tilt fallback")
        if v["test_delay_tick"] >= 0 and not v["simulation"]:
            raise ValueError("Fault injection is restricted to simulation")
        if v["simulation"] and not (v["listen"].startswith("tcp/127.0.0.1:") and not v["connect"]):
            raise ValueError("Simulation must use an isolated loopback Zenoh listener")
        if v["simulation"]:
            v["telemetry_listen"] = ""
            v["transport_diagnostics"] = False
        if v["control_cpu"] >= 0 and v["control_cpu"] not in os.sched_getaffinity(0):
            raise ValueError("Requested control CPU is unavailable")
        if v["controller"] not in ("policy", "mpc") or (v["controller"]=="mpc" and v["residual_model"]):
            raise ValueError("Choose policy/residual or the standalone MPC baseline")
        source = Path(v["loco_root"]).resolve()/"experiments/sanity_ctbr_sim2sim"
        mpc = v["controller"]=="mpc"
        worker_source = source/("mpc_controller.py" if mpc else "policy_worker.py")
        if mpc and not v["mpc_config"]:
            v["mpc_config"] = str(Path(v["loco_root"]).parent/"Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_mpc.json")
        if mpc:
            mpc_tilt = json.loads(Path(v["mpc_config"]).read_text()).get("tilt_limit_deg",30.)
            if not math.isfinite(mpc_tilt) or not v["reference_tilt_limit_deg"] <= mpc_tilt <= v["fallback_tilt_limit_deg"]:
                raise ValueError("MPC tilt limits must satisfy: reference <= MPC constraint <= fallback < hard")
        controller_file = Path(v["mpc_config"] if mpc else v["checkpoint"])
        model_file = (Path(v["mpc_model"]) if mpc and v["mpc_model"] else
            Path(v["loco_root"])/"loco_mjx/environments/robots/sanity_ctbr/data/plane.xml")
        worker_python = Path(sys.executable if mpc else v["policy_python"])
        for file in (worker_source, source/"takeoff_controller.py", controller_file, worker_python, model_file):
            if not file.is_file():
                raise ValueError(f"Missing {file}")
        residual = None
        actor_source = worker_source
        if v["residual_model"]:
            residual = residual_metadata(v["residual_model"],v["checkpoint"],v["residual_source"])
            if v["speed_m_s"]>3. or v["command_speed_limit_m_s"] not in (0.,3.):
                raise ValueError("Residual deployment reference speed <=3m/s and trained velocity cap3m/s required")
            v["command_speed_limit_m_s"] = 3.
            actor_source = Path(v["residual_source"])/"experiments/sanity_ctbr_sim2sim/policy_worker.py"
            if not v["simulation"]:
                import hashlib
                if hashlib.sha256(Path(v["esp_collective_profile"]).read_bytes()).hexdigest()!=CALIBRATION_SHA256:
                    raise ValueError("Residual handoff requires the recorded delivered-FC-voltage calibration")
        params = json.loads(Path(v["ctbr_map"]).read_text()) if v["ctbr_map"] else {}
        if v["enable_output"] and not v["motor_data_root"] and not {"rate_full_scale_deg_s","rate_signs","model_rate_limits_rad_s","max_thrust_newtons","thrust_newtons","throttle_rc"} <= params.keys():
            raise ValueError("Enabled CTBR requires explicit rate signs/scales/limits and a collective calibration")
        self.mapping = (FirmwareCollective(v["esp_collective_profile"]) if v["esp_collective_profile"] and not v["simulation"] else
            MotorCollective(v["motor_data_root"],json.loads(Path(v["motor_profile"]).read_text()),v["bypass_voltage_check"])
            if v["motor_data_root"] else CtbrMapping(**{key:tuple(value) if isinstance(value,list) else value for key,value in params.items()}))
        self.motor_mapping = isinstance(self.mapping,(MotorCollective,FirmwareCollective))
        if v["enable_output"] and (not v["frames_confirmed"] or (not self.motor_mapping and len(self.mapping.thrust_newtons)<2)):
            raise ValueError("Explicit output requires confirmed frames and collective RC calibration")
        sys.path.insert(0,str(source))
        from missions import DeploymentMission
        from takeoff_controller import PositionPD,TakeoffCircleReference,TakeoffSupervisor
        if mission_factory is not None:
            self.circle = mission_factory(v)
        else:
            self.circle = DeploymentMission(v["mission_pattern"],v["speed_m_s"],v["radius_m"],v["laps"],
                command_speed_limit=v["command_speed_limit_m_s"],fallback_speed_limit=v["fallback_speed_limit_m_s"],
                ramp_s=v["mission_ramp_s"],max_reference_acceleration=v["max_reference_acceleration_m_s2"],auto_radius=v["auto_radius"],reference_tilt_limit_deg=v["reference_tilt_limit_deg"],controller=v["controller"],ramp_profile=v["mission_ramp_profile"])
        self.mission_plan = self.circle.metadata()
        self.mission_plan.update(fallback_position_tolerance_m=v["fallback_position_tolerance_m"],fallback_tilt_limit_deg=v["fallback_tilt_limit_deg"],
            hard_position_error_limit_m=v["hard_position_error_limit_m"],hard_tilt_limit_deg=v["hard_tilt_limit_deg"],target_height_m=v["target_height_m"],hard_altitude_limit_m=v["maximum_altitude_m"])
        if residual:
            self.mission_plan.update(outer_controller=residual,vertical_command_limit_m_s=.2,yaw_gain_per_s=1.,yaw_rate_limit_rad_s=.6,reference_point="root",
                residual_reference_training_max_m_s=residual["manifest"]["configuration"]["reference_speed_max_m_s"],
                residual_reference_extrapolation=v["speed_m_s"]>residual["manifest"]["configuration"]["reference_speed_max_m_s"])
        v.update(radius_m=self.circle.radius,command_speed_limit_m_s=self.circle.command_speed_limit,
            fallback_speed_limit_m_s=self.circle.fallback_speed_limit)
        if max(self.circle.minimum_mission_time,v["duration_s"])+self.circle.landing_reserve_s > v["maximum_run_s"]:
            raise ValueError("Mission cannot finish within maximum_run_s with landing reserve; increase the runtime budget")
        self.output = Path(v["output"])
        self.output.mkdir(parents=True,exist_ok=False)
        self.worker = AsyncPolicy(worker_python.absolute(),actor_source,controller_file.resolve(),self.output,
            residual_model=v["residual_model"] or None,residual_source=v["residual_source"] or None,
            controller=v["controller"],mpc_model=model_file if mpc else None,isolated_recording=isolated_io)
        info = self.worker.info
        self.bounds = np.asarray(info["ctrlrange"])
        self.mission_plan.update(controller=v["controller"], controller_configuration=info.get("configuration") if mpc else None)
        if mpc:
            self.mission_plan.update(reference_point="root",policy_scope="Standalone trajectory-to-CTBR MPC; velocity-goal cap is unused",
                outer_controller=None,mpc_model_sha256=info["model_sha256"],mpc_config_sha256=info["config_sha256"])
        if mpc and info["configuration"].get("tilt_limit_deg",30.)>v["fallback_tilt_limit_deg"]:
            self.worker.close()
            raise ValueError("MPC prediction tilt must not exceed the active tracking tilt fallback")
        if not math.isclose(info["policy_dt_s"],.02):
            self.worker.close()
            raise ValueError("Expected the 50 Hz policy contract")
        if v["enable_output"] and ((not self.motor_mapping and self.mapping.thrust_newtons[-1]<self.bounds[0,1]) or
            np.any(np.asarray(self.mapping.model_rate_limits_rad_s)<np.max(abs(self.bounds[1:]),axis=1))):
            self.worker.close()
            raise ValueError("Command mapping must cover policy bounds")
        self.vertices = np.array([[float(x) for x in line.split()[1:4]] for line in
            (Path(v["loco_root"])/"loco_mjx/environments/robots/sanity_ctbr/data/assets/meshes/collision/envelope.obj").read_text().splitlines() if line.startswith("v ")])
        self.reference = (self.circle.takeoff_reference(v) if self.fleet_mode else
            TakeoffCircleReference(v["ground_root_height_m"],self.circle,v["target_height_m"],transfer_s=self.circle.transfer_s))
        self.pd = PositionPD(info["mass_kg"],integral_gains=v["pd_integral_gains"],integral_acceleration_limits=v["pd_integral_limits_m_s2"])
        self.supervisor = TakeoffSupervisor("hybrid",self.reference,self.bounds,info["history_length_steps"],
            handover_position_tolerance=v["handover_position_tolerance_m"],
            handover_thrust_tolerance=v["handover_thrust_tolerance_n"],
            fallback_position_tolerance=v["fallback_position_tolerance_m"],fallback_speed_limit=v["fallback_speed_limit_m_s"],
            fallback_tilt_limit_deg=v["fallback_tilt_limit_deg"],hard_tilt_limit_deg=v["hard_tilt_limit_deg"])
        self.estimator = MocapEstimator(v["parent_frame"],v["tracked_frame"],v["world_from_map_xyzw"],
            v["marker_to_base_xyz"],v["marker_to_base_xyzw"],v["velocity_filter_tau_s"],frames_confirmed=v["frames_confirmed"],
            angular_rate_filter_tau_s=v["angular_rate_filter_tau_s"])
        self.events = EventRecorder(self.output/"events.jsonl",isolated=isolated_io)
        self.events({"kind":"configuration","parameters":v,"mapping":self.mapping.provenance if self.motor_mapping else asdict(self.mapping)})
        try:
            self.sender = SanitySender(device=v["drone_id"],listen=v["listen"],connect=v["connect"] or None,telemetry_listen=v["telemetry_listen"] or None,diagnostics=v["transport_diagnostics"],ctbr_mapping=self.mapping,on_event=self.events,physical_units=isinstance(self.mapping,FirmwareCollective),transport=transport,disabled_only=self.fleet_mode and not v["enable_output"])
        except BaseException:
            self.worker.close()
            raise
        self.lock = threading.Lock()
        self.packet = None
        self.slot = None
        self.stage = "waiting"
        self.wait_started = time.monotonic()
        self.error = None
        self.started_ns = None
        self.tick = 0
        self.pending_observations = {}
        self.fallback_reference = None
        self.landing = None
        self.landing_request = None
        self.landed = False
        self.last_policy_weight = 0.
        self.latest_control = None
        self.home_xy = None
        self.ground_root = None
        self.finished = threading.Event()
        self.tf_log = None if isolated_io else (self.output/"tf.jsonl").open("w",buffering=1)
        self.control_log = (self.output/"control.jsonl").open("w",buffering=1)
        self.command_log = (self.output/"commands.jsonl").open("w",buffering=1)
        self.receipt = {"mapping":self.mapping.provenance if self.motor_mapping else asdict(self.mapping),"parameters":v,"policy":info,"controller":v["controller"],"history_contract":"FIFO 50 Hz observation/selected-command pairs; no dropping or resetting; 10 ms observation-to-PD phase",
                        "transport":"native Zenoh", "ros_command_topics":False,"isolated_mocap_and_recording":isolated_io,"mission_plan":self.mission_plan,
                        "landing_exit_condition":{"height_tolerance_m":v["landing_exit_height_m"],"dwell_s":v["landing_exit_dwell_s"],"spool_down_s":.6,"criterion":"absolute tracked height relative to initial ground; landing phases only"}}
        self.tf_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.command_group = MutuallyExclusiveCallbackGroup()
        self.subscription = (self.create_subscription(TFMessage,v["tf_topic"],self.receive_tf,
            QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT),callback_group=self.tf_group) if subscribe_tf else None)
        self.observation_timer = self.create_timer(.02,self.observe,callback_group=self.control_group)
        self.control_timer = self.create_timer(.02,self.control_tick,callback_group=self.control_group)
        self.phase_timer = self.create_timer(.01,self.first_control,callback_group=self.control_group)
        self.observation_timer.cancel()
        self.control_timer.cancel()
        self.phase_timer.cancel()
        self.command_timer = self.create_timer(.01,self.command_tick,callback_group=self.command_group)
        if v["control_cpu"] >= 0:
            os.sched_setaffinity(0,{v["control_cpu"]})
        self.receipt["control_affinity"] = sorted(os.sched_getaffinity(0))
        self.get_logger().info("Ready: TF mocap; 50 Hz PD/selection; independent 100 Hz native Zenoh; background "+v["controller"])

    def receive_tf(self,message):
        for tf in message.transforms:
            if (tf.header.frame_id,tf.child_frame_id)!=(self.estimator.parent,self.estimator.child):
                continue
            t,q = tf.transform.translation,tf.transform.rotation
            self.events({"kind":"raw_tf","stamp_ns":tf.header.stamp.sec*10**9+tf.header.stamp.nanosec,
                "arrival_ros_ns":self.get_clock().now().nanoseconds,"parent":tf.header.frame_id,"child":tf.child_frame_id,
                "translation":[t.x,t.y,t.z],"quaternion_xyzw":[q.x,q.y,q.z,q.w]})
            try:
                packet = self.estimator.update(tf.header.frame_id,tf.child_frame_id,
                    tf.header.stamp.sec*10**9+tf.header.stamp.nanosec,[t.x,t.y,t.z],[q.x,q.y,q.z,q.w],
                    self.get_clock().now().nanoseconds,time.monotonic_ns())
                if packet is None:
                    continue
            except MocapOutlier as error:
                self.events({"kind":"mocap_rejected_sample", "reason":str(error), **error.diagnostics})
                continue
            except (ValueError,TypeError) as error:
                packet = {"ready":False,"error":str(error)}
            with self.lock:
                self.packet = packet
            self.events({"kind":"tf_estimate","record":{"input_stamp_ns":tf.header.stamp.sec*10**9+tf.header.stamp.nanosec,"arrival_ros_ns":self.get_clock().now().nanoseconds,"packet":packet}})

    def measured_state(self):
        external=getattr(self,"external_mocap",None)
        if external is not None:
            packet=external.snapshot()
            if packet.get("ready") and packet.get("tracked_frame") != self.values["tracked_frame"]:
                raise ValueError("Isolated mocap frame mismatch")
        else:
            with self.lock:
                packet = dict(self.packet or {})
        state,age = state_age(packet,packet.get("receipt_ns",time.monotonic_ns()) if self.sim_clock else time.monotonic_ns())
        if self.sim_clock:
            age = (self.get_clock().now().nanoseconds-packet["stamp_ns"])/1e9
            if not 0<=age<.04:
                raise ValueError("Mocap state expired on simulation clock")
        return state,age,packet

    def stop(self,reason=None):
        with self.lock:
            self.stage = "stopped"
            self.slot = None
            self.error = self.error or reason
        if not self.finished.is_set():
            self.events({"kind":"stopped","reason":self.error,"ros_ns":self.get_clock().now().nanoseconds,"landed":self.landed})
        self.finished.set()

    def request_landing(self, reason):
        if self.landing_request is None and self.stage=="running":
            self.landing_request = reason
            self.events({"kind":"landing_requested","reason":reason})

    def begin(self):
        state,_,_ = self.measured_state()
        if self.values["start_at_current_xy"]:
            self.values["origin_xy"] = state[:2].tolist()
        self.home_xy = np.asarray(self.values["origin_xy"])
        self.ground_root = float(state[2])
        self.pd.reset()
        self.started_ns = self.get_clock().now().nanoseconds
        self.started_monotonic_ns = time.monotonic_ns()
        if getattr(self,"fleet_mode",False):
            self.reference.bind_home(self.home_xy)
        self.stage = "running"
        self.observe()
        self.observation_timer.reset()
        self.phase_timer.reset()

    def elapsed_s(self, ros_ns=None):
        if getattr(self,"fleet_mode",False):
            return (time.monotonic_ns()-self.started_monotonic_ns)/1e9
        return ((self.get_clock().now().nanoseconds if ros_ns is None else ros_ns)-self.started_ns)/1e9


    def first_control(self):
        self.phase_timer.cancel()
        self.control_timer.reset()
        self.control_tick()

    def observe(self):
        if self.stage != "running" or self.tick in self.pending_observations:
            return
        try:
            state,age,packet = self.measured_state()
            t = SequenceNode.elapsed_s(self)
            rotation = Rotation.from_quat(np.r_[state[4:7],state[3]])
            offset = rotation.apply(self.worker.info["imu_position"])
            position = state[:3]+offset
            velocity = state[7:10]+np.cross(state[10:13],offset)
            ref = self.reference.sample(t)
            ref["position"] = (np.asarray(ref["position"])+np.r_[self.values["origin_xy"],0.]).tolist()
            if self.fallback_reference is not None and self.landing is None:
                ref = {"position":self.fallback_reference,"velocity":[0.]*3,"acceleration":[0.]*3,"phase":"fallback_hover"}
            if self.landing is not None:
                ref = self.landing.sample(t)
            if self.values.get("controller")=="mpc":
                observation = {"tick":self.tick,"state_stamp_ns":packet["stamp_ns"],"state":state.tolist(),"goal":None,"prepared_host_ns":time.monotonic_ns(),
                    "reference_preview":trajectory_preview(self.reference,self.landing,self.fallback_reference,t,self.values["origin_xy"],self.worker.info["preview_times_s"])}
            elif self.values.get("residual_model"):
                observation = {"tick":self.tick,"state_stamp_ns":packet["stamp_ns"],"state":state.tolist(),"goal":None,"prepared_host_ns":time.monotonic_ns(),
                    "preview":trajectory_preview(self.reference,self.landing,self.fallback_reference,t,self.values["origin_xy"]),"state_age_s":age,
                    "residual_enabled":self.supervisor.blend_start is not None and not self.supervisor.fallback and self.landing is None}
            else:
                target = np.asarray(ref["position"])+self.worker.info["imu_position"]
                desired = self.circle.world_velocity({**ref,"position":target},position,velocity)
                goal = [*rotation.inv().apply(desired),float(np.clip(-1.5*rotation.as_euler("xyz")[2],-1.,1.))]
                observation = {"tick":self.tick,"state_stamp_ns":packet["stamp_ns"],"state":state.tolist(),"goal":goal,"prepared_host_ns":time.monotonic_ns(),"goal_diagnostics":dict(self.circle.last_goal)}
            if self.values.get("controller")=="mpc" and self.worker.info.get("disturbance_compensation",{}).get("enabled"):
                observation["disturbance_context"]={"allow_learning":bool(self.values["enable_output"] and self.landing is None and not self.supervisor.fallback and t>=self.reference.climb_end+1.),
                    "height_above_ground_m":float(state[2]-self.ground_root),
                    "reference_velocity":ref["velocity"],"reference_acceleration":ref["acceleration"]}
            if self.tick==self.values["test_delay_tick"]:
                observation["test_delay_s"] = self.values["test_delay_s"]
            self.pending_observations[self.tick] = observation
            if not self.worker.error:
                self.worker.observe(observation)
        except Exception as error:
            self.stop(f"Observation: {type(error).__name__}: {error}")

    def control_tick(self):
        started = time.monotonic_ns()
        with self.lock:
            stage = self.stage
        if stage != "running":
            return
        try:
            self.receipt["last_control_stage"] = "state"
            state,age,packet = self.measured_state()
            now_ns = self.get_clock().now().nanoseconds
            t = SequenceNode.elapsed_s(self,now_ns)
            mission_deadline = self.values["duration_s"]
            if self.values["enable_output"]:
                mission_deadline = max(mission_deadline,self.circle.minimum_mission_time)
                if self.reference.orbit_start is not None:
                    mission_deadline = max(mission_deadline,self.reference.orbit_start+self.circle.duration+2.)
            self.receipt["mission_landing_deadline_s"] = mission_deadline
            if self.landing is None and t>=self.values["maximum_run_s"]-self.circle.landing_reserve_s:
                self.request_landing("mission_runtime_reserve")
            if not getattr(self,"fleet_mode",False) and t >= mission_deadline:
                if self.values["landing_enabled"] and self.values["enable_output"]:
                    self.request_landing("mission_complete")
                else:
                    self.stop()
                    return
            if t>=self.values["maximum_run_s"]:
                raise ValueError("Mission/landing total time limit exceeded")
            if self.landing is None and self.landing_request is not None:
                self.landing = Landing(t,state[:3],self.home_xy,self.ground_root,self.landing_request,self.last_policy_weight,velocity=state[7:10],
                    exit_height_m=self.values["landing_exit_height_m"],exit_dwell_s=self.values["landing_exit_dwell_s"])
                self.events({"kind":"landing_started","time_s":t,"reason":self.landing_request})
            rotation = Rotation.from_quat(np.r_[state[4:7],state[3]])
            offset = rotation.apply(self.worker.info["imu_position"])
            position = state[:3]+offset
            velocity = state[7:10]+np.cross(state[10:13],offset)
            ref = self.reference.sample(t)
            ref["position"] = (np.asarray(ref["position"])+np.r_[self.values["origin_xy"],0.]).tolist()
            if self.fallback_reference is not None and self.landing is None:
                ref = {"position":self.fallback_reference,"velocity":[0.]*3,"acceleration":[0.]*3,"phase":"fallback_hover"}
            if self.landing is not None:
                ref = self.landing.sample(t)
            ref_imu = {**ref,"position":(np.asarray(ref["position"])+self.worker.info["imu_position"]).tolist()}
            self.receipt["last_control_stage"] = "pd"
            pd_owns_command = (self.supervisor.blend_start is None or self.supervisor.fallback) if self.landing is None else t>=self.landing.start+2. and self.landing.touch_time is None
            integrate = self.values['enable_output'] and pd_owns_command and t>=self.reference.climb_start and state[2]-self.ground_root>.02
            if self.landing is None:
                pd = self.pd.takeoff_command(position,velocity,rotation,ref_imu,t,float(state[2]-self.ground_root),self.reference.climb_start,dt=.02,integrate=integrate)
            else:
                pd = self.pd.command(position,velocity,rotation,ref_imu,dt=.02,integrate=integrate)
            if self.tick not in self.pending_observations:
                self.observe()
            observation = self.pending_observations.pop(self.tick)
            goal = observation["goal"]
            self.receipt["last_control_stage"] = "policy_snapshot"
            result = self.worker.latest()
            policy_age = (now_ns-result["state_stamp_ns"])/1e9 if result else math.inf
            valid = bool(result and result.get("valid",True) and not self.worker.error and result["tick"] in (self.tick,self.tick-1) and 0<=policy_age<=.04)
            policy = result["ctbr"] if valid else None
            tilt = float(np.degrees(np.arccos(np.clip(rotation.as_matrix()[2,2],-1.,1.))))
            clearance = state[2]+rotation.apply(self.vertices)[:,2].min()
            self.receipt["last_control_stage"] = "selection"
            if self.landing is None:
                command,transition = self.supervisor.step(t,np.asarray(ref_imu["position"])-position,velocity,
                    tilt,clearance,pd,policy,result["history_age"] if result else 0,
                    policy_age_s=policy_age if valid else math.inf,state_age_s=age)
            else:
                if t>self.landing.timeout:
                    raise ValueError("Landing timeout: touchdown could not be confirmed")
                if self.landing.update_touchdown(t,state,tilt) and self.supervisor.previous[0]<=1e-6:
                    self.landed = True
                    self.events({"kind":"landed","time_s":t,"state":state.tolist()})
                    self.stop()
                    return
                weight = self.landing.weight(t) if valid else 0.
                selected = ((1-weight)*pd+weight*np.asarray(policy)) if valid else pd
                selected *= self.landing.spool(t)
                command = np.clip(selected,self.bounds[:,0],self.bounds[:,1])
                self.supervisor.previous = command.copy()
                transition = {"phase":ref["phase"],"policy_weight":weight,"handover_ready":False,
                    "blend_start_s":self.supervisor.blend_start,"policy_start_s":self.supervisor.policy_start,
                    "fallback":self.supervisor.fallback,"command_limited":bool(np.any(abs(command-selected)>1e-10))}
            if transition["fallback"] and self.fallback_reference is None:
                self.fallback_reference = state[:3].tolist()
                if self.values["land_on_policy_fallback"] and self.values["landing_enabled"]:
                    self.request_landing("policy_fallback")
            if not getattr(self,"fleet_mode",False) and self.reference.orbit_start is None and self.supervisor.policy_start is not None and not self.supervisor.fallback and t>=self.supervisor.policy_start+2.:
                self.reference.orbit_start = t
            if self.values["enable_output"] and (tilt>self.values.get("hard_tilt_limit_deg",45.) or state[2]>self.values["maximum_altitude_m"] or np.linalg.norm(state[:3]-ref["position"])>self.values["hard_position_error_limit_m"]):
                raise ValueError("Flight envelope exceeded")
            if self.motor_mapping and self.values["enable_output"]:
                status = self.sender.receiver_status()
                if status is None or status.fc_age_ms>=150:
                    raise ValueError("Fresh FC status required by collective mapping")
                self.mapping.set_voltage(status.voltage_mv/1000)
                limited = self.mapping.limit(command[0])
                if limited!=command[0]:
                    command[0] = limited
                    self.supervisor.previous = command.copy()
                    transition["command_limited"] = True
            self.pd.accept_output(command)
            outgoing = command if self.values["enable_output"] else np.zeros(4)
            raw_action = bool(self.values.get("controller")!="mpc" and self.values["enable_output"] and valid and transition["policy_weight"]==1 and not transition["command_limited"] and t>=self.reference.climb_start)
            event = {"tick":self.tick,
                     **({"selected_action":result["action"]} if raw_action else {"selected_ctbr":outgoing.tolist()})}
            if "disturbance_context" in observation:
                event["controller_context"]={"selected_stamp_ns":now_ns,"disturbance_learning_allowed":bool(observation["disturbance_context"]["allow_learning"] and not transition["command_limited"])}
            if self.values.get("residual_model"):
                pd_velocity = np.r_[ref["velocity"],0.]
                selected_velocity = pd_velocity
                if valid and transition["policy_weight"]>0:
                    weight = transition["policy_weight"]
                    selected_velocity = (1-weight)*pd_velocity+weight*np.asarray(result["velocity_world"])
                if not self.values["enable_output"]:
                    selected_velocity = np.zeros(4)
                event.update(selected_velocity_world=selected_velocity.tolist(),outgoing_ctbr=outgoing.tolist())
                goal = result["goal"] if valid else None
            with self.lock:
                if self.stage != "running":
                    return
                self.slot = {"ctbr":outgoing.tolist(),"control_ns":started,"control_ros_ns":now_ns,"packet":packet,"tick":self.tick}
            try:
                if self.worker.error:
                    raise RuntimeError(self.worker.error)
                self.worker.commit(event)
            except RuntimeError:
                # No policy restart or history reset: keep PD alive with fresh state.
                self.supervisor.fallback = True
                self.fallback_reference = state[:3].tolist()
                if self.values["landing_enabled"]:
                    self.request_landing("policy_worker_failed")
            self.last_policy_weight = transition["policy_weight"]
            row = {"tick":self.tick,"time_s":t,"control_ns":started,"state_stamp_ns":packet["stamp_ns"],
                "state":state.tolist(),"control_ros_ns":now_ns,"state_age_s":age,"goal":goal,"goal_diagnostics":observation.get("goal_diagnostics"),"reference":ref,"pd_ctbr":pd.tolist(),
                "command":outgoing.tolist(),"pd_integral":dict(self.pd.last),"policy_ctbr":result["ctbr"] if result else None,
                "policy_tick":result["tick"] if result else None,"policy_age_s":policy_age if math.isfinite(policy_age) else None,
                "outer":result.get("outer") if valid else None,"controller":self.values.get("controller","policy"),
                "mpc":result.get("mpc") if result else None,"controller_weight":transition["policy_weight"],
                "policy_valid":valid,"history_observation":observation,"history_event":event,"policy_error":self.worker.error,
                "callback_ms":(time.monotonic_ns()-started)/1e6,**transition}
            self.latest_control = row
            self.events({"kind":"control","record":row})
            self.tick += 1
        except Exception as error:
            self.stop(f"PD: {type(error).__name__}: {error}")

    def command_tick(self):
        now = time.monotonic_ns()
        with self.lock:
            stage,slot = self.stage,self.slot
        try:
            if self.events.error:
                self.request_landing("recording_failed: "+self.events.error)
                if stage in ("waiting","enabling"):
                    raise ValueError(self.events.error)
            if self.values["recorder_ready_file"] and not Path(self.values["recorder_ready_file"]).is_file():
                if stage in ("waiting","enabling"):
                    self.sender.ctbr(False,0.,(0.,0.,0.))
                    return
                self.request_landing("rosbag_recorder_lost")
            if getattr(self,"fleet_mode",False) and stage == "waiting" and not self.fleet_admitted:
                self.sender.ctbr(False,0.,(0.,0.,0.))
                return
            if stage in ("waiting","enabling") and time.monotonic()-self.wait_started>30.:
                raise ValueError("Startup timeout waiting for fresh mocap/receiver readiness")
            if stage == "waiting":
                self.sender.ctbr(False,0.,(0.,0.,0.))
                try:
                    state,_,_ = self.measured_state()
                except ValueError:
                    return
                if self.values["enable_output"] and not self.pd.liftoff.preflight(state,now/1e9):
                    self.receipt["liftoff_preflight"] = dict(self.pd.liftoff.preflight_status)
                    return
                self.receipt["liftoff_preflight"] = dict(self.pd.liftoff.preflight_status)
                status = self.sender.receiver_status()
                if status and status.session==self.sender.session_id and not status.enabled and not status.failsafe:
                    with self.lock:
                        self.stage = "enabling"
                    if not self.values["enable_output"]:
                        self.begin()
                return
            if stage == "enabling":
                state,_,_ = self.measured_state()
                if self.values["enable_output"] and not self.pd.liftoff.preflight(state,now/1e9):
                    self.sender.ctbr(False,0.,(0.,0.,0.))
                    with self.lock:
                        self.stage = "waiting"
                    return
                if self.motor_mapping:
                    status = self.sender.receiver_status()
                    if status is None or status.fc_age_ms>=150:
                        raise ValueError("Fresh FC status required")
                    self.mapping.set_voltage(status.voltage_mv/1000)
                self.sender.ctbr(True,0.,(0.,0.,0.))
                status = self.sender.receiver_status()
                if status and status.session==self.sender.session_id and status.enabled and not status.failsafe:
                    self.begin()
                return
            if stage == "stopped":
                self.sender.ctbr(False,0.,(0.,0.,0.))
                return
            if slot is None:
                if self.started_ns and (self.get_clock().now().nanoseconds-self.started_ns)>40_000_000:
                    raise ValueError("No fresh controller output")
                return
            self.measured_state()
            if self.motor_mapping and self.values["enable_output"]:
                status = self.sender.receiver_status()
                if status is None or status.fc_age_ms>=150:
                    raise ValueError("FC status stale")
                self.mapping.set_voltage(status.voltage_mv/1000)
            command_age_ns = self.get_clock().now().nanoseconds-slot["control_ros_ns"] if self.sim_clock else now-slot["control_ns"]
            if command_age_ns>=40_000_000:
                raise ValueError("PD/controller output expired")
            command = slot["ctbr"]
            self.sender.ctbr(self.values["enable_output"],command[0],tuple(command[1:]))
            status = self.sender.receiver_status()
            if status is None or (self.values["enable_output"] and (not status.enabled or status.failsafe or self.sender.rearm_required)):
                raise ValueError("Zenoh receiver interlock/lease lost")
            self.events({"kind":"command","record":{"host_ns":now,"ros_ns":self.get_clock().now().nanoseconds,"tick":slot["tick"],"ctbr":command,"enabled":self.values["enable_output"],
                "mapping":self.mapping.last if self.motor_mapping else None}})
        except Exception as error:
            self.sender.invalidate(str(error))
            self.stop(f"Command: {type(error).__name__}: {error}")

    def close(self):
        self.stop(self.error)
        self.sender.close()
        self.worker.close()
        self.receipt.update(sender_stop_reason=self.sender.stop_reason,sender_shutdown_acknowledged=self.sender.shutdown_acknowledged,
            sender_shutdown_error=self.sender.shutdown_error,sender_shutdown_status=self.sender.shutdown_status,
            error=self.error,started_ros_ns=self.started_ns,control_ticks=self.tick,history_submitted=self.worker.submitted,
            history_processed=self.worker.processed,worker_error=self.worker.error,
            blend_start_s=self.supervisor.blend_start,policy_start_s=self.supervisor.policy_start,
            orbit_start_s=self.reference.orbit_start,fallback=self.supervisor.fallback)
        if self.tf_log:self.tf_log.close()
        self.control_log.close()
        self.command_log.close()
        self.events.close()
        logs = {"control":"control.jsonl","command":"commands.jsonl"}
        if not self.isolated_io:logs["tf_estimate"]="tf.jsonl"
        streams = {kind:(self.output/name).open("w") for kind,name in logs.items()}
        try:
            for line in (self.output/"events.jsonl").open():
                event = json.loads(line)
                if event["kind"] in streams:
                    streams[event["kind"]].write(json.dumps(event["record"])+"\n")
        finally:
            for stream in streams.values():
                stream.close()
        self.receipt.update(controller_start_s=self.supervisor.policy_start,landed=self.landed,landing_reason=self.landing.reason if self.landing else None,
            landing_start_s=self.landing.start if self.landing else None,landing_touchdown_s=self.landing.touch_time if self.landing else None,
            recording_error=self.events.error,recorded_events=self.events.count,home_xy=self.home_xy.tolist() if self.home_xy is not None else None,
            ground_root=self.ground_root)
        (self.output/"receipt.json").write_text(json.dumps(self.receipt,indent=2)+"\n")


def main(args=None):
    rclpy.init(args=args,signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = SequenceNode()
        interruptions = 0
        def interrupt(signum,frame):
            nonlocal interruptions
            interruptions += 1
            if signum==signal.SIGINT and interruptions==1 and node.stage=="running" and node.values["enable_output"] and node.values["landing_enabled"]:
                node.request_landing("operator_interrupt")
            else:
                node.stop("Emergency termination")
        signal.signal(signal.SIGINT,interrupt)
        signal.signal(signal.SIGTERM,interrupt)
        gc.collect()
        gc.disable()
        executor.add_node(node)
        while rclpy.ok() and not node.finished.is_set():
            executor.spin_once(timeout_sec=.005)
    except KeyboardInterrupt:
        if node:
            node.stop("Interrupted")
    finally:
        executor.shutdown(timeout_sec=2)
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        gc.enable()
    if node and node.error:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
