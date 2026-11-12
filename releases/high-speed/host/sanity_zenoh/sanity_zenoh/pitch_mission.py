"""Experimental flight adapter for the separately tested full-attitude MPC."""
import math
import os
import sys
import threading
import time
from pathlib import Path
import json
import select
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation


class PitchWorker:
    def __init__(self, workspace, config, output):
        self.log=(output/"solver.log").open("w")
        self.process=subprocess.Popen([str(workspace/".venv-sanity-deploy/bin/python"),"-u",
            str(workspace/"loco_mjx/experiments/sanity_pitch_mpc/controller.py"),"--config",str(config)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,
            env={**os.environ,"OMP_NUM_THREADS":"1","OPENBLAS_NUM_THREADS":"1","MKL_NUM_THREADS":"1"})
        try:
            self.info=self.read(180.,startup=True)
        except BaseException:
            self.process.terminate();self.process.wait(timeout=5);self.log.close()
            raise
        self.condition=threading.Condition();self.pending=None;self.result=None;self.error=None;self.closed=False
        self.thread=threading.Thread(target=self.run,name="pitch_mpc",daemon=True);self.thread.start()


    def read(self, timeout, startup=False):
        started=time.monotonic();deadline=started+timeout
        while True:
            remaining=deadline-time.monotonic()
            if remaining<=0:raise TimeoutError("Pitch MPC response timeout; see solver.log")
            if select.select([self.process.stdout],[],[],min(2.,remaining) if startup else remaining)[0]:break
            if startup:print(f"Loading MPC solver: {time.monotonic()-started:.0f}s elapsed; output remains disabled (details: solver.log)",flush=True)
        line=self.process.stdout.readline()
        if not line:raise RuntimeError("Pitch MPC exited; see solver.log")
        return json.loads(line)


    def submit(self, request):
        with self.condition:
            self.pending=request;self.condition.notify_all()


    def latest(self):
        with self.condition:return self.result


    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.pending is not None or self.closed)
                if self.closed:return
                request=self.pending;self.pending=None
            try:
                self.process.stdin.write(json.dumps(request)+"\n");self.process.stdin.flush()
                result=self.read(1.)
                result.update(request_ns=request["request_ns"],reference_epoch=request["reference_epoch"],request=request)
                with self.condition:self.result=result
            except Exception as error:
                self.error=str(error);return


    def close(self):
        with self.condition:self.closed=True;self.condition.notify_all()
        self.thread.join(timeout=1.2)
        if self.process.poll() is None:
            self.process.stdin.write('{"op":"close"}\n');self.process.stdin.flush()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=3)
        self.log.close()


def wait_for_pitch_receiver(sender, lease_s, timeout_s=30., on_event=None):
    from sanity_zenoh.protocol import CTBR
    if not math.isfinite(timeout_s) or timeout_s<=0:raise ValueError("Positive receiver startup timeout required")
    started=time.monotonic();deadline=started+timeout_s;last_progress=-math.inf
    while True:
        sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=lease_s)
        status=sender.receiver_status();now=time.monotonic()
        if status and status.mode==CTBR and status.session==sender.session_id and status.pitch_loop_supported and not status.enabled and not status.failsafe and status.fc_ready and status.fc_age_ms<150:
            if on_event:on_event({"kind":"startup_receiver_ready","elapsed_s":now-started,"receiver_status":vars(status)})
            return status
        if now>=deadline:
            if status is None:reason="no fresh UDP command/status packets (TCP telemetry alone is insufficient)"
            elif status.session!=sender.session_id:reason="receiver has not acknowledged this sender session"
            elif status.enabled:reason="receiver has not acknowledged disabled output"
            elif not status.pitch_loop_supported:reason="SZC3 command capability has not been acknowledged"
            elif status.failsafe:reason="receiver remains in failsafe/interlock"
            else:reason=f"FC not ready or UART4 status stale (age {status.fc_age_ms}ms)"
            raise ValueError(f"No fresh SZC3 disabled acknowledgement after {timeout_s:g}s: {reason}")
        if now-last_progress>=2.:
            print(f"Waiting for disabled receiver acknowledgement: {now-started:.0f}/{timeout_s:g}s; output disabled",flush=True)
            if on_event:on_event({"kind":"startup_receiver_wait","elapsed_s":now-started,"receiver_status":vars(status) if status else None})
            last_progress=now
        time.sleep(.01)


class PitchMission:
    def __init__(self, workspace, configuration, laps=5, direction=1, bottom=.5, command_delay_s=0., coordinated=False, skip_horizontal_limit=False):
        sys.path.insert(0,str(workspace/"loco_mjx/experiments/sanity_pitch_mpc"))
        sys.path.insert(0,str(workspace/"loco_mjx/experiments/sanity_ctbr_sim2sim"))
        from reference import PitchLoopReference
        from takeoff_controller import PositionPD
        if configuration.get("reference_mode","run_in")=="run_in":
            from run_in_reference import RunInPitchReference
            self.reference=RunInPitchReference(configuration,laps,bottom,direction)
        elif configuration["reference_mode"]=="ellipse":
            self.reference=PitchLoopReference(.4769230769230769,.62,5.856231172724236,.65,laps,bottom,direction)
        else:raise ValueError("Unknown pitch reference mode")
        if not math.isfinite(command_delay_s) or not 0<=command_delay_s<=.04:raise ValueError("Command prediction delay must be in0..40ms")
        if type(skip_horizontal_limit) is not bool:raise ValueError('skip_horizontal_limit must be a boolean')
        self.skip_horizontal_limit=skip_horizontal_limit;self.horizontal_distance_m=0.
        self.coordinated=bool(coordinated);self.motion_ready=False;self.control_source="disabled"
        self.config=configuration;self.delay=command_delay_s
        self.rate_observer=None
        if configuration.get("rate_prediction","legacy")=="flight":
            from flight_prediction import FlightRatePrediction
            self.rate_observer=FlightRatePrediction(configuration)
        self.handover_limits={"rate_tolerance_rad_s":float(configuration.get("handover_rate_tolerance_rad_s",.5)),
            "thrust_tolerance_N":float(configuration.get("handover_thrust_tolerance_N",.20)),"dwell_s":1.}
        if not all(math.isfinite(v) and v>0 for v in self.handover_limits.values()):raise ValueError("Handover tolerances must be finite and positive")
        self.selection_delay=float(configuration.get("host_selection_delay_s",0.))
        self.host_output_delay=float(configuration.get("host_output_delay_s",0.))
        if not all(math.isfinite(v) and 0<=v<=.04 for v in (self.selection_delay,self.host_output_delay)):raise ValueError("Host selection/output delay must each be in0..40ms")
        self.output_delay=self.host_output_delay+self.delay
        self.prediction_delay=self.selection_delay+self.output_delay
        self.bounds=np.array([[0.,configuration["maximum_thrust_N"]],[-2.3,2.3],[-9.5,9.5],[-2.6,2.6]])
        self.pd=PositionPD(configuration["mass_kg"],integral_gains=(.8,.8,1.2))
        self.phase="waiting";self.started=None;self.epoch=None;self.settled_since=None;self.landing=None
        self.landing_requested=None;self.done=False;self.history=[];self.force=0.;self.previous=np.zeros(4)
        self.origin=None;self.ground=None;self.tick=0;self.gate=None;self.mpc_rejection=None
        self.motion_epoch=None;self.hover_gate=None;self.hover_stable_since=None
        self.transfer_s=6. if configuration.get("reference_mode","run_in")=="run_in" else 0.


    def begin(self, now, state):
        from takeoff_controller import TakeoffReference
        self.started=now;self.origin=np.array(state[:2]);self.ground=float(state[2]);self.phase="pd_takeoff"
        if not self.ground<self.reference.bottom:raise ValueError("Aircraft must start on the ground below the0.5m takeoff target")
        self.takeoff=TakeoffReference(self.ground,self.reference.bottom)
        self.history=[(now,self.force,self.previous.copy())]


    def request_landing(self, reason):
        self.landing_requested=self.landing_requested or reason


    def sample(self, now):
        if self.landing is not None:return self.landing.sample(now)
        r=self.reference.sample(self.reference_time(now))
        rotation=Rotation.from_quat(r[[4,5,6,3]])
        acceleration=rotation.as_matrix()[:,2]*r[13]/self.config["mass_kg"]-[0.,0.,9.81]
        return {"position":(r[:3]+np.r_[self.origin,0.]).tolist(),"velocity":r[7:10].tolist(),"acceleration":acceleration.tolist(),"phase":self.phase}


    def schedule_motion(self, epoch, now):
        if not self.coordinated or not self.motion_ready or self.motion_epoch is not None or not np.isfinite(epoch) or epoch<=now:
            raise ValueError("Coordinated motion requires a ready mission and a future epoch")
        self.motion_epoch=float(epoch)


    def reference_time(self, now):
        if self.motion_epoch is None:return 0.
        return max(0.,self.reference.hold+now-self.motion_epoch) if self.coordinated else self.reference.hold+max(0.,now-self.motion_epoch)


    def applied_command_at(self, time_s):
        index=max(0,min(len(self.history)-1,int(np.searchsorted([r[0] for r in self.history],time_s-self.output_delay,side="right")-1)))
        return self.history[index][2]


    def force_at(self, time_s):
        index=max(0,min(len(self.history)-1,int(np.searchsorted([r[0] for r in self.history],time_s,side="right")-1)))
        cursor=self.history[index][0];force=self.history[index][1]
        changes=[r[0]+self.output_delay for r in self.history if cursor<r[0]+self.output_delay<time_s]
        for end in [*changes,time_s]:
            target=float(self.applied_command_at((cursor+end)/2)[0])
            tau=self.config["collective_tau_up_s"] if target>=force else self.config["collective_tau_down_s"]
            force+=(-np.expm1(-max(0.,end-cursor)/tau))*(target-force);cursor=end
        return float(force)


    def inference_request(self, now, state, age, pose_filter_delay=None):
        rotation=Rotation.from_quat(np.asarray(state)[[4,5,6,3]])
        sampled=now-age
        times=np.array([r[0] for r in self.history])
        force=self.force_at(sampled) if self.output_delay else np.interp(sampled,times,[r[1] for r in self.history])
        measured=np.r_[state[:10],rotation.inv().apply(state[10:13]),force]
        rate_observer=self.rate_observer.observe(sampled,measured[10:13],self.history,self.output_delay) if self.rate_observer is not None else None
        if pose_filter_delay is None:
            pose_filter_delay={key:(.005/np.expm1(.005/self.config.get(field,0.)) if self.config.get(field,0.) else 0.)
                for key,field in (("position","position_filter_tau_s"),("attitude","attitude_filter_tau_s"))}
        if not all(math.isfinite(pose_filter_delay[k]) and 0<=pose_filter_delay[k]<=.04 for k in ("position","attitude")):raise ValueError("Invalid pose-filter prediction delay")
        segments=[];cursor=sampled
        while cursor<now+self.prediction_delay-1e-8:
            end=min(cursor+.005,now+self.prediction_delay)
            changes=[r[0]+self.output_delay for r in self.history if cursor+1e-9<r[0]+self.output_delay<end]
            if changes:end=min(changes)
            segments.append((end-cursor,self.applied_command_at((cursor+end)/2).tolist()));cursor=end
        offsets=np.arange(self.config.get("nodes",20)+1)*self.config.get("prediction_dt_s",.04)
        t=self.reference_time(now)
        preview=self.reference.sample(t+self.prediction_delay+offsets) if self.motion_epoch is not None else np.tile(self.reference.sample(0.),(len(offsets),1))
        preview[:,:2]+=self.origin
        return {"op":"infer","state":measured.tolist(),"preview":preview.tolist(),"sample_command":self.applied_command_at(sampled).tolist(),
            "prediction_segments":segments,"filter_compensation_s":.0225,"request_ns":int(now*1e9),"reference_epoch":self.epoch,
            "motion_epoch":self.motion_epoch,"pose_filter_delay_s":pose_filter_delay,"rate_observer":rate_observer,"timing":{"host_selection_s":self.selection_delay,"host_output_s":self.host_output_delay,"external_command_s":self.delay,"prediction_s":self.prediction_delay}}


    def choose(self, now, state, result, force_limit):
        from sanity_zenoh.landing import Landing
        self.control_source="pd" if self.epoch is None or self.landing is not None else "mpc"
        state=np.asarray(state);rotation=Rotation.from_quat(state[[4,5,6,3]])
        tilt=float(np.degrees(np.arccos(np.clip(rotation.as_matrix()[2,2],-1.,1.))))
        elapsed=now-self.started
        if elapsed>120.:raise ValueError("Pitch mission exceeded120s runtime")
        horizontal_distance=float(np.linalg.norm(state[:2]-self.origin));self.horizontal_distance_m=horizontal_distance
        if state[2]>2. or (horizontal_distance>2. and not self.skip_horizontal_limit):
            limits="2m/bypassed" if self.skip_horizontal_limit else "2m/2m"
            raise ValueError(f"Pitch mission flight envelope exceeded:height={state[2]:.3f}m, horizontal distance={horizontal_distance:.3f}m (limits{limits})")
        if self.epoch is not None and self.landing is None and state[2]<.15:raise ValueError("Pitch mission below minimum flight height")
        valid=bool(result and result["mpc"]["valid"] and 0<=(now*1e9-result["request_ns"])/1e9<.04)
        if self.epoch is not None and result and result["reference_epoch"]!=self.epoch:valid=False
        if self.epoch is None and self.landing is None:
            from takeoff_controller import smooth_fraction
            self.phase="pd_takeoff"
            ref=self.takeoff.sample(elapsed);ref["position"][:2]=self.origin.tolist()
            transfer_start=self.takeoff.climb_end+1.
            if self.transfer_s and elapsed>=transfer_start:
                u=float(np.clip((elapsed-transfer_start)/self.transfer_s,0.,1.));delta=self.reference.sample(0.)[:2]
                ref["position"][:2]=(self.origin+delta*smooth_fraction(u)).tolist()
                ref["velocity"][:2]=(delta*30*u**2*(1-u)**2/self.transfer_s).tolist()
                ref["acceleration"][:2]=(delta*60*u*(1-u)*(1-2*u)/self.transfer_s**2).tolist()
                self.phase="pd_reposition" if u<1. else "pd_run_in_hold"
            target=self.reference.sample(0.)[:3]+np.r_[self.origin,0.]
            command=self.pd.takeoff_command(state[:3],state[7:10],rotation,ref,elapsed,float(state[2]-self.ground),self.takeoff.climb_start,
                integrate=elapsed>self.takeoff.climb_start and state[2]-self.ground>.02)
            yaw=rotation.as_euler("xyz")[2]
            position_error=float(np.linalg.norm(state[:3]-target))
            speed=float(np.linalg.norm(state[7:10]))
            thrust_difference=float(abs(result["ctbr"][0]-command[0])) if result else None
            rate_difference=float(np.max(abs(np.asarray(result["ctbr"])[1:]-command[1:]))) if result else None
            gates={"climb_complete":elapsed>=self.takeoff.climb_end+(1.+self.transfer_s if self.transfer_s else 0.),"position":position_error<.08,"speed":speed<.2,"tilt":tilt<10.,"yaw":abs(yaw)<.15,
                "mpc_valid_fresh":valid,"thrust_match":thrust_difference is not None and thrust_difference<self.handover_limits["thrust_tolerance_N"],"rate_match":rate_difference is not None and rate_difference<self.handover_limits["rate_tolerance_rad_s"]}
            settled=all(gates[k] for k in ("climb_complete","position","speed","tilt","yaw"))
            match=all(gates[k] for k in ("mpc_valid_fresh","thrust_match","rate_match"))
            self.gate={"passed":{k:bool(v) for k,v in gates.items()},"blocking":[k for k,v in gates.items() if not v],"position_error_m":position_error,"speed_m_s":speed,
                "tilt_deg":tilt,"yaw_rad":float(yaw),"thrust_difference_N":thrust_difference,"rate_difference_rad_s":rate_difference,"limits":self.handover_limits.copy()}
            self.settled_since=(now if self.settled_since is None else self.settled_since) if settled and match else None
            if self.landing_requested:
                self.landing=Landing(now,state[:3],self.origin,self.ground,self.landing_requested,0.,velocity=state[7:10]);self.phase="landing"
            elif self.settled_since is not None and now-self.settled_since>=self.handover_limits["dwell_s"]:
                self.epoch=now;self.phase="mpc_initial_hover";self.settled_since=None
            elif elapsed>35.:self.request_landing("takeoff_gate_timeout")
        elif self.landing is None:
            t=self.reference_time(now)
            self.phase=self.reference.phase(t) if hasattr(self.reference,"phase") else "mpc_initial_hover" if t<self.reference.hold else "pitch_loops" if t<self.reference.hold+self.reference.moving_s else "mpc_recovery"
            if not valid:
                diagnostics=result.get("mpc",{}) if result else {}
                reasons=[]
                age=(now*1e9-result["request_ns"])/1e9 if result else None
                if result is None:reasons.append("no MPC result")
                else:
                    if not 0<=age<.04:reasons.append(f"result age={age*1000:.1f}ms (limit40ms)")
                    if result["reference_epoch"]!=self.epoch:reasons.append("reference epoch mismatch")
                    if diagnostics.get("status",0):reasons.append("solver status="+str(diagnostics["status"]))
                    if diagnostics.get("predicted_height_max_m",0)>2.:reasons.append(f"predicted ceiling={diagnostics['predicted_height_max_m']:.4f}m (limit2m)")
                    if diagnostics.get("predicted_height_min_m",1)<.12:reasons.append(f"predicted floor={diagnostics['predicted_height_min_m']:.4f}m (limit0.12m)")
                    if diagnostics.get("total_ms",0)>25.:reasons.append(f"computation={diagnostics['total_ms']:.1f}ms (limit25ms)")
                    if diagnostics.get("input_violation",0)>=1e-4:reasons.append("predicted input bounds violated")
                    if diagnostics.get("reason"):reasons.append(str(diagnostics["reason"]))
                    if not reasons:reasons.append("MPC result marked invalid")
                self.mpc_rejection={"time_s":elapsed,"phase":self.phase,"reasons":reasons,"result_age_s":age,"result":result}
                raise ValueError("Pitch MPC stale/invalid: "+"; ".join(reasons)+"; upright PD is not an inverted recovery controller")
            command=np.asarray(result["ctbr"])
            if self.motion_epoch is None or (self.coordinated and now<self.motion_epoch):
                hover=self.reference.sample(0.)[:3]+np.r_[self.origin,0.]
                body_rate=rotation.inv().apply(state[10:13])
                gates={"minimum_hover_time":now-self.epoch>=self.reference.hold,"position":np.linalg.norm(state[:3]-hover)<.08,
                    "speed":np.linalg.norm(state[7:10])<.2,"tilt":tilt<6.,"body_rate":np.linalg.norm(body_rate)<.5,
                    "command_rate":np.max(abs(command[1:]))<.5,"thrust":abs(command[0]-self.config["mass_kg"]*9.81)<.20,
                    "prediction_consistent":result["mpc"].get("rollout_vs_raw_state_max_error",0.)<.1}
                self.hover_gate={"passed":{k:bool(v) for k,v in gates.items()},"blocking":[k for k,v in gates.items() if not v]}
                self.hover_stable_since=(now if self.hover_stable_since is None else self.hover_stable_since) if all(gates.values()) else None
                self.motion_ready=self.hover_stable_since is not None and now-self.hover_stable_since>=1.
                if self.landing_requested or now-self.epoch>(45. if self.coordinated else 12.):
                    self.landing=Landing(now,state[:3],self.origin,self.ground,self.landing_requested or "mpc_hover_gate_timeout",0.,velocity=state[7:10]);self.phase="landing";self.pd.reset()
                elif self.hover_stable_since is not None and now-self.hover_stable_since>=1. and self.config.get("hover_only",False):
                    self.landing=Landing(now,state[:3],self.origin,self.ground,"hover_qualification_complete",0.,velocity=state[7:10]);self.phase="landing";self.pd.reset()
                elif self.hover_stable_since is not None and now-self.hover_stable_since>=1. and not self.coordinated:
                    self.motion_epoch=now;t=self.reference_time(now);self.phase=self.reference.phase(t) if hasattr(self.reference,"phase") else "pitch_loops"
            ref=self.sample(now)
            if np.linalg.norm(state[:3]-ref["position"])>1.5:raise ValueError("Pitch tracking error exceeded1.5m")
            recovered=self.motion_epoch is not None and t>=self.reference.hold+self.reference.moving_s+2. and np.linalg.norm(state[:3]-(self.reference.sample(self.reference.duration)[:3]+np.r_[self.origin,0.]))<.12 and np.linalg.norm(state[7:10])<.2 and tilt<6.
            self.settled_since=(now if self.settled_since is None else self.settled_since) if recovered else None
            if self.landing is None and self.settled_since is not None and now-self.settled_since>=1.:
                self.landing=Landing(now,state[:3],self.origin,self.ground,self.landing_requested or "mission_complete",0.,velocity=state[7:10]);self.phase="landing";self.pd.reset()
            elif t>self.reference.duration+6.:raise ValueError("Pitch MPC hover recovery timeout")
        if self.landing is not None:
            self.control_source="pd_landing"
            if now>self.landing.timeout:raise ValueError("Landing timeout: touchdown not confirmed")
            ref=self.landing.sample(now);self.phase=ref["phase"]
            self.done=self.landing.update_touchdown(now,state,tilt)
            command=self.pd.command(state[:3],state[7:10],rotation,ref,integrate=self.landing.touch_time is None)*self.landing.spool(now)
        if self.epoch is None or self.landing is not None:
            if tilt>45.:raise ValueError("Upright PD tilt limit exceeded")
            command=np.clip(command,[0.,-2.3,-2.3,-2.6],[self.bounds[0,1],2.3,2.3,2.6])
            if self.epoch is None and elapsed<self.takeoff.settle_s:command=np.zeros(4)
            elif self.epoch is None and elapsed<self.takeoff.climb_start:command*=np.clip((elapsed-self.takeoff.settle_s)/(self.takeoff.climb_start-self.takeoff.settle_s),0.,1.)
            command[0]=min(command[0],force_limit);self.pd.accept_output(command)
        elif command[0]>force_limit+1e-4:raise ValueError("MPC thrust exceeds delivered-voltage capability")
        if command.shape!=(4,) or not np.isfinite(command).all() or np.any(command<self.bounds[:,0]-1e-6) or np.any(command>self.bounds[:,1]+1e-6):raise ValueError("Pitch CTBR outside physical command bounds")
        return command,ref


    def commit(self, now, command):
        self.force=self.force_at(now)
        self.previous=np.array(command,copy=True);self.history.append((now,self.force,self.previous.copy()))
        self.history=self.history[-256:];self.tick+=1
