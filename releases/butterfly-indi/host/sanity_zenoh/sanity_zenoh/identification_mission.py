"""PD flight identification with explicit, short CTBR excitation windows."""
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
from sanity_zenoh.landing import Landing


class IdentificationMission:
    def __init__(self, workspace, mission="heights", height=1., heights=(.5,.8,1.1,.8,.5), hold=10., mass=.06985, seed=0, floor_z=None):
        sys.path.insert(0,str(Path(workspace)/"loco_mjx/experiments/sanity_ctbr_sim2sim"))
        from takeoff_controller import PositionPD
        if mission not in ("hover","heights","collective","roll","pitch","yaw"):
            raise ValueError("Unknown identification mission")
        if not all(math.isfinite(v) for v in (height,hold,mass,*heights)) or not .3<=height<=1.2 or not 5<=hold<=30 or not .04<=mass<=.12:
            raise ValueError("Height0.3..1.2m, hold5..30s, mass0.04..0.12kg required")
        if not 1<=len(heights)<=8 or not all(.3<=h<=1.2 for h in heights):
            raise ValueError("One to eight heights in0.3..1.2m required")
        if floor_z is not None and not math.isfinite(floor_z):raise ValueError("Finite floor datum required")
        self.name=mission;self.height=float(height);self.heights=list(heights);self.hold=float(hold);self.mass=mass;self.floor_z=floor_z
        self.pd=PositionPD(mass,integral_gains=(.8,.8,1.2))
        self.started=None;self.origin=None;self.ground=None;self.datum=None;self.takeoff=None;self.target=None
        self.phase="waiting";self.landing=None;self.landing_requested=None;self.done=False;self.tick=0;self.history=[]
        self.gate=None;self.stable_since=None;self.phase_started=None;self.hold_started=None;self.transfer=None
        self.gate_rate_limit=.6;self.gate_rate_tau=.05;self.gate_rate_filtered=None;self.gate_rate_source=None;self.gate_rate_raw=None
        self.height_index=0;self.trial_index=0;self.pulse_started=None;self.baseline=None;self.baseline_samples=[]
        self.trials=[];self.trial_results=[];self.pending_events=[];self.last_now=None;self.active_trial=None
        rng=np.random.default_rng(seed)
        if mission=="collective":
            for fraction in (.03,.05):
                group=[{"delta_thrust_N":sign*fraction*mass*9.81,"rate_axis":None,"delta_rate_rad_s":0.,"duration_s":duration,"fraction_weight":sign*fraction}
                    for duration in (.16,.24) for repeat in range(2) for sign in (-1,1)]
                self.trials.extend(group[int(i)] for i in rng.permutation(len(group)))
        elif mission in ("roll","pitch","yaw"):
            for amplitude in ((.2,.4) if mission=="yaw" else (.3,.6)):
                group=[{"delta_thrust_N":0.,"rate_axis":("roll","pitch","yaw").index(mission)+1,
                    "delta_rate_rad_s":sign*amplitude,"duration_s":.24 if mission=="yaw" else .16}
                    for repeat in range(3) for sign in (-1,1)]
                self.trials.extend(group[int(i)] for i in rng.permutation(len(group)))
        self.recovery_s=3. if mission=="collective" else 2.
        self.info={"mission":mission,"controller":"PD with explicit short collective/tested-axis overrides; no policy/MPC",
            "mass_kg":mass,"height_m":height,"height_sequence_m":self.heights if mission=="heights" else [height],
            "height_datum":"floor_z if supplied; otherwise initial tracked ground height (not a measured rotor-floor offset)",
            "floor_z":floor_z,"hold_s":hold,"pulse_trials":self.trials,"seed":seed,"recovery_minimum_s":self.recovery_s,
            "command_lease_s":1.,"control_hz":50.,"transport":"Zenoh physical CTBR; existing FC600deg/s mapping",
            "ESP_power":"Normal battery-powered flight path","native_FC_logging":False,
            "settling_rate_gate":{"limit_rad_s":self.gate_rate_limit,"filter_tau_s":self.gate_rate_tau,
                "source":"Fresh scaled FC gyro in flight; mocap fallback for offline callers only",
                "scope":"Probe/height-hold entry only; controller observations and outgoing commands unchanged","continuous_dwell_s":1.},
            "pulse_envelope":{"height_error_m":.3,"horizontal_excursion_m":.3,"tilt_deg":10.,"speed_m_s":.6},
            "hard_envelope":{"height_above_datum_m":1.6,"horizontal_radius_m":2.,"tilt_deg":45.},
            "interrupt":"First Ctrl-C cancels excitation and requests PD landing; second Ctrl-C/SIGTERM disarms immediately"}


    def excitation_command(self, command, trial, elapsed):
        """Default short-step override; subclasses may supply other excitations."""
        command=command.copy()
        command[0]=self.baseline[0]+trial["delta_thrust_N"]
        axis=trial["rate_axis"]
        if axis is not None:command[axis]=self.baseline[axis]+trial["delta_rate_rad_s"]
        return command


    def begin(self, now, state):
        from takeoff_controller import TakeoffReference
        state=np.asarray(state)
        self.started=now;self.last_now=now;self.phase_started=now;self.origin=state[:2].copy();self.ground=float(state[2])
        self.datum=self.ground if self.floor_z is None else self.floor_z
        initial_height=self.heights[0] if self.name=="heights" else self.height
        self.target=np.r_[self.origin,self.datum+initial_height]
        if self.target[2]-self.ground<.2:raise ValueError("Takeoff target must be at least0.2m above the initial tracked ground height")
        self.takeoff=TakeoffReference(self.ground,self.target[2]);self.phase="takeoff"
        self.pending_events.append({"kind":"identification_begin","ground_height_m":self.ground,"height_datum_m":self.datum,"origin_xy":self.origin.tolist()})


    def request_landing(self, reason):
        self.landing_requested=self.landing_requested or reason


    def settled(self, state, rotation, command, telemetry_ok):
        tilt=float(np.degrees(np.arccos(np.clip(rotation.as_matrix()[2,2],-1.,1.))))
        gates={"position":np.linalg.norm(state[:3]-self.target)<.08,"speed":np.linalg.norm(state[7:10])<.15,
            "tilt":tilt<5.,"body_rate":self.gate_rate_filtered is not None and np.linalg.norm(self.gate_rate_filtered)<self.gate_rate_limit,
            "command_rate":np.max(abs(command[1:]))<.25,"telemetry":bool(telemetry_ok)}
        self.gate={"passed":{k:bool(v) for k,v in gates.items()},"blocking":[k for k,v in gates.items() if not v],
            "rate_source":self.gate_rate_source,"rate_raw_rad_s":self.gate_rate_raw.tolist(),
            "rate_filtered_rad_s":self.gate_rate_filtered.tolist(),"rate_raw_norm_rad_s":float(np.linalg.norm(self.gate_rate_raw)),
            "rate_filtered_norm_rad_s":float(np.linalg.norm(self.gate_rate_filtered)),
            "rate_limit_rad_s":self.gate_rate_limit,"rate_filter_tau_s":self.gate_rate_tau}
        return all(gates.values())


    def choose(self, now, state, force_limit, telemetry_ok=True, gyro_rad_s_fc=None):
        state=np.asarray(state,dtype=float)
        if self.started is None:raise ValueError("Identification mission not started")
        if state.shape!=(13,) or not np.isfinite(state).all() or not math.isfinite(now) or not math.isfinite(force_limit) or force_limit<=0:
            raise ValueError("Finite state13/time and positive force limit required")
        if now<self.last_now:raise ValueError("Identification controller clock moved backwards")
        if now-self.last_now>.04000001:
            self.request_landing('controller_deadline_missed')
            self.pending_events.append({'kind':'controller_deadline_missed','gap_s':now-self.last_now,
                                        'response':'Cancel excitation; PD landing using current validated state'})
        dt=now-self.last_now;self.last_now=now
        rotation=Rotation.from_quat(state[[4,5,6,3]])
        source="fc_gyro" if gyro_rad_s_fc is not None else "mocap_offline_fallback"
        rate=np.asarray(gyro_rad_s_fc,dtype=float) if gyro_rad_s_fc is not None else rotation.inv().apply(state[10:13])
        if rate.shape!=(3,) or not np.isfinite(rate).all():raise ValueError("Finite three-axis settling gyro required")
        self.gate_rate_raw=rate.copy()
        if self.gate_rate_filtered is None or source!=self.gate_rate_source:self.gate_rate_filtered=rate.copy()
        else:self.gate_rate_filtered+=(-math.expm1(-dt/self.gate_rate_tau))*(rate-self.gate_rate_filtered)
        self.gate_rate_source=source
        tilt=float(np.degrees(np.arccos(np.clip(rotation.as_matrix()[2,2],-1.,1.))))
        horizontal=float(np.linalg.norm(state[:2]-self.origin))
        if state[2]>self.datum+1.6 or horizontal>2. or tilt>45. or state[2]<self.ground-.1:
            raise ValueError("Identification hard flight envelope exceeded")
        if now-self.started>self.info.get("hard_runtime_s",240.):raise ValueError("Identification mission runtime exceeded")
        if self.phase=="pulse" and (abs(state[2]-self.target[2])>.3 or horizontal>.3 or tilt>10. or np.linalg.norm(state[7:10])>.6):
            self.request_landing("probe_envelope")
        if self.landing_requested and self.landing is None:
            if self.active_trial is not None:
                self.pending_events.append({"kind":"probe_cancelled","trial_index":self.trial_index,"reason":self.landing_requested,"actual_duration_s":now-self.pulse_started if self.pulse_started is not None else 0.})
                self.active_trial=None
            self.landing=Landing(now,state[:3],self.origin,self.ground,self.landing_requested,0.,velocity=state[7:10])
            self.phase="landing";self.phase_started=now
        if self.landing is not None:
            if now>self.landing.timeout:raise ValueError("Identification landing timeout")
            reference=self.landing.sample(now);self.phase=reference["phase"]
            self.done=self.landing.update_touchdown(now,state,tilt)
            command=self.pd.command(state[:3],state[7:10],rotation,reference,dt=.02,integrate=self.landing.touch_time is None)*self.landing.spool(now)
            self.pd.accept_output(command)
            command=np.clip(command,[0.,-2.3,-2.3,-2.6],[min(2.85,force_limit),2.3,2.3,2.6])
            self.pd.accept_output(command)
            return command,reference,{"trial_index":None,"pulse_active":False}
        reference={"position":self.target.tolist(),"velocity":[0.,0.,0.],"acceleration":[0.,0.,0.],"phase":self.phase}
        if self.phase=="takeoff":
            reference=self.takeoff.sample(now-self.started);reference["position"][:2]=self.origin.tolist()
        elif self.phase=="height_transfer":
            start,initial=self.transfer;u=float(np.clip((now-start)/4.,0.,1.));delta=self.target-initial
            fraction=u**3*(10-15*u+6*u*u)
            reference.update(position=(initial+delta*fraction).tolist(),velocity=(delta*30*u*u*(1-u)**2/4.).tolist(),acceleration=(delta*60*u*(1-u)*(1-2*u)/16.).tolist())
            if u>=1.:self.phase="settle";self.phase_started=now;self.stable_since=None
        pulse_active=self.phase=="pulse" and (self.pulse_started is None or now-self.pulse_started<self.active_trial["duration_s"]-1e-8)
        if pulse_active and self.pulse_started is None:
            self.pulse_started=now
            self.pending_events.append({"kind":"probe_begin","trial_index":self.trial_index,"actual_start_s":now-self.started,
                "baseline_ctbr":self.baseline.tolist(),**self.active_trial})
        if self.phase=="takeoff" or not self.pd.liftoff.complete:
            command=self.pd.takeoff_command(state[:3],state[7:10],rotation,reference,now-self.started,float(state[2]-self.ground),self.takeoff.climb_start,
                integrate=now-self.started>self.takeoff.climb_start and state[2]-self.ground>.02)
        else:
            command=self.pd.command(state[:3],state[7:10],rotation,reference,dt=.02,
                integrate=not pulse_active and now-self.started>self.takeoff.climb_start and state[2]-self.ground>.02)
        baseline_pd=command.copy()
        if self.phase=="takeoff":
            elapsed=now-self.started
            if elapsed<self.takeoff.settle_s:command=np.zeros(4)
            elif elapsed<self.takeoff.climb_start:command*=np.clip((elapsed-self.takeoff.settle_s)/self.takeoff.spool_s,0.,1.)
            if elapsed>=self.takeoff.climb_end:self.phase="settle";self.phase_started=now
        if self.phase=="pulse":
            if pulse_active:
                command=self.excitation_command(command,self.active_trial,now-self.pulse_started)
            else:
                self.pending_events.append({"kind":"probe_end","trial_index":self.trial_index,"actual_duration_s":now-self.pulse_started if self.pulse_started is not None else 0.})
                self.trial_results.append({"trial_index":self.trial_index,"started_s":self.pulse_started-self.started,"actual_duration_s":now-self.pulse_started if self.pulse_started is not None else 0.,**self.active_trial})
                self.trial_index+=1;self.active_trial=None;self.phase="recovery";self.phase_started=now;self.stable_since=None;self.baseline_samples=[]
        elif self.phase in ("settle","recovery","height_hold"):
            ready=self.settled(state,rotation,command,telemetry_ok)
            self.stable_since=(now if self.stable_since is None else self.stable_since) if ready else None
            self.baseline_samples.append(command.copy());self.baseline_samples=self.baseline_samples[-25:]
            stable=self.stable_since is not None and now-self.stable_since>=1.-1e-8
            if self.phase=="height_hold":
                if now-self.hold_started>=self.hold:
                    self.height_index+=1
                    if self.name=="hover" or self.height_index>=len(self.heights):self.request_landing("identification_complete")
                    else:
                        initial=self.target.copy();self.target[2]=self.datum+self.heights[self.height_index]
                        self.transfer=(now,initial);self.phase="height_transfer";self.phase_started=now;self.stable_since=None
            elif stable and now-self.phase_started>=(self.recovery_s if self.phase=="recovery" else 2.):
                if self.name in ("hover","heights"):
                    self.phase="height_hold";self.hold_started=now;self.phase_started=now
                    self.pending_events.append({"kind":"height_hold_begin","height_index":self.height_index,"target_position":self.target.tolist(),"height_above_datum_m":self.target[2]-self.datum})
                elif self.trial_index>=len(self.trials):self.request_landing("identification_complete")
                else:
                    self.baseline=np.mean(self.baseline_samples,axis=0);self.active_trial=self.trials[self.trial_index]
                    self.phase="pulse";self.pulse_started=None
                    self.pending_events.append({"kind":"probe_armed","trial_index":self.trial_index,"first_tick_expected_s":now+.02-self.started,"baseline_ctbr":self.baseline.tolist(),**self.active_trial})
            elif now-self.phase_started>15. and self.phase!="height_hold":self.request_landing("settle_timeout")
        clipped=np.clip(command,[0.,-2.3,-2.3,-2.6],[min(2.85,force_limit),2.3,2.3,2.6])
        if pulse_active and not np.allclose(clipped,command,atol=1e-10,rtol=0.):
            self.request_landing("probe_command_clipped");clipped=np.clip(baseline_pd,[0.,-2.3,-2.3,-2.6],[min(2.85,force_limit),2.3,2.3,2.6]);pulse_active=False
        self.pd.accept_output(clipped)
        reference["phase"]=self.phase
        details={"trial_index":self.trial_index if self.active_trial is not None else None,"pulse_active":bool(pulse_active),
            "baseline_pd_ctbr":baseline_pd.tolist(),"held_baseline_ctbr":self.baseline.tolist() if self.active_trial is not None else None,
            "excitation":self.active_trial if self.active_trial is not None else None,"target_height_above_datum_m":self.target[2]-self.datum}
        return clipped,reference,details


    def commit(self, now, command):
        self.history.append((now,np.array(command,copy=True)));self.history=self.history[-256:];self.tick+=1
