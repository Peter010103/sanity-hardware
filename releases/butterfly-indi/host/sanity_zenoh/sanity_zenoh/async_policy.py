"""Ordered observation/selection pairs; inference never blocks ROS timers."""

import json
import threading
import time
from sanity_zenoh.local_policy import ActorProcess
from sanity_zenoh.recording import EventRecorder


class AsyncPolicy:
    def __init__(self, python, source, checkpoint, output, residual_model=None, residual_source=None, controller="policy", mpc_model=None, isolated_recording=False):
        self.actor = ActorProcess(python, source, checkpoint, output / "policy_worker.log", realtime=True, residual_model=residual_model, residual_source=residual_source, controller=controller, mpc_model=mpc_model)
        try:
            self.info = self.actor.info
            self.controller = controller
            self.condition = threading.Condition()
            self.observations = {}
            self.selections = {}
            self.result = None
            self.error = None
            self.accepting = True
            self.observed = self.submitted = self.processed = 0
            self.history_recorder = EventRecorder(output / "history.jsonl",isolated=True,stamp=False) if isolated_recording else None
            self.log = None if isolated_recording else (output / "history.jsonl").open("w", buffering=1)
            self.thread = threading.Thread(target=self.run, name="ordered_policy_history", daemon=True)
            self.thread.start()
        except BaseException:
            for entity in (getattr(self, 'history_recorder', None), getattr(self, 'log', None)):
                if entity is not None:
                    try:entity.close()
                    except Exception:pass
            self.actor.close()
            raise

    def observe(self, observation):
        with self.condition:
            if not self.accepting or self.error:
                raise RuntimeError(self.error or "Policy stream closed")
            if observation["tick"] != self.observed:
                raise ValueError("Observation tick gap or duplicate")
            if len(self.observations)>=256:
                self.error = "Causal observation queue overflow"
                raise RuntimeError(self.error)
            self.observations[self.observed] = observation
            self.observed += 1
            self.condition.notify_all()

    def commit(self, selection):
        with self.condition:
            if not self.accepting or self.error:
                raise RuntimeError(self.error or "Policy stream closed")
            if selection["tick"] != self.submitted or self.submitted>=self.observed:
                raise ValueError("Selection must follow its observation exactly once")
            self.selections[self.submitted] = selection
            self.submitted += 1
            self.condition.notify_all()

    def latest(self):
        with self.condition:
            return dict(self.result) if self.result else None

    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.processed in self.observations or not self.accepting)
                if self.processed not in self.observations:
                    return
                observation = self.observations.pop(self.processed)
            try:
                extra = {key:observation[key] for key in ("preview","state_age_s","residual_enabled","reference_preview","prediction_context") if key in observation}
                if self.controller=="mpc":
                    extra.update(state_stamp_ns=observation["state_stamp_ns"])
                    if "disturbance_context" in observation:extra["disturbance_context"]=observation["disturbance_context"]
                result = self.actor.call("infer",deadline=time.monotonic()+1.,state=observation["state"],
                    goal=observation["goal"],commit=False,record_inputs=True,test_delay_s=observation.get("test_delay_s",0.),**extra)
                result.update(tick=self.processed,state_stamp_ns=observation["state_stamp_ns"],finished_host_ns=time.monotonic_ns())
                with self.condition:
                    self.result = result
                    self.condition.wait_for(lambda:self.processed in self.selections or not self.accepting)
                    if self.processed not in self.selections:
                        return
                    selection = self.selections.pop(self.processed)
                extra = {key:selection[key] for key in ("selected_velocity_world","outgoing_ctbr") if key in selection}
                if self.controller=="mpc" and "controller_context" in selection:extra["controller_context"]=selection["controller_context"]
                committed = self.actor.call("commit",deadline=time.monotonic()+1.,**extra,
                    **({"action":selection["selected_action"]} if "selected_action" in selection else {"ctbr":selection["selected_ctbr"]}))
                entry={"observation":observation,"selection":selection,"result":result,"committed":committed}
                if self.history_recorder:
                    if self.history_recorder.error:raise RuntimeError(self.history_recorder.error)
                    self.history_recorder(entry)
                else:self.log.write(json.dumps(entry)+"\n")
                self.processed += 1
            except Exception as error:
                self.error = f"{type(error).__name__}: {error}"
                return

    def close(self):
        with self.condition:
            self.accepting = False
            self.condition.notify_all()
        self.thread.join(timeout=2)
        self.actor.close()
        if self.thread.is_alive():
            self.thread.join(timeout=1)
        if self.history_recorder:
            self.history_recorder.close()
            if self.history_recorder.error:self.error=self.error or 'History recording: '+self.history_recorder.error
        if self.log:self.log.close()
