"""One takeoff, bounded rate-gain search during hover, validation and landing."""
import json
from pathlib import Path
import queue
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
from sanity_zenoh.identification_mission import IdentificationMission
from sanity_zenoh.rate_autotune import RatePulseLog,RateGainSearch,batch_score


class AutotuneMission(IdentificationMission):
    def __init__(self,workspace,height=1.,mass=.06985,axes=('roll','pitch'),evaluations=4,tune_d=False,save_best=False,floor_z=None,tuning_profile="legacy",step_duration=.4,step_amplitudes=(.2,.3)):
        super().__init__(workspace,'roll',height=height,mass=mass,floor_z=floor_z)
        if not axes or any(a not in ('roll','pitch') for a in axes) or len(set(axes))!=len(axes) or not 2<=evaluations<=6:raise ValueError('Distinct roll/pitch axes and2..6evaluations required')
        if tuning_profile not in ("legacy","damping","boost"):raise ValueError("Unknown tuning profile")
        amplitudes=tuple(float(v) for v in step_amplitudes)
        if not np.isfinite(step_duration) or not .16<=step_duration<=.5 or len(amplitudes)!=2 or not all(np.isfinite(v) and .1<=v<=.6 for v in amplitudes) or not amplitudes[0]<amplitudes[1]:
            raise ValueError('Step duration0.16..0.5s and two increasing amplitudes0.1..0.6rad/s required')
        if max(amplitudes)*step_duration>.120000001:
            raise ValueError('Rate times duration must not exceed0.12rad; reduce amplitude for longer holds')
        self.step_duration=float(step_duration);self.step_amplitudes=amplitudes
        self.tuning_profile=tuning_profile
        self.axes=tuple(axes);self.evaluations=evaluations;self.tune_d=tune_d;self.save_best=save_best;self.rpc=None;self.search=None;self.output=None
        self.measurements=RatePulseLog();self.analysis=None;self.worker=None;self.results=queue.Queue();self.cancel=threading.Event();self.batch_retries=0;self.tuning_error=None;self.tuning_complete=False;self.finalized=False;self.tuning_receipt={}
        self.name=self.axes[0];self.trials=self.next_trials()
        self.info.update(mission='rate_autotune',axes=list(axes),evaluations_per_axis=evaluations,validation_batches_per_axis=1,nominal_pulses=4*(evaluations+1)*len(axes),pulse_trials=self.trials,
            step_duration_s=self.step_duration,step_amplitudes_rad_s=list(self.step_amplitudes),nominal_max_step_rotation_deg=float(np.degrees(max(self.step_amplitudes)*self.step_duration)),
            scoring_window='Entire commanded hold; late gain from final quarter for holds>0.2s; settling within10% for at least60ms',
            response_filter='Betaflight-reported gyro; no additional host low-pass filter',
            tuned_terms=['F','P']+(['D'] if tune_d else []),I_yaw_filters_unchanged=True,save_best_after_disarm=save_best,
            relative_gain_bounds={'P':[.8,1.1],'D':[1.,1.1],'F':[.5,1.1]},update_steps={'P':.05,'D':.05,'F':.1},
            quality={'minimum_clean_pulses':3,'both_amplitudes_and_signs':True,'maximum_received_command_age_ms':40.,'maximum_gyro_gap_ms':35.,'pulse_duration_error_ms':45.,'batch_retries':1},
            analysis_process='spawned before arming; CPU-only; current-batch snapshots',deadline_miss_behavior='Cancel probing and land on fresh state/FC; hard stop remains for unavailable state/FC',runtime_budget_s=210.,tuning_profile=tuning_profile,scope='Bounded empirical step-response tuning; not a general stability certificate or high-frequency vibration tune')

        if tuning_profile!='legacy':
            self.info.update(evaluations_per_axis=5,validation_batches_per_axis=2,nominal_pulses=28*len(axes),
                tuned_terms=['D','D_min'] if tuning_profile=='damping' else ['feedforward_boost'],
                comparison_sequence=['baseline','candidate_1','baseline','candidate_2','baseline','validation_1','validation_2'],
                candidate_changes=['D and D-min +5%','D and D-min +10%'] if tuning_profile=='damping' else ['boost -5','boost -10'],
                acceptance='>=5% lower score and >=20% relative overshoot reduction (minimum3 percentage points), repeated twice; late gain0.8..1.2',
                update_steps={'D_and_D_min':.05} if tuning_profile=='damping' else {'feedforward_boost':-5},
                runtime_budget_s=285.,hard_runtime_s=330.)

    def next_trials(self):
        axes=self.axes if self.tuning_profile=='boost' else (self.name,)
        return [trial for axis in axes for trial in self.batch_trials(axis)]

    def batch_trials(self,name):
        values=[sign*amplitude for amplitude in self.step_amplitudes for sign in (1,-1)]
        return [{'delta_thrust_N':0.,'rate_axis':('roll','pitch').index(name)+1,'delta_rate_rad_s':value,'duration_s':self.step_duration} for value in values]

    def attach(self,rpc,output,keepalive):
        self.rpc=rpc;self.output=Path(output);baseline=rpc.snapshot(keepalive)
        if self.tuning_profile=='legacy':
            if self.tune_d and any(baseline.advanced[39+('roll','pitch').index(a)]!=baseline.gain(('roll','pitch').index(a),'D') for a in self.axes):
                raise ValueError('Paired D tuning requires D-min equal to D')
            if self.tune_d:rpc.require_extensions(1,keepalive)
            self.search=RateGainSearch(baseline,self.axes,self.evaluations,self.tune_d)
        else:
            from sanity_zenoh.damping_search import DampingSearch
            rpc.require_extensions(1 if self.tuning_profile=='damping' else 2,keepalive)
            self.search=DampingSearch(baseline,self.axes,self.tuning_profile)
        (self.output/'original_tune.json').write_text(json.dumps(baseline.json(),indent=2)+'\n')
        (self.output/'restore_original.cli').write_text(f'profile {baseline.profile}\n'+baseline.cli())
        rpc.verify_setters(baseline,keepalive)
        from sanity_zenoh.rate_analysis import RateAnalysisWorker
        self.analysis=RateAnalysisWorker()
        self.tuning_receipt={'original':baseline.json(),'best':baseline.json(),'saved_to_EEPROM':False,'original_restored':False,'success':False}

    def observe_event(self,event):self.measurements.observe(event)

    def probe_event(self,event):
        self.measurements.mark(event,self.started)
        event.update(tuning_axis=self.name,tuning_evaluation=self.search.evaluation if self.search else 0,tuning_validation=self.search.validation if self.search else False)

    def run_batch_job(self):
        try:
            metrics=self.analysis.evaluate(self.measurements)
            if any(m.get("valid") and m["peak_gain"]>2. for m in metrics):raise ValueError("Excessive rate response; reverting original gains")
            try:
                batch_score(metrics)
                if self.tuning_profile=='boost':
                    for axis in self.axes:batch_score([m for m in metrics if m.get('rate_axis')==('roll','pitch').index(axis)+1])
            except ValueError:
                if self.batch_retries<1:
                    self.batch_retries+=1;self.results.put({'repeat':True,'metrics':metrics});return
                raise
            self.batch_retries=0;target=self.search.accept(metrics)
            (self.output/'tuning_history.json').write_text(json.dumps(self.search.history,indent=2)+'\n')
            (self.output/'best_tune.json').write_text(json.dumps(self.search.best.json(),indent=2)+'\n')
            (self.output/'best_tune.cli').write_text(f'profile {self.search.best.profile}\n'+self.search.best.cli())
            self.rpc.apply(target if target is not None else self.search.best,self.search.baseline,cancel=self.cancel)
            self.results.put({'complete':target is None,'metrics':metrics,'axis':None if target is None else self.search.axes[self.search.axis_index]})
        except BaseException as error:
            rollback=None
            try:self.rpc.apply(self.search.baseline,self.search.baseline)
            except Exception as caught:rollback=str(caught)
            self.results.put({'error':str(error) or type(error).__name__,'rollback_error':rollback})

    def choose(self,now,state,force_limit,telemetry_ok=True,gyro_rad_s_fc=None):
        if self.search is None:raise ValueError('Tuning snapshot must be acquired before arming')
        if self.started is not None and now-self.last_now>.04000001:self.request_landing('controller_deadline_missed')
        if self.phase=='recovery' and self.trial_index>=len(self.trials):self.phase='tuning_hold';self.phase_started=now;self.stable_since=None
        if self.started is not None and now-self.started>self.info['runtime_budget_s'] and self.landing_requested is None:self.request_landing('autotune_time_budget')
        if self.landing_requested:self.cancel.set()
        try:result=self.results.get_nowait()
        except queue.Empty:result=None
        if result is not None:
            self.worker=None;self.pending_events.append({'kind':'autotune_batch_result','result':result,'best':self.search.best.json()})
            if result.get('error'):
                self.tuning_error=result['error'];self.request_landing('autotune_error')
            elif result.get('complete'):
                self.tuning_complete=True;self.request_landing('autotune_complete')
            elif not self.landing_requested:
                if not result.get('repeat'):self.name=result['axis']
                self.trials.extend(self.next_trials());self.phase='settle';self.phase_started=now;self.stable_since=None;self.baseline_samples=[];self.measurements.clear_marks()
        command,reference,probe=super().choose(now,state,force_limit,telemetry_ok,gyro_rad_s_fc)
        if self.phase=='tuning_hold' and not self.landing_requested:
            rotation=Rotation.from_quat(np.asarray(state)[[4,5,6,3]])
            ready=self.settled(np.asarray(state),rotation,command,telemetry_ok)
            self.stable_since=(now if self.stable_since is None else self.stable_since) if ready else None
            if self.worker is None and self.stable_since is not None and now-self.stable_since>=1. and now-self.phase_started>=2.:
                self.worker=threading.Thread(target=self.run_batch_job,name='sanity_gain_search',daemon=True);self.worker.start()
                self.pending_events.append({'kind':'autotune_evaluation_started','axis':self.name,'validation':self.search.validation,'evaluation':self.search.evaluation})
            if now-self.phase_started>15. and self.worker is None:self.request_landing('autotune_settle_timeout')
        return command,reference,{**probe,'tuning_axis':self.name,'tuning_evaluation':self.search.evaluation,'tuning_validation':self.search.validation}

    def finalize(self,sender,events):
        if self.finalized:return
        self.finalized=True;self.cancel.set()
        if self.worker is not None:
            self.worker.join(timeout=3.)
            if self.worker.is_alive():
                self.rpc.close();
                if self.analysis:self.analysis.close()
                self.worker.join(timeout=.2);self.tuning_receipt['cleanup_error']='Gain worker did not stop; receiver will restore after owner lease loss and confirmed disarm';events({'kind':'autotune_finalization',**self.tuning_receipt});(self.output/'tuning_result.json').write_text(json.dumps(self.tuning_receipt,indent=2)+'\n');return
        def disabled():sender.ctbr(False,0.,(0.,0.,0.),valid_for_s=1.)
        try:
            deadline=time.monotonic()+5.
            while True:
                disabled();cap=self.rpc.call(0,keepalive=disabled)
                if cap[6]:break
                if time.monotonic()>deadline:raise TimeoutError('FC did not confirm disarm for tune finalization')
                time.sleep(.05)
            success=self.tuning_complete and self.done and not self.tuning_error and self.landing and self.landing.reason=='autotune_complete'
            if success and self.save_best:
                self.rpc.apply(self.search.best,self.search.baseline,keepalive=disabled);self.rpc.save(self.search.best,disabled);self.tuning_receipt['saved_to_EEPROM']=True
            else:
                self.rpc.apply(self.search.baseline,self.search.baseline,keepalive=disabled);self.tuning_receipt['original_restored']=True
            self.tuning_receipt.update(success=bool(success),improved=self.search.best!=self.search.baseline,best=self.search.best.json(),error=self.tuning_error,batches=len(self.search.history),tuning_profile=self.tuning_profile,validation_rejections=getattr(self.search,"rejections",[]))
        except Exception as error:self.tuning_receipt['cleanup_error']=str(error)
        finally:
            if self.analysis:self.analysis.close()
            self.rpc.close();events({'kind':'autotune_finalization',**self.tuning_receipt});(self.output/'tuning_result.json').write_text(json.dumps(self.tuning_receipt,indent=2)+'\n')
