"""Timed manual pull/release or known-weight cues; no force measurement."""
import math
from pathlib import Path
import shutil
import struct
import subprocess
import wave


class PullProtocol:
    def __init__(self, cycles=3, pull_s=3.0, release_s=5.0, baseline_s=5.0, countdown_s=5.0,
                 weights_g=None, base_load_g=0.0, require_enter=None):
        if type(cycles) is not int or cycles<1:
            raise ValueError('At least one load/release cycle is required')
        if any(not math.isfinite(v) or v<=0 for v in (pull_s,release_s,baseline_s)) or not math.isfinite(countdown_s) or countdown_s<0:
            raise ValueError('Durations must be positive; the countdown may be zero')
        if not math.isfinite(base_load_g) or base_load_g<0:
            raise ValueError('The constant base load must be nonnegative')
        if weights_g is not None and (not weights_g or any(not math.isfinite(v) or v<=0 for v in weights_g)):
            raise ValueError('Known weights must be positive grams; the released baseline is recorded automatically')
        self.require_enter=bool(weights_g) if require_enter is None else require_enter
        self.pending=None;self.pending_since_s=None;self.setup_wait_s=0.0;self.confirmations=0
        self.weights_g=None if weights_g is None else list(weights_g)
        self.base_load_g=base_load_g
        self.repeats=cycles
        self.cycles=cycles*(len(weights_g) if weights_g else 1)
        self.stages=[];cursor=0.0
        entries=[]
        if countdown_s:
            entries.append(dict(phase='get_ready',cycle=0,repeat=0,duration_s=countdown_s,instruction='GET READY — keep hands off',added_weight_g=0.0 if weights_g else None,removed_weight_g=None))
        entries.append(dict(phase='baseline',cycle=0,repeat=0,duration_s=baseline_s,
            instruction='BASELINE — leave all added weights off' if weights_g else 'BASELINE — hands off; leave the sensor alone',
            added_weight_g=0.0 if weights_g else None,removed_weight_g=None))
        trial=0
        for repeat in range(1,cycles+1):
            for weight in weights_g if weights_g else [None]:
                trial+=1
                instruction=('LOAD %g g — attach this total added mass, including its hanger, then leave it hanging' % weight) if weight is not None else 'PULL GENTLY AND HOLD'
                entries.append(dict(phase='pull',cycle=trial,repeat=repeat,duration_s=pull_s,instruction=instruction,added_weight_g=weight,removed_weight_g=None))
                instruction=('UNLOAD — remove the %g g added mass completely; keep the base load unchanged' % weight) if weight is not None else 'LET GO NOW — fully release the sensor'
                entries.append(dict(phase='release',cycle=trial,repeat=repeat,duration_s=release_s,instruction=instruction,added_weight_g=0.0 if weights_g else None,removed_weight_g=weight))
        for stage in entries:
            stage['offset_s']=cursor
            stage['base_load_g']=base_load_g if weights_g else None
            stage['total_reference_mass_g']=base_load_g+stage['added_weight_g'] if weights_g else None
            stage['reference_force_N']=stage['total_reference_mass_g']/1000*9.80665 if weights_g else None
            self.stages.append(stage);cursor+=stage['duration_s']
        self.duration_s=cursor
        self.origin_s=None;self.next_index=0;self.phase='waiting_for_data';self.cycle=0
        self.complete=False;self.phase_end_s=None;self.current=None

    def start(self, elapsed_s):
        if self.origin_s is None:
            self.origin_s=elapsed_s

    def update(self, elapsed_s):
        if self.origin_s is None or self.complete or self.pending is not None:
            return []
        planned=self.origin_s if self.next_index==0 else self.phase_end_s
        if elapsed_s<planned:
            return []
        if self.next_index<len(self.stages):
            stage=self.stages[self.next_index]
            if self.require_enter and stage['phase'] in ('baseline','pull','release'):
                self.pending=stage;self.pending_since_s=elapsed_s
                setup=dict(stage,phase='setup_'+stage['phase'],duration_s=0.0,
                    added_weight_g=None,total_reference_mass_g=None,reference_force_N=None)
                setup['instruction']=stage['instruction']+' — press Enter once ready and hands off'
                self.phase=setup['phase'];self.cycle=stage['cycle'];self.current=setup;self.phase_end_s=None
                return [dict(setup,elapsed_s=elapsed_s,planned_elapsed_s=planned,
                    late_by_s=max(0.0,elapsed_s-planned),missed=False,requires_enter=True,
                    confirmed=False,confirmation_wait_s=0.0,target_phase=stage['phase'],
                    target_added_weight_g=stage['added_weight_g'])]
            self.phase=stage['phase'];self.cycle=stage['cycle'];self.current=stage
            self.phase_end_s=elapsed_s+stage['duration_s'];self.next_index+=1
            return [dict(stage,elapsed_s=elapsed_s,planned_elapsed_s=planned,
                         late_by_s=max(0.0,elapsed_s-planned),missed=False,requires_enter=False,
                         confirmed=False,confirmation_wait_s=0.0,target_phase=None,target_added_weight_g=None)]
        self.phase='complete';self.complete=True;self.phase_end_s=elapsed_s
        event=dict(phase='complete',cycle=self.cycles,repeat=self.repeats,duration_s=0.0,
            elapsed_s=elapsed_s,planned_elapsed_s=planned,
            late_by_s=max(0.0,elapsed_s-planned),missed=False,requires_enter=False,confirmed=False,confirmation_wait_s=0.0,target_phase=None,target_added_weight_g=None,
            added_weight_g=0.0 if self.weights_g else None,removed_weight_g=None,
            base_load_g=self.base_load_g if self.weights_g else None,
            total_reference_mass_g=self.base_load_g if self.weights_g else None,
            reference_force_N=self.base_load_g/1000*9.80665 if self.weights_g else None,
            instruction='DONE — leave added weights off; saving data and plot' if self.weights_g else 'DONE — hands off; saving data and plot')
        self.current=event
        return [event]


    def confirm(self, elapsed_s):
        if self.pending is None:
            return []
        stage=dict(self.pending)
        waited=max(0.0,elapsed_s-self.pending_since_s)
        self.setup_wait_s+=waited;self.confirmations+=1
        if stage['phase']=='pull':
            stage['instruction']='RECORDING %g g added mass — leave it hanging, hands off' % stage['added_weight_g']
        else:
            stage['instruction']='RECORDING RELEASED BASELINE — leave all added weights off'
        self.pending=None;self.pending_since_s=None
        self.phase=stage['phase'];self.cycle=stage['cycle'];self.current=stage
        self.phase_end_s=elapsed_s+stage['duration_s'];self.next_index+=1
        return [dict(stage,elapsed_s=elapsed_s,planned_elapsed_s=elapsed_s,late_by_s=0.0,
            missed=False,requires_enter=True,confirmed=True,confirmation_wait_s=waited,
            target_phase=stage['phase'],target_added_weight_g=stage['added_weight_g'])]


class CueAudio:
    def __init__(self, directory, enabled=True):
        self.enabled=enabled
        self.player=shutil.which('paplay') or shutil.which('aplay')
        self.processes=[]
        self.failed=False
        self.files={}
        if not enabled:
            return
        directory=Path(directory)/'cue_audio';directory.mkdir()
        for phase,frequency in (('pull',1000),('release',440),('get_ready',660),('baseline',660),('complete',1320)):
            path=directory/(phase+'.wav');rate=16000;count=int(rate*.18)
            values=[]
            for i in range(count):
                fade=min(1.0,i/(rate*.008),(count-1-i)/(rate*.008))
                values.append(int(6000*fade*math.sin(2*math.pi*frequency*i/rate)))
            with wave.open(str(path),'wb') as file:
                file.setnchannels(1);file.setsampwidth(2);file.setframerate(rate)
                file.writeframes(struct.pack('<%dh' % count,*values))
            self.files[phase]=path

    def play(self, phase):
        if not self.enabled:
            return False
        active=[]
        for process in self.processes:
            code=process.poll()
            if code is None:
                active.append(process)
            elif code:
                self.failed=True
        self.processes=active
        if self.player and not self.failed:
            try:
                self.processes.append(subprocess.Popen([self.player,str(self.files[phase])],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL))
                return True
            except OSError:
                self.failed=True
        # A terminal bell is a fallback, not a guarantee that audio is audible.
        print('\a',end='',flush=True)
        return False

    def close(self):
        for process in self.processes:
            try:
                process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                process.terminate()
