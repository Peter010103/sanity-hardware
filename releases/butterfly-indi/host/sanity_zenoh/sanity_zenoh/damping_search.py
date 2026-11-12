"""Isolated damping or boost comparisons with interleaved baselines and repeat validation."""
import math
import numpy as np
from sanity_zenoh.rate_autotune import batch_score


class DampingSearch:
    def __init__(self,baseline,axes=('roll','pitch'),profile='damping'):
        if profile not in ('damping','boost') or not axes or len(set(axes))!=len(axes):
            raise ValueError('Distinct axes and damping/boost profile required')
        if any(a not in ('roll','pitch') for a in axes):raise ValueError('Only roll/pitch supported')
        if profile=='damping' and any(baseline.advanced[39+('roll','pitch').index(a)]!=baseline.gain(('roll','pitch').index(a),'D') for a in axes):
            raise ValueError('Damping comparisons require constant D: D-min equals D at entry')
        if profile=='boost' and baseline.advanced[52]<5:raise ValueError('Boost already below5; no bounded decrease candidates')
        self.baseline=baseline;self.best=baseline;self.current=baseline;self.axes=tuple(axes);self.profile=profile
        self.axis_index=0;self.evaluation=0;self.validation=False;self.complete=False;self.history=[];self.rejections=[]
        self.axis_baseline=baseline;self.selected=None;self.references={};self.validation_entries=[]

    @property
    def axis(self):return ('roll','pitch').index(self.axes[self.axis_index])

    @property
    def active_axes(self):return self.axes if self.profile=='boost' else (self.axes[self.axis_index],)

    def improving(self,candidate,references):
        for axis in self.active_axes:
            current=candidate['axis_metrics'][axis];reference=references[axis]
            excess=max(0.,reference['median_peak_gain']-1.)
            required=max(.03,.2*excess)
            if not (current['score']<=reference['score']*.95 and
                    current['median_peak_gain']<=reference['median_peak_gain']-required and
                    .8<=current['median_late_gain']<=1.2):return False
        return True

    def accept(self,metrics):
        # Keep the legacy rising-edge score in raw metrics; this profile also
        # penalizes gyro/setpoint mismatch during PD-controlled recovery.
        metrics=[{**m,'score':m['score']+.25*m.get('recovery_score',0.)} if m.get('valid') else m for m in metrics]
        per_axis={name:batch_score([m for m in metrics if m.get('rate_axis',('roll','pitch').index(name)+1)==('roll','pitch').index(name)+1]) for name in self.active_axes}
        score=batch_score(metrics)
        phase=('baseline' if self.evaluation in (0,2,4) else 'candidate' if self.evaluation in (1,3) else 'validation')
        entry={'axis':self.axes[self.axis_index] if self.profile=='damping' else 'roll_pitch','axes':list(self.active_axes),
               'profile':self.profile,'evaluation':self.evaluation,'validation':phase=='validation','phase':phase,
               'gains':self.current.json(),'metrics':metrics,'axis_metrics':per_axis,**score,'accepted':False}
        self.history.append(entry)
        if self.evaluation==4:
            local=self.history[-5:];reference_rows=[x for x in local if x['phase']=='baseline']
            self.references={name:{key:float(np.median([x['axis_metrics'][name][key] for x in reference_rows]))
                for key in ('score','median_peak_gain','median_late_gain')} for name in self.active_axes}
            candidates=[x for x in local if x['phase']=='candidate' and self.improving(x,self.references)]
            self.selected=min(candidates,key=lambda x:x['score']) if candidates else None
            if self.selected:
                self.selected['accepted']=True
                from sanity_zenoh.tuning_rpc import TuneSnapshot
                value=self.selected['gains'];self.current=TuneSnapshot(bytes.fromhex(value['pid_hex']),bytes.fromhex(value['advanced_hex']),value['profile'])
            else:self.current=self.axis_baseline
            self.validation_entries=[]
        elif self.evaluation in (5,6):
            self.validation_entries.append(entry)
            if self.evaluation==6:
                if self.selected and all(self.improving(x,self.references) for x in self.validation_entries):
                    self.best=self.current
                    for x in self.validation_entries:x['accepted']=True
                else:
                    if self.selected:
                        self.selected['accepted']=False;self.rejections.append({'axes':list(self.active_axes),'reason':'Overshoot/score improvement did not repeat in both validation batches'})
                    self.best=self.axis_baseline
                if self.profile=='boost' or self.axis_index+1==len(self.axes):
                    self.current=self.best;self.complete=True;return None
                self.axis_index+=1;self.evaluation=0;self.validation=False;self.axis_baseline=self.best;self.current=self.best;return self.current
        self.evaluation+=1;self.validation=self.evaluation>=5
        if self.evaluation in (1,3):
            if self.profile=='damping':
                original=self.axis_baseline.gain(self.axis,'D');fraction=.05 if self.evaluation==1 else .1
                value=min(math.floor(original*1.1),original+max(1,round(original*fraction)))
                self.current=self.axis_baseline.with_damping(self.axis,value)
            else:
                decrease=5 if self.evaluation==1 else 10
                self.current=self.axis_baseline.with_boost(max(0,self.axis_baseline.advanced[52]-decrease))
        elif self.evaluation in (2,4):self.current=self.axis_baseline
        return self.current
