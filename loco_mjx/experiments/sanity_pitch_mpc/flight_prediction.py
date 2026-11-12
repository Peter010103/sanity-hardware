import hashlib
import json
import math
from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.linalg import expm


class FlightRatePrediction:
    def __init__(self, configuration):
        root=Path(__file__).resolve().parents[3]
        path=Path(configuration.get('flight_rate_profile',root/'loco_mjx/loco_mjx/environments/robots/sanity_ctbr/data/flight_rate_20261022.json'))
        if not path.is_absolute():path=root/path
        profile=json.loads(path.read_text())
        if profile.get('kind')!='Sanity_CTBR_flight_rate_response':raise ValueError('Expected identified Sanity flight-rate profile')
        self.timestamp=configuration.get('rate_prediction_timestamp','query_proxy')
        if self.timestamp not in ('query_proxy','response'):raise ValueError('Unknown flight-rate timestamp hypothesis')
        self.filter_tau=float(configuration.get('angular_rate_filter_tau_s',.05))
        if not math.isfinite(self.filter_tau) or self.filter_tau<=0:raise ValueError('Positive angular-rate measurement filter time required')
        self.coefficients=[];matrices=[];inputs=[];outputs=[]
        for axis in ('roll','pitch'):
            row=profile['axes'][axis]
            values=([row[k] for k in ('gain','natural_frequency_hz','damping_ratio','zero_time_s','internal_delay_s')]
                if self.timestamp=='query_proxy' else row['response_timestamp_sensitivity_parameters'])
            gain,frequency,damping,zero,delay=map(float,values)
            if not np.isfinite(values).all() or min(gain,frequency,damping,zero,delay)<=0:raise ValueError('Finite positive flight-rate coefficients required')
            wn=2*np.pi*frequency;delay_tau=delay/3
            a=np.zeros((6,6));b=np.zeros(6);c=np.array([gain,gain*zero,0.,0.,0.,0.])
            a[0,1]=1.;a[1,:2]=[-wn**2,-2*damping*wn];a[1,4]=wn**2
            a[2,2]=-1/delay_tau;b[2]=1/delay_tau
            a[3,2:4]=[1/delay_tau,-1/delay_tau];a[4,3:5]=[1/delay_tau,-1/delay_tau]
            a[5]=c/self.filter_tau;a[5,5]=-1/self.filter_tau
            matrices.append(a);inputs.append(b);outputs.append(c)
            self.coefficients.append((gain,wn,damping,zero,delay_tau))
        self.a=np.array(matrices);self.b=np.array(inputs);self.c=np.array(outputs)
        self.state=None;self.covariance=None;self.time=None
        self.info={'kind':'identified_second_order_roll_pitch','profile':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'timestamp_hypothesis':self.timestamp,'scope':profile['scope'],
            'internal_delay':'Three equal first-order stages per axis approximate only fitted internal delay; host output delay is separate',
            'angular_measurement':'Causal linear observer includes the configured mocap angular-rate low-pass; no derivative-based angular-rate extrapolation',
            'angular_rate_filter_tau_s':self.filter_tau,'observer_noise_scope':'Engineering covariance assumptions, not identified sensor noise',
            'yaw':'Retained assumed first-order yaw response','collective':'Retained bench-derived collective lag',
            'command_output_filter':None}


    @lru_cache(maxsize=512)
    def transition(self, duration_ticks):
        dt=duration_ticks*.0001
        augmented=np.zeros((2,7,7));augmented[:,:6,:6]=self.a;augmented[:,:6,6]=self.b
        values=np.stack([expm(a*dt) for a in augmented])
        return values[:,:6,:6],values[:,:6,6]


    def observe(self, sampled, measured_rates, history, output_delay):
        measured=np.asarray(measured_rates,dtype=float)[:2]
        if measured.shape!=(2,) or not np.isfinite(measured).all():raise ValueError('Finite measured roll/pitch rates required')
        if self.time is None:
            self.state=np.zeros((2,6));self.state[:,0]=measured/self.c[:,0];self.state[:,5]=measured
            self.covariance=np.broadcast_to(np.diag([.01,25.,.04,.04,.04,.0064]),(2,6,6)).copy();self.time=sampled
        if sampled<self.time-1e-6:raise ValueError('Flight-rate observer timestamp moved backwards')
        if sampled>self.time+1e-7:
            times=np.array([r[0]+output_delay for r in history]);cursor=self.time
            changes=times[(times>cursor+1e-8)&(times<sampled-1e-8)]
            for end in [*changes,sampled]:
                index=max(0,int(np.searchsorted(times,(cursor+end)/2,side='right')-1));command=np.asarray(history[index][2])[1:3]
                dt=end-cursor;ad,bd=self.transition(max(1,round(dt/.0001)))
                self.state=np.einsum('aij,aj->ai',ad,self.state)+bd*command[:,None]
                q=np.diag([.000004,.25,.0001,.0001,.0001,.000016])*dt/.02
                self.covariance=ad@self.covariance@np.swapaxes(ad,1,2)+q
                cursor=end
            innovation=measured-self.state[:,5];variance=self.covariance[:,5,5]+.08**2
            gain=self.covariance[:,:,5]/variance[:,None]
            self.state+=gain*innovation[:,None]
            self.covariance-=gain[:,:,None]*self.covariance[:,None,5,:]
            self.covariance=(self.covariance+np.swapaxes(self.covariance,1,2))/2
            self.time=sampled
        return {'state':self.state[:,:5].tolist(),'rate_rad_s':np.einsum('ai,ai->a',self.c,self.state).tolist(),
            'filtered_prediction_rad_s':self.state[:,5].tolist(),'sample_time_s':sampled,'profile_sha256':self.info['sha256']}
