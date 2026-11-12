import hashlib
import json
import math
from pathlib import Path

import numpy as np
from udp_broadcaster.ctbr_protocol import CtbrMapping, RcCommand
from sanity_zenoh.protocol import ctbr_si_values


class FirmwareCollective:
    def __init__(self, path, high_rate=False, selected_high_rate=False):
        self.profile=json.loads(Path(path).read_text())
        payload={k:v for k,v in self.profile.items() if k not in ('calibration_id','calibration_sha256')}
        digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        if digest!=self.profile['calibration_sha256'] or int(digest[:8],16)!=self.profile['calibration_id']:
            raise ValueError('ESP collective profile content/hash mismatch')
        if self.profile['kind']!='esp_ctbr_acro100_FC_voltage_RPM_inverse':
            raise ValueError('Expected voltage-aware RPM calibration')
        self.calibration_id=self.profile['calibration_id']
        self.coefficients=np.asarray(self.profile['speed_model']['coefficients_rad_s'])
        self.hull=np.asarray(self.profile['convex_hull_rc_FC_voltage'])
        self.k=self.profile['effective_armed_kT_N_per_rad_s_squared']
        self.high_rate=bool(high_rate)
        self.selected_high_rate=bool(selected_high_rate)
        if self.selected_high_rate and not self.high_rate:raise ValueError("Selected high rate requires SZC6 1e-3 scale")
        self.base=CtbrMapping(rate_full_scale_deg_s=(600.,)*3,model_rate_limits_rad_s=(9.5 if selected_high_rate else 2.3,9.5 if high_rate else 2.3,9.5 if selected_high_rate else 2.6))
        self.model_rate_limits_rad_s=self.base.model_rate_limits_rad_s
        self.max_thrust_newtons=3.2
        self.voltage=None
        self.last=None
        self.provenance={'kind':'ESP-side RPM/FC-voltage inverse',**self.profile,'wire_version':6 if self.selected_high_rate else 3 if self.high_rate else 2,'model_rate_limits_rad_s':list(self.model_rate_limits_rad_s)}

    def set_voltage(self, voltage):
        if not math.isfinite(voltage) or voltage<0:
            raise ValueError('Invalid FC voltage telemetry')
        self.voltage=voltage

    def curve(self):
        reported=self.voltage
        fallback=reported is None or reported==0
        supplied=self.profile['voltage_policy']['missing_voltage_V'] if fallback else reported
        minimum,maximum=self.profile['voltage_range_V']
        v=float(np.clip(supplied,math.ceil(minimum*1000)/1000,math.floor(maximum*1000)/1000))
        self.model_voltage=v
        self.voltage_fallback=fallback
        self.voltage_clamped=v!=supplied
        low,high=self.profile['positive_rc_range']
        rc=np.arange(low,high+1)
        q=(rc-1500)/400;z=(v-7.4)/.8;c=self.coefficients
        speed=c[0]+c[1]*q+c[2]*z+c[3]*q*q+c[4]*q*z+c[5]*z*z
        force=4*self.k*speed**2
        return rc,force

    def limit(self, thrust):
        return min(float(thrust),float(self.curve()[1][-1]))

    def convert(self, device, armed, thrust, rates):
        values=ctbr_si_values(armed,thrust,rates,self.high_rate,self.selected_high_rate)
        mapped=self.base.convert(device,armed,0.,tuple(v/(1000 if self.high_rate else 10000) for v in values[:3]))
        quantized=values[3]/10000
        throttle=1000;low=high=False
        if armed and values[3]:
            rc,force=self.curve()
            throttle=int(np.floor(np.interp(quantized,force,rc)+.5))
            low=bool(quantized<force[0]);high=bool(quantized>force[-1])
        self.last={'requested_thrust_N':thrust,'wire_thrust_N':quantized,'expected_throttle_rc':throttle,
                   'reported_FC_voltage_V':self.voltage,'model_voltage_V':getattr(self,'model_voltage',None),
                   'voltage_fallback':getattr(self,'voltage_fallback',False),'voltage_clamped':getattr(self,'voltage_clamped',False),
                   'voltage_use':'bounded RPM model input,8V if missing; no range interlock or additional sag compensation',
                   'target_equivalent_speed_rad_s':math.sqrt(quantized/(4*self.k)) if quantized else 0.,
                   'calibration_id':self.calibration_id,'saturated_low':low,'saturated_high':high}
        return RcCommand(device,bool(armed),mapped.roll,mapped.pitch,mapped.yaw,throttle)
