"""Motor-data-derived CTBR feedforward, including the non-3D Betaflight path."""

import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from udp_broadcaster.ctbr_protocol import CtbrMapping, RcCommand


class MotorCollective:
    def __init__(self, hardware_root, profile, bypass_voltage_check=False):
        root = Path(hardware_root)
        path = root/"parameters/actuators.json"
        self.data = json.loads(path.read_text())
        self.bypass_voltage_check = bool(bypass_voltage_check)
        self.reported_voltage = None
        self.voltage_source = None
        self.profile = dict(profile)
        if profile.get("rate_full_scale_deg_s")!=[600,600,600]:
            raise ValueError("Expected the user-confirmed600deg/s FC rate scales")
        self.base = CtbrMapping(rate_full_scale_deg_s=(600.,)*3)
        self.max_thrust_newtons = 3.2
        self.model_rate_limits_rad_s = self.base.model_rate_limits_rad_s
        self.coefficients = np.asarray(self.data["command_to_speed"]["coefficients_a_b_c_rad_s"])
        self.voltages = self.data["command_to_speed"]["voltage_range_V"]
        self.k = self.data["thrust"]["k_N_per_rad_s_squared"]
        self.idle = float(profile["dshot_idle_value"])/10000
        self.mincheck = float(profile["min_check"])
        self.sag = float(profile["vbat_sag_compensation"])/100
        self.cells = int(profile["battery_cells"])
        if not 0<=self.idle<.2 or not 1000<=self.mincheck<1200 or not 0<=self.sag<=1 or self.cells!=2:
            raise ValueError("Unsupported FC throttle profile")
        if any(profile.get(k,0)!=0 for k in ("thr_expo","thrust_linear","dyn_idle_min_rpm")) or profile.get("motor_output_limit",100)!=100 or profile.get("throttle_limit_type","OFF")!="OFF":
            raise ValueError("Derived mapping requires linear throttle, no dynamic idle/thrust linearisation/output limit")
        self.commands = np.r_[1100.,1200.,np.arange(1250.,1451.)]
        low = {}
        static = root/"data/processed/actuator/static_points.csv"
        with static.open() as stream:
            for r in csv.DictReader(stream):
                u = int(r["command_M1"])
                if r["source"]=="legacy_sweep" and u in (1100,1200):
                    low[u] = float(r["thrust_N"])*(8/float(r["voltage_V"]))**2
        if sorted(low)!=[1100,1200]:
            raise ValueError("Measured low-throttle points missing")
        q = (self.commands[2:]-1350)/100
        speed = self.coefficients[:,0,None]+self.coefficients[:,1,None]*q+self.coefficients[:,2,None]*q*q
        self.forces_at_8v = np.r_[low[1100],low[1200],self.k*np.sum(speed**2,axis=0)]
        if np.any(np.diff(self.forces_at_8v)<=0):
            raise ValueError("Collective curve is not monotonic")
        self.voltage = None
        self.last = None
        self.provenance = {"kind":"motor-data-derived armed feedforward; not an armed-flight calibration",
            "sources":{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (path,static)},
            "command_range":[1100,1450],"fitted_command_range":[1250,1450],"voltage_range":self.voltages,
            "low_range":"interpolate measured collective1100/1200 points, scaled by V squared",
            "sag_voltage":"MSP displayed voltage used as proxy for internal FC sag-filtered voltage", "fc_profile":profile,"bypass_voltage_check":self.bypass_voltage_check}

    def set_voltage(self, voltage):
        if not math.isfinite(voltage) or voltage<0:
            raise ValueError("Invalid FC voltage")
        self.reported_voltage = voltage
        if self.bypass_voltage_check:
            self.voltage = voltage if voltage>0 else 8.0
            self.voltage_source = "fc_reported_extrapolation_allowed" if voltage>0 else "assumed_8V_receiver_voltage_unavailable"
        else:
            if voltage==0:
                raise ValueError("FC voltage unavailable (receiver reports0V); not a measured empty battery")
            if not self.voltages[0]<=voltage<=self.voltages[1]:
                raise ValueError(f"Battery {voltage:.3f}V outside motor data range {self.voltages[0]:.3f}..{self.voltages[1]:.3f}V")
            self.voltage = voltage
            self.voltage_source = "fc_reported_in_calibration_range"

    def curve(self):
        if self.voltage is None:
            raise ValueError("Fresh FC battery voltage required")
        return self.forces_at_8v*(self.voltage/8)**2

    def attenuation(self):
        full = 4.2
        warning = self.profile.get("vbat_warning_cell_voltage",350)/100
        return self.sag*min(max(self.voltage/self.cells-warning,0.),full-warning)/full

    def limit(self, thrust):
        return min(float(thrust),float(self.curve()[-1]))

    def convert(self, device, armed, thrust, rates):
        rate = self.base.convert(device,armed,0.,rates)
        if not armed or thrust==0:
            return rate
        if not math.isfinite(thrust) or not 0<thrust<=self.curve()[-1]+1e-9:
            raise ValueError("Collective demand outside supplied data; no high-command extrapolation")
        force = self.curve()
        u = float(np.interp(thrust,force,self.commands))
        fraction = (u-1001)/999
        fraction = (fraction-self.idle)/((1-self.idle)*(1-self.attenuation()))
        throttle = round(self.mincheck+(2000-self.mincheck)*max(0.,fraction))
        # Below the measured idle thrust, minimum armed throttle is the only endpoint.
        throttle = min(2000,max(1000,throttle))
        self.last = {"voltage_V":self.voltage,"reported_voltage_V":self.reported_voltage,"voltage_source":self.voltage_source,"bypass_voltage_check":self.bypass_voltage_check,"thrust_N":thrust,"external_motor_counts":u,
            "throttle_rc":throttle,"below_measured_idle":bool(thrust<force[0]),"maximum_collective_N":float(force[-1]),
            "sag_attenuation_estimate":self.attenuation()}
        return RcCommand(device,True,rate.roll,rate.pitch,rate.yaw,throttle)

    def thrust_from_rc(self, throttle):
        fraction = max(0.,(throttle-self.mincheck)/(2000-self.mincheck))
        u = 1001+999*(self.idle+(1-self.idle)*(1-self.attenuation())*fraction)
        return float(np.interp(u,self.commands,self.curve()))
