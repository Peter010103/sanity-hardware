"""Fixed-gain small rate injections on top of the existing PD hover controller."""
import math
import numpy as np
from sanity_zenoh.identification_mission import IdentificationMission


def sine_sample(elapsed, frequency, amplitude, cycles=5):
    """Derivative of a windowed cosine angle: zero rate and angle at both ends.

    One raised-cosine cycle ramps in/out; three middle cycles have pure sine
    rate. The angle construction avoids a DC angle offset from integrating a
    sine rate started at zero. Units: seconds, Hz, rad/s -> rad/s and rad.
    """
    period=1./frequency;duration=cycles*period
    if elapsed<=0. or elapsed>=duration:return 0.,0.,0.
    if elapsed<period:
        u=elapsed/period;window=.5*(1.-math.cos(math.pi*u));derivative=.5*math.pi/period*math.sin(math.pi*u)
    elif elapsed>duration-period:
        u=(duration-elapsed)/period;window=.5*(1.-math.cos(math.pi*u));derivative=-.5*math.pi/period*math.sin(math.pi*u)
    else:window=1.;derivative=0.
    omega=2.*math.pi*frequency;phase=omega*elapsed
    angle=amplitude/omega*window*math.cos(phase)
    rate=amplitude*(derivative/omega*math.cos(phase)-window*math.sin(phase))
    return rate,angle,window


class SineIdentificationMission(IdentificationMission):
    def __init__(self,workspace,height=1.,mass=.06985,axes=('roll','pitch'),frequencies=(.5,1.,2.),amplitude=.3,repeats=2,floor_z=None):
        super().__init__(workspace,'hover',height=height,mass=mass,floor_z=floor_z)
        axes=tuple(axes);frequencies=tuple(float(v) for v in frequencies)
        if not axes or len(set(axes))!=len(axes) or any(a not in ('roll','pitch') for a in axes):raise ValueError('Distinct roll/pitch axes required')
        if not 1<=len(frequencies)<=3 or len(set(frequencies))!=len(frequencies) or not all(math.isfinite(f) and .5<=f<=4. for f in frequencies):raise ValueError('One to three distinct sine frequencies in0.5..4Hz required')
        if not math.isfinite(amplitude) or not .1<=amplitude<=.3 or type(repeats) is not int or not 1<=repeats<=2:raise ValueError('Sine amplitude0.1..0.3rad/s and one or two repeats required')
        self.name='rate_sine';self.sine_details=None;self.trials=[]
        for repeat in range(repeats):
            for axis in axes:
                for frequency in sorted(frequencies):
                    self.trials.append(dict(delta_thrust_N=0.,rate_axis=('roll','pitch').index(axis)+1,delta_rate_rad_s=0.,
                        duration_s=5./frequency,waveform='windowed_angle_derivative',frequency_hz=frequency,
                        amplitude_rad_s=float(amplitude),cycles=5,ramp_cycles=1,steady_cycles=3,repeat=repeat+1,axis=axis))
        self.info.update(mission=self.name,controller='PD hover plus additive tested-axis rate injection; no policy/MPC',
            pulse_trials=self.trials,axes=list(axes),frequencies_hz=sorted(frequencies),steady_rate_amplitude_rad_s=float(amplitude),
            repeats=repeats,nominal_blocks=len(self.trials),excitation_duration_s=sum(t['duration_s'] for t in self.trials),
            gains='Read-only PID/advanced snapshot before arming; no setters or EEPROM writes',
            waveform='angle=A/(2*pi*f)*w(t)*cos(2*pi*f*t); injected rate is its analytic derivative; one-cycle ramps',
            analysis='Three middle cycles; actual UART-derived total command versus FC gyro, including live PD contributions',
            nominal_peak_injected_angle_deg=math.degrees(amplitude/(2*math.pi*min(frequencies))),
            injection_rate_bound_rad_s=1.25*float(amplitude),
            response_filter='FC-filtered gyro; no extra host response low-pass',
            PD_during_excitation='Live position/velocity/yaw and collective feedback; integral correction frozen during each block',
            runtime_budget_s=180.,hard_runtime_s=240.)

    def excitation_command(self,command,trial,elapsed):
        rate,angle,window=sine_sample(elapsed,trial['frequency_hz'],trial['amplitude_rad_s'],trial['cycles'])
        result=command.copy();result[trial['rate_axis']]+=rate
        self.sine_details=dict(injected_rate_rad_s=rate,nominal_injected_angle_rad=angle,window=window,
            frequency_hz=trial['frequency_hz'],axis=trial['axis'],repeat=trial['repeat'],elapsed_in_block_s=elapsed,
            steady_cycle=1./trial['frequency_hz']<=elapsed<4./trial['frequency_hz'])
        return result

    def choose(self,now,state,force_limit,telemetry_ok=True,gyro_rad_s_fc=None):
        self.sine_details=None
        if self.phase=='pulse' and (not telemetry_ok or gyro_rad_s_fc is None):self.request_landing('sine_telemetry_lost')
        if self.started is not None and now-self.started>self.info['runtime_budget_s']:self.request_landing('sine_time_budget')
        command,reference,probe=super().choose(now,state,force_limit,telemetry_ok,gyro_rad_s_fc)
        if probe['pulse_active']:probe['rate_sine']=self.sine_details
        return command,reference,probe
