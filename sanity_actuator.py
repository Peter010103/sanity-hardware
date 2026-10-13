#!/usr/bin/env python3
"""Offline Sanity actuator model. Standard library only; no hardware I/O.

SpinningModel applies the empirically observed interface dynamics to already
spinning rotors. It does not implement ESC startup, arming, spin-down, battery
dynamics, yaw reaction torque, or flight-controller attitude control.
"""
import json
import math
from pathlib import Path


class SanityModel:
    def __init__(self, filename=None):
        filename = filename or Path(__file__).with_name("parameters") / "actuators.json"
        self.data = json.loads(Path(filename).read_text())
        self.speed = self.data["command_to_speed"]
        self.force = self.data["thrust"]
        self.L = self.data["response"]["delay_s"]
        self.tau = self.data["response"]["tau_s"]

    @staticmethod
    def vector(values):
        a=tuple(float(x) for x in values)
        if len(a)!=4 or not all(math.isfinite(x) for x in a):
            raise ValueError("Four finite values in hardware output order M1..M4 required")
        return a

    def steady_omega(self,commands,voltage_V):
        commands=self.vector(commands)
        low, high = self.speed["command_range_counts"]
        if any(not low<=u<=high for u in commands):
            raise ValueError("Spinning command map valid only for 1250..1450 counts; startup/STOP not modeled")
        lo,hi=self.speed["voltage_range_V"]
        if not math.isfinite(voltage_V) or not lo<=voltage_V<=hi:
            raise ValueError("Voltage outside measured plateau envelope %.3f..%.3f V"%(lo,hi))
        coef=self.speed["coefficients_a_b_c_rad_s"]
        return tuple(voltage_V/8*(a+b*((u-1350)/100)+c*((u-1350)/100)**2) for u,(a,b,c) in zip(commands,coef))

    def thrusts(self,omega_rad_s):
        omega=self.vector(omega_rad_s)
        bounds=self.force["measured_speed_range_rad_s"]
        if any(w<0 or w>bounds[1] or (0<w<bounds[0]) for w in omega):
            raise ValueError("Nonzero rotor speed outside measured envelope")
        return tuple(self.force["k_N_per_rad_s_squared"] * w*w for w in omega)


class SpinningModel:
    def __init__(self, model, commands, voltage_V, initial_omega_rad_s=None):
        self.model=model
        self.current=model.vector(commands)
        target=model.steady_omega(self.current,voltage_V)
        self.omega=model.vector(initial_omega_rad_s) if initial_omega_rad_s is not None else target
        if any(w<=0 for w in self.omega):
            raise ValueError("Initialize already-spinning rotors; ESC startup not modeled")
        self.time=0.
        self.events=[]
        self.last_requested=self.current

    def step(self, dt_s, commands, voltage_V):
        """Command is applied at the start of this step; voltage held for the step.

        Exact first-order integration, splitting at every delayed command event.
        Returned speed is rad/s. The fitted delay includes telemetry effects.
        """
        if not math.isfinite(dt_s) or dt_s<=0:
            raise ValueError("dt_s must be finite and positive")
        command=self.model.vector(commands)
        self.model.steady_omega(command,voltage_V)
        if command!=self.last_requested:
            self.events.append((self.time+self.model.L,command))
            self.last_requested=command
        end=self.time+dt_s
        while self.events and self.events[0][0]<=end:
            event,active=self.events.pop(0)
            self._advance(max(0.,event-self.time),voltage_V)
            self.current=active
        self._advance(end-self.time,voltage_V)
        return self.omega

    def _advance(self, dt_s, voltage_V):
        target=self.model.steady_omega(self.current,voltage_V)
        decay=math.exp(-dt_s/self.model.tau)
        self.omega=tuple(f+(w-f)*decay for w,f in zip(self.omega,target))
        self.time+=dt_s


if __name__=="__main__":
    model=SanityModel()
    speed=model.steady_omega([1350]*4,7.8)
    print("omega rad/s:",speed)
    print("estimated rotor thrust N:",model.thrusts(speed))
    print("estimated collective thrust N:",sum(model.thrusts(speed)))
