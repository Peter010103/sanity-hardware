import hashlib
import json
import math
from pathlib import Path

import numpy as np


class AcroBenchMapping:
    def __init__(self, profile_path, *, source_voltage_V, voltage_definition):
        path = Path(profile_path)
        self.sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        self.profile = json.loads(path.read_text())
        if voltage_definition != "PSU_source_V" or self.profile["voltage_definition"] != voltage_definition:
            raise ValueError("Acro bench mapping requires known PSU source voltage; FC/battery bus is not interchangeable")
        domains = {r["source_V"]: r for r in self.profile["fit"]["domain_by_source_V"]}
        if source_voltage_V not in domains:
            raise ValueError("Source voltage outside measured conditions")
        table = self.profile["deployment_inverse"]
        self.thrust = np.asarray(table["thrust_N"])
        self.rc = np.asarray(table["rc"])
        if np.any(np.diff(self.thrust) <= 0) or np.any(np.diff(self.rc) <= 0):
            raise ValueError("Inverse table must be monotonic")
        self.rc_min = domains[source_voltage_V]["x_min"]
        self.rc_max = min(domains[source_voltage_V]["x_max"], self.rc[-1])


    def throttle_for_thrust(self, thrust_N):
        if not math.isfinite(thrust_N) or not 0 <= thrust_N <= 3.2:
            raise ValueError("Expected total-aircraft thrust command in newtons within the existing policy contract")
        return int(np.clip(np.floor(np.interp(thrust_N, self.thrust, self.rc) + .5), self.rc_min, self.rc_max))


    def rates_to_rc(self, rates_rad_s):
        rates = np.asarray(rates_rad_s)
        if rates.shape != (3,) or not np.isfinite(rates).all() or np.any(np.abs(rates) > np.array([2.3, 2.3, 2.6]) + 1e-6):
            raise ValueError("Expected body rates within +/-2.3/2.3/2.6 rad/s")
        return tuple(int(round(1500 + 500 * sign * math.degrees(rate) / 600)) for sign, rate in zip((1, 1, -1), rates))
