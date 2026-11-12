"""Invert the supplied Sanity steady spinning actuator fit; never extrapolate."""

import json
import math
from pathlib import Path


class DirectThrustMapping:
    def __init__(self, model_path, model_to_output):
        data = json.loads(Path(model_path).read_text())
        speed, thrust = data["command_to_speed"], data["thrust"]
        self.coefficients = tuple(tuple(c) for c in speed["coefficients_a_b_c_rad_s"])
        self.commands = tuple(speed["command_range_counts"])
        self.voltages = tuple(speed["voltage_range_V"])
        self.k = thrust["k_N_per_rad_s_squared"]
        self.speed_range = tuple(thrust["measured_speed_range_rad_s"])
        self.order = tuple(model_to_output)
        if data.get("schema_version") != 2 or data["output_order"] != [
            "M1",
            "M2",
            "M3",
            "M4",
        ]:
            raise ValueError(
                "Expected Sanity actuator schema v2 in hardware output order"
            )
        if sorted(self.order) != [1, 2, 3, 4] or any(
            type(i) is not int for i in self.order
        ):
            raise ValueError(
                "model_to_output must explicitly map RR, FR, FL, RL to a permutation of M1..M4"
            )
        if self.commands != (1250, 1450) or not math.isfinite(self.k) or self.k <= 0:
            raise ValueError(
                "Unsupported calibration command range or thrust coefficient"
            )
        if len(self.coefficients) != 4 or any(
            len(c) != 3 or not all(math.isfinite(v) for v in c)
            for c in self.coefficients
        ):
            raise ValueError("Four finite quadratic coefficient triples required")
        if (
            len(self.voltages) != 2
            or not 0 < self.voltages[0] < self.voltages[1]
            or not all(math.isfinite(v) for v in self.voltages)
        ):
            raise ValueError("Invalid voltage envelope")
        for a, b, c in self.coefficients:
            if a - b + c <= 0 or min(b - 2 * c, b + 2 * c) <= 0:
                raise ValueError(
                    "Actuator fit must be positive and increasing over its calibrated interval"
                )

    def convert(self, thrusts, voltage):
        if len(thrusts) != 4 or not all(
            math.isfinite(t) and 0 <= t <= 0.8 + 1e-6 for t in thrusts
        ):
            raise ValueError(
                "Four finite rotor thrusts in 0..0.8 N required, in RR, FR, FL, RL order"
            )
        if not any(thrusts):
            return (1000,) * 4
        if (
            not math.isfinite(voltage)
            or not self.voltages[0] <= voltage <= self.voltages[1]
        ):
            raise ValueError(
                "Fresh battery voltage within the measured calibration envelope is required"
            )
        commands = [1000] * 4
        for thrust, output in zip(thrusts, self.order):
            if thrust == 0:
                continue  # Explicit STOP, not extrapolation of the spinning model.
            target = math.sqrt(thrust / self.k)
            if not self.speed_range[0] <= target <= self.speed_range[1]:
                raise ValueError(
                    "Requested rotor speed lies outside the measured envelope"
                )
            a, b, c = self.coefficients[output - 1]
            target *= 8 / voltage
            lower, upper = a - b + c, a + b + c
            if not lower <= target <= upper:
                # Inverting T=k*omega**2 and voltage scaling can differ by a few
                # floating-point ulps at an exact calibrated endpoint.
                if math.isclose(target, lower, rel_tol=1e-12, abs_tol=1e-9):
                    target = lower
                elif math.isclose(target, upper, rel_tol=1e-12, abs_tol=1e-9):
                    target = upper
                else:
                    raise ValueError(
                        f"M{output}: thrust lies outside the calibrated spinning command range"
                    )
            low, high = -1.0, 1.0
            for _ in range(48):
                q = (low + high) / 2
                if a + b * q + c * q * q < target:
                    low = q
                else:
                    high = q
            commands[output - 1] = round(1350 + 100 * (low + high) / 2)
        return tuple(commands)
