"""CTBR v1 transport and physical-command conversion, independent of ROS."""

from dataclasses import dataclass
import math
import struct

HEADER = struct.Struct(">4sBBHII")
RECORD = struct.Struct(">BBHHHH")
MAX_AGENTS = 128


@dataclass(frozen=True)
class RcCommand:
    drone_id: int
    armed: bool = False
    roll: int = 1500
    pitch: int = 1500
    yaw: int = 1500
    throttle: int = 1000


def construct_ctbr_packet(commands, session, sequence):
    if not 1 <= len(commands) <= MAX_AGENTS:
        raise ValueError("CTBR requires 1..128 commands")
    if not 1 <= session <= 0xffffffff or not 0 <= sequence <= 0xffffffff:
        raise ValueError("Invalid session or sequence")
    seen = set()
    records = []
    for command in commands:
        if not 1 <= command.drone_id <= 255 or command.drone_id in seen:
            raise ValueError("Drone IDs must be unique and in 1..255")
        seen.add(command.drone_id)
        values = (command.roll, command.pitch, command.yaw, command.throttle)
        if any(type(v) is not int or not 1000 <= v <= 2000 for v in values):
            raise ValueError("RC channels must be integers in 1000..2000")
        if not command.armed and values != (1500, 1500, 1500, 1000):
            raise ValueError("Disarmed CTBR commands must be neutral with minimum throttle")
        records.append(RECORD.pack(command.drone_id, int(command.armed), *values))
    return HEADER.pack(b"SCTB", 1, len(commands), 0, session, sequence) + b"".join(records)


@dataclass(frozen=True)
class CtbrMapping:
    rate_full_scale_deg_s: tuple = (600.0, 600.0, 600.0)
    rate_signs: tuple = (1.0, 1.0, -1.0)
    model_rate_limits_rad_s: tuple = (2.3, 2.3, 2.6)
    max_thrust_newtons: float = 3.2
    thrust_newtons: tuple = ()
    throttle_rc: tuple = ()

    def __post_init__(self):
        for values in (self.rate_full_scale_deg_s, self.rate_signs, self.model_rate_limits_rad_s):
            if len(values) != 3 or not all(math.isfinite(v) for v in values):
                raise ValueError("Rate settings require three finite values")
        if any(v <= 0 for v in self.rate_full_scale_deg_s + self.model_rate_limits_rad_s):
            raise ValueError("Rate limits must be positive")
        if any(v not in (-1.0, 1.0) for v in self.rate_signs):
            raise ValueError("Rate signs must be +1 or -1")
        if any(math.degrees(limit) > scale for limit, scale in zip(self.model_rate_limits_rad_s, self.rate_full_scale_deg_s)):
            raise ValueError("FC full-scale rates must cover the model's rate limits")
        if not math.isfinite(self.max_thrust_newtons) or self.max_thrust_newtons <= 0:
            raise ValueError("Maximum thrust must be finite and positive")
        if len(self.thrust_newtons) != len(self.throttle_rc):
            raise ValueError("Thrust and throttle tables must have equal length")
        if self.thrust_newtons:
            if not all(math.isfinite(v) for v in self.thrust_newtons + self.throttle_rc):
                raise ValueError("Thrust table requires finite points")
            if self.thrust_newtons[0] != 0 or self.throttle_rc[0] != 1000:
                raise ValueError("Thrust table must start at (0 N, 1000 RC)")
            if any(b <= a for values in (self.thrust_newtons, self.throttle_rc) for a, b in zip(values, values[1:])):
                raise ValueError("Thrust and throttle tables must be strictly increasing")
            if any(not 1000 <= v <= 2000 for v in self.throttle_rc):
                raise ValueError("Throttle table must remain in 1000..2000")

    def convert(self, drone_id, armed, thrust, rates):
        if len(rates) != 3 or not all(math.isfinite(v) for v in (thrust, *rates)):
            raise ValueError("CTBR commands must be finite and have three body rates")
        if not armed:
            return RcCommand(drone_id)
        if not -1e-6 <= thrust <= self.max_thrust_newtons + 1e-6:
            raise ValueError("Collective thrust exceeds the model limits")
        if any(abs(v) > limit + 1e-6 for v, limit in zip(rates, self.model_rate_limits_rad_s)):
            raise ValueError("Body rate exceeds the model limits")
        thrust = min(self.max_thrust_newtons, max(0.0, thrust))
        throttle = 1000
        if thrust > 0:
            if not self.thrust_newtons or thrust > self.thrust_newtons[-1]:
                raise ValueError("Positive thrust requires a measured RC-to-collective-thrust table covering the command")
            for index in range(1, len(self.thrust_newtons)):
                if thrust <= self.thrust_newtons[index]:
                    low, high = self.thrust_newtons[index - 1:index + 1]
                    fraction = (thrust - low) / (high - low)
                    throttle = round(self.throttle_rc[index - 1] + fraction * (self.throttle_rc[index] - self.throttle_rc[index - 1]))
                    break
        channels = tuple(round(1500 + 500 * sign * math.degrees(rate) / scale)
                         for rate, sign, scale in zip(rates, self.rate_signs, self.rate_full_scale_deg_s))
        return RcCommand(drone_id, True, *channels, throttle)


class CommandBuffer:
    def __init__(self, drone_ids, timeout_s):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("Command timeout must be positive and finite")
        self.timeout_s = timeout_s
        self.commands = {i: RcCommand(i) for i in drone_ids}
        self.updated = {i: -math.inf for i in drone_ids}
        self.rearm_required = {i: True for i in drone_ids}

    def update(self, command, now):
        i = command.drone_id
        if i not in self.commands:
            raise ValueError("Unknown drone ID")
        if now - self.updated[i] > self.timeout_s:
            self.rearm_required[i] = True
        if not command.armed:
            self.rearm_required[i] = False
        if command.armed and (self.rearm_required[i] or
                             (not self.commands[i].armed and command.throttle != 1000)):
            self.commands[i] = RcCommand(i)
            return False
        self.commands[i] = command
        self.updated[i] = now
        return True

    def invalidate(self, drone_id):
        self.commands[drone_id] = RcCommand(drone_id)
        self.rearm_required[drone_id] = True
        self.updated[drone_id] = -math.inf

    def snapshot(self, now):
        for i in self.commands:
            if now - self.updated[i] > self.timeout_s:
                self.invalidate(i)
        return list(self.commands.values())
