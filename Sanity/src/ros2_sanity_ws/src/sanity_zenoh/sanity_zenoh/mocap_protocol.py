"""Compact calibrated mocap state and conservative monotonic clock bounds.

World: configured Z-up; body: FLU; quaternion: world-from-body WXYZ.
Clock bounds assume nonnegative transport time and <=200 ppm relative drift.
ROS TF source/PC clock alignment remains a separate precondition.
"""
import math
import struct
from dataclasses import dataclass

STATE = struct.Struct('>4sBBBBQIIQQQ10f')  # 88 bytes
CLOCK_REQUEST = struct.Struct('>4sBBHQQ')
CLOCK_REPLY = struct.Struct('>4sBBH5Q')
STATUS = struct.Struct('>4sBBBBQ4I2Q')
MAX_AGE_US = 40000
MAX_CLOCK_WIDTH_US = 20000

def state_packet(device, epoch, session, sequence, source_ros_ns, lower_us, upper_us, state):
    values=tuple(float(v) for v in state)
    if not 1<=device<=255 or not 0<epoch<2**64 or not 0<session<2**32 or not 0<=sequence<2**32:
        raise ValueError('Invalid mocap identity')
    if not 0<source_ros_ns<2**64 or not 0<lower_us<=upper_us<2**64 or upper_us-lower_us>MAX_CLOCK_WIDTH_US:
        raise ValueError('Invalid capture clock bounds')
    if len(values)!=10 or not all(math.isfinite(v) for v in values):raise ValueError('Ten finite p/q/v values required')
    if any(abs(v)>1000 for v in values[:3]) or any(abs(v)>100 for v in values[7:]):raise ValueError('Mocap range exceeded')
    if abs(sum(v*v for v in values[3:7])-1)>.002:raise ValueError('Unit WXYZ quaternion required')
    return STATE.pack(b'SZM1',1,device,1,3,epoch,session,sequence,source_ros_ns,lower_us,upper_us,*values)

def decode_status(payload, device):
    if len(payload)!=STATUS.size:raise ValueError('Wrong mocap status size')
    magic,version,drone,frame,flags,epoch,session,seq,accepted,rejected,received,age=STATUS.unpack(payload)
    if (magic,version,drone,frame)!=(b'SZMS',1,device,1) or flags&~3 or not epoch:raise ValueError('Invalid mocap status')
    return dict(epoch=epoch,session=session,sequence=seq,accepted=accepted,rejected=rejected,
                present=bool(flags&1),usable=bool(flags&2),received_esp_us=received,
                source_age_upper_us=None if age==2**64-1 else age)

@dataclass
class ClockBounds:
    epoch: int
    low: float
    high: float
    host_us: int
    ttl_us: int = 2000000
    drift_ppm: float = 200.

    @classmethod
    def exchange(cls, epoch, host_send, esp_receive, esp_send, host_receive):
        if not epoch or not 0<host_send<=host_receive or not 0<esp_receive<=esp_send:
            raise ValueError('Invalid clock exchange order')
        low,high=esp_send-host_receive,esp_receive-host_send
        if high<low:raise ValueError('Inconsistent clock exchange')
        margin=(host_receive-host_send)*200./1e6+2
        return cls(epoch,low-margin,high+margin,host_receive)

    def capture(self, capture_host_us, now_host_us):
        if not 0<=now_host_us-self.host_us<=self.ttl_us or capture_host_us>now_host_us:
            raise ValueError('Expired clock bound or future capture')
        growth=abs(capture_host_us-self.host_us)*self.drift_ppm/1e6+2
        low=math.floor(capture_host_us+self.low-growth)
        high=math.ceil(capture_host_us+self.high+growth)
        if low<=0 or high-low>MAX_CLOCK_WIDTH_US:raise ValueError('Clock uncertainty too wide')
        return low,high

    def intersect(self, candidate):
        """Combine causal offset bounds, allowing drift between exchanges.

        This does not assume symmetric Wi-Fi delays or subtract a measured
        ping minimum. Both intervals must contain the same clock offset.
        """
        if candidate.epoch != self.epoch:
            return candidate
        elapsed=candidate.host_us-self.host_us
        if elapsed<0:raise ValueError('Reversed clock exchange')
        if elapsed>self.ttl_us:return candidate
        drift=max(self.drift_ppm,candidate.drift_ppm)
        growth=elapsed*drift/1e6+2
        low=max(self.low-growth,candidate.low)
        high=min(self.high+growth,candidate.high)
        if low>high:raise ValueError('Inconsistent clock offset intervals')
        return ClockBounds(self.epoch,low,high,candidate.host_us,
                           min(self.ttl_us,candidate.ttl_us),drift)
