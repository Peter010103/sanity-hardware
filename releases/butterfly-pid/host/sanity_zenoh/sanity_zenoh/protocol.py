"""Sanity Zenoh versioned commands; network byte order, calibrated SI on ESP."""

from dataclasses import dataclass
import struct

CTBR = 1
DIRECT = 2
PAPER_NATIVE = 3
COMMAND = struct.Struct(">4sBBBBQIIIHH4H")
STATUS = struct.Struct(">4sBBBBQIIIIHH")
STATUS_V2 = struct.Struct(">4sBBBBQIIIIHHI4H")
SI_SCALE = 10000
MAX_LEASE_MS = 1000
MAX_LEASE_S = MAX_LEASE_MS / 1000


def neutral(mode):
    if mode not in (CTBR, DIRECT, PAPER_NATIVE):
        raise ValueError("Unknown output mode")
    return (1500, 1500, 1500, 1000) if mode in (CTBR, PAPER_NATIVE) else (1000,) * 4


def command_packet(
    device, mode, enabled, epoch, session, sequence, lease_ms, ttl_ms, values
):
    if mode not in (CTBR,DIRECT):
        raise ValueError("Legacy command requires CTBR or DIRECT mode")
    if not 1 <= device <= 255 or type(enabled) is not bool:
        raise ValueError("Invalid device or enable flag")
    if not 0 < epoch < 2**64 or not 0 < session < 2**32:
        raise ValueError("Invalid epoch or session")
    if (
        not 0 <= sequence < 2**32
        or not 0 <= lease_ms < 2**32
        or not 1 <= ttl_ms <= MAX_LEASE_MS
    ):
        raise ValueError("Invalid sequence or receiver lease")
    stopped = neutral(mode)
    if len(values) != 4 or any(
        type(v) is not int or not 1000 <= v <= 2000 for v in values
    ):
        raise ValueError("Four integer output values in 1000..2000 required")
    if mode == DIRECT and any(v != 1000 and not 1250 <= v <= 1450 for v in values):
        raise ValueError(
            "MSP values must be STOP or within the calibrated 1250..1450 range"
        )
    if not enabled and tuple(values) != stopped:
        raise ValueError("Disabled commands must be neutral")
    return COMMAND.pack(
        b"SZC1",
        1,
        mode,
        device,
        int(enabled),
        epoch,
        session,
        sequence,
        lease_ms,
        ttl_ms,
        0,
        *values,
    )


def ctbr_si_values(armed, thrust, rates, high_rate=False, selected_high_rate=False):
    import math
    if len(rates) != 3 or not all(math.isfinite(v) for v in (thrust,*rates)) or not 0 <= thrust <= 3.2:
        raise ValueError("Finite CTBR thrust in 0..3.2N and three body rates required")
    if any(abs(v)>limit+1e-9 for v,limit in zip(rates,(9.5 if selected_high_rate else 2.3,9.5 if high_rate else 2.3,9.5 if selected_high_rate else 2.6))):
        raise ValueError("Body rate outside policy limits")
    if selected_high_rate and not high_rate:raise ValueError("Selected high rates require the1e-3 wire scale")
    if not armed:
        return (0,0,0,0)
    return tuple(int(math.floor(abs(v)*(1000 if high_rate else SI_SCALE)+.5))*(1 if v>=0 else -1) for v in rates)+(int(math.floor(thrust*SI_SCALE+.5)),)


def si_command_packet(device, enabled, epoch, session, sequence, lease_ms, ttl_ms, values, calibration_id, high_rate=False, controller_selection=False, indi_requested=False):
    if len(values)!=4 or any(type(v) is not int for v in values) or any(abs(v)>limit for v,limit in zip(values[:3],(9500 if controller_selection else 2300,9500,9500 if controller_selection else 2600) if high_rate else (23000,23000,26000))) or not 0<=values[3]<=32000:
        raise ValueError("Invalid fixed-point CTBR values")
    if not enabled and any(values):
        raise ValueError("Disabled physical commands must be zero")
    if type(controller_selection) is not bool or type(indi_requested) is not bool or (indi_requested and (not controller_selection or not enabled)) :
        raise ValueError("Explicit SZC4 selection, normal rate scale and enabled output required for INDI request")
    if not 0 < calibration_id < 2**32:
        raise ValueError("Explicit firmware calibration ID required")
    packet=bytearray(command_packet(device,CTBR,enabled,epoch,session,sequence,lease_ms,ttl_ms,neutral(CTBR)))
    packet[:5]=b"SZC3\x03" if high_rate else b"SZC2\x02"
    if controller_selection:
        packet[:5]=b"SZC6\x06" if high_rate else b"SZC4\x04"
        packet[7]=int(enabled)|(int(indi_requested)<<1)
    struct.pack_into(">H3hH",packet,30,calibration_id & 0xffff,*values)
    return bytes(packet)


@dataclass(frozen=True)
class ReceiverStatus:
    mode: int
    device: int
    flags: int
    epoch: int
    receiver_ms: int
    session: int
    sequence: int
    rejected: int
    voltage_mv: int
    fc_age_ms: int
    calibration_id: int = 0
    output_values: tuple = ()

    @property
    def enabled(self):
        return bool(self.flags & 1)

    @property
    def failsafe(self):
        return bool(self.flags & 2)

    @property
    def voltage_supported(self):
        return not bool(self.flags & 8)

    @property
    def indi_high_rate_supported(self):
        return bool(self.flags & 64)

    @property
    def indi_pitch_loop_supported(self):
        return self.indi_high_rate_supported

    @property
    def pitch_loop_supported(self):
        return bool(self.flags & 16)

    @property
    def tuning_supported(self):
        return bool(self.flags & 32)

    @property
    def fc_ready(self):
        return bool(self.flags & 4)

    @classmethod
    def decode(cls, payload):
        if len(payload) == STATUS.size:
            magic,version,*fields=STATUS.unpack(payload)
            if magic!=b"SZS1" or version!=1:
                raise ValueError("Invalid legacy status header")
        elif len(payload) == STATUS_V2.size:
            magic,version,*fields=STATUS_V2.unpack(payload)
            if magic!=b"SZS2" or version!=2:
                raise ValueError("Invalid physical status header")
            fields=fields[:11]+[tuple(fields[11:])]
        else:
            raise ValueError("Wrong status size")
        if fields[0] not in (CTBR,DIRECT,PAPER_NATIVE) or not fields[1] or fields[2] & ~(7 if version==1 else 127) or not fields[3]:
            raise ValueError("Invalid status header")
        return cls(*fields)


# Frozen PIN1 contract: see outputs/.../PAPER_NATIVE_WIRE_V1.txt.
PAPER_MODEL_ID = 0x50494e31
PAPER_COMMAND = struct.Struct(">4sBBBBQIIIHBBI7f")

@dataclass(frozen=True)
class PaperDemand:
    """Physical body FRD input; companion rates serve guards/PID fallback only."""
    thrust_n: float = 0.
    alpha_rad_s2: tuple = (0., 0., 0.)
    fallback_rate_rad_s: tuple = (0., 0., 0.)

    def validate(self):
        import math
        if len(self.alpha_rad_s2)!=3 or len(self.fallback_rate_rad_s)!=3:
            raise ValueError("Three FRD alpha and companion-rate values required")
        values=(self.thrust_n,*self.alpha_rad_s2,*self.fallback_rate_rad_s)
        if not all(math.isfinite(v) for v in values) or not 0<=self.thrust_n<=4:
            raise ValueError("Finite paper demand and collective in 0..4 N required")
        if any(abs(v)>1000 for v in self.alpha_rad_s2) or any(abs(v)>lim for v,lim in zip(self.fallback_rate_rad_s,(2.3,2.3,2.6))):
            raise ValueError("Paper demand exceeds PIN1 numeric bounds")
        return values


def paper_command_packet(device,enabled,epoch,session,sequence,lease_ms,ttl_ms,demand=PaperDemand(),paper_requested=False,model_id=PAPER_MODEL_ID):
    # Reuse the established identity/lease validation without changing SZC1-4.
    command_packet(device,CTBR,enabled,epoch,session,sequence,lease_ms,ttl_ms,neutral(CTBR))
    if type(paper_requested) is not bool or (paper_requested and not enabled) or model_id!=PAPER_MODEL_ID:
        raise ValueError("Invalid paper request/model")
    values=demand.validate()
    if not enabled and any(values):raise ValueError("Disabled paper demands must be zero")
    return PAPER_COMMAND.pack(b"SZC5",5,PAPER_NATIVE,device,int(enabled)|(int(paper_requested)<<1),epoch,session,sequence,lease_ms,ttl_ms,1,0,model_id,*values)
