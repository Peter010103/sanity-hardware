"""PIN1 capability and FC application-consumption snapshot, separate from UART ACK."""
import math
import struct
from sanity_zenoh.controller_status import crc8
from sanity_zenoh.protocol import PAPER_MODEL_ID
COMMAND=0x7f16

def decode_wrapped(payload):
    if len(payload)!=86:raise ValueError("Paper status wrapped length")
    flags,command,size=struct.unpack_from('<BHH',payload)
    if flags or command!=COMMAND or size!=80 or crc8(payload[:-1])!=payload[-1]:raise ValueError("Paper status header/CRC")
    return decode(payload[5:-1])

def decode(p):
    if len(p)!=80 or p[:4]!=b'SIPS' or p[4] not in (1,2) or p[5]>1 or p[6]!=1 or p[7]&128 or p[55]:raise ValueError("Paper status schema")
    if p[4]==1 and any(p[76:]):raise ValueError("Paper status v1 reserved")
    if p[4]==2 and (p[76]&~0x8f or any(v&~15 for v in p[77:80])):raise ValueError("Paper status model state")
    model,period=struct.unpack_from('>II',p,8)
    epoch,session,sequence,cons_session,cons_sequence,accepted_us,consumed_us,remaining,reason=struct.unpack_from('>Q6I2H',p,16)
    residual=struct.unpack_from('>4f',p,56)
    if model!=PAPER_MODEL_ID or reason>(11 if p[4]==2 else 10) or p[52]&~15 or p[53]&~15 or p[54] not in (0,1,2,3,4,255) or not all(math.isfinite(x) for x in residual):raise ValueError("Paper status model/value")
    bits=p[7]
    return dict(version=p[4],backend=p[5],frame='FRD',native_capable=bool(bits&1),demand_valid=bool(bits&2),paper_active=bool(bits&4),g2_present=bool(bits&8),servo_enabled=bool(bits&16),armed=bool(bits&32),observations_valid=bool(bits&64),model_contract_id=model,control_period_us=period,accepted_epoch=epoch,accepted_session=session,accepted_sequence=sequence,consumed_session=cons_session,consumed_sequence=cons_sequence,accepted_fc_us=accepted_us,consumed_fc_us=consumed_us,remaining_lease_ms=remaining,reason=reason,rpm_mask=p[52],saturation_mask=p[53],solver_result=p[54],residual_si=residual,rejected=struct.unpack_from('>I',p,72)[0],model_reason=p[76] if p[4]==2 else None,degraded_mask=p[77] if p[4]==2 else None,recovering_mask=p[78] if p[4]==2 else None,seeded_mask=p[79] if p[4]==2 else None,consumption_scope='FC control tick; UART enqueue/ACK does not prove consumption')
