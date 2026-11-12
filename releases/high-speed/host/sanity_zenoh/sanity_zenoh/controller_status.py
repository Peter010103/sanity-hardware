"""Read-only Sanity PID/INDI authority status, MSPv2 command 0x7f11."""
import math
import struct

COMMAND=0x7F11
SIZE=80
MODES=('pid','to_indi','indi','to_pid')
REASONS=('ok','disabled','disarmed','mode','config','rpm','timing','numeric',
         'allocator','motion','saturation','tracking','takeoff','request_reset','idle','warmup','rx')

def crc8(data):
    crc=0
    for value in data:
        crc^=value
        for _ in range(8):crc=((crc<<1)^(0xd5 if crc&0x80 else 0))&255
    return crc

def wrapped_query():
    inner=struct.pack('<BHH',0,COMMAND,0)
    inner+=bytes([crc8(inner)])
    outer=bytes([len(inner),255])+inner
    checksum=0
    for value in outer:checksum^=value
    return b'$M<'+outer+bytes([checksum])

def decode_wrapped(payload):
    if len(payload)!=SIZE+6:raise ValueError('Controller MSP2-over-v1 length')
    flags,command,length=struct.unpack_from('<BHH',payload)
    if flags or command!=COMMAND or length!=SIZE or crc8(payload[:-1])!=payload[-1]:
        raise ValueError('Controller MSP2-over-v1 header/CRC')
    return decode(payload[5:-1])

def decode(data):
    if len(data)!=SIZE or data[0]!=1 or data[1]>=len(MODES) or data[2]&128:
        raise ValueError('Controller status version/mode/flags')
    if any(data[i]>=len(REASONS) for i in (3,4,76)) or data[5]&~15 or data[6]&~15 or data[43] or any(data[77:]):
        raise ValueError('Controller status reason/masks/reserved fields')
    weight=struct.unpack_from('<f',data,24)[0]
    agreement=struct.unpack_from('<f',data,36)[0]
    collective,*rates=struct.unpack_from('<4f',data,60)
    if not all(math.isfinite(x) for x in (weight,agreement,collective,*rates)) or not 0<=weight<=1 or agreement<0 or collective<0:
        raise ValueError('Controller status nonfinite/out-of-range values')
    if (data[1]==0 and weight!=0) or (data[1]==2 and weight!=1):raise ValueError('Controller mode/weight disagreement')
    now,sequence,steps=struct.unpack_from('<3I',data,8)
    transitions,fallbacks=struct.unpack_from('<2H',data,20)
    runtime,max_runtime,cycle,max_cycle=struct.unpack_from('<4H',data,28)
    bits=data[2]
    return {'version':1,'mode':MODES[data[1]],'permission':bool(bits&1),'requested':bool(bits&2),
        'eligible':bool(bits&4),'fault_latched':bool(bits&8),'armed':bool(bits&16),
        'takeoff_guard_triggered':bool(bits&32),'supported':bool(bits&64),
        'reason':REASONS[data[3]],'last_fault':REASONS[data[4]],'rpm_mask':data[5],
        'saturated_mask':data[6],'solver_exit':data[7],'fc_time_us':now,'sequence':sequence,
        'allocator_steps':steps,'transitions':transitions,'fallbacks':fallbacks,'indi_weight':weight,
        'runtime_us':runtime,'max_runtime_us':max_runtime,'cycle_us':cycle,'max_cycle_us':max_cycle,
        'candidate_disagreement':agreement,'iterations':struct.unpack_from('<H',data,40)[0],
        'request_aux':data[42],'fc_uid_msp':data[44:56].hex(),'entry_healthy_us':struct.unpack_from('<I',data,56)[0],
        'nominal_collective_N':collective,'gyro_native_rad_s':rates,'fault':REASONS[data[76]]}
