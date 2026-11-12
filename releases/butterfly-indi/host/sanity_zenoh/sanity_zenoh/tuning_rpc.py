"""Bounded runtime PID transactions for the recorded Betaflight4.5.1/API1.46."""
from dataclasses import dataclass
import json
import struct
import threading
import time

REQUEST=struct.Struct('>4sBBBBQIIIHH')
REPLY=struct.Struct('>4sBBBBQIIQ')
ERRORS={1:'FC rejected MSP',2:'Receiver denied ownership/bounds/state',3:'FC response timed out',4:'Transaction busy/serial drain',5:'Unsupported command/payload/profile'}


@dataclass(frozen=True)
class TuneSnapshot:
    pid:bytes
    advanced:bytes
    profile:int

    def __post_init__(self):
        if not 9<=len(self.pid)<=30 or len(self.pid)%3 or len(self.advanced)!=61 or not self.advanced[47] or not 0<=self.profile<3:
            raise ValueError('Expected Betaflight4.5.1 PID and61-byte advanced payloads')

    def gain(self,axis,term):
        if axis not in (0,1) or term not in ('P','D','F'):raise ValueError('Only roll/pitch P,D,F are tunable')
        return struct.unpack_from('<H',self.advanced,32+2*axis)[0] if term=='F' else self.pid[3*axis+(0 if term=='P' else 2)]

    def changed(self,axis,term,value):
        if not isinstance(value,int) or not 0<=value<=(65535 if term=='F' else 255):raise ValueError('Integer gain out of range')
        self.gain(axis,term);pid=bytearray(self.pid);advanced=bytearray(self.advanced)
        if term=='F':struct.pack_into('<H',advanced,32+2*axis,value)
        else:pid[3*axis+(0 if term=='P' else 2)]=value
        return TuneSnapshot(bytes(pid),bytes(advanced),self.profile)

    def with_damping(self,axis,value):
        if self.advanced[39+axis]!=self.gain(axis,'D'):
            raise ValueError('Paired damping search requires D-min equal to D at entry')
        changed=self.changed(axis,'D',value);advanced=bytearray(changed.advanced);advanced[39+axis]=value
        return TuneSnapshot(changed.pid,bytes(advanced),changed.profile)

    def with_boost(self,value):
        if not isinstance(value,int) or not 0<=value<=255:raise ValueError('Integer boost out of range')
        advanced=bytearray(self.advanced);advanced[52]=value
        return TuneSnapshot(self.pid,bytes(advanced),self.profile)

    def json(self):
        return {'d_min_roll':self.advanced[39],'d_min_pitch':self.advanced[40],'feedforward_boost':self.advanced[52],'profile':self.profile,'pid_hex':self.pid.hex(),'advanced_hex':self.advanced.hex(),'roll':{k:self.gain(0,k) for k in ('P','D','F')},'pitch':{k:self.gain(1,k) for k in ('P','D','F')}}

    def cli(self):
        return '\n'.join(f'set {term.lower()}_{name} = {self.gain(axis,term)}' for axis,name in enumerate(('roll','pitch')) for term in ('P','D','F'))+'\n'+f'set d_min_roll = {self.advanced[39]}\nset d_min_pitch = {self.advanced[40]}\nset feedforward_boost = {self.advanced[52]}\n'


class MspTuningRpc:
    def __init__(self,sender):
        self.sender=sender;self.condition=threading.Condition();self.serial=threading.Lock();self.responses={};self.request_id=0;self.closed=threading.Event()
        import zenoh
        root=f'sanity/{sender.device:02d}/tuning/msp'
        self.subscriber=sender.session.declare_subscriber(root+'/reply',self.receive)
        # Pico1.10.1 does not deliver this RPC on the best-effort UDP channel.
        # Configuration transactions use reliable delivery; CTBR traffic is unchanged.
        self.publisher=sender.session.declare_publisher(root+'/request',priority=zenoh.Priority.REAL_TIME,congestion_control=zenoh.CongestionControl.DROP,reliability=zenoh.Reliability.RELIABLE,express=True)

    def record(self,event):
        if self.sender.on_event:self.sender.on_event(event)

    def receive(self,sample):
        raw=bytes(sample.payload)
        try:
            if not 32<=len(raw)<=128:raise ValueError('MSP RPC reply length')
            magic,version,device,command,code,epoch,session,identifier,stamp=REPLY.unpack_from(raw)
            if (magic,version,device)!=(b'SZA1',1,self.sender.device) or session!=self.sender.session_id or code>5:raise ValueError('MSP RPC reply header')
            value={'command':command,'code':code,'epoch':epoch,'session':session,'id':identifier,'esp_us':stamp,'payload':raw[32:]}
            with self.condition:
                self.responses[identifier]=value
                if len(self.responses)>16:self.responses.pop(next(iter(self.responses)))
                self.condition.notify_all()
            self.record({'kind':'msp_rpc_reply','payload_hex':raw.hex(),'id':identifier,'command':command,'code':code})
        except (ValueError,struct.error) as error:self.record({'kind':'msp_rpc_decode_error','payload_hex':raw.hex(),'error':str(error)})

    def call(self,command,payload=b'',timeout=1.5,keepalive=None,cancel=None):
        if command not in (0,94,95,112,150,202,250) or len(payload)>96:raise ValueError('MSP command outside tuning whitelist')
        with self.serial:
            self.request_id=(self.request_id+1)&0xffffffff or 1;identifier=self.request_id;deadline=time.monotonic()+timeout;next_send=0.
            while time.monotonic()<deadline:
                if self.closed.is_set():raise InterruptedError("MSP RPC closed")
                if cancel is not None and cancel.is_set():raise InterruptedError('Tuning cancelled')
                if keepalive:keepalive()
                status=self.sender.receiver_status()
                if status is None or not status.tuning_supported:raise ValueError('Fresh tuning-capable ESP status required')
                if status.session!=self.sender.session_id or status.failsafe:raise ValueError('MSP tuning requires fresh command ownership')
                if time.monotonic()>=next_send:
                    raw=REQUEST.pack(b'SZM1',1,self.sender.device,command,len(payload),status.epoch,self.sender.session_id,identifier,status.receiver_ms,800,0)+payload
                    self.publisher.put(raw);self.record({'kind':'msp_rpc_request','id':identifier,'command':command,'payload_hex':raw.hex()});next_send=time.monotonic()+.1
                with self.condition:
                    response=self.responses.pop(identifier,None)
                    if response is None:self.condition.wait(.02);continue
                if response['epoch']!=status.epoch or response['command']!=command:raise ValueError('RPC context changed')
                if response['code']==4:time.sleep(.03);continue
                if response['code']:raise ValueError(ERRORS[response['code']])
                return response['payload']
            raise TimeoutError('No acknowledged MSP tuning response; no blind write retry')

    def snapshot(self,keepalive=None):
        cap=self.call(0,keepalive=keepalive)
        if len(cap) not in (7,8) or tuple(cap[:5])!=(1,46,4,5,1):raise ValueError('Tuner requires Betaflight4.5.1/API1.46')
        pid=self.call(112,keepalive=keepalive);advanced=self.call(94,keepalive=keepalive);status=self.call(150,keepalive=keepalive)
        if len(status)<11 or status[10]!=cap[5]:raise ValueError('Active PID profile changed')
        return TuneSnapshot(pid,advanced,cap[5])

    def require_extensions(self,mask,keepalive):
        cap=self.call(0,keepalive=keepalive)
        if len(cap)!=8 or cap[7]&mask!=mask:
            raise ValueError('Install extended D-min/boost tuning firmware before flight')

    def verify_setters(self,snapshot,keepalive):
        cap=self.call(0,keepalive=keepalive)
        if len(cap) not in (7,8) or not cap[6]:raise ValueError('Setter preflight requires FC disarmed')
        for write,read,payload in ((202,112,snapshot.pid),(95,94,snapshot.advanced)):
            self.call(write,payload,keepalive=keepalive)
            if self.call(read,keepalive=keepalive)!=payload:raise ValueError('Unchanged-value setter preflight readback mismatch')

    def apply(self,target,baseline,cancel=None,keepalive=None):
        current=self.snapshot(keepalive)
        if current.profile!=target.profile or baseline.profile!=target.profile:raise ValueError('PID profile changed')
        for snapshot in (current,target):
            reconstructed=baseline
            for axis in (0,1):
                for term in ('P','D','F'):reconstructed=reconstructed.changed(axis,term,snapshot.gain(axis,term))
            advanced=bytearray(reconstructed.advanced);advanced[39:41]=snapshot.advanced[39:41];advanced[52]=snapshot.advanced[52]
            reconstructed=TuneSnapshot(reconstructed.pid,bytes(advanced),reconstructed.profile)
            if reconstructed!=snapshot:raise ValueError('Non-tuning settings changed; refusing to overwrite')
        for _ in range(30):
            if current==target:return current
            step=current
            for axis in (0,1):
                for term in ('P','D','F'):
                    previous=current.gain(axis,term);desired=target.gain(axis,term)
                    maximum=max(1,int(baseline.gain(axis,term)*(.15 if term=='F' else .1)))
                    value=previous+max(-maximum,min(maximum,desired-previous));step=step.changed(axis,term,value)
            advanced=bytearray(step.advanced)
            for axis in (0,1):
                previous=current.advanced[39+axis];desired=target.advanced[39+axis];maximum=max(1,int(baseline.gain(axis,'D')*.1))
                advanced[39+axis]=previous+max(-maximum,min(maximum,desired-previous))
            previous=current.advanced[52];desired=target.advanced[52];advanced[52]=previous+max(-5,min(5,desired-previous))
            step=TuneSnapshot(step.pid,bytes(advanced),step.profile)
            for command,read,data,old in ((202,112,step.pid,current.pid),(95,94,step.advanced,current.advanced)):
                if data!=old:
                    self.call(command,data,keepalive=keepalive,cancel=cancel)
                    if self.call(read,keepalive=keepalive,cancel=cancel)!=data:raise ValueError('MSP gain readback mismatch')
            current=step
        raise ValueError('Gain change exceeded bounded update budget')

    def save(self,target,keepalive):
        cap=self.call(0,keepalive=keepalive);status=self.sender.receiver_status()
        if len(cap) not in (7,8) or not cap[6] or status is None or status.enabled:raise ValueError('EEPROM save requires confirmed FC disarmed')
        if self.snapshot(keepalive)!=target:raise ValueError('Cannot save unverified gains')
        self.call(250,timeout=3.,keepalive=keepalive)
        if self.snapshot(keepalive)!=target:raise ValueError('Saved gain readback mismatch')

    def close(self):
        if self.closed.is_set():return
        self.closed.set()
        with self.condition:self.condition.notify_all()
        self.subscriber.undeclare();self.publisher.undeclare()
