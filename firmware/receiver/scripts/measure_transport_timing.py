#!/usr/bin/env python3
"""Disabled-only command stream plus residence-timestamped Zenoh probes.

Requires the optional ESP timing build. No probe can issue an FC command. The
producer retains the production 50 Hz source, 100 Hz transmitter and 100 ms TTL.
"""
import argparse,datetime,json,struct,subprocess,sys,threading,time
from dataclasses import asdict
from pathlib import Path
import zenoh
from sanity_zenoh.client import SanitySender
from sanity_zenoh.protocol import CTBR,DIRECT,COMMAND,ReceiverStatus,neutral
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'sanity-hardware/tools/acquisition'))
import betaflight_msp as bf
from uart4_mode import PORT
from measure_command_latency import msp_frame
PING=struct.Struct('>4sBBHIQ');PONG=struct.Struct('>4sBBHIQQQQbBBB');HEADER=struct.Struct('>4sBBBBI');EVENT=struct.Struct('>4B3I5Q2I')
EVENT_KEYS=['type','mode','flags','code','session','sequence','tick','epoch','t0','t1','t2','t3','arg0','arg1']

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--duration',type=float,default=120,help='Seconds in each disabled mode');ap.add_argument('--expected-uid',required=True);ap.add_argument('--reset-esp',action='store_true');a=ap.parse_args()
    if a.output.exists():ap.error('Choose a new output directory')
    if not 10<=a.duration<=600:ap.error('duration must be 10..600 seconds per mode')
    a.output.mkdir(parents=True);rows={k:[] for k in ['source','tx','status','probe_tx','pong','trace','invalidate','fc','phase','tcp']};lock=threading.RLock();fatal=[];sender=None;fc_stop=threading.Event();fc_thread=None;tcp_thread=None;phase='startup';error=None;guarded=False;initial_inhibit=None;boxes=None;started=time.monotonic_ns()
    def record(kind,**values):
        with lock:rows[kind].append({'host_ns':time.monotonic_ns(),'phase':phase,**values})
    t=bf.USBSerial(PORT);c=bf.MSP(t,.5);bf.COMMANDS[105]='RC'
    def fc_sample():
        raw=c.query(101);s=bf.status(raw,boxes);mask=struct.unpack_from('<I',raw,17+(raw[15]&15))[0]
        motors=list(struct.unpack_from('<4H',c.query(104)));rpm=[m['rpm'] for m in bf.motor_telemetry(c.query(139))['motors']];raw=c.query(105);rc=list(struct.unpack('<'+'H'*(len(raw)//2),raw))
        record('fc',armed=s['armed'],mask=mask,motors=motors,rpm=rpm,rc=rc)
        if s['armed'] is not False or not mask&(1<<16) or motors!=[1000]*4 or any(rpm) or rc[4]>1100:raise RuntimeError('FC disarmed/inhibited/STOP verification failed')
    class Sender(SanitySender):
        def __init__(self,mode):
            self.api_id=-1;self.tx_original=None
            super().__init__(device=9,mode=mode,listen='tcp/0.0.0.0:7447')
            self.tx_original=self.publisher
            owner=self
            class Guard:
                def put(self,payload):
                    f=COMMAND.unpack(payload)
                    if f[4] or tuple(f[11:])!=neutral(owner.mode):raise RuntimeError('Disabled-only guard rejected command')
                    h=time.monotonic_ns();owner.tx_original.put(payload)
                    record('tx',start_ns=h,end_ns=time.monotonic_ns(),api_id=owner.api_id,epoch=f[5],session=f[6],sequence=f[7],echo_ms=f[8],ttl_ms=f[9])
            self.publisher=Guard()
            self.pong_sub=self.session.declare_subscriber('sanity/09/diagnostics/pong',self.pong)
            self.trace_sub=self.session.declare_subscriber('sanity/09/diagnostics/trace',self.trace)
            self.probe_pub=self.session.declare_publisher('sanity/09/diagnostics/ping',priority=zenoh.Priority.REAL_TIME,congestion_control=zenoh.CongestionControl.DROP,reliability=zenoh.Reliability.BEST_EFFORT,express=True)
        def transmit(self):
            # Hold the existing RLock through put so a source API id cannot be
            # attached to a packet constructed just before that API update.
            with self.lock:super().transmit()
        def receive_status(self,sample):
            h=time.monotonic_ns()
            try:
                s=ReceiverStatus.decode(bytes(sample.payload));record('status',arrival_ns=h,**asdict(s))
                if s.enabled:fatal.append('Unexpected enabled ESP status')
            except (ValueError,TypeError):pass
            super().receive_status(sample)
        def invalidate(self,reason='Command expired'):
            record('invalidate',reason=reason);super().invalidate(reason)
        def pong(self,sample):
            h=time.monotonic_ns()
            try:
                f=PONG.unpack(bytes(sample.payload))
                if f[:4]!=(b'SZR1',1,9,0):return
                record('pong',arrival_ns=h,id=f[4],sent_ns=f[5],esp_rx_us=f[6],esp_tx_us=f[7],epoch=f[8],rssi_dbm=f[9],channel=f[10])
            except (ValueError,struct.error):pass
        def trace(self,sample):
            raw=bytes(sample.payload)
            if len(raw)<HEADER.size:return
            magic,version,device,size,n,dropped=HEADER.unpack_from(raw)
            if (magic,version,device,size)!=(b'SZT1',1,9,64) or len(raw)!=HEADER.size+n*size:return
            h=time.monotonic_ns()
            with lock:
                for i in range(n):rows['trace'].append({'host_ns':h,'phase':phase,'dropped':dropped,**dict(zip(EVENT_KEYS,EVENT.unpack_from(raw,HEADER.size+i*size)))})
        def update(self,i):
            with self.lock:
                h=time.monotonic_ns()
                if self.mode==CTBR:self.ctbr(False,0.,(0.,0.,0.))
                else:self.direct_thrust(False,(0.,)*4)
                self.api_id=i;record('source',start_ns=h,end_ns=time.monotonic_ns(),api_id=i,session=self.session_id)
        def probe(self,i):
            h=time.monotonic_ns();self.probe_pub.put(PING.pack(b'SZP1',1,9,0,i,h));record('probe_tx',id=i,start_ns=h,end_ns=time.monotonic_ns())
    try:
        if c.query(160).hex()!=a.expected_uid.lower():raise RuntimeError('FC UID mismatch')
        boxes=c.query(119);raw=c.query(101);status=bf.status(raw,boxes)
        initial_inhibit=bool(struct.unpack_from('<I',raw,17+(raw[15]&15))[0]&(1<<16))
        motors=struct.unpack_from('<4H',c.query(104));rpm=bf.motor_telemetry(c.query(139))['motors'];raw=c.query(105);rc=struct.unpack('<'+'H'*(len(raw)//2),raw)
        if status['armed'] is not False or motors!=(1000,)*4 or any(m['rpm'] for m in rpm) or rc[4]>1100:raise RuntimeError('Start disarmed, ARM low and motors stopped')
        c._exchange(msp_frame(99,b'\x01'),99);guarded=True;fc_sample()
        def monitor():
            while not fc_stop.is_set():
                try:fc_sample()
                except Exception as exc:fatal.append(str(exc));return
                fc_stop.wait(.2)
        fc_thread=threading.Thread(target=monitor,daemon=True);fc_thread.start()
        def monitor_tcp():
            while not fc_stop.is_set():
                try:
                    result=subprocess.run(['ss','-tin','dst','10.0.0.109'],capture_output=True,text=True,timeout=2)
                    record('tcp',text=result.stdout,error=result.stderr)
                except Exception as exc:record('tcp',error=str(exc))
                fc_stop.wait(1)
        tcp_thread=threading.Thread(target=monitor_tcp,daemon=True);tcp_thread.start()
        for index,mode in enumerate([CTBR,DIRECT]):
            phase='warmup'
            if sender is None:sender=Sender(mode)
            else:
                new_pub=sender.session.declare_publisher('sanity/09/command/direct_thrust',priority=zenoh.Priority.REAL_TIME,congestion_control=zenoh.CongestionControl.DROP,reliability=zenoh.Reliability.BEST_EFFORT,express=True)
                with sender.lock:
                    old_pub=sender.tx_original;sender.mode=mode;sender.invalidate('Explicit disabled mode transition');sender.tx_original=new_pub
                old_pub.undeclare()
            if index==0 and a.reset_esp:
                cmd=['/home/hw527/.espressif/python_env/idf6.2_py3.13_env/bin/python','-m','esptool','--chip','esp32s3','--port','/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_64:E8:33:50:9C:34-if00','--before','default-reset','--after','hard-reset','read-mac']
                with (a.output/'esp_reset.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=15)
            ready=time.monotonic()+30;i=0
            while time.monotonic()<ready:
                sender.update(i);sender.probe(i);i+=1;s=sender.receiver_status()
                if s and s.mode==mode and s.session==sender.session_id and not s.enabled and not s.failsafe and any(x['phase']=='warmup' for x in rows['pong']):break
                if fatal:raise RuntimeError(str(fatal))
                time.sleep(.02)
            else:raise TimeoutError('No disabled claim and timing pong')
            # Drain boot/startup traces before the measured interval.
            until=time.monotonic()+2
            while time.monotonic()<until:sender.update(i);sender.probe(i);i+=1;time.sleep(.02)
            phase='ctbr' if mode==CTBR else 'direct';begin=time.monotonic();next_tick=begin;next_probe=begin;next_report=begin+30
            record('phase',event='start',mode=mode,session=sender.session_id)
            while time.monotonic()-begin<a.duration:
                sender.update(i);i+=1
                if time.monotonic()>=next_probe:sender.probe(i);next_probe+=.05
                if fatal:raise RuntimeError(str(fatal))
                if time.monotonic()>=next_report:
                    print(json.dumps({'phase':phase,'elapsed_s':round(time.monotonic()-begin,1),'pongs':sum(x['phase']==phase for x in rows['pong']),'traces':sum(x['phase']==phase for x in rows['trace']),'expiry_examples':[{'evaluated_ms':x['t1'],'last_rx_ms':x['t2'],'lease_ms':x['t3'],'ttl_ms':x['arg0'],'cause_bits':x['arg1']} for x in rows['trace'] if x['type']==4 and x['code']==6][-3:]}),flush=True);next_report+=30
                next_tick=max(next_tick+.02,time.monotonic());time.sleep(max(0,next_tick-time.monotonic()))
            record('phase',event='end',mode=mode,session=sender.session_id)
            phase='drain';until=time.monotonic()+1
            while time.monotonic()<until:sender.update(i);i+=1;time.sleep(.02)
    except BaseException as exc:error=f'{type(exc).__name__}: {exc}'
    finally:
        phase='cleanup';fc_stop.set()
        if fc_thread:fc_thread.join(timeout=2)
        if tcp_thread:tcp_thread.join(timeout=3)
        if sender:
            try:sender.close()
            except Exception as exc:error=(error+'; ' if error else '')+'close: '+str(exc)
        if guarded:
            try:
                fc_sample();c._exchange(msp_frame(99,bytes((int(initial_inhibit),))),99)
                record('phase',event='inhibit_restored',initial_inhibit=initial_inhibit)
            except Exception as exc:
                error=(error+'; ' if error else '')+'final: '+str(exc)
                try:c._exchange(msp_frame(99,b'\x01'),99)
                except Exception:pass
        t.close()
        for name,data in rows.items():(a.output/(name+'.json')).write_text(json.dumps(data)+'\n')
        result={'timestamp_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'elapsed_s':(time.monotonic_ns()-started)/1e9,'error':error,'fatal':fatal,'disabled_only':True,'arming_commands_sent':False,'positive_thrust_sent':False}
        (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    if error:raise SystemExit(1)
if __name__=='__main__':main()
