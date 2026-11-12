#!/usr/bin/env python3
"""Props-off command-to-FC readback/RPM benchmark through the production Zenoh link.

CTBR: zero thrust, FC arming inhibited over independent USB. Direct-counts:
uses physical-unit inputs calculated for bounded count targets within the fit.
"""
import argparse,datetime,functools,json,math,secrets,struct,sys,threading,time
from pathlib import Path
from dataclasses import asdict
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'sanity-hardware/tools/acquisition'))
import betaflight_msp as bf
from sanity_zenoh.client import SanitySender
from sanity_zenoh.actuator import DirectThrustMapping
from sanity_zenoh.protocol import CTBR,DIRECT,COMMAND,ReceiverStatus
from uart4_mode import PORT


def msp_frame(command,payload=b''):
    body=bytes((len(payload),command))+payload
    return b'$M<'+body+bytes((functools.reduce(int.__xor__,body,0),))


def query_batch(client,commands):
    """One outstanding request per ID, sent together; timestamp each parsed reply.

    This avoids serializing USB reply aggregation delay across independent queries.
    Request time is a conservative earliest sample bound for every reply.
    """
    if len(set(commands))!=len(commands):raise ValueError('Batch command IDs must be unique')
    start=time.monotonic();deadline=start+client.timeout
    client.transport.write(b''.join(bf.request_frame(cmd) for cmd in commands),deadline)
    pending=set(commands);responses={};buffer=client.buffer
    while pending:
        while len(buffer)>=3:
            pos=buffer.find(b'$M')
            if pos<0:del buffer[:-1];break
            if pos:del buffer[:pos]
            if len(buffer)<5:break
            if buffer[2] not in (ord('>'),ord('!')):del buffer[0];continue
            size,cmd=buffer[3:5]
            if size==255:raise bf.MSPError('Jumbo reply unsupported in telemetry batch')
            if len(buffer)<size+6:break
            frame=bytes(buffer[:size+6]);del buffer[:size+6]
            if functools.reduce(int.__xor__,frame[3:-1],0)!=frame[-1] or cmd not in pending:continue
            if frame[2]==ord('!'):raise bf.MSPError('FC rejected batched command '+str(cmd))
            responses[cmd]=(frame[5:-1],start,time.monotonic());pending.remove(cmd)
        if pending:buffer.extend(client.transport.read(deadline))
    return responses

def allowed_packet(fields,mode):
    if fields[2]!=mode:return False
    if not fields[4] and tuple(fields[11:])!=((1500,1500,1500,1000) if mode==CTBR else (1000,)*4):return False
    v=tuple(fields[11:])
    if mode==CTBR:return v[3]==1000 and v[1:3]==(1500,1500) and 1400<=v[0]<=1600
    return all(x==1000 or 1250<=x<=1300 for x in v) and sum(x!=1000 for x in v)<=1


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',choices=['ctbr','direct-counts'],required=True)
    ap.add_argument('--expected-uid',required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--props-removed',action='store_true',required=True)
    ap.add_argument('--trials',type=int,default=80)
    ap.add_argument('--motors',type=int,nargs='+',choices=[1,2,3,4],default=[1,2,3,4])
    args=ap.parse_args()
    if args.output.exists():ap.error('Choose a new output directory')
    if not 1<=args.trials<=160:ap.error('trials must be 1..160')
    args.output.mkdir(parents=True)
    mode=CTBR if args.mode=='ctbr' else DIRECT
    bf.COMMANDS[105]='RC'
    t=bf.USBSerial(PORT);c=bf.MSP(t,timeout=.3)
    rows=[];source_rows=[];status_rows=[];tx=[];trials=[];events=[];fatal=[]
    lock=threading.RLock();stop=threading.Event()
    desired={'enabled':False,'values':(1500,1500,1500,1000) if mode==CTBR else (1000,)*4,'trial':None}
    sender=producer=observer=None;verified=False;initial_inhibit=False;inhibit=False;error=None;latest={}
    begin=time.monotonic()
    def event(kind,**kw):events.append({'t':time.monotonic(), 'kind':kind,**kw})
    def set_inhibit(value):return c._exchange(msp_frame(99,bytes((int(value),))),99)
    def usb_stop():return c._exchange(msp_frame(214,struct.pack('<4H',1000,1000,1000,1000)),214)
    def status_check():
        raw=c.query(101);s=bf.status(raw,boxes);extra=raw[15]&15
        mask=struct.unpack_from('<I',raw,17+extra)[0]
        if s['armed'] is not False or not mask&(1<<16):
            set_inhibit(True)
            raise RuntimeError('FC arming inhibit or confirmed disarmed state lost')
        return mask
    def set_target(enabled,values,trial=None):
        with lock:desired.update(enabled=enabled,values=tuple(values),trial=trial)
    def reader():
        nonlocal latest
        try:
            while not stop.is_set():
                batch=query_batch(c,[101,105,104,139,130])
                raw,q0,q1=batch[101];status=bf.status(raw,boxes);extra=raw[15]&15
                mask=struct.unpack_from('<I',raw,17+extra)[0]
                if status['armed'] is not False or not mask&(1<<16):
                    set_inhibit(True);raise RuntimeError('FC arming inhibit or disarmed state lost')
                status_row={'kind':'status','start':q0,'end':q1,'armed':False,'inhibit_mask':mask}
                raw,q0,q1=batch[105];rc=tuple(struct.unpack('<'+'H'*(len(raw)//2),raw))
                rc_row={'kind':'rc','start':q0,'end':q1,'values':rc}
                if rc[3]!=1000:raise RuntimeError('Test throttle is not minimum')
                raw,q0,q1=batch[104];values=tuple(struct.unpack_from('<4H',raw))
                motor_row={'kind':'motors','start':q0,'end':q1,'values':values}
                raw,q0,q1=batch[139];rpm=tuple(m['rpm'] for m in bf.motor_telemetry(raw)['motors'])
                rpm_row={'kind':'rpm','start':q0,'end':q1,'values':rpm}
                if mode==CTBR and (values!=(1000,)*4 or any(rpm)):raise RuntimeError('Unexpected motor output in inhibited CTBR test')
                if mode==DIRECT and (any(x!=1000 and not 1250<=x<=1300 for x in values) or sum(x!=1000 for x in values)>1):raise RuntimeError('Unexpected motor command outside the single-motor bench envelope')
                battery=bf.battery(batch[130][0])
                if mode==DIRECT and (battery['detected_cells']!=2 or not 7.514<=battery['voltage_V']<=8.112):raise RuntimeError('Battery outside production actuator calibration envelope')
                with lock:
                    rows.extend([status_row,rc_row,motor_row,rpm_row])
                    latest.update(inhibit_mask=mask,rc=rc,rc_time=rc_row['end'],motors=values,motor_time=motor_row['end'],rpm=rpm,battery=battery)
                stop.wait(.001)
        except Exception as exc:
            fatal.append(str(exc));event('observer_error',error=str(exc));stop.set()
    def source():
        while not stop.is_set():
            with lock:d=desired.copy()
            try:
                started=time.monotonic()
                if mode==CTBR:
                    rate=math.radians((d['values'][0]-1500)*sender.ctbr_mapping.rate_full_scale_deg_s[0]/500)
                    sender.ctbr(d['enabled'],0.,(rate,0.,0.))
                else:
                    with sender.lock:
                        if d['enabled'] and any(v!=1000 for v in d['values']):
                            status=sender.receiver_status()
                            if not status:raise RuntimeError('Fresh voltage required')
                            voltage=status.voltage_mv/1000
                            hardware=[]
                            for v,(a,b,c0) in zip(d['values'],sender.direct_mapping.coefficients):
                                q=(v-1350)/100
                                hardware.append(0. if v==1000 else sender.direct_mapping.k*((voltage/8)*(a+b*q+c0*q*q))**2)
                            thrusts=tuple(hardware[i-1] for i in sender.direct_mapping.order)
                        else:voltage=None;thrusts=(0.,)*4
                        started=time.monotonic()
                        sender.direct_thrust(d['enabled'],thrusts)

                with lock:
                    source_rows.append({'start':started,'end':time.monotonic(),'trial':d['trial'],'enabled':d['enabled'],'values':d['values'], 'thrusts_N':thrusts if mode==DIRECT else None, 'voltage_V':voltage if mode==DIRECT else None})
            except Exception as exc:
                event('source_fault',error=str(exc),trial=d['trial'])
                set_target(False,(1500,1500,1500,1000) if mode==CTBR else (1000,)*4)
                if mode==DIRECT:
                    fatal.append('Direct output stopped after source/interlock fault: '+str(exc));stop.set();return
            stop.wait(.02)
    def check():
        if fatal or stop.is_set():raise RuntimeError('; '.join(fatal) or 'Test stopped')
    def handshake():
        neutral=(1500,1500,1500,1000) if mode==CTBR else (1000,)*4
        for enabled in [False,True]:
            set_target(enabled,neutral)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                check();s=sender.receiver_status()
                if s and s.mode==mode and s.session==sender.session_id and s.enabled==enabled and not s.failsafe and s.fc_ready:break
                time.sleep(.005)
            else:raise TimeoutError('Disabled/zero-enable handshake failed')
    try:
        if c.query(160).hex()!=args.expected_uid.lower():raise RuntimeError('FC UID mismatch')
        if bf.identify(c)['firmware_version']!='4.5.1':raise RuntimeError('Expected BF 4.5.1')
        boxes=c.query(119);initial=c.query(101);extra=initial[15]&15
        initial_inhibit=bool(struct.unpack_from('<I',initial,17+extra)[0]&(1<<16))
        if bf.status(initial,boxes)['armed'] is not False:raise RuntimeError('FC not disarmed')
        if any(m['rpm'] for m in bf.motor_telemetry(c.query(139))['motors']):raise RuntimeError('Motors must initially be stopped')
        verified=True;set_inhibit(True);inhibit=True;status_check()
        model=Path(__file__).resolve().parents[4]/'sanity-hardware/parameters/actuators.json'
        mapping=DirectThrustMapping(model,(1,2,4,3)) if mode==DIRECT else None
        class ObservedSender(SanitySender):
            def receive_status(self,sample):
                try:status_rows.append({'t':time.monotonic(),**asdict(ReceiverStatus.decode(bytes(sample.payload)))})
                except (ValueError,TypeError):pass
                super().receive_status(sample)
            def invalidate(self,reason='Command expired'):
                event('sender_invalidation',reason=reason,trial=desired['trial'])
                super().invalidate(reason)
        sender=ObservedSender(mode=mode,listen='tcp/0.0.0.0:7447',direct_mapping=mapping)
        publisher=sender.publisher
        class Guard:
            def put(self,payload):
                fields=COMMAND.unpack(payload)
                if not allowed_packet(fields,mode):raise RuntimeError('Benchmark packet guard rejected command')
                timestamp=time.monotonic()
                with lock:
                    active=desired.copy()
                    tx.append({'t':timestamp,'sequence':fields[7],'enabled':bool(fields[4]),'values':fields[11:],'trial':active['trial'] if tuple(fields[11:])==active['values'] else None})
                publisher.put(payload)
        sender.publisher=Guard()
        observer=threading.Thread(target=reader,daemon=True);observer.start()
        producer=threading.Thread(target=source,daemon=True);producer.start()
        handshake()
        motors=[None] if mode==CTBR else [m-1 for m in args.motors]
        for motor in motors:
            if mode==DIRECT:
                handshake()
                baseline=[1000]*4;baseline[motor]=1250;set_target(True,baseline)
                until=time.monotonic()+.8
                while time.monotonic()<until:check();time.sleep(.005)
                targets=[1300,1251,1299,1252,1298,1253,1297,1254,1296,1255,1295,1256][:min(args.trials,12)]
            else:
                import random
                targets=random.Random(527).sample([v for v in range(1420,1581) if v!=1500],args.trials)
            for target in targets:
                check()
                ident=len(trials)
                values=(target,1500,1500,1000) if mode==CTBR else tuple(target if i==motor else 1000 for i in range(4))
                trial={'id':ident,'motor':None if motor is None else motor+1,'values':values,'requested':time.monotonic()};trials.append(trial)
                set_target(True,values,ident)
                end=time.monotonic()+(.20 if mode==CTBR else .35)
                while time.monotonic()<end:check();time.sleep(.002)
                trial['end']=time.monotonic()
                if mode==CTBR:
                    s=sender.receiver_status()
                    if not s or not s.enabled or sender.rearm_required:
                        trial['faulted']=True;event('explicit_recovery',trial=ident);handshake()
            if mode==DIRECT:
                set_target(False,(1000,)*4)
                end=time.monotonic()+3
                while time.monotonic()<end:
                    check()
                    with lock:stopped=latest.get('motors')==(1000,)*4 and not any(latest.get('rpm',(1,)))
                    if stopped:break
                    time.sleep(.01)
                else:raise TimeoutError('Motors did not stop between blocks')
    except BaseException as exc:error=f'{type(exc).__name__}: {exc}';event('test_error',error=error)
    finally:
        stop.set()
        for thread in [producer,observer]:
            if thread:thread.join(timeout=2)
        if sender:
            try:sender.close()
            except Exception as exc:event('sender_close_error',error=str(exc))
        final={}
        if verified:
            try:
                time.sleep(.25)
                for _ in range(3):usb_stop();time.sleep(.02)
                final['status']=bf.status(c.query(101),boxes)
                rcraw=c.query(105);final['rc']=list(struct.unpack('<'+'H'*(len(rcraw)//2),rcraw))
                final['motors']=list(struct.unpack_from('<4H',c.query(104)))
                deadline=time.monotonic()+3
                while True:
                    final['rpm']=[m['rpm'] for m in bf.motor_telemetry(c.query(139))['motors']]
                    if not any(final['rpm']) or time.monotonic()>deadline:break
                    time.sleep(.02)
                final['battery']=bf.battery(c.query(130))
                safe=final['status']['armed'] is False and final['rc'][4]<=1100 and final['motors']==[1000]*4 and not any(final['rpm'])
                if safe and inhibit and not initial_inhibit:set_inhibit(False);final['inhibit_restored']=True
                else:final['inhibit_left_on']=bool(inhibit)
                if not safe:error=(error+'; ' if error else '')+'Final stop/ARM-low verification failed; inhibit retained'
            except Exception as exc:error=(error+'; ' if error else '')+'Cleanup: '+str(exc)
        t.close()
        result={'timestamp_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'mode':args.mode,'trials':trials,'events':events,'final':final,'error':error,'source_start':'First physical-unit API call for each target; direct thrusts are generated to produce known bounded count targets','telemetry':'Batched independent USB queries; request-to-reply bounds retained', 'scope':'CTBR is arming-inhibited RC application readback; direct is bounded props-off count application and reported RPM, not measured thrust.'}
        for name,data in [('results',result),('usb',rows),('source',source_rows),('tx',tx),('receiver_status',status_rows)]:
            (args.output/(name+'.json')).write_text(json.dumps(data)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ['trials','events']},indent=2),flush=True)
    if error:raise SystemExit(1)


if __name__=='__main__':main()
