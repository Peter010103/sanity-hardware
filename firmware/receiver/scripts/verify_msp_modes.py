#!/usr/bin/env python3
"""Disabled-only CTBR -> direct MSP bench -> CTBR check, with no FC mode changes."""
import argparse
from dataclasses import asdict
import datetime,json,struct,sys,threading,time
from pathlib import Path
import zenoh
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'sanity-hardware/tools/acquisition'))
import betaflight_msp as bf
from sanity_zenoh.client import SanitySender
from sanity_zenoh.protocol import CTBR,DIRECT,COMMAND,neutral
from uart4_mode import PORT,serial_records


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--expected-uid',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=10,help='Seconds in each of three disabled phases')
    a=p.parse_args()
    if a.output.exists():p.error('Choose a new output file')
    if not 0<a.duration<=60:p.error('duration must be finite and in (0,60]')
    bf.COMMANDS.update({54:'CF_SERIAL_CONFIG',105:'RC'})
    t=bf.USBSerial(PORT);c=bf.MSP(t,timeout=.5)
    peer=sender=worker=None;stop=threading.Event();phases=[];error=None;checks=[]
    def sample():
        status=bf.status(c.query(101),c.query(119))
        rpm=[m['rpm'] for m in bf.motor_telemetry(c.query(139))['motors']]
        motors=list(struct.unpack_from('<4H',c.query(104)))
        raw=c.query(105);rc=list(struct.unpack('<'+'H'*(len(raw)//2),raw))
        ports=serial_records(c.query(54));features=struct.unpack_from('<I',c.query(36))[0]
        row={'t':time.monotonic(),'armed':status['armed'],'rpm':rpm,'motor_counts':motors,'rc':rc,'ports':ports,'features':features}
        checks.append(row)
        if status['armed'] is not False or any(rpm) or motors!=[1000]*4:
            raise RuntimeError('FC must stay disarmed with STOP outputs and zero RPM')
        if ports!=before_ports or features!=before_features:
            raise RuntimeError('FC configuration changed during mode transitions')
        return row
    def disabled(mode):
        if mode==CTBR:sender.ctbr(False,0.,(0.,0.,0.))
        else:sender.direct_thrust(False,(0.,)*4)
    try:
        if c.query(160).hex()!=a.expected_uid.lower():raise RuntimeError('FC UID mismatch')
        before_ports=serial_records(c.query(54));before_features=struct.unpack_from('<I',c.query(36))[0]
        if not before_features&(1<<14) or before_features&(1<<3):raise RuntimeError('Select RX_MSP once before this test')
        uart4=next(r for r in before_ports if r[0]==3)
        if uart4[1:3]!=[1,5]:raise RuntimeError('UART4 must stay MSP 115200')
        sample()
        config=zenoh.Config()
        for key,value in {'mode':'peer','listen/endpoints':['tcp/0.0.0.0:7447'],'scouting/multicast/enabled':False,'scouting/gossip/enabled':False}.items():
            config.insert_json5(key,json.dumps(value))
        peer=zenoh.open(config)
        for mode in [CTBR,DIRECT,CTBR]:
            sender=SanitySender(mode=mode,connect='tcp/127.0.0.1:7447')
            publisher=sender.publisher;packets=[0];source_errors=[]
            class DisabledPublisher:
                def put(self,packet):
                    f=COMMAND.unpack(packet)
                    if f[4] or tuple(f[11:])!=neutral(f[2]):raise RuntimeError('Disabled-only guard blocked output')
                    packets[0]+=1;publisher.put(packet)
            sender.publisher=DisabledPublisher()
            stop.clear()
            def produce():
                while not stop.is_set():
                    try:disabled(mode)
                    except Exception as exc:source_errors.append(str(exc));return
                    stop.wait(.02)
            worker=threading.Thread(target=produce,daemon=True);worker.start()
            deadline=time.monotonic()+20
            while True:
                s=sender.receiver_status()
                if s and s.mode==mode and s.session==sender.session_id and not s.enabled and not s.failsafe and s.fc_age_ms<150 and (mode==DIRECT or s.fc_ready):break
                if time.monotonic()>=deadline:raise TimeoutError('No fresh disabled claim/FC status for selected mode')
                time.sleep(.02)
            epoch=s.epoch;start=time.monotonic();next_fc=start;statuses=[]
            while time.monotonic()-start<a.duration:
                s=sender.receiver_status()
                if s:
                    statuses.append(asdict(s))
                    if s.enabled or s.mode!=mode or s.epoch!=epoch:raise RuntimeError('Unexpected enable/mode/epoch during phase')
                if source_errors:raise RuntimeError(str(source_errors))
                if time.monotonic()>=next_fc:
                    row=sample()
                    if row['rc'][:6]!=[1500,1500,1500,1000,1000,2000]:raise RuntimeError('FC RC readback does not match disabled AETR/ARM/Acro channels')
                    next_fc=time.monotonic()+.2
                time.sleep(.02)
            stop.set();worker.join();worker=None;sender.close();sender=None
            phases.append({'mode':mode,'epoch':epoch,'command_packets':packets[0],'statuses':statuses})
            print(json.dumps({'phase_complete':mode,'epoch':epoch,'packets':packets[0]}),flush=True)
    except BaseException as exc:error=f'{type(exc).__name__}: {exc}'
    finally:
        stop.set()
        if worker:worker.join(timeout=2)
        if sender:
            try:sender.close()
            except Exception as exc:error=(error+'; ' if error else '')+'Sender cleanup: '+str(exc)
        if peer:peer.close()
        try:sample()
        except Exception as exc:error=(error+'; ' if error else '')+'Final FC check: '+str(exc)
        t.close()
        report={'timestamp_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'phases':phases,'fc_checks':checks,'error':error,'stop_only':True,'arming_commands_sent':False,'positive_thrust_sent':False,'configuration_writes_during_test':False}
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('phases','fc_checks')},indent=2))
    if error:raise SystemExit(1)


if __name__=='__main__':main()
