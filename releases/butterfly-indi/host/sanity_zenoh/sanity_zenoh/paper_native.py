"""Explicit FRD T/alpha publisher/replay. No tracker, MPC or attitude conversion."""
import argparse
import json
import math
from pathlib import Path
import time
from sanity_zenoh.protocol import PAPER_NATIVE, PAPER_MODEL_ID, PaperDemand, paper_command_packet


def load_replay(path):
    data=json.loads(Path(path).read_text())
    if data.get('schema')!='PIN1-replay-v1' or data.get('frame')!='FRD' or data.get('model_contract_id')!=PAPER_MODEL_ID:
        raise ValueError('Explicit PIN1 FRD replay schema/model required')
    rows=data['commands'];last=-1.
    if not rows:raise ValueError('Nonempty replay required')
    for row in rows:
        t=row['time_s']
        if last>=0 and t-last>.8:raise ValueError('Replay gap exceeds bounded source lease')
        if not math.isfinite(t) or t<0 or t<=last:raise ValueError('Strictly increasing replay times required')
        last=t
        if type(row['enabled']) is not bool or type(row['paper_requested']) is not bool:raise ValueError('Explicit boolean flags required')
        demand=PaperDemand(row['thrust_n'],tuple(row['alpha_rad_s2']),tuple(row['fallback_rate_rad_s']))
        paper_command_packet(1,row['enabled'],1,1,0,0,250,demand,row['paper_requested'])
    if rows[0]['enabled'] or rows[-1]['enabled']:raise ValueError('Replay must begin and end disabled')
    return rows


def dry_run(rows,device=15,epoch=1,session=1):
    return [dict(time_s=row['time_s'],packet_hex=paper_command_packet(device,row['enabled'],epoch,session,i,0,250,
        PaperDemand(row['thrust_n'],tuple(row['alpha_rad_s2']),tuple(row['fallback_rate_rad_s'])),row['paper_requested']).hex()) for i,row in enumerate(rows)]


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay',type=Path,required=True)
    parser.add_argument('--device',type=int,default=15)
    parser.add_argument('--execute',action='store_true',help='Open command transport; default dry-run opens no sockets')
    parser.add_argument('--listen',default='tcp/0.0.0.0:7447')
    parser.add_argument('--connect')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv);rows=load_replay(args.replay)
    if not args.execute:
        result={'mode':'dry-run','frame':'FRD','model_contract_id':PAPER_MODEL_ID,'source':'explicit input; no paper tracker','packets':dry_run(rows,args.device)}
        text=json.dumps(result,indent=2)+'\n'
        if args.output:args.output.write_text(text)
        else:print(text,end='')
        return
    from sanity_zenoh.client import SanitySender
    events=[]
    sender=SanitySender(device=args.device,mode=PAPER_NATIVE,listen=args.listen,connect=args.connect,on_event=events.append)
    def wait(predicate,reason):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            if predicate():return
            time.sleep(.01)
        raise RuntimeError(reason)
    try:
        sender.paper_native(False)
        wait(lambda:(s:=sender.receiver_status()) is not None and s.mode==PAPER_NATIVE and s.session==sender.session_id and not s.enabled and not s.failsafe and (cap:=sender.paper_status()) is not None and cap['accepted_epoch']==s.epoch and cap['accepted_session']==sender.session_id,'Fresh disabled claim and FC paper capability not acknowledged')
        started=time.monotonic()
        for index,row in enumerate(rows):
            while time.monotonic()<started+row['time_s']:time.sleep(.002)
            sender.paper_native(row['enabled'],row['thrust_n'],row['alpha_rad_s2'],row['fallback_rate_rad_s'],valid_for_s=min(1.,rows[index+1]['time_s']-row['time_s']+.1) if index+1<len(rows) else .25,paper_requested=row['paper_requested'])
            # Preserve caller's zero-enable acknowledgement before positive T.
            if row['enabled'] and row['thrust_n']==0:
                wait(lambda:(s:=sender.receiver_status()) is not None and s.enabled and s.session==sender.session_id,'Enabled-zero not acknowledged')
    finally:
        sender.close()
        receipt={'frame':'FRD','source':'explicit replay; no tracker','events':events,'shutdown_acknowledged':sender.shutdown_acknowledged}
        if args.output:args.output.write_text(json.dumps(receipt,indent=2)+'\n')

if __name__=='__main__':main()
