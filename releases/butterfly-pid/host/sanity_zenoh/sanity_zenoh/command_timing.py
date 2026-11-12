"""Offline command-path schema1 analysis. Never opens ROS, a socket or hardware.

ESP callback -> decision -> UART API submission is observable. UART return and
FIFO-correlated MSP ACK do not measure a particular command's FC application.
The SZC4 lease field echoes a receiver clock snapshot, not host generation time.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import statistics

REASONS={0:'accepted',1:'decode',2:'profile',3:'calibration',4:'fc_not_ready',
         5:'epoch',6:'device',7:'mode_or_mode_reclaim',8:'lease_not_fresh',
         9:'sequence',10:'disabled_claim_required',11:'rearm_or_initial_output',12:'other_policy'}
MASK32=0xffffffff


def decode_command_trace(record):
    """Annotate new record types; preserve the raw trace alongside annotations."""
    r=record;kind=r['type']
    identity={k:r[k] for k in ('epoch','session','sequence','mode')}
    if kind==9:
        if r['flags']&~7 or r['code'] not in REASONS or not 0<=r['t2']<=MASK32:
            raise ValueError('Invalid command-decision schema1 record')
        if r['t1']<r['t0']:
            raise ValueError('Reversed receiver decision timestamps')
        decoded=bool(r['flags']&1);accepted=bool(r['flags']&2)
        if accepted and (not decoded or r['code']!=0):
            raise ValueError('Inconsistent accepted command-decision record')
        previous=r['t3']
        gap=r['t0']-previous if accepted and previous and r['t0']>=previous else None
        return dict(identity,schema=1,stage='receiver_decision',decoded=decoded,
            accepted=accepted,enabled=bool(r['flags']&4),reason=REASONS[r['code']],
            receiver_rx_us=r['t0'],receiver_decision_us=r['t1'],decision_duration_us=r['t1']-r['t0'],
            receiver_now_ms=r['tick'],echoed_receiver_lease_ms=r['t2'],ttl_ms=r['arg0'],
            lease_age_ms=(r['tick']-r['t2'])&MASK32 if decoded else None,
            previous_accepted_rx_us=previous,accepted_gap_us=gap,rejected_counter=r['arg1'])
    if kind==10:
        if r['flags']&~7 or r['code'] not in (200,214) or r['t3']<r['t2'] or r['t2']<r['t1']:
            raise ValueError('Invalid UART-submission schema1 record')
        returned=r['arg1'] if r['arg1']<2**31 else r['arg1']-2**32
        valid=bool(r['flags']&1)
        return dict(identity,schema=1,stage='uart_submission',output_tick=r['tick'],
            valid=valid,enabled=bool(r['flags']&2),driver_full_write=bool(r['flags']&4),
            msp_command=r['code'],accepted_rx_us=r['t0'],state_snapshot_us=r['t1'],
            uart_api_enter_us=r['t2'],uart_api_return_us=r['t3'],
            received_to_submit_us=r['t2']-r['t0'] if valid and r['t0'] and r['t2']>=r['t0'] else None,
            uart_api_duration_us=r['t3']-r['t2'],lease_age_at_submit_ms=r['arg0'] if valid else None,
            driver_returned_bytes=returned,fc_application_time_us=None,
            boundary='UART driver API enqueue/return; not wire completion or FC acceptance')
    if kind==4 and r['code']==6:
        return dict(identity,schema=1,stage='receiver_expiry',receiver_event_us=r['t0'],
            evaluated_receiver_ms=r['t1'],last_accepted_receiver_ms=r['t2'],
            echoed_receiver_lease_ms=r['t3'],ttl_ms=r['arg0'],
            accepted_rx_age_ms=(r['t1']-r['t2'])&MASK32,
            lease_age_ms=(r['t1']-r['t3'])&MASK32,
            receive_gap_expired=bool(r['arg1']&1),echoed_lease_expired=bool(r['arg1']&2))
    if kind==3:
        return dict(identity,schema=1,stage='msp_ack',msp_command=r['code'],
            association='FIFO assumption only; MSP ACK does not echo SZC4 identity',fc_application_time_us=None)
    return None


def _stats(values):
    values=sorted(v for v in values if v is not None)
    if not values:return {'samples':0,'median':None,'p95':None,'maximum':None}
    p=(len(values)-1)*.95;i=int(p);p95=values[i]+(values[min(i+1,len(values)-1)]-values[i])*(p-i)
    return {'samples':len(values),'median':statistics.median(values),'p95':p95,'maximum':values[-1]}


def summarize_command_timing(events):
    from sanity_zenoh.protocol import COMMAND
    host={};decisions=[];submissions=[];expiry=[];acks=0;errors=[];drops=[]
    for event in events:
        if event.get('kind')=='zenoh_tx_command':
            try:
                h=COMMAND.unpack(bytes.fromhex(event['payload_hex']))
                host[(h[5],h[6],h[7])]=event
            except (ValueError,KeyError):errors.append('Invalid host command record')
        if event.get('kind')!='zenoh_rx_trace':continue
        if 'decode_error' in event:errors.append(event['decode_error']);continue
        if 'dropped' in event:drops.append(event['dropped'])
        for record in event.get('records',[]):
            try:a=decode_command_trace(record)
            except (KeyError,TypeError,ValueError) as exc:errors.append(str(exc));continue
            if not a:continue
            if a['stage']=='receiver_decision':decisions.append(a)
            elif a['stage']=='uart_submission':submissions.append(a)
            elif a['stage']=='receiver_expiry':expiry.append(a)
            elif a['stage']=='msp_ack':acks+=1
    accepted=[r for r in decisions if r['accepted']]
    rejected=[r for r in decisions if not r['accepted']]
    key=lambda r:(r['epoch'],r['session'],r['sequence'])
    contexts=defaultdict(list)
    for r in accepted:contexts[(r['epoch'],r['session'])].append(r)
    missing=0;nonforward=0
    for values in contexts.values():
        values.sort(key=lambda r:r['receiver_rx_us'])
        for a,b in zip(values,values[1:]):
            delta=(b['sequence']-a['sequence'])&MASK32
            if 0<delta<2**31:missing+=delta-1
            else:nonforward+=1
    counter_resets=sum(b<a and not(a>=0xf0000000 and b<0x10000000) for a,b in zip(drops,drops[1:]))
    drop_delta=sum((b-a)&MASK32 for a,b in zip(drops,drops[1:])) if drops and not counter_resets else None
    relevant=[r for r in submissions if r['valid']]
    matched=[r for r in accepted if key(r) in host]
    prepared=[];publication=[];source_to_build=[]
    for r in matched:
        e=host[key(r)]
        if all(k in e for k in ('publication_started_ns','publication_completed_ns')):
            publication.append((e['publication_completed_ns']-e['publication_started_ns'])/1000)
        if all(k in e for k in ('packet_build_started_ns','publication_started_ns')):
            prepared.append((e['publication_started_ns']-e['packet_build_started_ns'])/1000)
        if e.get('source_command_generation_ns') is not None and 'packet_build_started_ns' in e:
            source_to_build.append((e['packet_build_started_ns']-e['source_command_generation_ns'])/1000)
    return {'schema':'sanity_command_timing_analysis_v1',
        'scope':'Host local publication timing and receiver local decision/UART API timing. No cross-clock one-way latency or FC application time inferred.',
        'counts':{'host_commands':len(host),'receiver_decisions':len(decisions),'accepted':len(accepted),
            'rejected':len(rejected),'accepted_matched_to_host':len(matched),'UART_submissions':len(submissions),
            'receiver_expiries':len(expiry),'legacy_FIFO_ACKs':acks,'contexts':len(contexts),
            'missing_sequence_numbers_between_recorded_acceptances':missing,'nonforward_accepted_sequences':nonforward,
            'UART_incomplete_or_error_returns':sum(not r['driver_full_write'] for r in submissions)},
        'rejection_reasons':dict(Counter(r['reason'] for r in rejected)),
        'metrics':{'host_source_to_packet_build_us':_stats(source_to_build),
            'host_build_to_publication_us':_stats(prepared),'host_publication_call_us':_stats(publication),
            'receiver_callback_to_decision_us':_stats(r['decision_duration_us'] for r in accepted),
            'accepted_callback_gap_us':_stats(r['accepted_gap_us'] for r in accepted),
            'echoed_lease_age_at_decision_ms':_stats(r['lease_age_ms'] for r in accepted),
            'received_to_UART_submit_us':_stats(r['received_to_submit_us'] for r in relevant),
            'UART_API_call_us':_stats(r['uart_api_duration_us'] for r in submissions),
            'echoed_lease_age_at_UART_submit_ms':_stats(r['lease_age_at_submit_ms'] for r in relevant)},
        'trace_loss':{'counter_first':drops[0] if drops else None,'counter_last':drops[-1] if drops else None,
            'counter_increase':drop_delta,'counter_resets_or_reordering':counter_resets,'coverage_complete':False,
            'interpretation':'Counter increases and missing sequences may reflect trace loss. Publication loss/reordering is not covered by the ESP queue-drop counter; complete coverage is not inferred.'},
        'matched_command_examples':[dict(epoch=r['epoch'],session=r['session'],sequence=r['sequence'],
            host_publication_started_ns=host[key(r)].get('publication_started_ns'),
            receiver_rx_us=r['receiver_rx_us'],receiver_decision_us=r['receiver_decision_us']) for r in matched[:20]],
        'expiry_events':expiry,'rejection_examples':rejected[:20],'decode_errors':errors,
        'schema1_records_present':bool(decisions or submissions),
        'FC_application_time_available':False,
        'lease_semantics':'Echoed receiver status clock snapshot age, not host generation or network transit age.'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--events',type=Path,required=True)
    p.add_argument('--output',type=Path);a=p.parse_args()
    with a.events.open() as f:result=summarize_command_timing(json.loads(line) for line in f)
    text=json.dumps(result,indent=2)+'\n'
    if a.output:a.output.write_text(text)
    else:print(text,end='')


if __name__=='__main__':main()
