"""Receiver-local arrival windows; avoids inferring command gaps from sampled traces."""
from collections import defaultdict


def summarize_windows(events, session=None):
    contexts=defaultdict(dict)
    profiles=set()
    for row in events:
        for record in row.get('records', []):
            if record.get('type') != 8 or (session is not None and record['session'] != session):
                continue
            if (not record['epoch'] or not record['session'] or record['code'] not in (0,1,2)
                    or record['t1'] <= record['t0'] or record['t2'] > record['t3']
                    or record['arg1'] > record['tick']):
                raise ValueError('Invalid receiver arrival window')
            contexts[(record['epoch'],record['session'])][record['t0']]=record
            profiles.add(record['code'])
    windows=[];holes=[]
    for context, values in contexts.items():
        group=sorted(values.values(), key=lambda r:r['t0'])
        for before,after in zip(group,group[1:]):
            if after['t0'] != before['t1']:
                holes.append({'epoch':context[0],'session':context[1], 'gap_us':after['t0']-before['t1']})
        windows.extend(group)
    return {'received':bool(windows),'windows':len(windows),'contexts':len(contexts),
            'profiles':sorted(profiles),'accepted_commands':sum(w['tick'] for w in windows),
            'rejected_commands':sum(w['arg0'] for w in windows),
            'maximum_accepted_gap_ms':max((w['t2']/1000 for w in windows),default=None),
            'gaps_over_50ms':sum(w['arg1'] for w in windows),'report_holes':holes,
            'contiguous':bool(windows) and not holes,
            'complete_for_received_context':bool(windows) and not holes and len(contexts)==1,
            'largest_report_span_ms':max(((w['t1']-w['t0'])/1000 for w in windows),default=None),
            'scope':'Actual ESP accepted-callback intervals, not one-way latency. No per-gap median/p99 is inferred from window maxima. Missing windows or context changes invalidate a complete-run maximum.'}
