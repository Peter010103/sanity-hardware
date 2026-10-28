"""Causal transport/ADC health only; deliberately NOT a cable-state estimator.

sample_usable means locally well-formed, continuous raw evidence. It establishes
neither a verified zero nor absolute network freshness/mocap alignment.
"""
import math

U32=0xffffffff
TIMING_FIELDS=('receiver_boot_count','ready_lower_us','shift_end_us','acquired_us','reset_us','queue_drops')


def validate_timing_extension(packet):
    if 'timing_v' not in packet:
        if any(k in packet for k in (*TIMING_FIELDS,'receiver_boot_id')):
            raise ValueError('Partial HX711 timing extension')
        return
    if type(packet['timing_v']) is not int or packet['timing_v']!=1:
        raise ValueError('Unsupported HX711 timing extension')
    identity=packet.get('receiver_boot_id')
    if not isinstance(identity,str) or len(identity)!=16:
        raise ValueError('Missing receiver boot association')
    int(identity,16)
    for key in TIMING_FIELDS:
        value=packet.get(key)
        limit=U32 if key in ('receiver_boot_count','queue_drops') else (1<<63)-1
        if type(value) is not int or not 0<=value<=limit:
            raise ValueError('Invalid HX711 extension '+key)
    if not packet['reset_us']<=packet['acquired_us']<=packet['read_us']:
        raise ValueError('Invalid acquisition/dispatch order')
    if packet['flags']&1:
        if packet['ready_us'] or packet['shift_end_us'] or packet['raw']:
            raise ValueError('Missing status presented as a conversion')
    elif not (0<packet['ready_us']<=packet['shift_end_us']<=packet['acquired_us']):
        raise ValueError('Invalid conversion timing')
    if packet['ready_lower_us'] and packet['ready_us'] and packet['ready_lower_us']>packet['ready_us']:
        raise ValueError('Invalid observed readiness bracket')


class HX711Health:
    def __init__(self, max_source_gap_s=.1, max_receive_age_s=.25, max_queue_age_s=.1):
        if any(not math.isfinite(x) or x<=0 for x in (max_source_gap_s,max_receive_age_s,max_queue_age_s)):
            raise ValueError('Positive health limits required')
        self.source_gap_us=max_source_gap_s*1e6
        self.age_ns=max_receive_age_s*1e9
        self.queue_age_us=max_queue_age_s*1e6
        self.previous=None;self.last_usable_ns=None;self.generation=0;self.retired=set()
        self.last_host_ns=None;self.last_reasons=['waiting_for_data']

    def invalidate(self, reason):
        """Decoder/source-validity failure: clear health without refreshing time."""
        self.last_reasons=[str(reason)]

    def check(self, host_ns):
        stale=self.last_usable_ns is None or host_ns-self.last_usable_ns>self.age_ns
        return {'stale':stale,'health_ok':not stale and not self.last_reasons,
                'last_reasons':list(self.last_reasons),'baseline_generation':self.generation,'clock_alignment_verified':False}

    def observe(self, packet, host_ns):
        validate_timing_extension(packet)
        key=(packet['boot_id'],packet.get('receiver_boot_id'))
        previous=self.previous;reasons=[];gap=0;source_dt=None;reset=False
        if key in self.retired:
            self.last_reasons=['retired_boot']
            return dict(sample_usable=False,reasons=['retired_boot'],sequence_gap=0,
                        source_interval_us=None,baseline_invalidated=False,**self.check(host_ns))
        if self.last_host_ns is not None and host_ns<self.last_host_ns:
            self.last_reasons=['host_clock_reversed']
            return dict(sample_usable=False,reasons=['host_clock_reversed'],sequence_gap=0,
                        source_interval_us=None,baseline_invalidated=False,**self.check(host_ns))
        if previous is None or key!=(previous['boot_id'],previous.get('receiver_boot_id')):
            if previous is not None:
                self.retired.add((previous['boot_id'],previous.get('receiver_boot_id')))
                reasons.append('boot_changed')
            previous=None;self.generation+=1;reset=True;self.last_usable_ns=None
        if previous is not None:
            delta=(packet['seq']-previous['seq'])&U32
            if not 0<delta<0x80000000:
                self.last_reasons=['duplicate_or_old']
                return dict(sample_usable=False,reasons=['duplicate_or_old'],sequence_gap=0,
                            source_interval_us=None,baseline_invalidated=False,**self.check(host_ns))
            gap=delta-1
            if gap:reasons.append('sequence_gap')
            if self.last_host_ns is not None and host_ns-self.last_host_ns>self.age_ns:
                reasons.append('receive_gap')
            for field in ('send_drops','queue_drops'):
                if field in packet and field in previous and packet[field]!=previous[field]:
                    reasons.append(field+'_changed')
            if packet['resets']!=previous['resets'] or packet.get('reset_us')!=previous.get('reset_us'):
                reasons.append('converter_reset');self.generation+=1;reset=True;self.last_usable_ns=None
            if packet['ready_us'] and previous['ready_us']:
                source_dt=packet['ready_us']-previous['ready_us']
                if source_dt<=0:reasons.append('source_clock_reversed');self.generation+=1;reset=True
                elif source_dt>self.source_gap_us:reasons.append('source_gap')
            if packet['read_us']<previous['read_us']:
                reasons.append('dispatch_clock_reversed');self.generation+=1;reset=True
        if packet['flags']:reasons.append('ADC_flags')
        queue_age=None
        if 'acquired_us' in packet:
            queue_age=packet['read_us']-packet['acquired_us']
            if queue_age>self.queue_age_us:reasons.append('local_queue_age')
        self.previous=dict(packet);self.last_host_ns=host_ns
        self.last_reasons=reasons
        usable=not reasons
        if usable:self.last_usable_ns=host_ns
        return dict(sample_usable=usable,reasons=reasons,sequence_gap=gap,
                    source_interval_us=source_dt,local_queue_age_us=queue_age,
                    baseline_invalidated=reset,**self.check(host_ns))
