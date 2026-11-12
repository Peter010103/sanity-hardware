#include "timing_wire.h"
#include <string.h>
static void p32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> (24 - i * 8));
}
static void p64(uint8_t *p, uint64_t v) {
    p32(p, (uint32_t)(v >> 32)); p32(p + 4, (uint32_t)v);
}
bool sz_probe_reply(const uint8_t *in, size_t n, uint8_t device, uint64_t rx_us,
                    uint64_t tx_us, uint64_t epoch, int8_t rssi, uint8_t channel,
                    uint8_t out[SZ_PONG_SIZE]) {
    if (n != SZ_PROBE_SIZE || memcmp(in, "SZP1", 4) || in[4] != 1 ||
        in[5] != device || in[6] || in[7]) return false;
    memcpy(out, in, SZ_PROBE_SIZE); memcpy(out, "SZR1", 4);
    p64(out + 20, rx_us); p64(out + 28, tx_us); p64(out + 36, epoch);
    out[44] = (uint8_t)rssi; out[45] = channel; out[46] = 0; out[47] = 0;
    return true;
}
void sz_trace_encode(const struct sz_trace *e, uint8_t out[SZ_TRACE_SIZE]) {
    out[0] = e->type; out[1] = e->mode; out[2] = e->flags; out[3] = e->code;
    p32(out+4,e->session); p32(out+8,e->sequence); p32(out+12,e->tick);
    p64(out+16,e->epoch); p64(out+24,e->t0); p64(out+32,e->t1);
    p64(out+40,e->t2); p64(out+48,e->t3); p32(out+56,e->arg0); p32(out+60,e->arg1);
}

void sz_timing_accept(struct sz_timing_window *w, uint64_t epoch, uint32_t session,
                      uint32_t sequence, uint8_t mode, bool accepted, uint64_t now_us) {
    if (!accepted) { if (w->epoch == epoch && w->session == session) ++w->rejected; return; }
    if (w->epoch != epoch || w->session != session) {
        memset(w, 0, sizeof(*w)); w->epoch = epoch; w->session = session; w->start_us = now_us;
    }
    if (w->previous_us && now_us >= w->previous_us) {
        uint64_t gap = now_us - w->previous_us;
        if (gap > w->max_gap_us) { w->max_gap_us = gap; w->max_at_us = now_us; }
        if (gap > 50000) ++w->over_50ms;
    }
    w->previous_us = now_us; w->sequence = sequence; w->mode = mode; ++w->accepted;
}
bool sz_timing_report(struct sz_timing_window *w, uint64_t now_us, uint8_t profile, struct sz_trace *out) {
    if (!w->epoch || now_us < w->start_us || now_us - w->start_us < 200000) return false;
    *out = (struct sz_trace){.type=8,.mode=w->mode,.flags=1,.code=profile,
        .session=w->session,.sequence=w->sequence,.tick=w->accepted,.epoch=w->epoch,
        .t0=w->start_us,.t1=now_us,.t2=w->max_gap_us,.t3=w->max_at_us,
        .arg0=w->rejected,.arg1=w->over_50ms};
    w->start_us=now_us; w->accepted=0; w->rejected=0; w->over_50ms=0;
    w->max_gap_us=0; w->max_at_us=0;
    return true;
}

uint8_t sz_command_reject_reason(const struct sz_state *s,const struct sz_command *c,
                                bool accepted,uint8_t stage,uint32_t now) {
    if (accepted) return SZ_CMD_ACCEPTED;
    if (stage) return stage;
    if (c->device!=s->device) return SZ_CMD_DEVICE;
    if (c->epoch!=s->epoch) return SZ_CMD_EPOCH;
    if (c->mode!=s->mode) return SZ_CMD_MODE;
    if (now-c->lease_ms>=c->ttl_ms) return SZ_CMD_LEASE;
    if (s->valid && c->session==s->command.session) {
        uint32_t delta=c->sequence-s->command.sequence;
        if (!delta || delta>=0x80000000u) return SZ_CMD_SEQUENCE;
    } else if (c->enabled) return SZ_CMD_CLAIM;
    bool expired=!s->valid || now-s->command.lease_ms>=s->command.ttl_ms ||
        now-s->last_rx_ms>=SZ_MAX_LEASE_MS;
    bool zero=c->mode==SZ_CTBR ? c->values[3]==1000 :
        c->values[0]==1000 && c->values[1]==1000 && c->values[2]==1000 && c->values[3]==1000;
    if (c->enabled && (s->rearm_required || expired || (!s->enabled && !zero))) return SZ_CMD_REARM;
    return SZ_CMD_POLICY;
}

struct sz_trace sz_command_decision_trace(const struct sz_command *c,bool decoded,bool accepted,
    uint8_t reason,uint32_t now,uint64_t rx,uint64_t done,uint64_t previous,uint32_t rejected) {
    return (struct sz_trace){.type=9,.mode=c->mode,.flags=(decoded?1:0)|(accepted?2:0)|(c->enabled?4:0),
        .code=reason,.session=c->session,.sequence=c->sequence,.epoch=c->epoch,.tick=now,
        .t0=rx,.t1=done,.t2=c->lease_ms,.t3=previous,.arg0=c->ttl_ms,.arg1=rejected};
}

struct sz_trace sz_command_uart_trace(const struct sz_state *s,uint32_t tick,uint8_t command,
    uint64_t rx,uint64_t snapshot,uint64_t before,uint64_t after,unsigned requested,int returned) {
    return (struct sz_trace){.type=10,.mode=s->mode,
        .flags=(s->valid?1:0)|(s->enabled?2:0)|(returned>=0 && (unsigned)returned==requested?4:0),
        .code=command,.session=s->valid?s->command.session:0,.sequence=s->valid?s->command.sequence:0,
        .epoch=s->epoch,.tick=tick,.t0=rx,.t1=snapshot,.t2=before,.t3=after,
        .arg0=s->valid?(uint32_t)(before/1000)-s->command.lease_ms:0,.arg1=(uint32_t)returned};
}
