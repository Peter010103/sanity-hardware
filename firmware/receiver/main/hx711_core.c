#include "hx711_core.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
void sz_hx_reset_done(struct sz_hx_state *s, int64_t now) {
    s->reset_us=now; s->settling_until_us=now+SZ_HX_SETTLING_US;
    s->discard=SZ_HX_DISCARD;
}
void sz_hx_init(struct sz_hx_state *s, int64_t now) {
    memset(s,0,sizeof(*s));s->resets=1;sz_hx_reset_done(s,now);
}
bool sz_hx_step(struct sz_hx_state *s, bool have, int32_t raw, int64_t ready,
    int64_t lower, int64_t shift_end, uint32_t high, bool stuck, int64_t now,
    struct sz_hx_sample *out, bool *reset) {
    *reset=false;
    if (!have && now-s->last_packet_us<1000000) return false;
    memset(out,0,sizeof(*out));
    if (!have) out->flags=SZ_HX_MISSING;
    else {
        out->raw=raw;out->ready_us=ready;out->ready_lower_us=lower;
        out->shift_end_us=shift_end;out->max_high_us=high;
        if (high>SZ_HX_HIGH_LIMIT_US || stuck) {
            out->flags|=SZ_HX_CLOCK;*reset=true;++s->resets;
            sz_hx_reset_done(s,now);
        }
        if (raw==8388607 || raw==-8388608) out->flags|=SZ_HX_SATURATION;
        if (now<s->settling_until_us || s->discard) out->flags|=SZ_HX_SETTLING;
        if (s->discard && !*reset) --s->discard;
    }
    out->sequence=s->sequence++;out->resets=s->resets;
    out->reset_us=s->reset_us;out->acquired_us=now;s->last_packet_us=now;
    return true;
}
int sz_hx_format(char *out, size_t size, const struct sz_hx_sample *s,
    unsigned device, uint64_t boot, uint64_t receiver_boot,
    uint32_t receiver_boot_count, int64_t prepare, uint32_t send_drops,
    uint32_t queue_drops) {
    return snprintf(out,size,
        "{\"schema\":\"sanity_hx711_v1\",\"device_id\":%u,"
        "\"boot_id\":\"%016" PRIx64 "\",\"seq\":%" PRIu32 ","
        "\"ready_us\":%" PRId64 ",\"read_us\":%" PRId64 ","
        "\"raw\":%" PRId32 ",\"flags\":%" PRIu32 ",\"max_high_us\":%" PRIu32 ","
        "\"send_drops\":%" PRIu32 ",\"resets\":%" PRIu32 ","
        "\"dout_gpio\":1,\"sck_gpio\":2,\"channel\":\"A\",\"gain\":128,"
        "\"timing_v\":1,\"receiver_boot_id\":\"%016" PRIx64 "\","
        "\"receiver_boot_count\":%" PRIu32 ",\"ready_lower_us\":%" PRId64 ","
        "\"shift_end_us\":%" PRId64 ",\"acquired_us\":%" PRId64 ","
        "\"reset_us\":%" PRId64 ",\"queue_drops\":%" PRIu32 "}",
        device,boot,s->sequence,s->ready_us,prepare,s->raw,s->flags,s->max_high_us,
        send_drops,s->resets,receiver_boot,receiver_boot_count,s->ready_lower_us,
        s->shift_end_us,s->acquired_us,s->reset_us,queue_drops);
}
