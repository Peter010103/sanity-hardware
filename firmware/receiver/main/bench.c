#include "bench.h"
#include <string.h>
static uint16_t le16(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }
static uint32_t le32(const uint8_t *p) { return le16(p) | ((uint32_t)le16(p + 2) << 16); }
void bench_init(struct bench_state *b) {
    memset(b, 0, sizeof(*b));
    b->arm_bit = -1;
}
static void response(struct bench_state *b, uint32_t now) {
    const uint8_t *p = b->payload;
    unsigned n = b->length;
    int slot = -1;
    bool ok = false;
    switch (b->command) {
    case 1:
        slot = 0;
        ok = n >= 3 && p[1] == 1 && p[2] == 46;
        break;
    case 2:
        slot = 1;
        ok = n == 4 && !memcmp(p, "BTFL", 4);
        break;
    case 3:
        slot = 2;
        ok = n >= 3 && p[0] == 4 && p[1] == 5 && p[2] == 1;
        break;
    case 36:
        slot = 3;
        ok = n >= 4 && !(le32(p) & (1u << 12));
        b->msp_receiver = ok && (le32(p) & (1u << 14)) &&
            !(le32(p) & ((1u << 0) | (1u << 3) | (1u << 13) | (1u << 25)));
        break;
    case 90:
        slot = 4;
        ok = n >= 8 && p[3] == 6;
        break; /* DSHOT300 */
    case 131:
        slot = 5;
        ok = n >= 7 && le16(p + 2) == 2000 && le16(p + 4) == 1000 && p[6] == 4;
        break;
    case 119:
        slot = 6;
        b->arm_bit = -1;
        for (unsigned i = 0; i < n && i < 32; ++i)
            if (p[i] == 0)
                b->arm_bit = (int)i;
        ok = b->arm_bit >= 0;
        break;
    case 101:
        slot = 7;
        ok = n >= 11 && b->arm_bit >= 0;
        b->disarmed = ok && !(le32(p + 6) & (1u << b->arm_bit));
        if(ok)b->pid_profile=p[10];
        break;
    case 130: {
        slot = 8;
        ok = n >= 11 && p[0] > 0;
        uint32_t mv = ok ? (uint32_t)le16(p + 9) * 10u : 0;
        ok = ok && mv > 0 && mv <= UINT16_MAX;
        b->voltage_mv = ok ? (uint16_t)mv : 0;
        b->battery_in_fit = ok && p[0] == 2 && mv >= 7514 && mv <= 8112;
        break;
    }
    case 200:
        b->rc_ack_seen = n == 0;
        b->rc_ack_ms = now;
        return;
    case 214:
        slot = 9;
        ok = n == 0;
        break;
    default:
        return;
    }
    if (ok) {
        b->seen |= 1u << slot;
        b->stamp[slot] = now;
    } else
        b->seen &= ~(1u << slot);
}
void bench_byte(struct bench_state *b, uint8_t v, uint32_t now) {
    switch (b->parse_state) {
    case 0:
        if (v == '$')
            b->parse_state = 1;
        break;
    case 1:
        b->parse_state = v == 'M' ? 2 : (v == '$' ? 1 : 0);
        break;
    case 2:
        b->parse_state = v == '>' ? 3 : 0;
        break;
    case 3:
        b->length = v;
        b->index = 0;
        b->checksum = v;
        b->parse_state = 4;
        break;
    case 4:
        b->command = v;
        b->checksum ^= v;
        b->parse_state = b->length ? 5 : 6;
        break;
    case 5:
        b->payload[b->index++] = v;
        b->checksum ^= v;
        if (b->index == b->length)
            b->parse_state = 6;
        break;
    case 6:
        if (v == b->checksum)
            response(b, now);
        b->parse_state = 0;
        break;
    default:
        b->parse_state = 0;
        break;
    }
}
bool bench_ready(const struct bench_state *b, uint32_t now) {
    if (b->seen != 1023 || !b->disarmed || !b->battery_in_fit)
        return false;
    for (unsigned i = 0; i < 10; ++i)
        if (now - b->stamp[i] >= (i < 7 ? 1500u : 150u))
            return false;
    return true;
}

bool bench_disarmed(const struct bench_state *b, uint32_t now) {
    return (b->seen & (1u << 7)) && now - b->stamp[7] < 150 && b->disarmed;
}
bool bench_ctbr_ready(const struct bench_state *b, uint32_t now) {
    /* CTBR can remain ready while the FC is armed. Direct bench cannot. */
    if ((b->seen & 255) != 255 || !b->msp_receiver || !b->rc_ack_seen ||
        now - b->rc_ack_ms >= 150 || !(b->seen & (1u << 9)) || now - b->stamp[9] >= 150)
        return false;
    for (unsigned i = 0; i < 8; ++i)
        if (now - b->stamp[i] >= (i < 7 ? 1500u : 150u))
            return false;
    return true;
}
