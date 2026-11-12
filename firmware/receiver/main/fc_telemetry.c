#include "fc_telemetry.h"
#include <string.h>

static int query_slot(uint8_t command) {
    switch (command) {
    case SZ_FC_PAPER_QUERY_SLOT: return 6;
    case 102: return 0;
    case 104: return 1;
    case 139: return 2;
    case 105: return 3;
    case 255: return 4; /* Native MSPv2-over-v1 read-only controller snapshot. */
    case SZ_FC_TIMING_QUERY_SLOT: return 5; /* Independent optional timing query. */
    default: return -1;
    }
}
static void wrapped_query(uint8_t low,uint8_t out[6]) {
    out[0]=0;out[1]=low;out[2]=0x7f;out[3]=0;out[4]=0;
    uint8_t crc=0;
    for(unsigned i=0;i<5;i++) {
        crc^=out[i];for(unsigned bit=0;bit<8;bit++)crc=(uint8_t)((crc<<1)^((crc&0x80)?0xd5:0));
    }
    out[5]=crc;
}
void sz_fc_controller_query_payload(uint8_t out[6]) {wrapped_query(0x11,out);}
void sz_fc_timing_query_payload(uint8_t out[6]) {wrapped_query(0x14,out);}
bool sz_fc_is_controller_record(const struct sz_fc_record *r) {
    return r->type==SZ_FC_RESPONSE && r->command==255 && r->length>=6 &&
        r->payload[1]==0x11 && r->payload[2]==0x7f;
}
static void put(uint8_t *out, uint64_t value, unsigned length) {
    for (unsigned i = 0; i < length; ++i) out[i] = (uint8_t)(value >> (8 * (length - 1 - i)));
}
void sz_fc_init(struct sz_fc_parser *p) { memset(p, 0, sizeof(*p)); }

bool sz_fc_query(struct sz_fc_parser *p, uint8_t command, uint64_t now_us, struct sz_fc_record *timeout) {
    int slot = query_slot(command);
    memset(timeout, 0, sizeof(*timeout));
    if (slot < 0 || now_us < p->retry_after[slot]) return false;
    if (p->outstanding[slot]) {
        if (now_us - p->pending[slot] >= 100000) {
            *timeout = (struct sz_fc_record){.type=SZ_FC_TIMEOUT,.command=command,.start_us=p->pending[slot],.end_us=now_us};
            p->outstanding[slot] = false;
            p->retry_after[slot] = now_us + 1000000;
        }
        ++p->skipped;
        return false;
    }
    p->pending[slot] = now_us;
    p->outstanding[slot] = true;
    return true;
}

bool sz_fc_byte(struct sz_fc_parser *p, uint8_t byte, uint64_t now_us, struct sz_fc_record *record) {
    if (p->state && now_us - p->first_us > 100000) { p->state=0; ++p->malformed; }
    switch (p->state) {
    case 0:
        if (byte == '$') { p->state=1; p->first_us=now_us; }
        break;
    case 1:
        p->state=byte=='M'?2:byte=='$'?1:0;
        if (byte=='$') p->first_us=now_us;
        break;
    case 2:
        p->direction=byte; p->state=(byte=='>' || byte=='!')?3:0;
        break;
    case 3:
        p->length=byte; p->index=0; p->checksum=byte; p->state=4;
        break;
    case 4:
        p->command=byte; p->checksum^=byte; p->state=p->length?5:6;
        break;
    case 5:
        p->payload[p->index++]=byte; p->checksum^=byte;
        if (p->index==p->length) p->state=6;
        break;
    case 6: {
        p->state=0;
        if (byte!=p->checksum) { ++p->malformed; return false; }
        int slot=query_slot(p->command);
        if(p->command==255 && p->length>=6 && p->payload[1]==0x16 && p->payload[2]==0x7f)slot=6;
        if(p->command==255 && p->length>=6 && p->payload[1]==0x15 && p->payload[2]==0x7f)slot=-1;
        if(p->command==255 && p->length>=6 && p->payload[1]==0x14 && p->payload[2]==0x7f)slot=5;
        /* Old firmware can return an empty outer-MSP error for0x7f14. Never
         * let that optional request consume controller query state/backoff. */
        if(p->command==255 && p->direction=='!' && !p->length && p->outstanding[6])slot=6;
        else if(p->command==255 && p->direction=='!' && !p->length && p->outstanding[5])slot=5;
        if (slot<0 && p->command!=255 && p->command!=130 && p->command!=200 && p->command!=94 && p->command!=95 && p->command!=112 && p->command!=150 && p->command!=202 && p->command!=250) return false;
        if (p->length>SZ_FC_PAYLOAD_MAX) { ++p->malformed; return false; }
        *record=(struct sz_fc_record){.type=p->direction=='!'?SZ_FC_ERROR:SZ_FC_RESPONSE,
            .command=p->command,.length=p->length,.start_us=0,.end_us=now_us,.command_rx_us=p->first_us};
        memcpy(record->payload,p->payload,p->length);
        if (slot>=0 && p->outstanding[slot]) {
            record->start_us=p->pending[slot]; record->flags=2;
            p->outstanding[slot]=false;
        }
        if (slot>=0 && p->direction=='!') p->retry_after[slot]=now_us+1000000;
        return true;
    }
    default: p->state=0; break;
    }
    return false;
}

size_t sz_fc_encode(const struct sz_fc_record *r, uint8_t *out, size_t capacity) {
    size_t length=SZ_FC_RECORD_HEADER_SIZE+r->length;
    if (r->length>SZ_FC_PAYLOAD_MAX || capacity<length) return 0;
    out[0]=r->type;out[1]=r->command;out[2]=r->flags;out[3]=r->length;
    put(out+4,r->epoch,8);put(out+12,r->session,4);put(out+16,r->sequence,4);
    put(out+20,r->command_rx_us,8);put(out+28,r->start_us,8);put(out+36,r->end_us,8);
    memcpy(out+SZ_FC_RECORD_HEADER_SIZE,r->payload,r->length);
    return length;
}
void sz_fc_batch_header(uint8_t *out, uint8_t device, uint16_t count, uint32_t dropped) {
    memcpy(out,"SZF1",4);out[4]=1;out[5]=device;put(out+6,count,2);put(out+8,dropped,4);
}

void sz_fc_health_record(struct sz_fc_record *r, const struct sz_fc_health *h, uint64_t epoch, uint64_t now_us) {
    *r=(struct sz_fc_record){.type=SZ_FC_HEALTH,.epoch=epoch,.end_us=now_us,.length=48};
    put(r->payload,h->boot_id,8);put(r->payload+8,h->boot_count,4);put(r->payload+12,h->reset_reason,4);
    put(r->payload+16,h->free_heap,4);put(r->payload+20,h->min_free_heap,4);
    put(r->payload+24,h->disconnects,4);put(r->payload+28,h->connects,4);put(r->payload+32,h->disconnect_reason,4);
    put(r->payload+36,h->last_disconnect_us,8);r->payload[44]=(uint8_t)h->rssi;r->payload[45]=h->channel;
}
