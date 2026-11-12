#include "fc_telemetry.h"
#include "control.h"
#include <string.h>
static struct sz_fc_parser parser;
static struct sz_fc_record record;
void test_fc_reset(void) {sz_fc_init(&parser);memset(&record,0,sizeof(record));}
int test_fc_feed(const uint8_t *bytes,unsigned length,uint64_t now) {
    int count=0;
    for(unsigned i=0;i<length;++i) if(sz_fc_byte(&parser,bytes[i],now,&record)) ++count;
    return count;
}
int test_fc_query(unsigned command,uint64_t now) {
    bool allowed=sz_fc_query(&parser,(uint8_t)command,now,&record);
    return allowed?1:record.type==SZ_FC_TIMEOUT?2:0;
}
unsigned test_fc_packet(uint8_t *out,unsigned capacity) {
    if(capacity<SZ_FC_HEADER_SIZE)return 0;
    record.epoch=UINT64_C(0x123456789abcdef0);
    sz_fc_batch_header(out,1,1,7);
    size_t n=sz_fc_encode(&record,out+SZ_FC_HEADER_SIZE,capacity-SZ_FC_HEADER_SIZE);
    return n?(unsigned)n+SZ_FC_HEADER_SIZE:0;
}
unsigned test_fc_malformed(void) {return parser.malformed;}
unsigned test_fc_skipped(void) {return parser.skipped;}
int test_fc_is_controller(void) {return sz_fc_is_controller_record(&record);}
uint64_t test_fc_query_start(void) {return record.start_us;}
void test_fc_output(uint64_t now) {
    uint16_t values[4]={1550,1450,1600,1350};uint8_t bytes[22];
    size_t n=sz_rc_msp(values,true,bytes);
    record=(struct sz_fc_record){.type=SZ_FC_OUTPUT,.command=200,.flags=1,.session=12,.sequence=99,
        .command_rx_us=now-1000,.start_us=now,.end_us=now+100,.length=(uint8_t)(n-6)};
    memcpy(record.payload,bytes+5,n-6);
}
void test_fc_health(unsigned reason) {
    struct sz_fc_health h={.boot_id=UINT64_C(0xfedcba9876543210),.boot_count=3,.reset_reason=reason,
        .free_heap=200000,.min_free_heap=180000,.disconnects=2,.connects=3,.disconnect_reason=200,
        .last_disconnect_us=UINT64_C(9999999999),.rssi=-62,.channel=6};
    sz_fc_health_record(&record,&h,100,UINT64_C(10000000000));
}
