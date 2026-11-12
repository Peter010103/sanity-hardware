#include "sdkconfig.h"
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
#include "diagnostics.h"
#include "telemetry_profile.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>
#if CONFIG_SANITY_DIAGNOSTIC_LEVEL == 0 && (defined(CONFIG_SANITY_COMMS_TEST_ONLY) || defined(CONFIG_SANITY_SPLIT_TELEMETRY))
#define ALL_COMMAND_TRACES 1
#else
#define ALL_COMMAND_TRACES 0
#endif
static QueueHandle_t events,probes;
struct probe {uint8_t data[SZ_PROBE_SIZE];uint64_t rx;};
static uint32_t dropped,ack_overflows[2];
static uint64_t epoch;
static struct sz_timing_window arrival_window;
static portMUX_TYPE arrival_lock=portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE epoch_lock=portMUX_INITIALIZER_UNLOCKED;
static volatile int8_t rssi;
static volatile uint8_t channel;
static struct { struct sz_trace v[32]; unsigned head, count; } pending[2];
static void emit(struct sz_trace *e) {
    if (xQueueSend(events, e, 0) != pdTRUE) {
        __atomic_add_fetch(&dropped, 1, __ATOMIC_RELAXED);
        if(e->type==5) {
            struct sz_trace old;
            if(xQueueReceive(events,&old,0)==pdTRUE && xQueueSend(events,e,0)!=pdTRUE)
                __atomic_add_fetch(&dropped,1,__ATOMIC_RELAXED);
        }
    }
}
void sz_diag_command_timing(const struct sz_trace *event) {
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
    struct sz_trace copy=*event;emit(&copy);
#else
    (void)event;
#endif
}
static struct sz_trace identity(const struct sz_state *s) {
    return (struct sz_trace){.mode=s->mode,.flags=(s->enabled?1:0)|(s->valid?2:0),.session=s->valid?s->command.session:0,
        .sequence=s->valid?s->command.sequence:0,.epoch=s->epoch};
}
void sz_diag_init(void) { events=xQueueCreate(128,sizeof(struct sz_trace));probes=xQueueCreate(8,sizeof(struct probe));if (!events || !probes) abort(); }
void sz_diag_epoch(uint64_t value) { portENTER_CRITICAL(&epoch_lock);epoch=value;portEXIT_CRITICAL(&epoch_lock); }
void sz_diag_command(const struct sz_command *c, bool accepted, uint64_t entered, uint64_t done, uint32_t now) {
    portENTER_CRITICAL(&arrival_lock);
    sz_timing_accept(&arrival_window,c->epoch,c->session,c->sequence,c->mode,accepted,entered);
    portEXIT_CRITICAL(&arrival_lock);
#if CONFIG_SANITY_DIAGNOSTIC_LEVEL != 0
    (void)done; (void)now; return;
#endif
    struct sz_trace e={.type=1,.mode=c->mode,.flags=(accepted?1:0)|(c->enabled?2:0),
        .session=c->session,.sequence=c->sequence,.epoch=c->epoch,.t0=entered,.t1=done,
        .arg0=now-c->lease_ms,.arg1=c->ttl_ms};if (ALL_COMMAND_TRACES || !(c->sequence%10) || !accepted) emit(&e);
}
void sz_diag_write(const struct sz_state *s, uint8_t cmd, uint32_t tick, uint64_t snapshot, uint64_t before, uint64_t after, unsigned bytes) {
#if CONFIG_SANITY_DIAGNOSTIC_LEVEL != 0
    (void)s;(void)cmd;(void)tick;(void)snapshot;(void)before;(void)after;(void)bytes; return;
#endif
    struct sz_trace e=identity(s);e.type=2;e.code=cmd;e.tick=tick;e.t0=snapshot;e.t1=before;e.t2=after;e.arg0=bytes;if ((e.flags&2) && ((ALL_COMMAND_TRACES && ((cmd==200 && e.mode==SZ_CTBR) || (cmd==214 && e.mode==SZ_DIRECT))) || !(e.sequence%10))) emit(&e);
    unsigned i=cmd==200?0:1;
    if (pending[i].count==32) {
        ++ack_overflows[i];
        if(ack_overflows[i]%100==1) {struct sz_trace loss={.type=7,.code=cmd,.arg0=ack_overflows[i],.t0=before};emit(&loss);}
        pending[i].head=(pending[i].head+1)%32;--pending[i].count;
    }
    pending[i].v[(pending[i].head+pending[i].count)%32]=e;++pending[i].count;
}
void sz_diag_ack(uint8_t cmd,uint64_t previous,uint64_t parsed) {
    unsigned i=cmd==200?0:1;
    if (!pending[i].count) return;
    struct sz_trace e=pending[i].v[pending[i].head];pending[i].head=(pending[i].head+1)%32;--pending[i].count;
    e.type=3;e.t0=e.t1;e.t1=e.t2;e.t2=previous;e.t3=parsed;e.arg1=pending[i].count;if ((e.flags&2) && ((ALL_COMMAND_TRACES && ((cmd==200 && e.mode==SZ_CTBR) || (cmd==214 && e.mode==SZ_DIRECT))) || !(e.sequence%10))) emit(&e);
}
void sz_diag_stop(const struct sz_state *s,uint8_t reason,uint16_t seen,uint16_t mv,uint16_t age) {
    struct sz_trace e=identity(s);e.type=4;e.code=reason;e.t0=esp_timer_get_time();e.arg0=seen;e.arg1=((uint32_t)mv<<16)|age;emit(&e);
}
void sz_diag_expiry(const struct sz_state *s,uint32_t evaluated) {
    struct sz_trace e=identity(s);e.type=4;e.code=6;e.t0=esp_timer_get_time();
    e.t1=evaluated;e.t2=s->last_rx_ms;e.t3=s->command.lease_ms;e.arg0=s->command.ttl_ms;
    e.arg1=((evaluated-s->last_rx_ms>=SZ_MAX_LEASE_MS)?1:0)|((evaluated-s->command.lease_ms>=s->command.ttl_ms)?2:0);emit(&e);
}
void sz_diag_poll_gap(uint64_t previous,uint64_t now) {
    struct sz_trace e={.type=6,.t0=previous+10000,.t1=now};emit(&e);
}
void sz_diag_publish_time(uint8_t kind,uint64_t before,uint64_t after,int result) {
#if defined(CONFIG_SANITY_COMMS_TEST_ONLY) || defined(CONFIG_SANITY_SPLIT_TELEMETRY)
#if CONFIG_SANITY_DIAGNOSTIC_LEVEL != 0
    if(after-before<50000 && result>=0)return;
#else
    if(after-before<5000 && result>=0)return;
#endif
#else
    if(after-before<1000 && result>=0)return;
#endif
#if CONFIG_SANITY_DIAGNOSTIC_LEVEL != 0
    static uint64_t next_report[8];
    if(kind>=8)return;
    portENTER_CRITICAL(&arrival_lock);
    bool allowed=after>=next_report[kind];if(allowed)next_report[kind]=after+200000;
    portEXIT_CRITICAL(&arrival_lock);
    if(!allowed)return;
#endif
    struct sz_trace e={.type=5,.code=kind,.t0=before,.t1=after,.arg0=(uint32_t)result};emit(&e);
}
void sz_diag_probe_bytes(const uint8_t *data,size_t size) {
    struct probe p={.rx=esp_timer_get_time()};uint8_t check[SZ_PONG_SIZE];
    if(size!=sizeof(p.data))return;
    memcpy(p.data,data,size);
    if(!sz_probe_reply(p.data,size,CONFIG_SANITY_DEVICE_ID,p.rx,p.rx,1,0,0,check))return;
    if(xQueueSend(probes,&p,0)!=pdTRUE)__atomic_add_fetch(&dropped,1,__ATOMIC_RELAXED);
}
void sz_diag_probe(z_loaned_sample_t *sample,void *context) {
    (void)context;uint8_t data[SZ_PROBE_SIZE];
    const z_loaned_bytes_t *payload=z_sample_payload(sample);
    if(z_bytes_len(payload)!=sizeof(data))return;
    z_bytes_reader_t reader=z_bytes_get_reader(payload);
    if(z_bytes_reader_read(&reader,data,sizeof(data))==sizeof(data))sz_diag_probe_bytes(data,sizeof(data));
}
static int zenoh_send(const uint8_t *data,size_t size,void *context) {
    z_owned_bytes_t payload;z_bytes_copy_from_buf(&payload,data,size);
    return z_publisher_put((const z_loaned_publisher_t *)context,z_bytes_move(&payload),NULL);
}
void sz_diag_flush_pongs_to(sz_diag_send_fn send,void *context) {
    struct probe probe;unsigned replies=0;
    while(replies++<4 && xQueueReceive(probes,&probe,0)==pdTRUE) {
        uint8_t response[SZ_PONG_SIZE];
        portENTER_CRITICAL(&epoch_lock);uint64_t current_epoch=epoch;portEXIT_CRITICAL(&epoch_lock);
        sz_probe_reply(probe.data,sizeof(probe.data),CONFIG_SANITY_DEVICE_ID,probe.rx,esp_timer_get_time(),current_epoch,rssi,channel,response);
        uint64_t before=esp_timer_get_time();int result=send(response,sizeof(response),context);
        sz_diag_publish_time(2,before,esp_timer_get_time(),result);
    }
}
void sz_diag_flush_pongs(const z_loaned_publisher_t *pub) {sz_diag_flush_pongs_to(zenoh_send,(void *)pub);}
void sz_diag_flush_traces_to(sz_diag_send_fn send,void *context) {
    uint8_t out[12+16*SZ_TRACE_SIZE];struct sz_trace e;unsigned n=0;
    portENTER_CRITICAL(&arrival_lock);
    bool report=sz_timing_report(&arrival_window,esp_timer_get_time(),CONFIG_SANITY_DIAGNOSTIC_LEVEL,&e);
    portEXIT_CRITICAL(&arrival_lock);
    if(report)sz_trace_encode(&e,out+12+SZ_TRACE_SIZE*n++);
    unsigned limit=CONFIG_SANITY_DIAGNOSTIC_LEVEL==0?16:4;
#ifdef CONFIG_SANITY_COMMAND_TIMING_TRACE
    /* Two records per100Hz command require >200 records/s drain capacity.
     * The20ms publishing task can drain800/s; all loss remains counted. */
    limit=16;
#endif
    while(n<limit && xQueueReceive(events,&e,0)==pdTRUE)sz_trace_encode(&e,out+12+SZ_TRACE_SIZE*n++);
    static uint64_t next_rssi;
    uint64_t now=esp_timer_get_time();
    if(now>=next_rssi){wifi_ap_record_t ap;if(esp_wifi_sta_get_ap_info(&ap)==ESP_OK){rssi=ap.rssi;channel=ap.primary;}next_rssi=now+1000000;}
    if(!n)return;
    memcpy(out,"SZT1",4);out[4]=1;out[5]=CONFIG_SANITY_DEVICE_ID;out[6]=SZ_TRACE_SIZE;out[7]=n;
    uint32_t d=__atomic_load_n(&dropped,__ATOMIC_RELAXED);for(unsigned i=0;i<4;++i)out[8+i]=(uint8_t)(d>>(24-i*8));
    uint64_t before=esp_timer_get_time();int result=send(out,12+n*SZ_TRACE_SIZE,context);
    sz_diag_publish_time(3,before,esp_timer_get_time(),result);
}
void sz_diag_flush_traces(const z_loaned_publisher_t *pub) {sz_diag_flush_traces_to(zenoh_send,(void *)pub);}
void sz_diag_flush(const z_loaned_publisher_t *pub,const z_loaned_publisher_t *pong_pub) {
    sz_diag_flush_pongs(pong_pub);sz_diag_flush_traces(pub);
}
#endif
