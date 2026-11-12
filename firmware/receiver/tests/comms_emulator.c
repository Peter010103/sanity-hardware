#define _POSIX_C_SOURCE 200809L
#include "control.h"
#include "test_config.h"
#include "fc_telemetry.h"
#include "timing_wire.h"
#include "zenoh-pico.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
static pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;
static struct sz_state state;
static struct sz_trace traces[1024];static unsigned count;static uint32_t dropped;
static bool telemetry=true,reply_pending,ping_pending;
static uint8_t reply[SZ_TEST_CONFIG_SIZE],ping[SZ_PROBE_SIZE];
static uint64_t command_rx,ping_rx,started,pause_after,pause_length;static bool paused;
static uint64_t micros(void) {struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;}
static void delay_ms(unsigned ms) {struct timespec t={.tv_sec=ms/1000,.tv_nsec=(ms%1000)*1000000};nanosleep(&t,NULL);}
static void push(struct sz_trace trace) {if(count<1024)traces[count++]=trace;else ++dropped;}
static void receive(z_loaned_sample_t *sample,void *context) {
    (void)context;uint64_t entered=micros();uint8_t bytes[SZ_COMMAND_SIZE];struct sz_command command;
    const z_loaned_bytes_t *payload=z_sample_payload(sample);if(z_bytes_len(payload)!=sizeof(bytes))return;
    z_bytes_reader_t reader=z_bytes_get_reader(payload);
    if(z_bytes_reader_read(&reader,bytes,sizeof(bytes))!=sizeof(bytes) || !sz_decode(bytes,sizeof(bytes),&command) || !sz_calibrate(&command,7800))return;
    if(pause_after && !paused && entered-started>=pause_after) {paused=true;delay_ms((unsigned)(pause_length/1000));}
    pthread_mutex_lock(&mutex);bool accepted=sz_accept_msp(&state,&command,(uint32_t)(micros()/1000),true);
    if(accepted)command_rx=entered;
    push((struct sz_trace){.type=1,.mode=command.mode,.flags=accepted?1:0,.session=command.session,.sequence=command.sequence,.epoch=command.epoch,.t0=entered,.t1=micros(),.arg1=command.ttl_ms});pthread_mutex_unlock(&mutex);
}
static void configure(z_loaned_sample_t *sample,void *context) {
    (void)context;uint8_t bytes[SZ_TEST_CONFIG_SIZE];const z_loaned_bytes_t *payload=z_sample_payload(sample);
    if(z_bytes_len(payload)!=sizeof(bytes))return;z_bytes_reader_t r=z_bytes_get_reader(payload);
    if(z_bytes_reader_read(&r,bytes,sizeof(bytes))!=sizeof(bytes))return;
    pthread_mutex_lock(&mutex);int result=sz_test_config(bytes,sizeof(bytes),1,state.command.session,state.valid,state.enabled,telemetry,reply);
    if(result>=0) {if(result)telemetry=bytes[6]!=0;reply_pending=true;}pthread_mutex_unlock(&mutex);
}
static void probe(z_loaned_sample_t *sample,void *context) {
    (void)context;const z_loaned_bytes_t *payload=z_sample_payload(sample);if(z_bytes_len(payload)!=SZ_PROBE_SIZE)return;
    uint8_t bytes[SZ_PROBE_SIZE];z_bytes_reader_t r=z_bytes_get_reader(payload);if(z_bytes_reader_read(&r,bytes,sizeof(bytes))!=sizeof(bytes))return;
    pthread_mutex_lock(&mutex);memcpy(ping,bytes,sizeof(bytes));ping_rx=micros();ping_pending=true;pthread_mutex_unlock(&mutex);
}
static void publish(const z_loaned_publisher_t *pub,const uint8_t *bytes,size_t length) {z_owned_bytes_t payload;z_bytes_copy_from_buf(&payload,bytes,length);z_publisher_put(pub,z_bytes_move(&payload),NULL);}
int main(int argc,char **argv) {
    if(argc!=4 && argc!=6)return 2;
    started=micros();unsigned duration=(unsigned)(atof(argv[3])*1000);if(argc==6){pause_after=(uint64_t)(atof(argv[4])*1000);pause_length=(uint64_t)(atof(argv[5])*1000);}
    sz_init(&state,1,SZ_CTBR,((uint64_t)started<<8)|(unsigned)getpid());
    z_owned_config_t config;z_config_default(&config);zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MODE_KEY,"client");zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_CONNECT_KEY,argv[1]);zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MULTICAST_SCOUTING_KEY,"false");
    z_owned_session_t session;if(z_open(&session,z_config_move(&config),NULL)<0)return 3;
    bool split=strcmp(argv[2],"-")!=0;z_owned_session_t telemetry_session;
    if(split) {z_config_default(&config);zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MODE_KEY,"client");zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_CONNECT_KEY,argv[2]);zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MULTICAST_SCOUTING_KEY,"false");if(z_open(&telemetry_session,z_config_move(&config),NULL)<0)return 4;}
    z_loaned_session_t *bulk=split?z_session_loan_mut(&telemetry_session):z_session_loan_mut(&session);
    const char *names[]={"sanity/01/status","sanity/01/telemetry/fc","sanity/01/diagnostics/trace","sanity/01/diagnostics/pong","sanity/01/diagnostics/telemetry/ack"};z_owned_publisher_t pubs[5];
    for(unsigned i=0;i<5;++i) {z_view_keyexpr_t key;z_view_keyexpr_from_str(&key,names[i]);z_publisher_options_t opts;z_publisher_options_default(&opts);opts.is_express=true;opts.congestion_control=Z_CONGESTION_CONTROL_DROP;opts.priority=(i==1 || i==2)?Z_PRIORITY_BACKGROUND:Z_PRIORITY_REAL_TIME;
#ifdef Z_FEATURE_UNSTABLE_API
        opts.reliability=Z_RELIABILITY_BEST_EFFORT;
#endif
        if(z_declare_publisher((i==1 || i==2)?bulk:z_session_loan(&session),&pubs[i],z_view_keyexpr_loan(&key),&opts)<0)return 5;
    }
    const char *subs[]={"sanity/01/command/*","sanity/01/diagnostics/telemetry/set","sanity/01/diagnostics/ping"};void(*callbacks[])(z_loaned_sample_t*,void*)={receive,configure,probe};z_owned_subscriber_t subscribers[3];
    for(unsigned i=0;i<3;++i) {z_view_keyexpr_t key;z_view_keyexpr_from_str(&key,subs[i]);z_owned_closure_sample_t cb;z_closure_sample(&cb,callbacks[i],NULL,NULL);if(z_declare_subscriber(z_session_loan(&session),&subscribers[i],z_view_keyexpr_loan(&key),z_closure_sample_move(&cb),NULL)<0)return 6;}
    unsigned tick=0;
    while(micros()-started<(uint64_t)duration*1000) {
        uint8_t status[SZ_STATUS_SIZE],raw[25];uint16_t values[4];uint64_t now=micros();struct sz_state copy;
        pthread_mutex_lock(&mutex);sz_output(&state,(uint32_t)(now/1000),values);copy=state;sz_status(&state,(uint32_t)(now/1000),true,7800,0,status);
        if(state.enabled){pthread_mutex_unlock(&mutex);return 7;}
        if(state.valid)push((struct sz_trace){.type=2,.mode=state.mode,.flags=2,.code=200,.session=state.command.session,.sequence=state.command.sequence,.epoch=state.epoch,.t0=now,.t1=now,.t2=micros()});
        uint8_t ack[SZ_TEST_CONFIG_SIZE],pong[SZ_PONG_SIZE];bool ack_ready=reply_pending,pong_ready=ping_pending;memcpy(ack,reply,sizeof(ack));reply_pending=false;ping_pending=false;bool enabled=telemetry;
        if(pong_ready)sz_probe_reply(ping,sizeof(ping),1,ping_rx,micros(),state.epoch,-50,11,pong);
        uint64_t received=command_rx;pthread_mutex_unlock(&mutex);
        sz_rc_msp(values,false,raw);
        if(ack_ready)publish(z_publisher_loan(&pubs[4]),ack,sizeof(ack));if(pong_ready)publish(z_publisher_loan(&pubs[3]),pong,sizeof(pong));
        if(!(tick++%2)) {
            publish(z_publisher_loan(&pubs[0]),status,sizeof(status));
            uint8_t trace[12+16*SZ_TRACE_SIZE];pthread_mutex_lock(&mutex);unsigned n=count<16?count:16;for(unsigned i=0;i<n;++i)sz_trace_encode(&traces[i],trace+12+i*SZ_TRACE_SIZE);memmove(traces,traces+n,(count-n)*sizeof(*traces));count-=n;memcpy(trace,"SZT1",4);trace[4]=1;trace[5]=1;trace[6]=64;trace[7]=(uint8_t)n;for(unsigned i=0;i<4;++i)trace[8+i]=(uint8_t)(dropped>>(24-8*i));pthread_mutex_unlock(&mutex);if(n)publish(z_publisher_loan(&pubs[2]),trace,12+n*SZ_TRACE_SIZE);
            if(enabled) {
                uint8_t batch[SZ_FC_BATCH_SIZE];size_t length=SZ_FC_HEADER_SIZE;
                struct sz_fc_record r={.type=SZ_FC_OUTPUT,.command=200,.flags=1,.length=16,.epoch=copy.epoch,.session=copy.command.session,.sequence=copy.command.sequence,.command_rx_us=received,.start_us=now,.end_us=now+5};memcpy(r.payload,raw+5,16);length+=sz_fc_encode(&r,batch+length,sizeof(batch)-length);
                memset(&r,0,sizeof(r));r.type=SZ_FC_RESPONSE;r.command=102;r.length=18;r.epoch=copy.epoch;r.end_us=now;length+=sz_fc_encode(&r,batch+length,sizeof(batch)-length);
                sz_fc_batch_header(batch,1,2,0);publish(z_publisher_loan(&pubs[1]),batch,length);
            }
        }
        delay_ms(10);
    }
    for(unsigned i=0;i<3;++i)z_subscriber_drop(z_subscriber_move(&subscribers[i]));for(unsigned i=0;i<5;++i)z_publisher_drop(z_publisher_move(&pubs[i]));if(split)z_session_drop(z_session_move(&telemetry_session));z_session_drop(z_session_move(&session));return 0;
}
