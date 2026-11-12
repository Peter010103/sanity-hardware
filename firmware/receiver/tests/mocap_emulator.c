/* Host-run Pico endpoint using the exact ESP wire validator/cache implementation. */
#define _POSIX_C_SOURCE 200809L
#include "mocap_state.h"
#include "control.h"
#include "zenoh-pico.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;
static struct sz_mocap_cache cache;
static struct sz_state control;
static uint64_t nonce,host,received;
static uint32_t version;
static uint64_t now_us(void) {struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;}
static void receive(z_loaned_sample_t *sample,void *unused) {
    (void)unused;uint8_t bytes[SZ_MOCAP_SIZE];struct sz_mocap_sample value;
    const z_loaned_bytes_t *payload=z_sample_payload(sample);size_t n=z_bytes_len(payload);bool ok=false;
    if(n==sizeof(bytes)) {z_bytes_reader_t reader=z_bytes_get_reader(payload);ok=z_bytes_reader_read(&reader,bytes,n)==n && sz_mocap_decode(bytes,n,9,&value);}
    pthread_mutex_lock(&mutex);if(ok)sz_mocap_accept(&cache,&value,now_us());else cache.rejected++;pthread_mutex_unlock(&mutex);
}
static void clock_receive(z_loaned_sample_t *sample,void *unused) {
    (void)unused;uint64_t at=now_us(),n,h;uint8_t bytes[SZ_MOCAP_CLOCK_REQUEST_SIZE];
    const z_loaned_bytes_t *payload=z_sample_payload(sample);if(z_bytes_len(payload)!=sizeof(bytes))return;
    z_bytes_reader_t reader=z_bytes_get_reader(payload);
    if(z_bytes_reader_read(&reader,bytes,sizeof(bytes))!=sizeof(bytes) || !sz_mocap_clock_request(bytes,sizeof(bytes),9,&n,&h))return;
    pthread_mutex_lock(&mutex);nonce=n;host=h;received=at;version++;pthread_mutex_unlock(&mutex);
}
static void command_receive(z_loaned_sample_t *sample,void *unused) {
    (void)unused;uint8_t bytes[SZ_COMMAND_SIZE];struct sz_command command;
    const z_loaned_bytes_t *payload=z_sample_payload(sample);if(z_bytes_len(payload)!=sizeof(bytes))return;
    z_bytes_reader_t reader=z_bytes_get_reader(payload);
    if(z_bytes_reader_read(&reader,bytes,sizeof(bytes))!=sizeof(bytes) || !sz_decode(bytes,sizeof(bytes),&command))return;
    /* Emulator is disarmed-only, even if an unexpected test publisher requests ARM. */
    pthread_mutex_lock(&mutex);if(!command.enabled)sz_accept(&control,&command,(uint32_t)(now_us()/1000));pthread_mutex_unlock(&mutex);
}
static void publish(z_owned_publisher_t *pub,const uint8_t *bytes,size_t n) {
    z_owned_bytes_t payload;z_bytes_copy_from_buf(&payload,bytes,n);z_publisher_put(z_publisher_loan(pub),z_bytes_move(&payload),NULL);
}
int main(int argc,char **argv) {
    if(argc<2)return 2;uint64_t epoch=argc>2?strtoull(argv[2],NULL,10):12345;
    sz_mocap_init(&cache,9,epoch);sz_init(&control,9,SZ_CTBR,epoch);
    z_owned_config_t config;z_config_default(&config);
    zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MODE_KEY,"client");
    zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_CONNECT_KEY,argv[1]);
    zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MULTICAST_SCOUTING_KEY,"false");
    z_owned_session_t session;if(z_open(&session,z_config_move(&config),NULL)<0)return 3;
    z_owned_subscriber_t subs[3];z_owned_publisher_t pubs[3];
    const char *inputs[]={"sanity/09/state/mocap","sanity/09/state/mocap/clock/request","sanity/09/command/ctbr"};
    const char *outputs[]={"sanity/09/state/mocap/status","sanity/09/state/mocap/clock/reply","sanity/09/status"};
    for(unsigned i=0;i<3;i++) {
        z_view_keyexpr_t key;z_view_keyexpr_from_str(&key,inputs[i]);z_owned_closure_sample_t closure;
        z_closure_sample(&closure,i==0?receive:i==1?clock_receive:command_receive,NULL,NULL);
        if(z_declare_subscriber(z_session_loan(&session),&subs[i],z_view_keyexpr_loan(&key),z_closure_sample_move(&closure),NULL)<0)return 4;
        z_publisher_options_t options;z_publisher_options_default(&options);options.congestion_control=Z_CONGESTION_CONTROL_DROP;options.is_express=true;
        z_view_keyexpr_from_str(&key,outputs[i]);
        if(z_declare_publisher(z_session_loan(&session),&pubs[i],z_view_keyexpr_loan(&key),&options)<0)return 5;
    }
    puts("ready");fflush(stdout);uint32_t sent=0;uint64_t end=now_us()+30000000;
    while(now_us()<end && !z_session_is_closed(z_session_loan(&session))) {
        struct sz_mocap_cache snapshot;uint64_t n,h,r;uint32_t v;uint8_t status[SZ_STATUS_SIZE];
        pthread_mutex_lock(&mutex);snapshot=cache;n=nonce;h=host;r=received;v=version;
        sz_status(&control,(uint32_t)(now_us()/1000),false,0,0,status);pthread_mutex_unlock(&mutex);
        if(n && v!=sent) {uint8_t reply[SZ_MOCAP_CLOCK_REPLY_SIZE];sz_mocap_clock_reply(9,epoch,n,h,r,now_us(),reply);publish(&pubs[1],reply,sizeof(reply));sent=v;}
        uint8_t data[SZ_MOCAP_STATUS_SIZE];sz_mocap_status(&snapshot,now_us(),data);publish(&pubs[0],data,sizeof(data));publish(&pubs[2],status,sizeof(status));
        struct timespec delay={.tv_nsec=5000000};nanosleep(&delay,NULL);
    }
    if(z_close(z_session_loan_mut(&session),NULL)<0)return 6;
    for(unsigned i=0;i<3;i++){z_subscriber_drop(z_subscriber_move(&subs[i]));z_publisher_drop(z_publisher_move(&pubs[i]));}
    z_session_drop(z_session_move(&session));return 0;
}
