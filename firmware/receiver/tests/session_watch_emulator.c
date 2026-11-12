/* Exercise pinned-Pico filtered publishers, callbacks and reconnect cleanup. */
#define _POSIX_C_SOURCE 200809L
#include "session_watch.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
static uint64_t millis(void) {struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000+t.tv_nsec/1000000;}
static void pause_ms(unsigned ms) {struct timespec t={.tv_sec=ms/1000,.tv_nsec=(ms%1000)*1000000};nanosleep(&t,NULL);}
static unsigned callbacks;
static void received(z_loaned_sample_t *sample,void *context) {
    (void)sample;(void)context;
    __atomic_add_fetch(&callbacks,1,__ATOMIC_RELAXED);
}
int main(int argc,char **argv) {
    if(argc<2 || argc>3)return 2;
    int partial=argc==3?atoi(argv[2]):-1;
    uint64_t started=millis();unsigned generation=0;
    while(millis()-started<90000) {
        z_owned_config_t config;z_config_default(&config);
        zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MODE_KEY,"client");
        zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_CONNECT_KEY,argv[1]);
        zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MULTICAST_SCOUTING_KEY,"false");
        z_owned_session_t session;if(z_open(&session,z_config_move(&config),NULL)<0){pause_ms(100);continue;}
        z_owned_publisher_t pub,trace;
        z_owned_subscriber_t subscriber;
        z_internal_null(&pub);z_internal_null(&trace);z_internal_null(&subscriber);
        struct sz_session_watch watch;z_owned_transport_events_listener_t listener;
        bool watched=false;
        if(partial==0)goto cleanup;
        z_publisher_options_t options;z_publisher_options_default(&options);options.congestion_control=Z_CONGESTION_CONTROL_DROP;options.is_express=true;
        z_view_keyexpr_t key;z_view_keyexpr_from_str(&key,"restart/probe");
        if(z_declare_publisher(z_session_loan(&session),&pub,z_view_keyexpr_loan(&key),&options)<0)return 3;
        if(partial==1)goto cleanup;
        z_view_keyexpr_from_str(&key,"restart/trace");
        if(z_declare_publisher(z_session_loan(&session),&trace,z_view_keyexpr_loan(&key),&options)<0)return 3;
        if(partial==2)goto cleanup;
        z_view_keyexpr_from_str(&key,"restart/input");
        z_owned_closure_sample_t closure;z_closure_sample(&closure,received,NULL,NULL);
        if(z_declare_subscriber(z_session_loan(&session),&subscriber,z_view_keyexpr_loan(&key),z_closure_sample_move(&closure),NULL)<0)return 3;
        if(partial==3)goto cleanup;
        watched=sz_session_watch_start(z_session_loan(&session),&watch,&listener)>=0;
        if(!watched)return 4;
        if(partial==4)goto cleanup;
        ++generation;printf("{\"generation\":%u,\"event\":\"opened\"}\n",generation);fflush(stdout);
        while(sz_session_watch_connected(&watch) && !z_session_is_closed(z_session_loan(&session)) && millis()-started<90000) {
            z_owned_bytes_t bytes;z_bytes_copy_from_buf(&bytes,(const uint8_t*)&generation,sizeof(generation));z_publisher_put(z_publisher_loan(&pub),z_bytes_move(&bytes),NULL);
            z_bytes_copy_from_buf(&bytes,(const uint8_t*)&generation,sizeof(generation));z_publisher_put(z_publisher_loan(&trace),z_bytes_move(&bytes),NULL);pause_ms(20);
        }
        printf("{\"generation\":%u,\"event\":\"reopen\",\"session_closed\":%s}\n",generation,z_session_is_closed(z_session_loan(&session))?"true":"false");fflush(stdout);
cleanup:
        if(z_close(z_session_loan_mut(&session),NULL)<0)return 5;
        if(!z_session_is_closed(z_session_loan(&session)))return 6;
        unsigned stopped=__atomic_load_n(&callbacks,__ATOMIC_RELAXED);
        pause_ms(10);
        if(stopped!=__atomic_load_n(&callbacks,__ATOMIC_RELAXED))return 7;
        printf("{\"event\":\"closed\",\"callbacks\":%u}\n",stopped);fflush(stdout);
        if(watched)z_transport_events_listener_drop(z_transport_events_listener_move(&listener));
        z_publisher_drop(z_publisher_move(&trace));
        z_publisher_drop(z_publisher_move(&pub));
        z_subscriber_drop(z_subscriber_move(&subscriber));
        z_session_drop(z_session_move(&session));
        if(partial>=0){printf("{\"event\":\"partial_closed\",\"stage\":%d}\n",partial);return 0;}
        pause_ms(100);
    }
    return 0;
}
