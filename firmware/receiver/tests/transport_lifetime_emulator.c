/* Loopback-only regression: a DROP publication must not cross a teardown lock. */
#define _POSIX_C_SOURCE 200809L
#include "zenoh-pico.h"
#include "zenoh-pico/session/utils.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
static void pause_ms(unsigned ms){struct timespec t={.tv_sec=ms/1000,.tv_nsec=(ms%1000)*1000000};nanosleep(&t,NULL);}
static unsigned long long now_us(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (unsigned long long)t.tv_sec*1000000+t.tv_nsec/1000;}
struct blocker { _z_session_t *session; atomic_bool held; };
static void *hold(void *arg){struct blocker *b=arg;_z_session_transport_mutex_lock(b->session);atomic_store(&b->held,true);pause_ms(250);_z_session_transport_mutex_unlock(b->session);return NULL;}
int main(int argc,char **argv){
    if(argc!=2 || (strncmp(argv[1],"tcp/127.0.0.1:",14) && strncmp(argv[1],"udp/127.0.0.1:",14)))return 2;
    z_owned_config_t config;z_config_default(&config);
    zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MODE_KEY,"client");
    zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_CONNECT_KEY,argv[1]);
    zp_config_insert(z_config_loan_mut(&config),Z_CONFIG_MULTICAST_SCOUTING_KEY,"false");
    z_owned_session_t session;if(z_open(&session,z_config_move(&config),NULL)<0)return 3;
    z_view_keyexpr_t key;z_view_keyexpr_from_str(&key,"guard/probe");
    z_publisher_options_t options;z_publisher_options_default(&options);options.congestion_control=Z_CONGESTION_CONTROL_DROP;options.is_express=true;
    z_owned_publisher_t pub;if(z_declare_publisher(z_session_loan(&session),&pub,z_view_keyexpr_loan(&key),&options)<0)return 4;
    z_matching_status_t matching={0};unsigned tries=0;
    while(tries++<300){if(z_publisher_get_matching_status(z_publisher_loan(&pub),&matching)>=0 && matching.matching)break;pause_ms(10);}
    if(!matching.matching)return 5;
    struct blocker b={.session=_Z_RC_IN_VAL(z_session_loan(&session))};pthread_t thread;
    if(pthread_create(&thread,NULL,hold,&b))return 6;
    while(!atomic_load(&b.held))pause_ms(1);
    z_owned_bytes_t bytes;z_bytes_copy_from_buf(&bytes,(const uint8_t *)"blocked",7);
    unsigned long long begin=now_us();int result=z_publisher_put(z_publisher_loan(&pub),z_bytes_move(&bytes),NULL);unsigned long long elapsed=now_us()-begin;
    pthread_join(thread,NULL);
    if(result>=0 || elapsed>50000){fprintf(stderr,"publication crossed teardown guard: result=%d elapsed_us=%llu\n",result,elapsed);return 7;}
    z_bytes_copy_from_buf(&bytes,(const uint8_t *)"after",5);if(z_publisher_put(z_publisher_loan(&pub),z_bytes_move(&bytes),NULL)<0)return 8;
    pause_ms(50);if(z_close(z_session_loan_mut(&session),NULL)<0)return 9;
    z_publisher_drop(z_publisher_move(&pub));z_session_drop(z_session_move(&session));
    printf("{\"guarded_drop\":true,\"elapsed_us\":%llu,\"post_release_send\":true}\n",elapsed);return 0;
}
