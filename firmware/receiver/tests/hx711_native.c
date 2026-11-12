/* Runs PRODUCTION hx711.c tasks on a POSIX fake HAL/queue. All GPIO, ADC and
 * network outcomes are synthetic. Never opens a network socket or serial port. */
#include "hx711_fakes/hx711_native_shim.h"
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <assert.h>
#include "../main/control.h"
#include "../main/hx711.c"
struct fake_queue {pthread_mutex_t mutex;pthread_cond_t cond;unsigned capacity,size,start,count;char *items;};
static const char *scenario;
static atomic_bool stopping;
static pthread_t threads[2];static unsigned thread_count;
static int64_t origin,next_ready;
static uint32_t cycles;
static unsigned pulses,high_level,completed,hardware_resets,network_attempts,network_success;
static bool clock_faulted,blocked_once;
static uint64_t nanos(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000000+t.tv_nsec;}
int64_t esp_timer_get_time(void){return (int64_t)(nanos()/1000)-origin;}
static bool mode(const char *value){return strcmp(scenario,value)==0;}
uint32_t esp_random(void){static uint32_t n=0x12345678;return ++n;}
uint32_t esp_cpu_get_cycle_count(void){return cycles;}
void esp_rom_delay_us(unsigned us){
    cycles+=us*240;
    if(us>=60){++hardware_resets;pulses=0;next_ready=esp_timer_get_time()+11073;}
    if(us==2 && mode("clock_fault") && !clock_faulted && esp_timer_get_time()>700000){cycles+=46*240;clock_faulted=true;}
}
int gpio_config(const gpio_config_t *config){(void)config;return mode("gpio_fail")?-1:0;}
int gpio_set_level(int pin,int value){
    if(pin!=2)return 0;
    high_level=(unsigned)value;
    if(value)++pulses;
    else if(pulses==25){++completed;pulses=0;next_ready=esp_timer_get_time()+11073;}
    return 0;
}
int gpio_get_level(int pin){
    assert(pin==1);
    if(mode("missing"))return 1;
    if(high_level && pulses>0 && pulses<=24){
        uint32_t raw=mode("saturation")?0x7fffff:0xfffe0c; /* signed -500 */
        return (raw>>(24-pulses))&1;
    }
    if(mode("stuck"))return 0;
    return esp_timer_get_time()<next_ready;
}
void vTaskDelay(unsigned ticks){
    if(atomic_load(&stopping))pthread_exit(NULL);
    struct timespec delay={.tv_sec=ticks/1000,.tv_nsec=(ticks%1000)*1000000};nanosleep(&delay,NULL);
}
void vTaskDelete(void *task){(void)task;pthread_exit(NULL);}
struct launch {void(*run)(void*);void *arg;};
static void *entry(void *arg){struct launch *l=arg;void(*run)(void*)=l->run;void *context=l->arg;free(l);run(context);return NULL;}
int xTaskCreate(void(*run)(void*),const char *name,unsigned stack,void *arg,unsigned priority,void *handle){
    (void)handle;assert(stack==4096);
    assert((strcmp(name,"hx711_read")==0 && priority==2)||(strcmp(name,"hx711_udp")==0 && priority==1));
    if((mode("read_task_fail") && priority==2)||(mode("udp_task_fail") && priority==1))return 0;
    assert(thread_count<2);struct launch *l=malloc(sizeof(*l));*l=(struct launch){run,arg};
    if(pthread_create(&threads[thread_count],NULL,entry,l)!=0)abort();++thread_count;return 1;
}
QueueHandle_t xQueueCreate(unsigned count,unsigned size){
    if(mode("queue_fail"))return NULL;
    struct fake_queue *q=calloc(1,sizeof(*q));q->capacity=count;q->size=size;q->items=calloc(count,size);
    pthread_mutex_init(&q->mutex,NULL);pthread_cond_init(&q->cond,NULL);return q;
}
int xQueueSend(QueueHandle_t q,const void *item,unsigned wait){
    assert(wait==0);pthread_mutex_lock(&q->mutex);
    if(q->count==q->capacity){pthread_mutex_unlock(&q->mutex);return 0;}
    memcpy(q->items+((q->start+q->count)%q->capacity)*q->size,item,q->size);++q->count;
    pthread_cond_signal(&q->cond);pthread_mutex_unlock(&q->mutex);return 1;
}
int xQueueReceive(QueueHandle_t q,void *item,unsigned wait){
    if(atomic_load(&stopping))pthread_exit(NULL);
    pthread_mutex_lock(&q->mutex);
    if(!q->count && wait){struct timespec until;clock_gettime(CLOCK_REALTIME,&until);until.tv_nsec+=(long)wait*1000000;until.tv_sec+=until.tv_nsec/1000000000;until.tv_nsec%=1000000000;pthread_cond_timedwait(&q->cond,&q->mutex,&until);}
    if(!q->count){pthread_mutex_unlock(&q->mutex);return 0;}
    memcpy(item,q->items+q->start*q->size,q->size);q->start=(q->start+1)%q->capacity;--q->count;
    pthread_mutex_unlock(&q->mutex);return 1;
}
void vQueueDelete(QueueHandle_t q){pthread_mutex_destroy(&q->mutex);pthread_cond_destroy(&q->cond);free(q->items);free(q);}
int hx_socket(int domain,int type,int protocol){
    assert(domain==AF_INET && type==SOCK_DGRAM && protocol==IPPROTO_UDP);
    if(mode("socket_fail") && esp_timer_get_time()<700000){errno=EMFILE;return -1;}return 42;
}
int hx_fcntl(int fd,int command,...){assert(fd==42 && command==F_SETFL);return 0;}
int hx_close(int fd){assert(fd==42);return 0;}
ssize_t hx_sendto(int fd,const void *data,size_t length,int flags,const struct sockaddr *to,socklen_t size){
    (void)to;(void)size;assert(fd==42 && flags==0);++network_attempts;
    if(mode("unreachable")){errno=ENETUNREACH;return -1;}
    if(mode("congested") && esp_timer_get_time()<700000){errno=EAGAIN;return -1;}
    if(mode("blocked") && !blocked_once){blocked_once=true;struct timespec delay={0,250000000};nanosleep(&delay,NULL);}
    ++network_success;fwrite(data,1,length,stdout);fputc('\n',stdout);return (ssize_t)length;
}
int main(int argc,char **argv){
    assert(argc==2);scenario=argv[1];origin=(int64_t)(nanos()/1000);next_ready=11073;
    sz_hx711_start(0xabcdef0123456789ULL,17);
    /* Synthetic normal receiver state/output workload. No UART or command socket.
     * Sensor tasks never receive its address. POSIX is not FreeRTOS timing proof. */
    struct sz_state control;sz_init(&control,9,SZ_CTBR,123);
    struct sz_command cmd={.device=9,.mode=SZ_CTBR,.epoch=123,.session=3,.ttl_ms=1000,.lease_ms=1000,.values={1500,1500,1500,1000}};
    unsigned ticks=0;uint64_t last=nanos(),max_gap=0;
    while(esp_timer_get_time()<1800000){
        uint64_t now=nanos();if(now-last>max_gap)max_gap=now-last;last=now;
        uint32_t ms=(uint32_t)(esp_timer_get_time()/1000);cmd.sequence++;cmd.lease_ms=ms;
        assert(sz_accept(&control,&cmd,ms));uint16_t values[4];sz_output(&control,ms,values);
        assert(!control.enabled && values[3]==1000);uint8_t frame[22];assert(sz_rc_msp(values,false,frame)==22);
        ++ticks;struct timespec delay={0,2000000};nanosleep(&delay,NULL);
    }
    atomic_store(&stopping,true);for(unsigned i=0;i<thread_count;++i)pthread_join(threads[i],NULL);
    if(samples)vQueueDelete(samples);
    printf("{\"native_summary\":true,\"scenario\":\"%s\",\"control_ticks\":%u,\"control_max_gap_us\":%.3f,\"reads\":%u,\"resets\":%u,\"send_attempts\":%u,\"sends\":%u}\n",scenario,ticks,max_gap/1000.,completed,hardware_resets,network_attempts,network_success);
    return 0;
}
