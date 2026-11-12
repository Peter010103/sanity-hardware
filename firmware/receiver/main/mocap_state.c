#include "mocap_state.h"
#include <math.h>
#include <string.h>

static uint32_t get32(const uint8_t *p) {
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
}
static uint64_t get64(const uint8_t *p) {return ((uint64_t)get32(p)<<32)|get32(p+4);}
static void put32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v;
}
static void put64(uint8_t *p, uint64_t v) {put32(p,(uint32_t)(v>>32));put32(p+4,(uint32_t)v);}
static float getfloat(const uint8_t *p) {uint32_t v=get32(p);float f;memcpy(&f,&v,4);return f;}
void sz_mocap_init(struct sz_mocap_cache *c, uint8_t device, uint64_t epoch) {
    memset(c,0,sizeof(*c));c->device=device;c->epoch=epoch;
}
bool sz_mocap_decode(const uint8_t *p,size_t n,uint8_t device,struct sz_mocap_sample *s) {
    if(n!=SZ_MOCAP_SIZE || memcmp(p,"SZM1",4) || p[4]!=1 || p[5]!=device || p[6]!=1 || p[7]!=3)return false;
    memset(s,0,sizeof(*s));s->epoch=get64(p+8);s->session=get32(p+16);s->sequence=get32(p+20);
    s->source_ros_ns=get64(p+24);s->capture_lower_us=get64(p+32);s->capture_upper_us=get64(p+40);
    if(!s->epoch || !s->session || !s->source_ros_ns || !s->capture_lower_us ||
       s->capture_upper_us<s->capture_lower_us || s->capture_upper_us-s->capture_lower_us>SZ_MOCAP_MAX_CLOCK_WIDTH_US)return false;
    float norm=0;
    for(unsigned i=0;i<10;i++) {
        float f=getfloat(p+48+4*i);if(!isfinite(f))return false;
        if(i<3) {if(fabsf(f)>1000.f)return false;s->position[i]=f;}
        else if(i<7) {s->quaternion[i-3]=f;norm+=f*f;}
        else {if(fabsf(f)>100.f)return false;s->velocity[i-7]=f;}
    }
    return fabsf(norm-1.f)<=.002f;
}
bool sz_mocap_usable(const struct sz_mocap_cache *c,uint64_t now) {
    return c->present && now>=c->received_us && now>=c->sample.capture_lower_us &&
        now-c->sample.capture_lower_us<=SZ_MOCAP_MAX_AGE_US;
}
bool sz_mocap_accept(struct sz_mocap_cache *c,const struct sz_mocap_sample *s,uint64_t now) {
    bool ok=s->epoch==c->epoch && now>=s->capture_lower_us && now-s->capture_lower_us<=SZ_MOCAP_MAX_AGE_US;
    if(c->present) {
        /* A frozen/replayed source must never refresh freshness, even across publishers. */
        ok=ok && s->source_ros_ns>c->sample.source_ros_ns && s->capture_lower_us>c->sample.capture_lower_us;
        if(s->session==c->sample.session) {
            uint32_t delta=s->sequence-c->sample.sequence;ok=ok && delta && delta<0x80000000u;
        } else ok=ok && !sz_mocap_usable(c,now);
    }
    if(!ok){c->rejected++;return false;}
    c->sample=*s;c->received_us=now;c->present=true;c->accepted++;return true;
}
void sz_mocap_status(const struct sz_mocap_cache *c,uint64_t now,uint8_t p[SZ_MOCAP_STATUS_SIZE]) {
    memset(p,0,SZ_MOCAP_STATUS_SIZE);memcpy(p,"SZMS",4);p[4]=1;p[5]=c->device;p[6]=1;
    p[7]=(c->present?1:0)|(sz_mocap_usable(c,now)?2:0);put64(p+8,c->epoch);
    put32(p+16,c->sample.session);put32(p+20,c->sample.sequence);put32(p+24,c->accepted);put32(p+28,c->rejected);
    put64(p+32,c->received_us);put64(p+40,c->present && now>=c->sample.capture_lower_us?now-c->sample.capture_lower_us:UINT64_MAX);
}
bool sz_mocap_clock_request(const uint8_t *p,size_t n,uint8_t device,uint64_t *nonce,uint64_t *host) {
    if(n!=SZ_MOCAP_CLOCK_REQUEST_SIZE || memcmp(p,"SZMQ",4) || p[4]!=1 || p[5]!=device || p[6] || p[7])return false;
    *nonce=get64(p+8);*host=get64(p+16);return *nonce && *host;
}
void sz_mocap_clock_reply(uint8_t device,uint64_t epoch,uint64_t nonce,uint64_t host,uint64_t received,uint64_t sent,uint8_t p[SZ_MOCAP_CLOCK_REPLY_SIZE]) {
    memset(p,0,SZ_MOCAP_CLOCK_REPLY_SIZE);memcpy(p,"SZMR",4);p[4]=1;p[5]=device;
    put64(p+8,epoch);put64(p+16,nonce);put64(p+24,host);put64(p+32,received);put64(p+40,sent);
}
