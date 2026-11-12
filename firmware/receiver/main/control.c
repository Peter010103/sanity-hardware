#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif
#include "control.h"
#include "actuator_profile.h"
#include <string.h>
#include <math.h>
static uint16_t u16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t u32(const uint8_t *p) { return ((uint32_t)u16(p) << 16) | u16(p + 2); }
static uint64_t u64(const uint8_t *p) { return ((uint64_t)u32(p) << 32) | u32(p + 4); }
static void put16(uint8_t *p, uint16_t v) {
    p[0] = v >> 8;
    p[1] = v;
}
static void put32(uint8_t *p, uint32_t v) {
    put16(p, v >> 16);
    put16(p + 2, v);
}
static void neutral(uint8_t mode, uint16_t v[4]) {
    for (unsigned i = 0; i < 4; ++i)
        v[i] = (mode == SZ_CTBR || mode == SZ_PAPER_NATIVE) && i < 3 ? 1500 : 1000;
}
void sz_init(struct sz_state *s, uint8_t device, uint8_t mode, uint64_t epoch) {
    memset(s, 0, sizeof(*s));
    s->device = device;
    s->mode = mode;
    s->epoch = epoch;
    s->rearm_required = true;
}
bool sz_decode(const uint8_t *p, size_t n, struct sz_command *c) {
    if (n == SZ_PAPER_COMMAND_SIZE) return sz_paper_decode(p,n,c);
    if (n != SZ_COMMAND_SIZE)
        return false;
    bool flip_selection = !memcmp(p, "SZC6", 4) && p[4] == 6;
    bool high_rate = flip_selection || (!memcmp(p, "SZC3", 4) && p[4] == 3);
    bool selection = flip_selection || (!memcmp(p, "SZC4", 4) && p[4] == 4);
    bool physical = selection || high_rate || (!memcmp(p, "SZC2", 4) && p[4] == 2);
    if ((!physical && (memcmp(p, "SZC1", 4) || p[4] != 1)) ||
        (p[5] != SZ_CTBR && p[5] != SZ_DIRECT) || !p[6] || (selection ? (p[7] & ~3) || ((p[7]&2) && !(p[7]&1)) : p[7] > 1) ||
        (!physical && u16(p + 30)) || (physical && (p[5] != SZ_CTBR ||
        (p[7] && u16(p + 30) != (uint16_t)SZ_CALIBRATION_ID))))
        return false;
    memset(c, 0, sizeof(*c));
    c->mode = p[5];
    c->device = p[6];
    c->enabled = (p[7] & 1) != 0;
    c->controller_selection = selection;
    c->indi_requested = selection && (p[7] & 2);
    c->physical_units = physical;
    c->high_rate = high_rate;
    c->epoch = u64(p + 8);
    c->session = u32(p + 16);
    c->sequence = u32(p + 20);
    c->lease_ms = u32(p + 24);
    c->ttl_ms = u16(p + 28);
    if (!c->epoch || !c->session || !c->ttl_ms || c->ttl_ms > SZ_MAX_LEASE_MS)
        return false;
    if (physical) {
        const int32_t rate_limits[3] = {flip_selection ? 9500 : high_rate ? 2300 : 23000,high_rate ? 9500 : 23000,flip_selection ? 9500 : high_rate ? 2600 : 26000};
        for (unsigned i=0;i<3;++i) {
            uint16_t raw = u16(p+32+2*i);
            int32_t rate = raw < 32768 ? raw : (int32_t)raw - 65536;
            if (rate < -rate_limits[i] || rate > rate_limits[i] || (!c->enabled && rate))
                return false;
            float offset = (float)rate * (i==2 ? -1.0f : 1.0f) * (high_rate ? 0.0477464829275686f : 0.00477464829275686f);
            c->values[i] = (uint16_t)(1500 + (int)(offset+(offset<0 ? -0.5f : 0.5f)));
        }
        uint32_t requested = u16(p+38);
        if (requested > 32000 || (!c->enabled && requested))
            return false;
        c->values[3] = 1000;
        c->thrust_q=(uint16_t)requested;
        return true;
    }
    uint16_t stopped[4];
    neutral(c->mode, stopped);
    for (unsigned i = 0; i < 4; ++i) {
        c->values[i] = u16(p + 32 + 2 * i);
        if (c->values[i] < 1000 || c->values[i] > 2000)
            return false;
        if (c->mode == SZ_DIRECT && c->values[i] != 1000 &&
            (c->values[i] < 1250 || c->values[i] > 1450))
            return false;
        if (!c->enabled && c->values[i] != stopped[i])
            return false;
    }
    return true;
}
bool sz_model_voltage_valid(uint16_t mv) {
    return mv >= SZ_MODEL_VOLTAGE_MIN_MV && mv <= SZ_MODEL_VOLTAGE_MAX_MV;
}
static double model_force(unsigned rc, double voltage) {
    double q=((double)rc-1500.0)/400.0, z=(voltage-7.4)/0.8;
    const double *c=sz_speed_coefficients;
    double speed=c[0]+c[1]*q+c[2]*z+c[3]*q*q+c[4]*q*z+c[5]*z*z;
    return 4.0*SZ_RPM_KT*speed*speed;
}
bool sz_calibrate(struct sz_command *c, uint16_t mv) {
    if (!c->physical_units || !c->thrust_q) return true;
    uint16_t model_mv=mv ? mv : SZ_MODEL_FALLBACK_VOLTAGE_MV;
    if (model_mv<SZ_MODEL_VOLTAGE_MIN_MV) model_mv=SZ_MODEL_VOLTAGE_MIN_MV;
    if (model_mv>SZ_MODEL_VOLTAGE_MAX_MV) model_mv=SZ_MODEL_VOLTAGE_MAX_MV;
    double voltage=(double)model_mv/1000.0;
    unsigned lo=SZ_MODEL_RC_MIN, hi=SZ_MODEL_RC_MAX;
    double target=(double)c->thrust_q/10000.0;
    if (target<=model_force(lo,voltage)) hi=lo;
    else if (target>=model_force(hi,voltage)) lo=hi;
    else while (hi-lo>1) {
        unsigned mid=(lo+hi)/2;
        if (model_force(mid,voltage)<=target) lo=mid;
        else hi=mid;
    }
    c->values[3]=(uint16_t)(lo==hi || 2.0*target<model_force(lo,voltage)+model_force(hi,voltage) ? lo : hi);
    return true;
}
void sz_stop(struct sz_state *s) {
    s->enabled = false;
    s->rearm_required = true;
}
bool sz_output(struct sz_state *s, uint32_t now, uint16_t values[4]) {
    bool expired =
        !s->valid || now - s->command.lease_ms >= s->command.ttl_ms || now - s->last_rx_ms >= SZ_MAX_LEASE_MS;
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
    if (expired && s->valid && !s->rearm_required)
        sz_diag_expiry(s,now);
#endif
    if (expired)
        sz_stop(s);
    if (!expired && s->enabled)
        memcpy(values, s->command.values, sizeof(s->command.values));
    else
        neutral(s->mode, values);
    return expired;
}
bool sz_profile_allows(const struct sz_command *c) {
#ifdef CONFIG_SANITY_COMMS_TEST_ONLY
    if(c->enabled) return false;
#endif
#if defined(CONFIG_SANITY_ESTIMATOR_PID_HOVER) || defined(CONFIG_SANITY_ESTIMATOR_INDI_HOVER)
    if(c->mode != SZ_CTBR || !c->controller_selection)
        return false;
#endif
#ifdef CONFIG_SANITY_ESTIMATOR_PID_HOVER
    if(c->indi_requested)
        return false;
#endif
#if defined(ESP_PLATFORM) && !defined(CONFIG_SANITY_PAPER_NATIVE)
    if(c->paper_native) return false;
#endif
#ifdef CONFIG_SANITY_PAPER_NATIVE
    if(c->mode != SZ_PAPER_NATIVE && (c->mode != SZ_CTBR || !c->controller_selection)) return false;
#endif
    (void)c;
    return true;
}
bool sz_accept(struct sz_state *s, const struct sz_command *c, uint32_t now) {
    if(!sz_profile_allows(c)) {++s->rejected;return false;}
    uint16_t values[4];
    sz_output(s, now, values);
    bool ok = c->device == s->device && c->mode == s->mode && c->epoch == s->epoch &&
              now - c->lease_ms < c->ttl_ms;
    if (s->valid && c->session == s->command.session) {
        uint32_t delta = c->sequence - s->command.sequence;
        ok = ok && delta && delta < 0x80000000u;
    } else if (c->enabled)
        ok = false;
    bool zero = (c->mode == SZ_CTBR || c->mode == SZ_PAPER_NATIVE) ? c->values[3] == 1000
                                   : c->values[0] == 1000 && c->values[1] == 1000 &&
                                         c->values[2] == 1000 && c->values[3] == 1000;
    if (c->enabled && (s->rearm_required || (!s->enabled && !zero)))
        ok = false;
    if (!ok) {
        ++s->rejected;
        return false;
    }
    s->command = *c;
    s->last_rx_ms = now;
    s->valid = true;
    s->enabled = c->enabled;
    if (!c->enabled)
        s->rearm_required = false;
    return true;
}
/* Unified MSP mode changes are disarmed-only and require a fresh disabled claim.
 * Keep the connection epoch/clock and rejection counter; invalidate the old lease.
 */
bool sz_accept_msp(struct sz_state *s, const struct sz_command *c, uint32_t now, bool fc_disarmed) {
    if(!sz_profile_allows(c)) {++s->rejected;return false;}
    if (c->mode != s->mode) {
        uint16_t stopped[4];
        sz_output(s, now, stopped);
        if (!fc_disarmed || s->enabled || c->enabled || c->device != s->device ||
            c->epoch != s->epoch || now - c->lease_ms >= c->ttl_ms ||
            (c->mode != SZ_CTBR && c->mode != SZ_DIRECT && c->mode != SZ_PAPER_NATIVE)) {
            ++s->rejected;
            return false;
        }
        /* Announce a new epoch before accepting the new controller's lease.
         * Buffered commands from the old mode can never become active again. */
        uint64_t epoch = s->epoch + 1;
        uint32_t rejected = s->rejected;
        sz_init(s, s->device, c->mode, epoch ? epoch : 1);
        s->rejected = rejected;
        return false;
    }
    return sz_accept(s, c, now);
}
void sz_status(struct sz_state *s, uint32_t now, bool ready, uint16_t mv, uint16_t age,
               uint8_t out[SZ_STATUS_SIZE]) {
    uint16_t values[4];
    bool expired = sz_output(s, now, values);
    memcpy(out, "SZS2", 4);
    out[4] = 2;
    out[5] = s->mode;
    out[6] = s->device;
    out[7] = (s->enabled ? 1 : 0) | (expired || s->rearm_required ? 2 : 0) | (ready ? 4 : 0) |
        (s->mode==SZ_CTBR && s->command.physical_units && !sz_model_voltage_valid(mv) ? 8 : 0) |
        (s->valid && s->command.high_rate ? 16 : 0) |
        (s->valid && s->command.high_rate && s->command.controller_selection ? 64 : 0);
#ifdef CONFIG_SANITY_MSP_TUNING
    out[7]|=32;
#endif
    put32(out + 8, s->epoch >> 32);
    put32(out + 12, s->epoch);
    put32(out + 16, now);
    put32(out + 20, s->valid ? s->command.session : 0);
    put32(out + 24, s->valid ? s->command.sequence : 0);
    put32(out + 28, s->rejected);
    put16(out + 32, mv);
    put16(out + 34, age);
    put32(out + 36, s->mode == SZ_CTBR ? SZ_CALIBRATION_ID : 0);
    for (unsigned i=0;i<4;++i) put16(out+40+2*i,values[i]);
}
void sz_sbus(const uint16_t v[4], bool enabled, bool failsafe, uint8_t out[25]) {
    uint16_t channels[16];
    for (unsigned i = 0; i < 16; ++i)
        channels[i] = 1000;
    channels[0] = v[0];
    channels[1] = v[1];
    channels[2] = v[3];
    channels[3] = v[2];
    channels[4] = enabled ? 2000 : 1000;
    channels[5] = 2000;
    memset(out, 0, 25);
    out[0] = 0x0f;
    for (unsigned i = 0; i < 16; ++i) {
        uint16_t raw = 192 + (channels[i] - 1000) * 1601 / 1000;
        for (unsigned bit = 0; bit < 11; ++bit)
            out[1 + (i * 11 + bit) / 8] |= ((raw >> bit) & 1u) << ((i * 11 + bit) % 8);
    }
    out[23] = failsafe ? 8 : 0;
}
size_t sz_msp(uint8_t cmd, const uint8_t *payload, uint8_t len, uint8_t *out) {
    memcpy(out, "$M<", 3);
    out[3] = len;
    out[4] = cmd;
    uint8_t crc = len ^ cmd;
    for (unsigned i = 0; i < len; ++i) {
        out[5 + i] = payload[i];
        crc ^= payload[i];
    }
    out[5 + len] = crc;
    return 6 + (size_t)len;
}
size_t sz_motors(const uint16_t v[4], uint8_t out[14]) {
    uint8_t payload[8];
    for (unsigned i = 0; i < 4; ++i) {
        payload[2 * i] = v[i];
        payload[2 * i + 1] = v[i] >> 8;
    }
    return sz_msp(214, payload, 8, out);
}

size_t sz_rc_msp_select(const uint16_t v[4], bool enabled, bool indi_requested, uint8_t out[22]) {
    /* Same AETR channel order as the verified SBUS mapping; ARM=AUX1, AUX2=Acro. */
    uint16_t channels[8] = {1500, 1500, 1000, 1500, 1000, 2000, 1000, 1000};
    if (enabled) {
        channels[0] = v[0]; channels[1] = v[1]; channels[2] = v[3]; channels[3] = v[2];
        channels[4] = 2000;
        channels[6] = indi_requested ? 2000 : 1000;
    }
    uint8_t payload[16];
    for (unsigned i = 0; i < 8; ++i) {
        payload[2*i] = channels[i]; payload[2*i+1] = channels[i] >> 8;
    }
    return sz_msp(200, payload, sizeof(payload), out);
}
size_t sz_rc_msp(const uint16_t v[4], bool enabled, uint8_t out[22]) {
    return sz_rc_msp_select(v,enabled,false,out);
}

/* PIN1 wire uses IEEE754 binary32 in network order. All loads are aligned local
 * copies; values[] is solely the legacy PID/authority companion, never alpha. */
static float paper_float(const uint8_t *p) { uint32_t bits=u32(p);float f;memcpy(&f,&bits,4);return f; }
static void paper_put_float(uint8_t *p,float f) {uint32_t bits;memcpy(&bits,&f,4);put32(p,bits);}
bool sz_paper_decode(const uint8_t *p,size_t n,struct sz_command *c) {
    if(n!=64 || memcmp(p,"SZC5",4) || p[4]!=5 || p[5]!=SZ_PAPER_NATIVE || !p[6] ||
       (p[7]&~3) || ((p[7]&2) && !(p[7]&1)) || p[30]!=1 || p[31] || u32(p+32)!=SZ_PAPER_MODEL_ID) return false;
    memset(c,0,sizeof(*c));c->paper_native=true;c->physical_units=true;c->controller_selection=true;
    c->mode=SZ_PAPER_NATIVE;c->device=p[6];c->enabled=p[7]&1;c->indi_requested=p[7]&2;
    c->epoch=u64(p+8);c->session=u32(p+16);c->sequence=u32(p+20);c->lease_ms=u32(p+24);c->ttl_ms=u16(p+28);
    if(!c->epoch || !c->session || !c->ttl_ms || c->ttl_ms>SZ_MAX_LEASE_MS) return false;
    c->thrust_n=paper_float(p+36);
    if(!isfinite(c->thrust_n)||c->thrust_n<0||c->thrust_n>4||(!c->enabled&&c->thrust_n!=0))return false;
    c->thrust_q=(uint16_t)(c->thrust_n*10000.0f+0.5f);c->values[3]=1000;
    for(unsigned i=0;i<3;i++) {
        float a=paper_float(p+40+4*i),r=paper_float(p+52+4*i),limit=i==2?2.6f:2.3f;
        if(!isfinite(a)||!isfinite(r)||fabsf(a)>1000||fabsf(r)>limit||(!c->enabled&&(a!=0||r!=0)))return false;
        c->alpha_rad_s2[i]=a;c->fallback_rate_rad_s[i]=r;
        /* FRD -> existing BF RC convention, only for PID companion stream. */
        float offset=r*(i==0?1.0f:-1.0f)*47.7464829275686f;
        c->values[i]=(uint16_t)(1500+(int)(offset+(offset<0?-0.5f:0.5f)));
    }
    return true;
}
static uint8_t paper_crc(const uint8_t *p,size_t n) {
    uint8_t crc=0;for(size_t i=0;i<n;i++){crc^=p[i];for(unsigned b=0;b<8;b++)crc=(uint8_t)((crc<<1)^((crc&128)?0xd5:0));}return crc;
}
static size_t paper_wrap(uint16_t id,const uint8_t *payload,uint8_t n,uint8_t *out) {
    uint8_t inner[70];inner[0]=0;inner[1]=id;inner[2]=id>>8;inner[3]=n;inner[4]=0;
    if(n)memcpy(inner+5,payload,n);
    inner[5+n]=paper_crc(inner,5+n);return sz_msp(255,inner,(uint8_t)(n+6),out);
}
size_t sz_paper_query(uint8_t out[12]) {return paper_wrap(SZ_PAPER_STATUS,NULL,0,out);}
bool sz_paper_capable(const uint8_t *p,size_t n) {
    if(n!=86)return false;
    if(p[9]==1) {for(unsigned i=81;i<85;i++)if(p[i])return false;}
    else if(p[9]==2) {if((p[81]&~0x8f) || (p[82]&0xf0) || (p[83]&0xf0) || (p[84]&0xf0))return false;}
    else return false;
    return n==86 && !p[0] && p[1]==0x16 && p[2]==0x7f && p[3]==80 && !p[4] &&
        paper_crc(p,n-1)==p[n-1] && !memcmp(p+5,"SIPS",4) && p[10]==1 && p[11]==1 &&
        (p[12]&1) && !(p[12]&128) && u32(p+13)==SZ_PAPER_MODEL_ID;
}
size_t sz_paper_msp(const struct sz_command *c,uint32_t now,uint8_t out[76]) {
    if(!c->paper_native)return 0;
    uint32_t age=now-c->lease_ms;
    if(c->enabled && age>=c->ttl_ms)return 0;
    uint8_t p[64]={0};memcpy(p,"SIP1",4);p[4]=1;p[5]=3;p[6]=c->device;p[7]=(c->enabled?1:0)|(c->enabled&&c->indi_requested?2:0);
    put32(p+8,c->epoch>>32);put32(p+12,c->epoch);put32(p+16,c->session);put32(p+20,c->sequence);
    put32(p+24,c->enabled && age<c->ttl_ms?c->ttl_ms-age:0);put16(p+28,c->ttl_ms);p[30]=1;put32(p+32,SZ_PAPER_MODEL_ID);
    paper_put_float(p+36,c->enabled?c->thrust_n:0);
    for(unsigned i=0;i<3;i++){paper_put_float(p+40+4*i,c->enabled?c->alpha_rad_s2[i]:0);paper_put_float(p+52+4*i,c->enabled?c->fallback_rate_rad_s[i]:0);}
    return paper_wrap(SZ_PAPER_SET,p,64,out);
}

void sz_paper_observe(struct sz_paper_link *l,uint64_t epoch,uint32_t now,const uint8_t *p,size_t n) {
    if(l->epoch!=epoch)l->negotiated=false;
    l->epoch=epoch;l->observed_ms=now;l->capable=sz_paper_capable(p,n);
    if(!l->capable)l->negotiated=false;
}
bool sz_paper_admit(const struct sz_paper_link *l,uint64_t epoch,uint32_t now) {
    return l->epoch==epoch && (l->negotiated || (l->capable && now-l->observed_ms<150));
}
bool sz_paper_forward(const struct sz_paper_link *l,uint64_t epoch,bool enabled) {
    return l->epoch==epoch && (l->negotiated || (!enabled && l->capable));
}
size_t sz_paper_after_rc(const struct sz_command *c,uint32_t now,unsigned expected,int written,uint8_t out[76]) {
    if(c->enabled && (!expected || written<0 || (unsigned)written!=expected))return 0;
    return sz_paper_msp(c,now,out);
}
