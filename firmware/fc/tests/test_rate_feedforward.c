#include "rate_feedforward.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
static srFfConfig_t cfg(float dt) {
    srFfConfig_t c={.dt=dt,.cutoffHz=20,.gain={.25f,.25f,0},.limit={20,20,20}};
    return c;
}
int main(void) {
    srFfState_t s;srFfConfig_t c=cfg(.001f);float r[3]={1,-1,.5f};
    assert(srFfInit(&s,&c));assert(srFfUpdate(&s,r,true));
    assert(s.seeded && s.primed && s.alpha[0]==0 && s.alpha[1]==0);
    for(unsigned n=1;n<2000;n++) {
        r[0]=1+5*n*c.dt;r[1]=-1-5*n*c.dt;r[2]=.5f+3*n*c.dt;
        assert(srFfUpdate(&s,r,true));
    }
    assert(fabsf(s.alpha[0]-1.25f)<.001f && fabsf(s.alpha[1]+1.25f)<.001f && s.alpha[2]==0);
    r[0]+=20;r[1]-=20;assert(srFfUpdate(&s,r,true));
    assert(s.alpha[0]==20 && s.alpha[1]==-20 && s.clippedMask==3);
    for(unsigned n=0;n<1000;n++)assert(srFfUpdate(&s,r,true));
    assert(fabsf(s.alpha[0])<1e-5f && !s.clippedMask); /* duplicates decay without new kicks */
    unsigned resets=s.resets;assert(srFfUpdate(&s,r,false));
    assert(!s.primed && s.resets==resets+1 && !s.alpha[0]);
    assert(srFfUpdate(&s,r,false));assert(s.resets==resets+1);
    r[0]=-100;assert(srFfUpdate(&s,r,true));assert(s.seeded && s.alpha[0]==0);
    r[1]=NAN;assert(!srFfUpdate(&s,r,true));
    assert(!s.primed && s.alpha[0]==0 && s.alpha[1]==0 && s.derivative[0]==0);
    r[1]=0;r[0]=FLT_MAX;assert(srFfUpdate(&s,r,true));
    r[0]=-FLT_MAX;assert(!srFfUpdate(&s,r,true));assert(!s.primed);
    for(unsigned a=0;a<3;a++)assert(s.raw[a]==0 && s.alpha[a]==0);
    for(unsigned trial=0;trial<5;trial++) {
        c=cfg(.001f);
        if(trial==0)c.dt=.002f;
        if(trial==1)c.cutoffHz=NAN;
        if(trial==2)c.gain[1]=1.001f;
        if(trial==3)c.limit[0]=0;
        if(trial==4)c.gain[2]=-1;
        assert(!srFfInit(&s,&c));assert(!s.configured);
    }
    for(unsigned dtcase=0;dtcase<2;dtcase++) {
        c=cfg(dtcase?.0005f:.001f);c.gain[0]=c.gain[1]=c.gain[2]=0;
        assert(srFfInit(&s,&c));
        for(unsigned n=0;n<10000;n++) {
            for(unsigned a=0;a<3;a++)r[a]=sinf(n*.07f+a)*2.3f;
            assert(srFfUpdate(&s,r,true));for(unsigned a=0;a<3;a++)assert(s.alpha[a]==0);
        }
    }
    puts("PASS physical rate feedforward: seed/ramp/sign/duplicates/caps/gaps/NaN/overflow/config/zero-gain/500us/1000us");
    return 0;
}
