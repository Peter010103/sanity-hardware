/* State/transition tests link the actual candidate paper_core.c. The injected
 * model independently controls support/failure to cover every branch. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "paper_core.h"
#include "speed_model_test_stub.h"
int smTestStatus[4];
float smTestCandidate[4];
static float measured[4], applied[4];
static float testDt=.001f;
static void model(int status, float count)
{for (unsigned i=0;i<4;i++) {smTestStatus[i]=status;smTestCandidate[i]=count;}}
static void init(spState_t *s)
{
    spConfig_t c={.dt=.001f,.cutoffHz=12,.inertia={3.32e-5f,4.28e-5f,5.43e-5f},
        .maxIterations=16,.kp=.04f,.ki=1,.kaw=10,.integralLimit=200,.countSlewPerSec=5000};
    for(unsigned i=0;i<4;i++){c.omegaMax[i]=5883;c.rowScale[i]=1;c.priority[i]=1;}
    c.dt=testDt;assert(spInit(s,&c));s->ready=true;
    for(unsigned i=0;i<4;i++){s->omegaCommand[i]=2250;measured[i]=2250;applied[i]=1350;}
    model(SM_TRUSTED_IN_SUPPORT,1350);
}
static void call(spState_t *s, bool seed, float voltage, float high)
{
    assert(spServo(s,measured,voltage,1100,high,applied,seed));
    for(unsigned i=0;i<4;i++) {
        assert(isfinite(s->servoCount[i]) && s->servoCount[i]>=1100 && s->servoCount[i]<=high);
        assert(fabsf(s->servoI[i])<=200);
    }
}
static void track(spState_t *s)
{
    memcpy(applied,s->servoCount,sizeof(applied));spTrack(s,applied);
    float old[4];memcpy(old,s->servoI,sizeof(old));spTrack(s,applied);
    assert(!memcmp(old,s->servoI,sizeof(old))); /* exactly-once tracking */
}
static void initialization_and_bounds(void)
{
    spState_t s;init(&s);model(SM_UNSUPPORTED,1999);
    assert(!spServo(&s,measured,6.95f,1100,1950,applied,false));
    assert(s.servoFailure==SP_SERVO_STATE && s.ffValidMask==0);
    call(&s,true,6.95f,1950);assert(s.ffSeededMask==15 && s.ffHeldMask==15);
    for(unsigned k=0;k<1000;k++) {
        for(unsigned i=0;i<4;i++) applied[i]=1350+(float)(k%100);
        call(&s,true,6.94f,1900);
        for(unsigned i=0;i<4;i++) assert(s.feedforward[i]==1350);
        track(&s);
    }
    const float volts[]={6.94f,6.95f,7.0f,7.04f,7.48f};
    const float highs[]={1900,1950,1999.99988f,2000};
    for(unsigned v=0;v<5;v++)for(unsigned h=0;h<4;h++) {
        for(unsigned i=0;i<4;i++) {applied[i]=1350;measured[i]=2500;}
        call(&s,false,volts[v],highs[h]);
        for(unsigned i=0;i<4;i++) assert(s.feedforward[i]==1350 && s.servoCount[i]<=applied[i]+5000*testDt);
        track(&s);
    }
    /* Moving hard bounds take precedence over slew. */
    for(unsigned i=0;i<4;i++) applied[i]=1900;
    call(&s,false,6.95f,1500);for(unsigned i=0;i<4;i++)assert(s.servoCount[i]<=1500);
    track(&s);
    spReset(&s);assert(!s.ready && !s.ffValidMask && !s.ffHeldMask && !s.ffSeededMask);
    assert(!s.ffDegradedEntries && !s.ffRecoveries && !s.servoISaturationTicks);
}
static void support_and_recovery(void)
{
    spState_t s;init(&s);call(&s,true,7.48f,2000);track(&s);
    model(SM_UNSUPPORTED,1950);call(&s,false,6.95f,1950);track(&s);
    assert(s.ffDegradedEntries==4 && s.ffHeldMask==15 && !s.ffSeededMask);
    for(unsigned i=0;i<4;i++) assert(s.feedforward[i]==1350);
    model(SM_TRUSTED_IN_SUPPORT,1450);
    for(unsigned k=0;k<(unsigned)ceilf(.05f/testDt)-1;k++) {
        call(&s,false,7.48f,2000);assert(s.ffHeldMask==15 && s.ffRecoveringMask==15);
        for(unsigned i=0;i<4;i++)assert(s.feedforward[i]==1350);
        track(&s);
    }
    call(&s,false,7.48f,2000);assert(s.ffHeldMask==0 && s.ffRecoveries==4);
    for(unsigned i=0;i<4;i++) assert(fabsf(s.feedforward[i]+s.servoI[i]-1350)<.001f);
    track(&s);
    /* A recovery larger than the I budget must remain partial, not clip I
     * after a full model jump. FF+I and actual output remain continuous. */
    model(SM_UNSUPPORTED,0);call(&s,false,6.95f,1950);track(&s);
    model(SM_TRUSTED_IN_SUPPORT,1800);
    for(unsigned k=0;k<100;k++){call(&s,false,7.48f,2000);track(&s);}
    assert(s.ffHeldMask==15 && s.ffRecoveringMask==15);
    for(unsigned i=0;i<4;i++) {
        assert(s.feedforward[i]<=1550.001f && s.servoI[i]>=-200);
        assert(fabsf(s.feedforward[i]+s.servoI[i]-1350)<.002f);
    }
    assert(s.servoISaturationTicks>0);
    /* Chatter cannot accumulate the 50 ms consecutive support dwell. */
    for(unsigned k=0;k<100;k++) {
        model(k%2?SM_TRUSTED_IN_SUPPORT:SM_UNSUPPORTED,1600);
        call(&s,false,7.48f,2000);track(&s);
        for(unsigned i=0;i<4;i++)assert(s.ffRecoveryTicks[i]<=1);
    }
}
static void faults_transaction_and_transfer(void)
{
    spState_t s;init(&s);call(&s,true,7.48f,2000);track(&s);
    spState_t old=s;model(SM_TRUSTED_IN_SUPPORT,1450);smTestStatus[3]=SM_INVALID;
    assert(!spServo(&s,measured,7.48f,1100,2000,applied,false));
    assert(s.servoFailure==SP_SERVO_MODEL && s.voltageSupport==SP_FF_INVALID);
    assert(!memcmp(old.feedforward,s.feedforward,sizeof(s.feedforward)));
    assert(!memcmp(old.servoI,s.servoI,sizeof(s.servoI)));
    assert(s.ffDegradedEntries==old.ffDegradedEntries && s.ffValidMask==old.ffValidMask);
    model(SM_TRUSTED_IN_SUPPORT,1350);measured[2]=NAN;
    assert(!spServo(&s,measured,7.48f,1100,2000,applied,false));
    assert(s.servoFailure==SP_SERVO_INPUT);measured[2]=2250;
    assert(!spServo(&s,measured,NAN,1100,2000,applied,false));
    assert(!spServo(&s,measured,7.48f,2000,1100,applied,false));
    s.omegaCommand[3]=INFINITY;assert(!spServo(&s,measured,7.48f,1100,2000,applied,false));
    s.omegaCommand[3]=2250;smTestCandidate[3]=NAN;
    assert(!spServo(&s,measured,7.48f,1100,2000,applied,false));
    assert(s.servoFailure==SP_SERVO_MODEL);
    model(SM_TRANSFER,1450);call(&s,false,6.95f,2000);
    assert(s.ffDegradedMask==15 && s.voltageSupport==SP_FF_TRANSFER);
#if SP_FF_ACCEPT_TRANSFER
    assert(!s.ffHeldMask);for(unsigned i=0;i<4;i++)assert(s.feedforward[i]==1450);
#else
    assert(s.ffHeldMask==15);for(unsigned i=0;i<4;i++)assert(s.feedforward[i]==1350);
#endif
    track(&s);
}
static void integral_and_antiwindup(void)
{
    spState_t s;init(&s);model(SM_UNSUPPORTED,0);call(&s,true,6.95f,2000);track(&s);
    for(unsigned i=0;i<4;i++)measured[i]=1250;
    for(unsigned k=0;k<300;k++){call(&s,false,6.95f,1400);track(&s);}
    for(unsigned i=0;i<4;i++) {
        assert(s.feedforward[i]==1350 && s.servoCount[i]<=1400 && s.servoI[i]<=200);
        /* Finite I is corrected toward actual bounds rather than winding to +200. */
        assert(s.servoI[i]<120);
    }
    call(&s,false,6.95f,1400);float old=s.servoI[0],raw=s.feedforward[0]+s.servoP[0]+old;
    for(unsigned i=0;i<4;i++)applied[i]=1300;
    spTrack(&s,applied);float expected=fminf(200,fmaxf(-200,old+10*testDt*(1300-raw)));
    assert(fabsf(s.servoI[0]-expected)<.0001f);
}
int main(void)
{
    for(unsigned rate=0;rate<2;rate++) {
        testDt=rate?.0005f:.001f;
        initialization_and_bounds();support_and_recovery();faults_transaction_and_transfer();integral_and_antiwindup();
    }
    printf("PASS actual servo: initialization/1000 shadow seeds/bounds/held FF/recovery/chatter/transaction/faults/antiwindup; transfer policy=%d\n",SP_FF_ACCEPT_TRANSFER);
    return 0;
}
