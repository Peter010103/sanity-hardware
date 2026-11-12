/* Actual direct speed model + actual servo: historical fault voltage/bound grid. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "paper_core.h"
#include "speed_model.h"
static void init(spState_t *s,float target)
{
    spConfig_t c={.dt=.001f,.cutoffHz=12,.inertia={3.32e-5f,4.28e-5f,5.43e-5f},
        .maxIterations=16,.kp=.04f,.ki=1,.kaw=10,.integralLimit=200,.countSlewPerSec=5000};
    for(unsigned i=0;i<4;i++){c.omegaMax[i]=5883;c.rowScale[i]=1;c.priority[i]=1;}
    assert(spInit(s,&c));s->ready=true;
    for(unsigned i=0;i<4;i++)s->omegaCommand[i]=target;
}
int main(void)
{
    const float voltages[]={7.48f,7.04f,7.0f,6.963889f,6.95f,6.94f,6.49f};
    const float highs[]={1900,1950,1999.99988f,2000};
    const float targets[]={2000,2250,2500,10};
    printf("policy,voltage,high,target,motor,success,reason,held,model_candidate,effective_ff,I,output,model_flags\n");
    for(unsigned t=0;t<4;t++)for(unsigned h=0;h<4;h++) {
        spState_t s;init(&s,targets[t]);float measured[4],applied[4];
        for(unsigned i=0;i<4;i++){measured[i]=targets[t];applied[i]=1350;}
        for(unsigned v=0;v<7;v++) {
            assert(spServo(&s,measured,voltages[v],1100,highs[h],applied,v==0));
            for(unsigned i=0;i<4;i++) {
                assert(isfinite(s.servoCount[i]) && s.servoCount[i]>=1100 && s.servoCount[i]<=highs[h]);
                assert(fabsf(s.servoCount[i]-applied[i])<=5.001f);
                assert(fabsf(s.servoI[i])<=200);
                assert(s.feedforward[i]<1600); /* Old hull escalation was >=1894.99. */
                if(targets[t]==10)assert(s.ffHeldMask==15 && s.ffCandidateMask==0);
                if(v==6)assert(s.ffHeldMask==15 && s.ffCandidateMask==0);
                printf("%d,%.9g,%.9g,%.9g,%u,1,%u,%u,%.9g,%.9g,%.9g,%.9g,%u\n",
                    SP_FF_ACCEPT_TRANSFER,voltages[v],highs[h],targets[t],i,s.ffReason[i],
                    !!(s.ffHeldMask&(1u<<i)),s.feedforwardCandidate[i],s.feedforward[i],
                    s.servoI[i],s.servoCount[i],s.ffModelFlags[i]);
            }
            memcpy(applied,s.servoCount,sizeof(applied));spTrack(&s,applied);
        }
        assert(!spServo(&s,measured,NAN,1100,2000,applied,false));
        measured[3]=NAN;assert(!spServo(&s,measured,7.48f,1100,2000,applied,false));
    }
    assert(spFeedforwardModelFingerprint()==SM_MODEL_FINGERPRINT);
    return 0;
}
