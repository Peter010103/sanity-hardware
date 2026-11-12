/* Policy changes authority; emergency disarm is handled by the flight adapter.
 * Faults inhibit re-engagement until disarm. A low request must be observed
 * after arming, so reboot/stale AUX-high cannot engage INDI at takeoff.
 * GPL-3.0-or-later.
 */
#include "platform.h"
#include <math.h>
#include "supervisor.h"

void shUpdate(shState_t *s, const shInput_t *in)
{
    if (!in->armed) {
        s->weight=0.0f; s->mode=SH_PID; s->reason=SH_DISARMED;
        s->fault=0; s->armedLast=false; s->sawRequestLow=false; s->requestLast=false;
        return;
    }
    if (!s->armedLast) {
        s->armedLast=true; s->weight=0.0f; s->mode=SH_PID;
        s->sawRequestLow=false; s->requestLast=false;
    }
    if (!in->request) s->sawRequestLow=true;
    if (in->fault && !s->fault) {
        s->fault=in->fault; s->lastFault=in->fault;
        if (s->weight>0.0f) s->fallbacks++;
    }
    bool target=false;
    if (!in->enabled) s->reason=SH_DISABLED;
    else if (s->fault) s->reason=s->fault;
    else if (!in->request) s->reason=SH_OK;
    else if (!s->sawRequestLow) s->reason=SH_REQUEST_RESET;
    else if (s->weight>0.0f && s->mode!=SH_TO_PID && in->eligible) target=true;
    else if (s->weight==0.0f && !s->requestLast && in->eligible) target=true;
    else if (s->mode==SH_TO_INDI && in->eligible) target=true;
    else s->reason=in->eligible ? SH_REQUEST_RESET : in->reason;
    // A denied engagement requires a deliberate low/high retry. An active
    // controller may continue while health is good; transient entry gates
    // (settled rate/attitude) are not applied after it gains authority.
    if (target) {
        s->reason=SH_OK;
        if (s->weight==0.0f) s->transitions++;
    }
    const float step=in->transitionUs ? (float)in->dtUs/(float)in->transitionUs : 0.0f;
    if (!isfinite(step) || step<=0.0f || step>0.25f) {
        s->fault=SH_TIMING; s->lastFault=SH_TIMING; s->reason=SH_TIMING;
        target=false;
    }
    const float boundedStep=fminf(0.25f,fmaxf(0.0f,step));
    s->weight=target ? fminf(1.0f,s->weight+boundedStep) : fmaxf(0.0f,s->weight-boundedStep);
    s->mode=s->weight==0.0f ? SH_PID : s->weight==1.0f && target ? SH_INDI : target ? SH_TO_INDI : SH_TO_PID;
    s->requestLast=in->request;
}
