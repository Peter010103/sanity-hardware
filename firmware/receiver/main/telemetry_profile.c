#include "telemetry_profile.h"
bool sz_telemetry_allow(struct sz_telemetry_profile *p, uint8_t type, uint8_t command, uint64_t now_us) {
    /* Controller authority/health is a flight interlock, not bulk diagnostics. */
    if (type==2 && command==255) return true;
#if CONFIG_SANITY_DIAGNOSTIC_LEVEL == 0
    (void)p; (void)type; (void)command; (void)now_us; return true;
#else
    if (type == 5) return true; /* Health producer is already limited to 1 Hz. */
#if CONFIG_SANITY_DIAGNOSTIC_LEVEL == 2
    (void)p; (void)command; (void)now_us; return false;
#else
    unsigned slot = type == 1 ? 0 : 1;
    uint64_t period = type == 1 ? 200000 : (command == 102 || command == 104) ? 100000 :
                      command == 130 ? 500000 : 200000;
    if (type == 3 || type == 4) period = 1000000;
    if (now_us < p->next[slot][command]) return false;
    p->next[slot][command] = now_us + period;
    return true;
#endif
#endif
}
