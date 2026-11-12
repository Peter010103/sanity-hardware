#ifndef SANITY_MOCAP_CLOCK_UDP_H
#define SANITY_MOCAP_CLOCK_UDP_H
#include <stdint.h>
/* Observation experiment only. Echoes clock probes; no state/control writes. */
void sz_mocap_clock_udp_start(uint8_t device, uint64_t (*current_epoch)(void));
#endif
