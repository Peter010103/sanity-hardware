#ifndef SANITY_TELEMETRY_PROFILE_H
#define SANITY_TELEMETRY_PROFILE_H
#include <stdbool.h>
#include <stdint.h>
#ifndef CONFIG_SANITY_DIAGNOSTIC_LEVEL
#define CONFIG_SANITY_DIAGNOSTIC_LEVEL 0
#endif
/* 0: full characterization, 1: sampled flight telemetry, 2: health only. */
struct sz_telemetry_profile { uint64_t next[2][256]; };
bool sz_telemetry_allow(struct sz_telemetry_profile *p, uint8_t type, uint8_t command, uint64_t now_us);
#endif
