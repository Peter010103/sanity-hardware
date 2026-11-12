/* Experimental flight controller handover policy. GPL-3.0-or-later. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum { SH_PID, SH_TO_INDI, SH_INDI, SH_TO_PID } shMode_e;
typedef enum {
    SH_OK, SH_DISABLED, SH_DISARMED, SH_MODE, SH_CONFIG, SH_RPM, SH_TIMING,
    SH_NUMERIC, SH_ALLOCATOR, SH_MOTION, SH_SATURATION, SH_TRACKING,
    SH_TAKEOFF, SH_REQUEST_RESET, SH_IDLE, SH_WARMUP, SH_RX
} shReason_e;
typedef struct {
    bool armed, enabled, request, eligible;
    uint8_t reason, fault;
    uint32_t nowUs, dtUs, transitionUs;
} shInput_t;
typedef struct {
    float weight;
    uint8_t mode, reason, fault, lastFault;
    bool armedLast, sawRequestLow, requestLast;
    uint16_t transitions, fallbacks;
} shState_t;
void shUpdate(shState_t *state, const shInput_t *input);
