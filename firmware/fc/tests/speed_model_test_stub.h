/* Deterministic model fault injection for tests of the real servo state machine.
 * This header is never selected by the firmware build. */
#pragma once
#include <math.h>
#include <stdint.h>
#define SM_MODEL_FINGERPRINT UINT32_C(0x54455354)
typedef enum { SM_TRUSTED_IN_SUPPORT, SM_TRANSFER, SM_UNSUPPORTED, SM_INVALID } smStatus_t;
extern int smTestStatus[4];
extern float smTestCandidate[4];
static inline smStatus_t smInverse(float omega, float voltage, unsigned motor,
                                  float *count, uint16_t *flags)
{
    if (!count || !flags || motor >= 4 || !isfinite(omega) || omega < 0 ||
        !isfinite(voltage) || voltage <= 0) return SM_INVALID;
    *flags = (uint16_t)(0x10u + motor);
    *count = smTestCandidate[motor];
    return (smStatus_t)smTestStatus[motor];
}
