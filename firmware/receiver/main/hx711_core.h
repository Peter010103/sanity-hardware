#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Pure acquisition bookkeeping. No baseline, force or cable-state inference. */
enum { SZ_HX_MISSING=1, SZ_HX_SATURATION=2, SZ_HX_CLOCK=4, SZ_HX_SETTLING=8 };
#define SZ_HX_SETTLING_US 600000
#define SZ_HX_DISCARD 4
#define SZ_HX_HIGH_LIMIT_US 45
#define SZ_HX_QUEUE_SIZE 4
struct sz_hx_state {
    uint32_t sequence, resets;
    int64_t last_packet_us, settling_until_us, reset_us;
    unsigned discard;
};
struct sz_hx_sample {
    uint32_t sequence, resets, flags, max_high_us;
    int32_t raw;
    int64_t ready_us, ready_lower_us, shift_end_us, acquired_us, reset_us;
};
void sz_hx_init(struct sz_hx_state *s, int64_t reset_done_us);
/* Return a sample/status packet; *reset requests the hardware reset after a fault.
 * That fault is not counted among the four conversions discarded AFTER reset. */
bool sz_hx_step(struct sz_hx_state *s, bool have, int32_t raw, int64_t ready,
    int64_t lower, int64_t shift_end, uint32_t high, bool stuck, int64_t now,
    struct sz_hx_sample *out, bool *reset);
void sz_hx_reset_done(struct sz_hx_state *s, int64_t now);
int sz_hx_format(char *out, size_t size, const struct sz_hx_sample *sample,
    unsigned device, uint64_t acquisition_boot, uint64_t receiver_boot,
    uint32_t receiver_boot_count, int64_t prepare_us, uint32_t send_drops,
    uint32_t queue_drops);
