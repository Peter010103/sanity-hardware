#ifndef SANITY_ZENOH_CONTROL_H
#define SANITY_ZENOH_CONTROL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define SZ_COMMAND_SIZE 40
#define SZ_STATUS_SIZE 48
#define SZ_CTBR 1
#define SZ_DIRECT 2
#define SZ_PAPER_NATIVE 3
#define SZ_PAPER_COMMAND_SIZE 64
#define SZ_PAPER_MODEL_ID 0x50494e31u
#define SZ_PAPER_SET 0x7f15
#define SZ_PAPER_STATUS 0x7f16
#define SZ_MAX_LEASE_MS 1000
struct sz_command {
    uint64_t epoch;
    uint32_t session, sequence, lease_ms;
    uint16_t ttl_ms, values[4], thrust_q;
    uint8_t mode, device;
    bool enabled, physical_units, high_rate, controller_selection, indi_requested, paper_native;
    float thrust_n, alpha_rad_s2[3], fallback_rate_rad_s[3];
};
struct sz_state {
    uint64_t epoch;
    struct sz_command command;
    uint32_t last_rx_ms, rejected;
    uint8_t device, mode;
    bool valid, enabled, rearm_required;
};
struct sz_paper_link {
    uint64_t epoch;
    uint32_t observed_ms;
    bool capable, negotiated;
};
void sz_paper_observe(struct sz_paper_link *link, uint64_t epoch, uint32_t now_ms, const uint8_t *wrapped, size_t length);
bool sz_paper_admit(const struct sz_paper_link *link, uint64_t epoch, uint32_t now_ms);
bool sz_paper_forward(const struct sz_paper_link *link, uint64_t epoch, bool enabled);
size_t sz_paper_after_rc(const struct sz_command *command, uint32_t now_ms, unsigned expected, int written, uint8_t out[76]);
bool sz_paper_decode(const uint8_t *data, size_t length, struct sz_command *command);
/* Called immediately before UART submission. Returns zero after expiry. */
size_t sz_paper_msp(const struct sz_command *command, uint32_t now_ms, uint8_t out[76]);
size_t sz_paper_query(uint8_t out[12]);
bool sz_paper_capable(const uint8_t *wrapped, size_t length);
void sz_init(struct sz_state *s, uint8_t device, uint8_t mode, uint64_t epoch);
bool sz_decode(const uint8_t *data, size_t length, struct sz_command *command);
bool sz_profile_allows(const struct sz_command *command);
bool sz_calibrate(struct sz_command *command, uint16_t voltage_mv);
bool sz_model_voltage_valid(uint16_t voltage_mv);
bool sz_accept(struct sz_state *s, const struct sz_command *command, uint32_t now);
bool sz_accept_msp(struct sz_state *s, const struct sz_command *command, uint32_t now, bool fc_disarmed);
void sz_stop(struct sz_state *s);
#ifdef CONFIG_SANITY_TIMING_DIAGNOSTICS
void sz_diag_expiry(const struct sz_state *s, uint32_t evaluated_ms);
#endif
bool sz_output(struct sz_state *s, uint32_t now, uint16_t values[4]);
void sz_status(struct sz_state *s, uint32_t now, bool fc_ready, uint16_t voltage_mv,
               uint16_t fc_age_ms, uint8_t out[SZ_STATUS_SIZE]);
void sz_sbus(const uint16_t values[4], bool enabled, bool failsafe, uint8_t out[25]);
size_t sz_msp(uint8_t command, const uint8_t *payload, uint8_t length, uint8_t *out);
size_t sz_rc_msp(const uint16_t values[4], bool enabled, uint8_t out[22]);
size_t sz_rc_msp_select(const uint16_t values[4], bool enabled, bool indi_requested, uint8_t out[22]);
size_t sz_motors(const uint16_t values[4], uint8_t out[14]);
#endif
