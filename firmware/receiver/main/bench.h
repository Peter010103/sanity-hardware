#ifndef SANITY_ZENOH_BENCH_H
#define SANITY_ZENOH_BENCH_H
#include <stdbool.h>
#include <stdint.h>
struct bench_state {
    uint8_t parse_state, length, command, index, checksum, payload[255];
    uint16_t seen;
    uint32_t stamp[10];
    uint16_t voltage_mv;
    uint8_t pid_profile;
    int arm_bit;
    bool disarmed, msp_receiver, rc_ack_seen, battery_in_fit;
    uint32_t rc_ack_ms;
};
void bench_init(struct bench_state *b);
void bench_byte(struct bench_state *b, uint8_t byte, uint32_t now);
bool bench_ctbr_ready(const struct bench_state *b, uint32_t now);
bool bench_disarmed(const struct bench_state *b, uint32_t now);
bool bench_ready(const struct bench_state *b, uint32_t now);
#endif
