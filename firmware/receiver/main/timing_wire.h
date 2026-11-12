#ifndef SANITY_TIMING_WIRE_H
#define SANITY_TIMING_WIRE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "control.h"
#define SZ_PROBE_SIZE 20
#define SZ_PONG_SIZE 48
#define SZ_TRACE_SIZE 64
struct sz_trace {
    uint8_t type, mode, flags, code;
    uint32_t session, sequence, tick;
    uint64_t epoch, t0, t1, t2, t3;
    uint32_t arg0, arg1;
};
bool sz_probe_reply(const uint8_t *in, size_t n, uint8_t device, uint64_t rx_us,
                    uint64_t tx_us, uint64_t epoch, int8_t rssi, uint8_t channel,
                    uint8_t out[SZ_PONG_SIZE]);
/* Type8 reports aggregate callback arrival timing over a bounded window.
 * t0/t1: window start/end us; t2: largest actual accepted-command gap us;
 * t3: callback time ending that gap; tick: accepted count; arg0: rejected count;
 * arg1: gaps >50ms; context is epoch/session, never mixed across a re-claim. */
struct sz_timing_window {
    uint64_t epoch, start_us, previous_us, max_gap_us, max_at_us;
    uint32_t session, sequence, accepted, rejected, over_50ms;
    uint8_t mode;
};
void sz_timing_accept(struct sz_timing_window *w, uint64_t epoch, uint32_t session,
                      uint32_t sequence, uint8_t mode, bool accepted, uint64_t now_us);
bool sz_timing_report(struct sz_timing_window *w, uint64_t now_us, uint8_t profile, struct sz_trace *out);
void sz_trace_encode(const struct sz_trace *event, uint8_t out[SZ_TRACE_SIZE]);
/* Optional command-timing schema1 carried by backwards-compatible SZT1 records.
 * Type9: tick=receiver now_ms, t0=callback rx_us, t1=decision complete_us,
 * t2=raw echoed receiver lease_ms (NOT host generation time), t3=previous
 * accepted callback rx_us in this owner/epoch; arg0=ttl_ms,arg1=rejected counter.
 * flags: decoded1,accepted2,enabled4. code is sz_command_reject below.
 * Type10: tick=output tick, t0=accepted callback rx_us,t1=state snapshot_us,
 * t2/t3=uart_write_bytes entry/return_us; arg0=modulo32 lease age_ms at entry,
 * arg1=signed driver return count. flags: valid1,enabled2,full write4.
 * code=MSP command. A UART API return is NOT wire completion or FC acceptance.
 */
enum sz_command_reject { SZ_CMD_ACCEPTED=0, SZ_CMD_DECODE=1, SZ_CMD_PROFILE=2,
    SZ_CMD_CALIBRATION=3, SZ_CMD_FC_NOT_READY=4, SZ_CMD_EPOCH=5,
    SZ_CMD_DEVICE=6, SZ_CMD_MODE=7, SZ_CMD_LEASE=8, SZ_CMD_SEQUENCE=9,
    SZ_CMD_CLAIM=10, SZ_CMD_REARM=11, SZ_CMD_POLICY=12 };
uint8_t sz_command_reject_reason(const struct sz_state *before,const struct sz_command *c,
                                bool accepted,uint8_t stage_reason,uint32_t now_ms);
struct sz_trace sz_command_decision_trace(const struct sz_command *c,bool decoded,bool accepted,
    uint8_t reason,uint32_t now_ms,uint64_t rx_us,uint64_t done_us,
    uint64_t previous_rx_us,uint32_t rejected);
struct sz_trace sz_command_uart_trace(const struct sz_state *s,uint32_t tick,uint8_t command,
    uint64_t rx_us,uint64_t snapshot_us,uint64_t before_us,uint64_t after_us,
    unsigned requested_bytes,int returned_bytes);
#endif
