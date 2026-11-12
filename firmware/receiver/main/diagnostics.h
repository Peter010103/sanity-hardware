#ifndef SANITY_DIAGNOSTICS_H
#define SANITY_DIAGNOSTICS_H
#include "control.h"
#include "timing_wire.h"
#include "zenoh-pico.h"
typedef int (*sz_diag_send_fn)(const uint8_t *data,size_t size,void *context);
void sz_diag_flush_traces_to(sz_diag_send_fn send,void *context);
void sz_diag_flush_pongs_to(sz_diag_send_fn send,void *context);
void sz_diag_probe_bytes(const uint8_t *data,size_t size);
void sz_diag_init(void);
void sz_diag_poll_gap(uint64_t previous_us,uint64_t now_us);
void sz_diag_flush_traces(const z_loaned_publisher_t *publisher);
void sz_diag_flush_pongs(const z_loaned_publisher_t *publisher);
void sz_diag_command(const struct sz_command *c, bool accepted, uint64_t entered_us, uint64_t done_us, uint32_t now_ms);
void sz_diag_write(const struct sz_state *s, uint8_t command, uint32_t tick, uint64_t snapshot_us, uint64_t before_us, uint64_t after_us, unsigned bytes);
void sz_diag_ack(uint8_t command, uint64_t previous_poll_us, uint64_t parsed_us);
void sz_diag_stop(const struct sz_state *s, uint8_t reason, uint16_t seen, uint16_t mv, uint16_t age);
void sz_diag_flush(const z_loaned_publisher_t *publisher, const z_loaned_publisher_t *pong_publisher);
void sz_diag_probe(z_loaned_sample_t *sample, void *context);
void sz_diag_epoch(uint64_t epoch);
void sz_diag_command_timing(const struct sz_trace *event);
void sz_diag_publish_time(uint8_t kind,uint64_t before,uint64_t after,int result);
#endif
