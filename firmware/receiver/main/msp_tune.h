#ifndef SANITY_MSP_TUNE_H
#define SANITY_MSP_TUNE_H
#include "control.h"
#include "fc_telemetry.h"
#define SZ_TUNE_HEADER 32
#define SZ_TUNE_MAX 96
#define SZ_TUNE_REPLY_MAX (SZ_TUNE_HEADER + SZ_TUNE_MAX)
struct sz_tune_request {
  uint64_t epoch;
  uint32_t session, id, lease_ms;
  uint16_t ttl;
  uint8_t device, command, length, payload[SZ_TUNE_MAX];
};
struct sz_tune_context {
  const struct sz_state *state;
  uint32_t now_ms;
  uint8_t profile;
  bool fc_ready, fc_disarmed;
};
struct sz_tune_rpc {
  struct sz_tune_request pending;
  uint64_t owner_epoch, sent_us;
  uint32_t owner_session, last_id, reply_version, cooldown_ms;
  uint8_t uncertain_mask;
  uint8_t profile, pid_length, pid[30], advanced[61], active_pid[30],
      active_advanced[61];
  bool queued, sent, internal, owned;
  uint8_t reply[SZ_TUNE_REPLY_MAX];
  size_t reply_length;
};
void sz_tune_init(struct sz_tune_rpc *rpc);
bool sz_tune_submit(struct sz_tune_rpc *rpc, const uint8_t *data, size_t size,
                    const struct sz_tune_context *context, uint64_t now_us);
size_t sz_tune_poll(struct sz_tune_rpc *rpc,
                    const struct sz_tune_context *context, uint64_t now_us,
                    uint8_t *frame);
void sz_tune_response(struct sz_tune_rpc *rpc,
                      const struct sz_fc_record *record, uint64_t now_us);
#endif
