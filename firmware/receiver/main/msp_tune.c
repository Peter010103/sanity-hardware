#include "msp_tune.h"
#include <string.h>
enum { OK = 0, MSP_ERROR = 1, DENIED = 2, TIMEOUT = 3, BUSY = 4, INVALID = 5 };
static uint16_t u16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t u32(const uint8_t *p) {
  return ((uint32_t)u16(p) << 16) | u16(p + 2);
}
static uint64_t u64(const uint8_t *p) {
  return ((uint64_t)u32(p) << 32) | u32(p + 4);
}
static void put(uint8_t *p, uint64_t x, unsigned n) {
  for (unsigned i = 0; i < n; ++i)
    p[i] = (uint8_t)(x >> (8 * (n - 1 - i)));
}
static unsigned le16(const uint8_t *p) { return p[0] + 256u * p[1]; }
void sz_tune_init(struct sz_tune_rpc *r) { memset(r, 0, sizeof(*r)); }
static bool live(const struct sz_tune_context *c) {
  const struct sz_state *s = c->state;
  return s->valid && !s->rearm_required &&
         c->now_ms - s->last_rx_ms < SZ_MAX_LEASE_MS &&
         c->now_ms - s->command.lease_ms < s->command.ttl_ms;
}
static bool dirty(const struct sz_tune_rpc *r) {
  return r->pid_length && (memcmp(r->pid, r->active_pid, r->pid_length) ||
                           memcmp(r->advanced, r->active_advanced, 61));
}
static void reply(struct sz_tune_rpc *r, const struct sz_tune_request *q,
                  uint8_t code, const uint8_t *payload, unsigned n,
                  uint64_t now) {
  memcpy(r->reply, "SZA1", 4);
  r->reply[4] = 1;
  r->reply[5] = q->device;
  r->reply[6] = q->command;
  r->reply[7] = code;
  put(r->reply + 8, q->epoch, 8);
  put(r->reply + 16, q->session, 4);
  put(r->reply + 20, q->id, 4);
  put(r->reply + 24, now, 8);
  if (n)
    memcpy(r->reply + 32, payload, n);
  r->reply_length = 32 + n;
  ++r->reply_version;
}
static bool bounds(unsigned value, unsigned baseline, unsigned previous,
                   unsigned low, unsigned high, unsigned step, bool restore) {
  if (restore)
    return value == baseline;
  unsigned delta = value > previous ? value - previous : previous - value;
  return value * 100 >= baseline * low && value * 100 <= baseline * high + 99 &&
         delta * 100 <= baseline * step + 99;
}
static bool valid_write(const struct sz_tune_rpc *r,
                        const struct sz_tune_request *q) {
  if (!r->pid_length || !r->advanced[47])
    return false;
  if (q->command == 202) {
    if (q->length != r->pid_length)
      return false;
    bool restore = !memcmp(q->payload, r->pid, q->length);
    for (unsigned i = 0; i < q->length; ++i) {
      unsigned axis = i / 3, term = i % 3;
      if (axis >= 2 || term == 1) {
        if (q->payload[i] != r->pid[i])
          return false;
      } else if (!bounds(q->payload[i], r->pid[i], r->active_pid[i],
                         term == 0 ? 80 : 100, 110, 10, restore))
        return false;
    }
    return true;
  }
  if (q->command == 95) {
    if (q->length != 61)
      return false;
    bool restore = !memcmp(q->payload, r->advanced, 61);
    for (unsigned i = 0; i < 61; ++i)
      if ((i < 32 || i >= 36) && i != 39 && i != 40 && i != 52 &&
          q->payload[i] != r->advanced[i])
        return false;
    for (unsigned i = 32; i < 36; i += 2)
      if (!bounds(le16(q->payload + i), le16(r->advanced + i),
                  le16(r->active_advanced + i), 50, 110, 15, restore))
        return false;
    for (unsigned axis = 0; axis < 2; ++axis) {
      unsigned i = 39 + axis;
      if (!bounds(q->payload[i], r->advanced[i], r->active_advanced[i], 100,
                  110, 10, restore) ||
          q->payload[i] > r->active_pid[3 * axis + 2])
        return false;
    }
    unsigned boost = q->payload[52], previous = r->active_advanced[52];
    unsigned delta = boost > previous ? boost - previous : previous - boost;
    if (boost > r->advanced[52] || (!restore && delta > 5))
      return false;
    return true;
  }
  return false;
}
bool sz_tune_submit(struct sz_tune_rpc *r, const uint8_t *p, size_t n,
                    const struct sz_tune_context *c, uint64_t now) {
  if (n < 32 || n > 128 || memcmp(p, "SZM1", 4) || p[4] != 1 ||
      p[5] != c->state->device || p[7] != n - 32 || u16(p + 30))
    return false;
  struct sz_tune_request q = {.device = p[5],
                              .command = p[6],
                              .length = p[7],
                              .epoch = u64(p + 8),
                              .session = u32(p + 16),
                              .id = u32(p + 20),
                              .lease_ms = u32(p + 24),
                              .ttl = u16(p + 28)};
  memcpy(q.payload, p + 32, q.length);
  if (q.epoch != c->state->epoch || q.session != c->state->command.session)
    return false;
  if (!q.id || !q.ttl || q.ttl > 1000 || c->now_ms - q.lease_ms >= q.ttl ||
      !live(c) || c->state->mode != SZ_CTBR || q.epoch != c->state->epoch ||
      q.session != c->state->command.session) {
    reply(r, &q, DENIED, NULL, 0, now);
    return false;
  }
  bool same =
      r->owned && r->owner_epoch == q.epoch && r->owner_session == q.session;
  if (!c->fc_ready && !(same && r->queued && q.id == r->last_id)) {
    reply(r, &q, BUSY, NULL, 0, now);
    return false;
  }
  if (!same) {
    if (r->queued || dirty(r) || r->uncertain_mask || c->state->enabled ||
        !c->fc_disarmed) {
      reply(r, &q, BUSY, NULL, 0, now);
      return false;
    }
    uint32_t version = r->reply_version;
    sz_tune_init(r);
    r->reply_version = version;
    r->cooldown_ms = c->now_ms;
    r->owned = true;
    r->owner_epoch = q.epoch;
    r->owner_session = q.session;
    r->profile = c->profile;
  }
  if (q.id == r->last_id) {
    if (!r->queued && r->reply_length)
      ++r->reply_version;
    return true;
  }
  if (r->last_id && (q.id - r->last_id) >= 0x80000000u)
    return false;
  if (r->queued || (int32_t)(c->now_ms - r->cooldown_ms) < 0) {
    reply(r, &q, BUSY, NULL, 0, now);
    return false;
  }
  r->last_id = q.id;
  bool read =
      q.command == 0 || q.command == 94 || q.command == 112 || q.command == 150;
  if (c->profile != r->profile || (read && q.length) ||
      (!read && q.command != 95 && q.command != 202 && q.command != 250)) {
    reply(r, &q, INVALID, NULL, 0, now);
    return false;
  }
  if (q.command == 250) {
    if (q.length || c->state->enabled || !c->fc_disarmed || !r->pid_length ||
        !r->advanced[47] || r->uncertain_mask) {
      reply(r, &q, DENIED, NULL, 0, now);
      return false;
    }
  } else if (!read &&
             ((!c->state->enabled && !c->fc_disarmed) || !valid_write(r, &q))) {
    reply(r, &q, DENIED, NULL, 0, now);
    return false;
  }
  if ((q.command == 112 && !r->pid_length) ||
      (q.command == 94 && !r->advanced[47]))
    if (c->state->enabled || !c->fc_disarmed) {
      reply(r, &q, DENIED, NULL, 0, now);
      return false;
    }
  if (q.command == 0) {
    uint8_t cap[8] = {1, 46, 4, 5, 1, c->profile, c->fc_disarmed ? 1 : 0, 3};
    reply(r, &q, OK, cap, sizeof(cap), now);
    return true;
  }
  r->pending = q;
  r->queued = true;
  r->sent = false;
  r->internal = false;
  return true;
}
size_t sz_tune_poll(struct sz_tune_rpc *r, const struct sz_tune_context *c,
                    uint64_t now, uint8_t *out) {
  if (r->queued && r->sent &&
      now - r->sent_us >= (r->pending.command == 250 ? 2000000u : 150000u)) {
    reply(r, &r->pending, TIMEOUT, NULL, 0, now);
    r->queued = false;
    r->cooldown_ms = c->now_ms + 500;
  }
  if (!r->queued && r->owned && (dirty(r) || r->uncertain_mask) &&
      c->fc_ready && c->fc_disarmed && !c->state->enabled &&
      c->profile == r->profile &&
      (!live(c) || c->state->epoch != r->owner_epoch ||
       c->state->command.session != r->owner_session) &&
      (int32_t)(c->now_ms - r->cooldown_ms) >= 0) {
    r->pending = (struct sz_tune_request){.device = c->state->device,
                                          .epoch = r->owner_epoch,
                                          .session = r->owner_session};
    if ((r->uncertain_mask & 1) ||
        memcmp(r->pid, r->active_pid, r->pid_length)) {
      r->pending.command = 202;
      r->pending.length = r->pid_length;
      memcpy(r->pending.payload, r->pid, r->pid_length);
    } else {
      r->pending.command = 95;
      r->pending.length = 61;
      memcpy(r->pending.payload, r->advanced, 61);
    }
    r->queued = true;
    r->sent = false;
    r->internal = true;
  }
  if (!r->queued || r->sent)
    return 0;
  if (c->profile != r->profile || !c->fc_ready ||
      (r->internal ? (!c->fc_disarmed || c->state->enabled)
                   : (!live(c) || r->pending.epoch != c->state->epoch ||
                      r->pending.session != c->state->command.session ||
                      c->now_ms - r->pending.lease_ms >= r->pending.ttl))) {
    reply(r, &r->pending, DENIED, NULL, 0, now);
    r->queued = false;
    return 0;
  }
  if (!r->internal &&
      ((r->pending.command == 250 && (c->state->enabled || !c->fc_disarmed)) ||
       ((r->pending.command == 95 || r->pending.command == 202) &&
        !c->state->enabled && !c->fc_disarmed))) {
    reply(r, &r->pending, DENIED, NULL, 0, now);
    r->queued = false;
    return 0;
  }
  if (r->pending.command == 202)
    r->uncertain_mask |= 1;
  if (r->pending.command == 95)
    r->uncertain_mask |= 2;
  r->sent = true;
  r->sent_us = now;
  return sz_msp(r->pending.command, r->pending.payload, r->pending.length, out);
}
void sz_tune_response(struct sz_tune_rpc *r, const struct sz_fc_record *record,
                      uint64_t now) {
  if (!r->queued || !r->sent || record->command != r->pending.command ||
      record->command_rx_us < r->sent_us)
    return;
  uint8_t code = record->type == SZ_FC_ERROR ? MSP_ERROR : OK;
  unsigned n = record->length;
  if (n > SZ_TUNE_MAX)
    code = INVALID;
  if (code == OK) {
    if (record->command == 112) {
      if (n < 9 || n > 30 || n % 3)
        code = INVALID;
      else {
        if (!r->pid_length) {
          r->pid_length = (uint8_t)n;
          memcpy(r->pid, record->payload, n);
        }
        memcpy(r->active_pid, record->payload, n);
      }
    } else if (record->command == 94) {
      if (n != 61)
        code = INVALID;
      else {
        if (!r->advanced[47])
          memcpy(r->advanced, record->payload, 61);
        memcpy(r->active_advanced, record->payload, 61);
      }
    } else if (record->command == 202 || record->command == 95 ||
               record->command == 250) {
      if (n)
        code = INVALID;
      else if (record->command == 202)
        memcpy(r->active_pid, r->pending.payload, r->pid_length);
      else if (record->command == 95)
        memcpy(r->active_advanced, r->pending.payload, 61);
      else {
        memcpy(r->pid, r->active_pid, r->pid_length);
        memcpy(r->advanced, r->active_advanced, 61);
      }
    }
  }
  if (code == OK && (record->command == 112 || record->command == 202))
    r->uncertain_mask &= (uint8_t)~1u;
  if (code == OK && (record->command == 94 || record->command == 95))
    r->uncertain_mask &= (uint8_t)~2u;
  reply(r, &r->pending, code, code == OK ? record->payload : NULL,
        code == OK ? n : 0, now);
  r->queued = false;
}
