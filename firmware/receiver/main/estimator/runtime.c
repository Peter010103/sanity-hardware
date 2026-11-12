#include "sdkconfig.h"
#ifdef CONFIG_SANITY_ESTIMATOR_EXPERIMENT
#include "eskf.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "runtime.h"
#include "sensor_wire.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#if configTICK_RATE_HZ != 1000
#error "Estimator experiment requires a 1000 Hz FreeRTOS tick"
#endif
#define EVENTS 32
#define RECORDS 64
struct event {
  uint8_t type;
  uint64_t received, epoch;
  union {
    se_sensor_packet sensor;
    se_clock_packet clock;
    struct sz_mocap_sample pose;
  } data;
};
struct record {
  uint16_t size;
  uint8_t bytes[256];
};
static StaticQueue_t eq, rq;
static uint8_t event_storage[EVENTS * sizeof(struct event)],
    record_storage[RECORDS * sizeof(struct record)];
static QueueHandle_t events, records;
static se_filter filter;
static se_clock_map mapping;
static se_unwrap gyro_unwrap, accel_unwrap;
static se_pose pose_history[32];
static unsigned pose_head, pose_count;
static uint32_t pose_session;
static uint64_t last_source_ns;
static uint64_t observation_epoch, requested_epoch;
static uint32_t requested_version, applied_version;
static uint64_t outstanding_nonce, outstanding_t1;
static uint32_t outstanding_sequence;
static uint32_t event_drops, record_drops, raw_sequence, max_task_us,
    clock_rejected, timing_rejected;
static portMUX_TYPE queue_lock = portMUX_INITIALIZER_UNLOCKED;
static void put32(uint8_t *b, uint32_t v) {
  for (int k = 0; k < 4; k++)
    b[k] = v >> (8 * k);
}
static void put64(uint8_t *b, uint64_t v) {
  for (int k = 0; k < 8; k++)
    b[k] = v >> (8 * k);
}
static void putf(uint8_t *b, float f) {
  uint32_t v;
  memcpy(&v, &f, 4);
  put32(b, v);
}
static void event_push(struct event *e) {
  if (!events)
    return;
  portENTER_CRITICAL(&queue_lock);
  if (xQueueSend(events, e, 0) != pdPASS) {
    struct event discarded;
    xQueueReceive(events, &discarded, 0);
    event_drops++;
    xQueueSend(events, e, 0);
  }
  portEXIT_CRITICAL(&queue_lock);
}
static void emit(uint8_t type, const uint8_t *data, unsigned n,
                 uint64_t received) {
  if (n > 228)
    return;
  struct record r = {.size = n + 28};
  memset(r.bytes, 0, sizeof(r.bytes));
  memcpy(r.bytes, "SE01", 4);
  r.bytes[4] = 1;
  r.bytes[5] = type;
  r.bytes[6] = r.size;
  r.bytes[7] = r.size >> 8;
  put32(r.bytes + 8, ++raw_sequence);
  put64(r.bytes + 12, received);
  put32(r.bytes + 20, record_drops);
  memcpy(r.bytes + 24, data, n);
  put32(r.bytes + 24 + n, se_crc32c(r.bytes, 24 + n));
  if (xQueueSend(records, &r, 0) != pdPASS) {
    struct record discarded;
    xQueueReceive(records, &discarded, 0);
    record_drops++;
    xQueueSend(records, &r, 0);
  }
}
size_t se_runtime_record(uint8_t out[256]) {
  struct record r;
  if (!records || xQueueReceive(records, &r, 0) != pdPASS)
    return 0;
  memcpy(out, r.bytes, r.size);
  return r.size;
}
void se_runtime_epoch(uint64_t epoch) {
  portENTER_CRITICAL(&queue_lock);
  outstanding_nonce = 0;
  requested_epoch = epoch;
  requested_version++;
  if (events)
    xQueueReset(events);
  portEXIT_CRITICAL(&queue_lock);
}
void se_runtime_pose(const struct sz_mocap_sample *p, uint64_t received) {
  struct event e = {.type = 4, .received = received};
  e.data.pose = *p;
  event_push(&e);
}
static uint8_t crc8(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (int j = 0; j < 8; j++)
    crc = crc & 128 ? (crc << 1) ^ 0xd5 : crc << 1;
  return crc;
}
/* Native MSPv2 parser; independent from the legacy MSPv1 telemetry parser. */
void se_runtime_byte(uint8_t b, uint64_t now) {
  static uint8_t state, crc, header[5], payload[80];
  static uint32_t parser_version;
  portENTER_CRITICAL(&queue_lock);
  uint32_t version = requested_version;
  portEXIT_CRITICAL(&queue_lock);
  if (version != parser_version) {
    state = 0;
    parser_version = version;
  }
  static unsigned index, length;
  static uint16_t command;
  if (state == 0) {
    state = b == '$' ? 1 : 0;
    return;
  }
  if (state == 1) {
    state = b == 'X' ? 2 : 0;
    return;
  }
  if (state == 2) {
    state = b == '>' ? 3 : 0;
    index = 0;
    crc = 0;
    return;
  }
  if (state == 3) {
    header[index++] = b;
    crc = crc8(crc, b);
    if (index == 5) {
      command = header[1] | header[2] << 8;
      length = header[3] | header[4] << 8;
      index = 0;
      if (header[0] || length > 80) {
        state = 0;
        return;
      }
      state = length ? 4 : 5;
    }
    return;
  }
  if (state == 4) {
    payload[index++] = b;
    crc = crc8(crc, b);
    if (index == length)
      state = 5;
    return;
  }
  state = 0;
  if (b != crc)
    return;
  struct event e = {.received = now};
  if (command == SE_SENSOR_MSP &&
      se_sensor_decode(payload, length, &e.data.sensor))
    e.type = 1;
  else if (command == SE_CLOCK_MSP &&
           se_clock_decode(payload, length, &e.data.clock)) {
    portENTER_CRITICAL(&queue_lock);
    bool match = outstanding_nonce && e.data.clock.nonce == outstanding_nonce &&
                 e.data.clock.t1 == outstanding_t1 &&
                 e.data.clock.sequence == outstanding_sequence &&
                 now >= outstanding_t1 && now - outstanding_t1 < 100000;
    if (match)
      outstanding_nonce = 0;
    portEXIT_CRITICAL(&queue_lock);
    if (!match)
      return;
    e.type = 2;
  } else
    return;
  event_push(&e);
}
size_t se_runtime_poll_in_slot(uint64_t now, uint8_t out[64], bool clock_slot) {
  static uint64_t next_sensor, next_clock;
  static uint32_t sequence;
  static uint32_t poll_version;
  portENTER_CRITICAL(&queue_lock);
  uint32_t version = requested_version;
  portEXIT_CRITICAL(&queue_lock);
  if (version != poll_version) {
    next_clock = next_sensor = 0;
    poll_version = version;
  }
  uint16_t cmd;
  unsigned n = 0;
  uint8_t p[48];
  if (clock_slot && now >= next_clock) {
    se_clock_packet c = {.nonce = ((uint64_t)esp_random() << 32) | esp_random(),
                         .t1 = now,
                         .sequence = ++sequence};
    portENTER_CRITICAL(&queue_lock);
    outstanding_nonce = c.nonce;
    outstanding_t1 = c.t1;
    outstanding_sequence = c.sequence;
    portEXIT_CRITICAL(&queue_lock);
    se_clock_encode(&c, p);
    n = 48;
    cmd = SE_CLOCK_MSP;
    next_clock = now + 20000; // 50 Hz: low-latency clock probes, with unchanged uncertainty gates.
  } else if (now >= next_sensor) {
    cmd = SE_SENSOR_MSP;
    next_sensor += 5000;
    if (next_sensor + 15000 < now)
      next_sensor = now + 5000;
  } else
    return 0;
  out[0] = '$';
  out[1] = 'X';
  out[2] = '<';
  out[3] = 0;
  out[4] = cmd;
  out[5] = cmd >> 8;
  out[6] = n;
  out[7] = 0;
  if (n)
    memcpy(out + 8, p, n);
  uint8_t crc = 0;
  for (unsigned j = 3; j < n + 8; j++)
    crc = crc8(crc, out[j]);
  out[n + 8] = crc;
  return n + 9;
}
size_t se_runtime_poll(uint64_t now, uint8_t out[64]) {
  return se_runtime_poll_in_slot(now,out,true);
}
static void snapshot(uint64_t now) {
  uint8_t b[228] = {0};
  const se_state *s = &filter.state;
  put64(b, observation_epoch);
  put64(b + 8, s->time_us);
  bool clock_current = mapping.valid && now >= mapping.last_local &&
                       now - mapping.last_local <= 1500000;
  bool fresh = filter.ready && now >= s->time_us && now - s->time_us < 40000 &&
               filter.last_pose_us && now >= filter.last_pose_us &&
               now - filter.last_pose_us < 300000;
  put32(b + 16, (filter.ready ? 1 : 0) | (filter.catching_up ? 2 : 0) |
                    (clock_current ? 4 : 0) |
                    ((fresh && clock_current && !filter.catching_up) ? 8 : 0));
  put32(b + 20, filter.last_fault);
  unsigned offset = 24;
  const se_real *arrays[] = {s->p, s->v, s->q, s->bg, s->ba};
  unsigned sizes[] = {3, 3, 4, 3, 3};
  for (unsigned a = 0; a < 5; a++)
    for (unsigned j = 0; j < sizes[a]; j++, offset += 4)
      putf(b + offset, arrays[a][j]);
  for (unsigned j = 0; j < 15; j++, offset += 4)
    putf(b + offset, s->P[j][j]);
  put32(b + 148, event_drops);
  put32(b + 152, record_drops);
  put32(b + 156, max_task_us);
  put32(b + 160, filter.fused);
  put32(b + 164, filter.replayed);
  put32(b + 168, filter.pending_replaced);
  put32(b + 172, clock_rejected);
  put32(b + 176, timing_rejected);
  put32(b + 180, sizeof(filter) + sizeof(event_storage) +
                     sizeof(record_storage) + sizeof(eq) + sizeof(rq) +
                     sizeof(pose_history));
  put64(b + 184, filter.last_pose_us);
  put64(b + 192, filter.history[filter.head].state.time_us);
  putf(b + 200, filter.nis);
  for (int j = 0; j < 6; j++)
    putf(b + 204 + 4 * j, filter.innovation[j]);
  emit(3, b, sizeof(b), now);
  uint8_t health[80] = {0};
  for (int j = 0; j < 11; j++)
    put32(health + 4 * j, filter.rejected[j]);
  put64(health + 44, now >= s->time_us ? now - s->time_us : UINT64_MAX);
  put64(health + 52, filter.last_pose_us && now >= filter.last_pose_us
                         ? now - filter.last_pose_us
                         : UINT64_MAX);
  put32(health + 60, filter.max_replay);
  put32(health + 64, mapping.uncertainty_us);
  put32(health + 68, filter.propagated);
  put32(health + 72, filter.count);
  put32(health + 76, filter.remaining);
  emit(5, health, sizeof(health), now);
}
void se_runtime_step(void) {
  static uint64_t next_output = 0;
  uint64_t begin = esp_timer_get_time();
  struct event e;
  portENTER_CRITICAL(&queue_lock);
  uint32_t version = requested_version;
  uint64_t epoch = requested_epoch;
  portEXIT_CRITICAL(&queue_lock);
  if (version != applied_version) {
    applied_version = version;
    e.epoch = epoch;

    observation_epoch = e.epoch;
    uint8_t epoch_bytes[8];
    put64(epoch_bytes, e.epoch);
    emit(6, epoch_bytes, 8, begin);
    se_reset(&filter, observation_epoch);
    memset(&mapping, 0, sizeof(mapping));
    memset(&gyro_unwrap, 0, sizeof(gyro_unwrap));
    memset(&accel_unwrap, 0, sizeof(accel_unwrap));
    pose_head = pose_count = 0;
    pose_session = 0;
    last_source_ns = 0;
  }
  for (unsigned budget = 0;
       budget < 8 && xQueueReceive(events, &e, 0) == pdPASS; budget++) {
    if (e.type == 2) {
      uint8_t b[56];
      se_clock_encode(&e.data.clock, b);
      put64(b + 48, e.received);
      emit(2, b, sizeof(b), e.received);
      se_clock_packet *c = &e.data.clock;
      if (mapping.epoch && mapping.epoch != c->epoch) {
        se_reset(&filter, observation_epoch);
        pose_head = pose_count = 0;
        pose_session = 0;
        last_source_ns = 0;
      }
      if (!se_clock_observe_uart(&mapping, c->epoch, c->t1, c->t2, c->t3,
                            e.received))
        clock_rejected++;
    } else if (e.type == 4) {
      struct sz_mocap_sample *p = &e.data.pose;
      uint8_t b[88]; /* Original decoded values and mapped capture bounds
                        retained. */
      put64(b, p->epoch);
      put64(b + 8, p->source_ros_ns);
      put64(b + 16, p->capture_lower_us);
      put64(b + 24, p->capture_upper_us);
      put32(b + 32, p->session);
      put32(b + 36, p->sequence);
      for (int j = 0; j < 3; j++)
        putf(b + 40 + 4 * j, p->position[j]);
      for (int j = 0; j < 4; j++)
        putf(b + 52 + 4 * j, p->quaternion[j]);
      for (int j = 0; j < 3; j++)
        putf(b + 68 + 4 * j, p->velocity[j]);
      memset(b + 80, 0, 8);
      emit(4, b, sizeof(b), e.received);
      if (p->epoch != observation_epoch ||
          p->capture_upper_us < p->capture_lower_us ||
          p->capture_upper_us - p->capture_lower_us > 2 * SE_MAX_CLOCK_US ||
          e.received < p->capture_upper_us ||
          e.received - p->capture_lower_us > 250000) {
        timing_rejected++;
        continue;
      }
      if (!p->session) {
        timing_rejected++;
        continue;
      }
      if (pose_session && p->session != pose_session) {
        se_reset(&filter, observation_epoch);
        pose_head = pose_count = 0;
        last_source_ns = 0;
      }
      if (p->source_ros_ns <= last_source_ns) {
        timing_rejected++;
        continue;
      }
      pose_session = p->session;
      last_source_ns = p->source_ros_ns;
      se_pose pose = {
          .epoch = p->epoch,
          .time_us = p->capture_lower_us +
                     (p->capture_upper_us - p->capture_lower_us) / 2,
          .sequence = p->sequence,
          .uncertainty_us = (p->capture_upper_us - p->capture_lower_us) / 2};
      for (int j = 0; j < 3; j++)
        pose.p[j] = p->position[j];
      for (int j = 0; j < 4; j++)
        pose.q[j] = p->quaternion[j];
      pose_history[pose_head] = pose;
      pose_head = (pose_head + 1) % 32;
      if (pose_count < 32)
        pose_count++;

    } else if (e.type == 1) {
      se_sensor_packet *p = &e.data.sensor;
      uint8_t b[76];
      se_sensor_encode(p, b);
      emit(1, b, sizeof(b), e.received);
      /* Seed raw32 unwrap near the FC64 clock anchor, including late joins.
       */
      se_unwrap *unwrappers[2] = {&gyro_unwrap, &accel_unwrap};
      uint32_t raw[2] = {p->gyro_us, p->accel_us};
      for (int j = 0; j < 2; j++)
        if (mapping.valid && mapping.epoch == p->epoch &&
            (!unwrappers[j]->present || unwrappers[j]->epoch != p->epoch)) {
          uint64_t near =
              (mapping.remote_anchor & ~UINT64_C(0xffffffff)) | raw[j];
          if (near > mapping.remote_anchor &&
              near - mapping.remote_anchor > UINT64_C(0x80000000) &&
              near >= UINT64_C(0x100000000))
            near -= UINT64_C(0x100000000);
          else if (mapping.remote_anchor > near &&
                   mapping.remote_anchor - near > UINT64_C(0x80000000))
            near += UINT64_C(0x100000000);
          unwrappers[j]->epoch = p->epoch;
          unwrappers[j]->extended = near - 1;
          unwrappers[j]->raw = raw[j] - 1;
          unwrappers[j]->present = true;
        }
      uint64_t gt, at, g, a;
      uint32_t gu, au;
      if ((p->flags & 3) != 3 ||
          !se_unwrap_time(&gyro_unwrap, p->epoch, p->gyro_us, &gt) ||
          !se_unwrap_time(&accel_unwrap, p->epoch, p->accel_us, &at) ||
          !se_clock_convert(&mapping, p->epoch, gt, e.received, &g, &gu) ||
          !se_clock_convert(&mapping, p->epoch, at, e.received, &a, &au)) {
        timing_rejected++;
        continue;
      }
      if (g < p->gyro_delay_us || a < p->accel_delay_us) {
        timing_rejected++;
        continue;
      }
      g -= p->gyro_delay_us;
      a -= p->accel_delay_us;
      uint64_t mismatch = g > a ? g - a : a - g;
      se_imu i = {.epoch = observation_epoch,
                  .sequence = p->sequence,
                  .time_us = g / 2 + a / 2 + (g % 2 + a % 2) / 2,
                  .uncertainty_us =
                      (gu > au ? gu : au) +
                      (p->gyro_uncertainty_us > p->accel_uncertainty_us
                           ? p->gyro_uncertainty_us
                           : p->accel_uncertainty_us) +
                      mismatch / 2,
                  .calibrated = (p->flags & 4) != 0,
                  .disarmed = (p->flags & 8) != 0};
      for (int j = 0; j < 3; j++) {
        i.gyro[j] = p->gyro[j];
        i.accel[j] = p->accel[j];
      }
      if (filter.ready)
        se_push_imu(&filter, &i);
      else {
        const se_pose *init = NULL;
        for (unsigned j = 0; j < pose_count; j++) {
          const se_pose *candidate = &pose_history[(pose_head + 31 - j) % 32];
          if (candidate->time_us <= i.time_us) {
            init = candidate;
            break;
          }
        }
        se_initialize(&filter, &i, init);
      }
    }
  }
  if (filter.ready) {
    uint64_t latest_imu = filter.history[filter.head].state.time_us;
    for (unsigned j = 0; j < pose_count; j++) {
      const se_pose *p = &pose_history[(pose_head + 31 - j) % 32];
      if (p->time_us <= latest_imu) {
        if (p->time_us > filter.last_pose_us &&
            (!filter.has_pending || p->time_us > filter.pending.time_us))
          se_submit_pose(&filter, p);
        break;
      }
    }
  }
  se_service(&filter);
  uint64_t end = esp_timer_get_time();
  if (end - begin > max_task_us)
    max_task_us = end - begin;
  if (end >= next_output) {
    snapshot(end);
    next_output = end + 20000;
  }
}
static void task(void *unused) {
  (void)unused;
  for (;;) {
    se_runtime_step();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
void se_runtime_start(void) {
  events = xQueueCreateStatic(EVENTS, sizeof(struct event), event_storage, &eq);
  records =
      xQueueCreateStatic(RECORDS, sizeof(struct record), record_storage, &rq);
  static StaticTask_t task_buffer;
  static StackType_t task_stack[8192 / sizeof(StackType_t)];
  if (!events || !records ||
      !xTaskCreateStaticPinnedToCore(task, "observation_eskf", 8192, NULL, 3,
                                     task_stack, &task_buffer, 1))
    abort();
}
#endif
