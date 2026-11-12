#include "estimator/runtime.h"
#include "estimator/sensor_wire.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
uint64_t fake_now;
void se_runtime_step(void);
static uint32_t r32(const uint8_t *b) {
  return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 |
         (uint32_t)b[3] << 24;
}
static uint64_t r64(const uint8_t *b) {
  return r32(b) | (uint64_t)r32(b + 4) << 32;
}
static uint8_t crc8(uint8_t c, uint8_t b) {
  c ^= b;
  for (int j = 0; j < 8; j++)
    c = c & 128 ? (c << 1) ^ 0xd5 : c << 1;
  return c;
}
static void response(unsigned cmd, const uint8_t *p, unsigned n) {
  uint8_t b[96] = {'$', 'X', '>', 0, cmd, cmd >> 8, n, 0};
  memcpy(b + 8, p, n);
  uint8_t c = 0;
  for (unsigned j = 3; j < n + 8; j++)
    c = crc8(c, b[j]);
  b[n + 8] = c;
  for (unsigned j = 0; j < n + 9; j++)
    se_runtime_byte(b[j], fake_now);
}
int main(int argc, char **argv) {
  uint64_t pose_delay = argc > 1 ? strtoull(argv[1], NULL, 10) : 500;
  se_runtime_start();
  se_runtime_epoch(1234);
  fake_now = 1000000;
  se_runtime_step();
  uint8_t request[64], reply[48], sensor[76], record[256];
  size_t n = se_runtime_poll(fake_now, request);
  assert(n == 57);
  se_clock_packet clock;
  assert(se_clock_decode(request + 8, 48, &clock));
  clock.epoch = 5678; /* join after multiple32bit wraps */
  clock.t2 = UINT64_C(0x200000000) + fake_now + 700;
  clock.t3 = clock.t2 + 10;
  se_clock_encode(&clock, reply);
  fake_now += 1410;
  response(SE_CLOCK_MSP, reply, 48);
  se_runtime_step();
  unsigned output = 0, raw = 0, health = 0;
  uint32_t flags = 0, max_memory = 0;
  bool saw_healthy = false;
  for (unsigned i = 1; i <= 500; i++) {
    fake_now =
        1000000 + i * 5000; /* Regular matched periodicclock exchanges. */
    n = se_runtime_poll(fake_now, request);
    if (n == 57) {
      assert(se_clock_decode(request + 8, 48, &clock));
      clock.epoch = 5678;
      clock.t2 = UINT64_C(0x200000000) + fake_now + 700;
      clock.t3 = clock.t2 + 10;
      se_clock_encode(&clock, reply);
      fake_now += 1410;
      response(SE_CLOCK_MSP, reply, 48);
    }
    struct sz_mocap_sample p = {.epoch = 1234,
                                .source_ros_ns = i * 5000000ULL,
                                .capture_lower_us = fake_now - pose_delay - 100,
                                .capture_upper_us = fake_now - pose_delay + 100,
                                .sequence = i,
                                .session = 3,
                                .quaternion = {1, 0, 0, 0}};
    // Match the configured maximum mocap rate: 100 Hz with 200 Hz IMU.
    if (i % 2 == 0) se_runtime_pose(&p, fake_now);
    se_sensor_packet s = {.epoch = 5678,
                          .sequence = i,
                          .gyro_us = (uint32_t)(fake_now - 400),
                          .accel_us = (uint32_t)(fake_now - 400),
                          .gyro_delay_us = 15915,
                          .accel_delay_us = 15915,
                          .gyro_uncertainty_us = 125,
                          .accel_uncertainty_us = 125,
                          .flags = 15,
                          .accel = {0, 0, 9.80665f}};
    se_sensor_encode(&s, sensor);
    response(SE_SENSOR_MSP, sensor, 76);
    se_runtime_step();
    for (unsigned sub = 0; sub < 3; sub++) {
      fake_now += 1000;
      se_runtime_step();
    }
    while ((n = se_runtime_record(record))) {
      assert(r32(record + n - 4) == se_crc32c(record, n - 4));
      if (record[5] == 1)
        raw++;
      if (record[5] == 3) {
        output++;
        flags = r32(record + 24 + 16);
        if ((flags & 9) == 9) saw_healthy = true;
        uint32_t m = r32(record + 24 + 180);
        if (m > max_memory)
          max_memory = m;
      }
      if (record[5] == 5)
        health++;
    }
  }
  // Stop IMU input, drain bounded replay, and observe the stale health flag.
  for (unsigned tick = 0; tick < 40; tick++) {
    fake_now += 1000;
    se_runtime_step();
    while ((n = se_runtime_record(record)))
      if (record[5] == 3) {
        flags = r32(record + 40);
      }
  }
  assert(raw == 500 && output >= 120 && health == output && saw_healthy);
  assert((flags & 3) == 1 && !(flags & 8)); /* Initialized, replay drained, stale IMU. */
  fake_now += 100000;
  se_runtime_step();
  uint64_t estimate_time = 0;
  while ((n = se_runtime_record(record)))
    if (record[5] == 3) {
      flags = r32(record + 40);
      estimate_time = r64(record + 32);
    }
  assert((flags & 1) && !(flags & 8) &&
         estimate_time < fake_now); /* Unmatchedclock cannotreplace mapping. */
  clock.nonce++;
  clock.epoch++;
  se_clock_encode(&clock, reply);
  response(SE_CLOCK_MSP, reply, 48);
  se_runtime_step(); /* Overflowbothboundedqueues, thenverifyreportedcount. */
  for (unsigned i = 0; i < 100; i++) {
    se_sensor_packet s = {.epoch = 5678,
                          .sequence = 1000 + i,
                          .gyro_us = i + 1,
                          .accel_us = i + 1,
                          .flags = 15,
                          .accel = {0, 0, 9.80665f}};
    se_sensor_encode(&s, sensor);
    response(SE_SENSOR_MSP, sensor, 76);
  }
  for (unsigned i = 0; i < 10; i++) {
    fake_now += 20000;
    se_runtime_step();
  }
  uint32_t drops = 0;
  while ((n = se_runtime_record(record)))
    if (record[5] == 3)
      drops = r32(record + 24 + 148);
  assert(drops >= 68);
  se_runtime_epoch(8888);
  unsigned polls = 0, clock_polls = 0;
  uint64_t poll_start = fake_now + 1000000;
  for (unsigned tick = 0; tick < 60000; tick++) {
    /* Periodic one-tick jitter collides clock and sensor deadlines. */
    if (tick % 1000 == 0)
      continue;
    size_t request_n = se_runtime_poll_in_slot(poll_start + tick * 1000ULL, request, tick%10>=6 && tick%10<8);
    if (request_n == 9)
      polls++;
    else if (request_n == 57)
      clock_polls++;
  }
  assert(polls >= 11999 && polls <= 12001 && clock_polls == 3000);
  assert(max_memory < 96 * 1024);
  printf("runtime: nativeMSPv2, matchedclock, latejoinwrap, stationaryinit, %u "
         "sensors, %u outputs, staleness, queueoverflow=%u, memory=%u passed\n",
         raw, output, drops, max_memory);
  return 0;
}
