#include "sensor_wire.h"
#include <math.h>
#include <string.h>
static uint16_t r16(const uint8_t *b) {
  return (uint16_t)b[0] | (uint16_t)b[1] << 8;
}
static uint32_t r32(const uint8_t *b) {
  return (uint32_t)r16(b) | (uint32_t)r16(b + 2) << 16;
}
static uint64_t r64(const uint8_t *b) {
  return (uint64_t)r32(b) | (uint64_t)r32(b + 4) << 32;
}
static void w16(uint8_t *b, uint16_t x) {
  b[0] = x;
  b[1] = x >> 8;
}
static void w32(uint8_t *b, uint32_t x) {
  w16(b, x);
  w16(b + 2, x >> 16);
}
static void w64(uint8_t *b, uint64_t x) {
  w32(b, x);
  w32(b + 4, x >> 32);
}
static float rf(const uint8_t *b) {
  uint32_t x = r32(b);
  float f;
  memcpy(&f, &x, 4);
  return f;
}
static void wf(uint8_t *b, float f) {
  uint32_t x;
  memcpy(&x, &f, 4);
  w32(b, x);
}
uint32_t se_crc32c(const uint8_t *b, size_t n) {
  uint32_t c = ~0u;
  for (size_t i = 0; i < n; i++) {
    c ^= b[i];
    for (unsigned k = 0; k < 8; k++)
      c = (c >> 1) ^ (0x82f63b78u & -(c & 1));
  }
  return ~c;
}
void se_sensor_encode(const se_sensor_packet *p,
                      uint8_t b[SE_WIRE_SENSOR_SIZE]) {
  memset(b, 0, 76);
  b[0] = 'S';
  b[1] = 'I';
  b[2] = 1;
  b[3] = 76;
  w64(b + 4, p->epoch);
  w32(b + 12, p->sequence);
  w32(b + 16, p->gyro_us);
  w32(b + 20, p->accel_us);
  w32(b + 24, p->gyro_uncertainty_us);
  w32(b + 28, p->accel_uncertainty_us);
  w32(b + 32, p->dropped);
  for (int a = 0; a < 3; a++) {
    wf(b + 36 + 4 * a, p->gyro[a]);
    wf(b + 48 + 4 * a, p->accel[a]);
  }
  w16(b + 60, p->flags);
  b[62] = p->gyro_source;
  b[63] = p->accel_source;
  w32(b + 64, p->gyro_delay_us);
  w32(b + 68, p->accel_delay_us);
  w32(b + 72, se_crc32c(b, 72));
}
bool se_sensor_decode(const uint8_t *b, size_t n, se_sensor_packet *p) {
  if (n != 76 || b[0] != 'S' || b[1] != 'I' || b[2] != 1 || b[3] != 76 ||
      r32(b + 72) != se_crc32c(b, 72))
    return false;
  p->gyro_delay_us = r32(b + 64);
  p->accel_delay_us = r32(b + 68);
  if (p->gyro_delay_us > 100000 || p->accel_delay_us > 100000)
    return false;
  p->epoch = r64(b + 4);
  p->sequence = r32(b + 12);
  p->gyro_us = r32(b + 16);
  p->accel_us = r32(b + 20);
  p->gyro_uncertainty_us = r32(b + 24);
  p->accel_uncertainty_us = r32(b + 28);
  p->dropped = r32(b + 32);
  for (int a = 0; a < 3; a++) {
    p->gyro[a] = rf(b + 36 + 4 * a);
    p->accel[a] = rf(b + 48 + 4 * a);
    if (!isfinite(p->gyro[a]) || !isfinite(p->accel[a]))
      return false;
  }
  p->flags = r16(b + 60);
  p->gyro_source = b[62];
  p->accel_source = b[63];
  return !(p->flags & ~15u) && p->epoch != 0;
}
void se_clock_encode(const se_clock_packet *p, uint8_t b[SE_WIRE_CLOCK_SIZE]) {
  memset(b, 0, 48);
  w64(b, p->epoch);
  w64(b + 8, p->nonce);
  w64(b + 16, p->t1);
  w64(b + 24, p->t2);
  w64(b + 32, p->t3);
  w32(b + 40, p->sequence);
  w32(b + 44, se_crc32c(b, 44));
}
bool se_clock_decode(const uint8_t *b, size_t n, se_clock_packet *p) {
  if (n != 48 || r32(b + 44) != se_crc32c(b, 44))
    return false;
  p->epoch = r64(b);
  p->nonce = r64(b + 8);
  p->t1 = r64(b + 16);
  p->t2 = r64(b + 24);
  p->t3 = r64(b + 32);
  p->sequence = r32(b + 40);
  return true;
}
bool se_unwrap_time(se_unwrap *s, uint64_t epoch, uint32_t raw, uint64_t *out) {
  if (!s->present || s->epoch != epoch) {
    s->epoch = epoch;
    s->raw = raw;
    s->extended = raw;
    s->present = true;
  } else {
    int32_t d = (int32_t)(raw - s->raw);
    if (d <= 0)
      return false;
    s->extended += (uint32_t)d;
    s->raw = raw;
  }
  *out = s->extended;
  return true;
}
static bool clockObserve(se_clock_map *m, uint64_t epoch, uint64_t t1, uint64_t t2,
                         uint64_t t3, uint64_t t4, uint32_t minimum_rtt_us) {
  if (t4 < t1 || t3 < t2 || t4 - t1 < t3 - t2)
    return false;
  uint64_t rtt = (t4 - t1) - (t3 - t2);
  if (rtt > 4000 || rtt < minimum_rtt_us)
    return false;
  uint32_t uncertainty = (uint32_t)((rtt - minimum_rtt_us) / 2 + 50);
  if (uncertainty > 2000)
    return false;
  uint64_t remote = t2 + (t3 - t2) / 2, local = t1 + (t4 - t1) / 2;
  if (!m->valid || m->epoch != epoch) {
    memset(m, 0, sizeof(*m));
    m->epoch = epoch;
    m->scale = 1;
    m->samples = 1;
    m->fit_remote = remote;
    m->fit_local = local;
  } else {
    if (t4 < m->last_local || local <= m->local_anchor || remote <= m->remote_anchor)
      return false;
    // Keep a tighter existing exchange instead of a slower UART reply.
    // Its uncertainty still grows at the same conservative rate.
    if (t4 - m->last_local <= 3000000 &&
        uncertainty > m->uncertainty_us + (t4 - m->last_local) / (m->samples >= 2 ? 1000 : 200))
      return true;
    uint64_t dr = remote - m->fit_remote, dl = local - m->fit_local;
    // UART scheduling jitter must not be interpreted as oscillator skew.
    // Fit drift across at least five seconds; offset bounds still refresh each exchange.
    if (dr >= 5000000) {
      double scale = (double)dl / dr;
      if (fabs(scale - 1) > (double).005) {
        m->valid = false;
        return false;
      }
      m->scale =
          m->samples < 2 ? scale : (double).9 * m->scale + (double).1 * scale;
      m->samples++;
      m->fit_remote = remote;
      m->fit_local = local;
    }
  }
  m->remote_anchor = remote;
  m->local_anchor = local;
  m->last_local = t4;
  m->uncertainty_us = uncertainty;
  m->valid = true;
  return true;
}
bool se_clock_observe(se_clock_map *m, uint64_t epoch, uint64_t t1, uint64_t t2,
                      uint64_t t3, uint64_t t4) {
  return clockObserve(m, epoch, t1, t2, t3, t4, 0);
}
bool se_clock_observe_uart(se_clock_map *m, uint64_t epoch, uint64_t t1, uint64_t t2,
                           uint64_t t3, uint64_t t4) {
  // Native MSP2 request/reply are each 57 bytes at 921600 baud, 8N1.
  // Each direction needs >=600 us (nominal 618.5 us), including 0.5% clock
  // tolerance. Subtract only this known minimum from the RTT uncertainty.
  return clockObserve(m, epoch, t1, t2, t3, t4, 1200);
}
bool se_clock_convert(const se_clock_map *m, uint64_t epoch, uint64_t remote,
                      uint64_t now, uint64_t *local, uint32_t *u) {
  if (!m->valid || epoch != m->epoch || now < m->last_local ||
      now - m->last_local > 3000000)
    return false;
  int64_t d = (int64_t)(remote - m->remote_anchor);
  double v = m->local_anchor + m->scale * (double)d;
  if (v < 0 || !isfinite(v))
    return false;
  uint64_t age = now - m->last_local;
  // Before drift calibration allow 5000 ppm oscillator error. After the
  // five-second fit, retain a conservative 1000 ppm residual-error bound.
  *u = m->uncertainty_us + (uint32_t)(age / (m->samples >= 2 ? 1000 : 200));
  if (*u > 2000)
    return false;
  *local = (uint64_t)(v + (double).5);
  return true;
}
