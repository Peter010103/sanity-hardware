/* Host-only deterministic replay ABI. Compile separately, never in firmware. */
#include "eskf.h"
#include "sensor_wire.h"
#include <string.h>
static se_filter f;
static se_clock_map map;
void se_host_reset(uint64_t epoch) {
  se_reset(&f, epoch);
  memset(&map, 0, sizeof(map));
}
int se_host_clock(const uint8_t *bytes, uint64_t t4) {
  se_clock_packet c;
  if (!se_clock_decode(bytes, 48, &c))
    return 0;
  bool reset = map.epoch && map.epoch != c.epoch;
  if (reset)
    se_reset(&f, f.epoch);
  int ok = se_clock_observe_uart(&map, c.epoch, c.t1, c.t2, c.t3, t4);
  return ok ? (reset ? 2 : 1) : 0;
}
int se_host_map(uint64_t epoch, uint32_t raw, uint64_t now, uint64_t *out,
                uint32_t *u) {
  uint64_t t = (map.remote_anchor & ~UINT64_C(0xffffffff)) | raw;
  if (t > map.remote_anchor && t - map.remote_anchor > UINT64_C(0x80000000) &&
      t >= UINT64_C(0x100000000))
    t -= UINT64_C(0x100000000);
  else if (map.remote_anchor > t &&
           map.remote_anchor - t > UINT64_C(0x80000000))
    t += UINT64_C(0x100000000);
  return se_clock_convert(&map, epoch, t, now, out, u);
}
int se_host_imu(uint64_t time, uint32_t sequence, uint32_t uncertainty,
                uint16_t flags, const double *gyro, const double *accel,
                const double *pose, uint64_t pose_time, uint32_t pose_sequence,
                uint32_t pose_uncertainty) {
  se_imu i = {.time_us = time,
              .epoch = f.epoch,
              .sequence = sequence,
              .uncertainty_us = uncertainty,
              .calibrated = !!(flags & 4),
              .disarmed = !!(flags & 8)};
  for (int a = 0; a < 3; a++) {
    i.gyro[a] = gyro[a];
    i.accel[a] = accel[a];
  }
  if (f.ready)
    return se_push_imu(&f, &i);
  se_pose p = {.epoch = f.epoch,
               .time_us = pose_time,
               .sequence = pose_sequence,
               .uncertainty_us = pose_uncertainty};
  if (pose) {
    for (int a = 0; a < 3; a++)
      p.p[a] = pose[a];
    for (int a = 0; a < 4; a++)
      p.q[a] = pose[a + 3];
  }
  return se_initialize(&f, &i, pose ? &p : NULL);
}
int se_host_pose(uint64_t time, uint32_t sequence, uint32_t uncertainty,
                 const double *pose) {
  se_pose p = {.epoch = f.epoch,
               .time_us = time,
               .sequence = sequence,
               .uncertainty_us = uncertainty};
  for (int a = 0; a < 3; a++)
    p.p[a] = pose[a];
  for (int a = 0; a < 4; a++)
    p.q[a] = pose[a + 3];
  return se_submit_pose(&f, &p);
}
unsigned se_host_service(void) { return se_service(&f); }
unsigned se_host_state(double *out, uint64_t *time, uint32_t *stats) {
  const se_real *arrays[] = {f.state.p, f.state.v, f.state.q, f.state.bg,
                             f.state.ba};
  unsigned lengths[] = {3, 3, 4, 3, 3}, at = 0;
  for (int a = 0; a < 5; a++)
    for (unsigned j = 0; j < lengths[a]; j++)
      out[at++] = arrays[a][j];
  for (int a = 0; a < 15; a++)
    for (int j = 0; j < 15; j++)
      out[at++] = f.state.P[a][j];
  *time = f.state.time_us;
  stats[0] = f.last_fault;
  stats[1] = f.fused;
  stats[2] = f.replayed;
  stats[3] = f.pending_replaced;
  for (int j = 0; j < 11; j++)
    stats[4 + j] = f.rejected[j];
  return (f.ready ? 1 : 0) | (f.catching_up ? 2 : 0) | (f.has_pending ? 4 : 0);
}
