#include "estimator/eskf.h"
#include "estimator/sensor_wire.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static se_filter f;
static se_imu imu(uint64_t t, uint32_t seq) {
  se_imu i = {.time_us = t,
              .epoch = 42,
              .sequence = seq,
              .uncertainty_us = 100,
              .calibrated = true,
              .disarmed = true,
              .accel = {0, 0, SE_GRAVITY},
              .gyro = {.01, -.02, .005}};
  return i;
}
static se_pose pose(uint64_t t, uint32_t seq) {
  se_pose p = {.time_us = t,
               .epoch = 42,
               .sequence = seq,
               .uncertainty_us = 100,
               .q = {1, 0, 0, 0}};
  return p;
}
static void initialize(void) {
  se_reset(&f, 42);
  for (unsigned n = 1; n <= 200; n++) {
    se_imu i = imu(n * 5000, n);
    se_pose p = pose(n * 5000, n);
    bool ok = se_initialize(&f, &i, &p);
    assert(ok == (n == 200));
  }
  assert(f.ready);
  assert(se_state_valid(&f.state));
  assert(fabs(f.state.bg[0] - .01) < 1e-6);
}
static void faults(void) {
  initialize();
  se_imu i = imu(1005000, 201);
  assert(se_push_imu(&f, &i));
  assert(!se_push_imu(&f, &i) && f.last_fault == SE_ORDER);
  i = imu(1010000, 202);
  i.epoch++;
  assert(!se_push_imu(&f, &i) && f.last_fault == SE_EPOCH);
  i.epoch = 42;
  i.uncertainty_us = 4000;
  assert(!se_push_imu(&f, &i) && f.last_fault == SE_CLOCK);
  i.uncertainty_us = 100;
  i.calibrated = false;
  assert(!se_push_imu(&f, &i) && f.last_fault == SE_CALIBRATION);
  se_pose p = pose(1005000, 201);
  p.p[0] = 100;
  assert(se_submit_pose(&f, &p));
  se_service(&f);
  assert(f.last_fault == SE_GATE);
  p.p[0] = 0;
  p.epoch++;
  assert(!se_submit_pose(&f, &p));
  i = imu(1040000, 203);
  assert(!se_push_imu(&f, &i) && !f.ready && f.last_fault == SE_GAP);
  initialize();
  f.last_imu_sequence = 0xfffffffeu;
  i = imu(1005000, 0xffffffffu);
  assert(se_push_imu(&f, &i));
  i = imu(1010000, 0);
  assert(se_push_imu(&f, &i));
  se_reset(&f, 42);
  for (unsigned n = 1; n < 200; n++) {
    i = imu(n * 5000, n);
    se_initialize(&f, &i, NULL);
  }
  i = imu(2000000, 200);
  assert(!se_initialize(&f, &i, NULL));
  assert(f.stationary_count == 1);
  initialize();
  for (unsigned n = 201; n <= 260; n++) {
    i = imu(n * 5000, n);
    assert(se_push_imu(&f, &i));
  }
  p = pose(1005000, 201);
  assert(se_submit_pose(&f, &p));
  se_service(&f);
  assert(f.last_fault == SE_TOO_OLD);
  p = pose(1250000, 250);
  assert(se_submit_pose(&f, &p));
  p = pose(1260000, 252);
  assert(se_submit_pose(&f, &p));
  assert(f.pending_replaced == 1);
  while (f.has_pending || f.catching_up)
    se_service(&f);
  assert(se_state_valid(&f.state));
  se_sensor_packet sp = {.epoch = 42,
                         .sequence = 0xffffffff,
                         .gyro_us = 123,
                         .accel_us = 120,
                         .flags = 15,
                         .accel = {0, 0, 9.80665f}},
                   decoded;
  uint8_t b[76];
  se_sensor_encode(&sp, b);
  assert(se_sensor_decode(b, 76, &decoded));
  assert(decoded.sequence == sp.sequence);
  for (unsigned n = 0; n < 76; n++) {
    b[n] ^= 1;
    assert(!se_sensor_decode(b, 76, &decoded));
    b[n] ^= 1;
  }
  assert(!se_sensor_decode(b, 75, &decoded));
  se_unwrap u = {0};
  uint64_t stamp;
  assert(se_unwrap_time(&u, 42, 0xfffffff0, &stamp));
  assert(se_unwrap_time(&u, 42, 0x10, &stamp) && stamp == 0x100000010ULL);
  assert(!se_unwrap_time(&u, 42, 0xf, &stamp));
  assert(se_unwrap_time(&u, 43, 1, &stamp) && stamp == 1);
  se_clock_map m = {0};
  uint64_t mapped;
  uint32_t uncertainty;
  assert(se_clock_observe(&m, 42, 1000, 2100, 2200, 1300));
  assert(se_clock_convert(&m, 42, 2150, 1300, &mapped, &uncertainty) &&
         mapped == 1150);
  assert(!se_clock_convert(&m, 43, 2150, 1300, &mapped, &uncertainty));
  assert(!se_clock_observe(&m, 42, 1000, 2000, 2100, 900));
  assert(!se_clock_convert(&m, 42, 2150, 4000000, &mapped, &uncertainty));
}

static void truth(double t, double position[3], double quaternion[4],
                  double body_accel[3], double omega[3]) {
  double r = .12 * sin(.8 * t), pitch = .1 * sin(.6 * t), yaw = .15 * t;
  double rd = .096 * cos(.8 * t), pd = .06 * cos(.6 * t), yd = .15;
  double cr = cos(r), sr = sin(r), cp = cos(pitch), sp = sin(pitch),
         cy = cos(yaw), sy = sin(yaw);
  double R[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                    {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                    {-sp, cp * sr, cp * cr}};
  double acceleration[3] = {.2 * .7 * .7 * cos(.7 * t),
                            .1 * .9 * .9 * cos(.9 * t),
                            .08 * .5 * .5 * cos(.5 * t) + SE_GRAVITY};
  position[0] = .2 * (1 - cos(.7 * t));
  position[1] = .1 * (1 - cos(.9 * t));
  position[2] = .08 * (1 - cos(.5 * t));
  for (int a = 0; a < 3; a++) {
    body_accel[a] = 0;
    for (int k = 0; k < 3; k++)
      body_accel[a] += R[k][a] * acceleration[k];
  }
  omega[0] = rd - yd * sp;
  omega[1] = pd * cr + yd * sr * cp;
  omega[2] = -pd * sr + yd * cr * cp;
  double cR = cos(r / 2), sR = sin(r / 2), cP = cos(pitch / 2),
         sP = sin(pitch / 2), cY = cos(yaw / 2), sY = sin(yaw / 2);
  quaternion[0] = cR * cP * cY + sR * sP * sY;
  quaternion[1] = sR * cP * cY - cR * sP * sY;
  quaternion[2] = cR * sP * cY + sR * cP * sY;
  quaternion[3] = cR * cP * sY - sR * sP * cY;
}
int main(int argc, char **argv) {
  faults();
  unsigned delay = argc > 1 ? atoi(argv[1]) : 0,
           stress = argc > 2 ? atoi(argv[2]) : 0;
  initialize();
  se_pose observations[2200];
  unsigned obs = 0, delivered = 0, samples = 0;
  double sum = 0, angle_sum = 0;
  for (unsigned n = 201; n <= 2200; n++) {
    double t = (n - 200) * .005, ptrue[3], qtrue[4], accel[3], omega[3];
    truth(t, ptrue, qtrue, accel, omega);
    se_imu i = imu(n * 5000, n);
    for (int a = 0; a < 3; a++) {
      i.accel[a] = accel[a] + .006 * sin(n * (1.3 + .2 * a)) +
                   (a == 0   ? .03
                    : a == 1 ? -.02
                             : .01);
      i.gyro[a] += omega[a];
    }
    assert(se_push_imu(&f, &i));
    if (n % 2 == 0) {
      se_pose p = pose(n * 5000, n);
      for (int a = 0; a < 3; a++)
        p.p[a] = ptrue[a] + .002 * sin(n * (.83 + .1 * a));
      for (int a = 0; a < 4; a++)
        p.q[a] = qtrue[a];
      observations[obs++] = p;
    }
    while (delivered < obs &&
           observations[delivered].time_us + delay * 1000 <= i.time_us) {
      se_pose p = observations[delivered++];
      if (stress && delivered % 13 == 0)
        continue;
      if (stress && delivered % 47 == 0)
        p.p[0] += 10;
      se_submit_pose(&f, &p);
    }
    assert(se_service(&f) <= SE_REPLAY_BUDGET);
    assert(f.ready && se_state_valid(&f.state));
    truth((f.state.time_us - 1000000) * 1e-6, ptrue, qtrue, accel, omega);
    for (int a = 0; a < 3; a++) {
      double e = f.state.p[a] - ptrue[a];
      sum += e * e;
    }
    double alignment = 0;
    for (int a = 0; a < 4; a++)
      alignment += f.state.q[a] * qtrue[a];
    alignment = fabs(alignment);
    if (alignment > 1)
      alignment = 1;
    double angle = 2 * acos(alignment);
    angle_sum += angle * angle;
    samples++;
  }
  unsigned drain = 0;
  while ((f.catching_up || f.has_pending) && drain++ < 100)
    se_service(&f);
  assert(!f.catching_up && !f.has_pending);
  assert(f.state.time_us == 11000000);
  printf("{\"delay_ms\":%u,\"stress\":%u,\"rmse_m\":%.12g,\"attitude_rmse_"
         "rad\":%.12g,\"fused\":%u,\"replayed\":%u,\"max_replay\":%u,\"gate_"
         "rejections\":%u,\"memory_bytes\":%zu,\"pending\":false,\"state\":[",
         delay, stress, sqrt(sum / samples), sqrt(angle_sum / samples), f.fused,
         f.replayed, f.max_replay, f.rejected[SE_GATE], sizeof(f));
  const se_real *arrays[] = {f.state.p, f.state.v, f.state.q, f.state.bg,
                             f.state.ba};
  unsigned sizes[] = {3, 3, 4, 3, 3}, written = 0;
  for (int a = 0; a < 5; a++)
    for (unsigned j = 0; j < sizes[a]; j++)
      printf("%s%.12g", written++ ? "," : "", (double)arrays[a][j]);
  printf("],\"covariance\":[");
  for (int a = 0; a < 15; a++)
    for (int j = 0; j < 15; j++)
      printf("%s%.12g", a || j ? "," : "", (double)f.state.P[a][j]);
  puts("]}");
  assert(sqrt(sum / samples) < .025);
  assert(sqrt(angle_sum / samples) < .025);
  return 0;
}
