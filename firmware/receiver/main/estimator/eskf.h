#ifndef SANITY_ESKF_H
#define SANITY_ESKF_H
/* Allocation-free observation-only ESKF. No controller or actuator
 * dependencies. */
#include <stdbool.h>
#include <stdint.h>
#ifdef SE_DOUBLE
typedef double se_real;
#else
typedef float se_real;
#endif
#define SE_N 15
#define SE_HISTORY 52
#define SE_REPLAY_BUDGET 8
#define SE_GRAVITY ((se_real)9.80665)
#define SE_MAX_CLOCK_US 2000
#define SE_MAX_GAP_US 15000
#define SE_MEMORY_BUDGET (96 * 1024)
enum se_fault {
  SE_OK = 0,
  SE_CLOCK = 1,
  SE_EPOCH = 2,
  SE_CALIBRATION = 3,
  SE_GAP = 4,
  SE_ORDER = 5,
  SE_TOO_OLD = 6,
  SE_GATE = 7,
  SE_NUMERIC = 8,
  SE_NOT_READY = 9,
  SE_OVERFLOW = 10
};
typedef struct {
  uint64_t time_us, epoch;
  uint32_t sequence, uncertainty_us;
  se_real gyro[3], accel[3];
  bool calibrated, disarmed;
} se_imu;
typedef struct {
  uint64_t time_us, epoch;
  uint32_t sequence, uncertainty_us;
  se_real p[3], q[4];
} se_pose;
typedef struct {
  se_real p[3], v[3], q[4], bg[3], ba[3], P[SE_N][SE_N];
  uint64_t time_us;
} se_state;
typedef struct {
  se_state state;
  se_imu imu;
} se_entry;
typedef struct {
  se_entry history[SE_HISTORY];
  se_state state, work;
  se_pose pending, init_pose;
  bool has_init_pose;
  se_real F[SE_N][SE_N], tmp[SE_N][SE_N], next[SE_N][SE_N];
  uint64_t epoch, last_pose_us;
  uint32_t last_pose_sequence, last_imu_sequence;
  uint32_t propagated, fused, rejected[11], pending_replaced, replayed,
      max_replay;
  unsigned head, count, cursor, remaining;
  bool ready, catching_up, has_pending;
  se_real innovation[6], nis;
  unsigned stationary_count;
  se_real gyro_sum[3];
  enum se_fault last_fault;
} se_filter;
void se_reset(se_filter *f, uint64_t epoch);
bool se_initialize(se_filter *f, const se_imu *imu, const se_pose *pose);
bool se_push_imu(se_filter *f, const se_imu *imu);
bool se_submit_pose(se_filter *f, const se_pose *pose);
unsigned
se_service(se_filter *f); /* at most SE_REPLAY_BUDGET propagation steps */
bool se_state_valid(const se_state *s);
#endif
