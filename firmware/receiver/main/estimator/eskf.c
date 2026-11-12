#include "eskf.h"
#include <math.h>
#include <string.h>
#ifdef SE_DOUBLE
#define se_sqrt sqrt
#define se_sin sin
#define se_cos cos
#define se_fabs fabs
#else
#define se_sqrt sqrtf
#define se_sin sinf
#define se_cos cosf
#define se_fabs fabsf
#endif
static bool newer(uint32_t a, uint32_t b) { return (int32_t)(a - b) > 0; }
#ifndef SE_DOUBLE
_Static_assert(sizeof(se_filter) <= SE_MEMORY_BUDGET,
               "ESKF static memory exceeds 96 KiB");
#endif
static bool reject(se_filter *f, enum se_fault e) {
  f->last_fault = e;
  f->rejected[e]++;
  return false;
}
static se_real dot(const se_real *a, const se_real *b, unsigned n) {
  se_real s = 0;
  for (unsigned i = 0; i < n; i++)
    s += a[i] * b[i];
  return s;
}
static bool normq(se_real *q) {
  se_real n = se_sqrt(dot(q, q, 4));
  if (!isfinite(n) || n < (se_real)0.5 || n > (se_real)1.5)
    return false;
  for (int i = 0; i < 4; i++)
    q[i] /= n;
  return true;
}
static void mulq(const se_real *a, const se_real *b, se_real *out) {
  se_real t[4] = {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
  memcpy(out, t, sizeof(t));
}
static void rot(const se_real *q, se_real R[3][3]) {
  se_real w = q[0], x = q[1], y = q[2], z = q[3];
  R[0][0] = 1 - 2 * (y * y + z * z);
  R[0][1] = 2 * (x * y - w * z);
  R[0][2] = 2 * (x * z + w * y);
  R[1][0] = 2 * (x * y + w * z);
  R[1][1] = 1 - 2 * (x * x + z * z);
  R[1][2] = 2 * (y * z - w * x);
  R[2][0] = 2 * (x * z - w * y);
  R[2][1] = 2 * (y * z + w * x);
  R[2][2] = 1 - 2 * (x * x + y * y);
}
static void skew(const se_real *a, se_real s[3][3]) {
  s[0][0] = s[1][1] = s[2][2] = 0;
  s[0][1] = -a[2];
  s[0][2] = a[1];
  s[1][0] = a[2];
  s[1][2] = -a[0];
  s[2][0] = -a[1];
  s[2][1] = a[0];
}
bool se_state_valid(const se_state *s) {
  const se_real *v =
      s->p; /* each array is checked explicitly, no struct aliasing */
  for (int i = 0; i < 3; i++)
    if (!isfinite(v[i]) || !isfinite(s->v[i]) || !isfinite(s->bg[i]) ||
        !isfinite(s->ba[i]))
      return false;
  se_real qn = dot(s->q, s->q, 4);
  if (!isfinite(qn) || se_fabs(qn - 1) > (se_real)0.001)
    return false;
  for (int i = 0; i < SE_N; i++)
    for (int j = 0; j < SE_N; j++)
      if (!isfinite(s->P[i][j]) ||
          se_fabs(s->P[i][j] - s->P[j][i]) > (se_real)0.001 ||
          (i == j && s->P[i][i] <= 0))
        return false;
  se_real L[SE_N][SE_N] = {{0}};
  for (int i = 0; i < SE_N; i++)
    for (int j = 0; j <= i; j++) {
      se_real v = s->P[i][j];
      for (int k = 0; k < j; k++)
        v -= L[i][k] * L[j][k];
      if (i == j) {
        if (!isfinite(v) || v <= 0)
          return false;
        L[i][j] = se_sqrt(v);
      } else
        L[i][j] = v / L[j][j];
    }
  return true;
}
void se_reset(se_filter *f, uint64_t epoch) {
  memset(f, 0, sizeof(*f));
  f->epoch = epoch;
  f->state.q[0] = 1;
}
static bool timing(se_filter *f, const se_imu *i) {
  if (i->epoch != f->epoch)
    return reject(f, SE_EPOCH);
  if (i->uncertainty_us > SE_MAX_CLOCK_US)
    return reject(f, SE_CLOCK);
  if (!i->calibrated)
    return reject(f, SE_CALIBRATION);
  for (int a = 0; a < 3; a++)
    if (!isfinite(i->gyro[a]) || !isfinite(i->accel[a]))
      return reject(f, SE_NUMERIC);
  return true;
}
bool se_initialize(se_filter *f, const se_imu *i, const se_pose *p) {
  if (f->ready)
    return true;
  if (!timing(f, i)) {
    f->stationary_count = 0;
    memset(f->gyro_sum, 0, sizeof(f->gyro_sum));
    return false;
  }
  if (f->stationary_count && (i->time_us <= f->work.time_us ||
                              i->time_us - f->work.time_us > SE_MAX_GAP_US)) {
    f->stationary_count = 0;
    memset(f->gyro_sum, 0, sizeof(f->gyro_sum));
  }
  f->work.time_us = i->time_us;
  if (!i->disarmed || dot(i->gyro, i->gyro, 3) > (se_real)0.04 ||
      se_fabs(se_sqrt(dot(i->accel, i->accel, 3)) - SE_GRAVITY) >
          (se_real)0.3) {
    f->stationary_count = 0;
    memset(f->gyro_sum, 0, sizeof(f->gyro_sum));
    return reject(f, SE_NOT_READY);
  }
  if (f->stationary_count && !newer(i->sequence, f->last_imu_sequence))
    return reject(f, SE_ORDER);
  f->last_imu_sequence = i->sequence;
  for (int a = 0; a < 3; a++)
    f->gyro_sum[a] += i->gyro[a];
  f->stationary_count++;
  if (p && p->epoch == f->epoch && p->uncertainty_us <= SE_MAX_CLOCK_US) {
    if (f->has_init_pose) {
      se_real distance = 0;
      for (int a = 0; a < 3; a++) {
        se_real d = p->p[a] - f->init_pose.p[a];
        distance += d * d;
      }
      se_real alignment = se_fabs(dot(p->q, f->init_pose.q, 4));
      if (distance > (se_real).0004 || alignment < (se_real).9998) {
        f->stationary_count = 0;
        memset(f->gyro_sum, 0, sizeof(f->gyro_sum));
        f->init_pose = *p;
        return reject(f, SE_NOT_READY);
      }
    } else {
      f->init_pose = *p;
      f->has_init_pose = true;
      f->stationary_count = 1;
      memcpy(f->gyro_sum, i->gyro, sizeof(f->gyro_sum));
    }
  }
  if (f->stationary_count < 200)
    return false;
  if (!p || p->epoch != f->epoch || p->uncertainty_us > SE_MAX_CLOCK_US ||
      p->time_us > i->time_us || i->time_us - p->time_us > 250000) {
    f->stationary_count = 0;
    f->has_init_pose = false;
    memset(f->gyro_sum, 0, sizeof(f->gyro_sum));
    return reject(f, SE_CLOCK);
  }
  se_state *s = &f->state;
  memset(s, 0, sizeof(*s));
  memcpy(s->q, p->q, sizeof(s->q));
  if (!normq(s->q))
    return reject(f, SE_NUMERIC);
  for (int a = 0; a < 3; a++) {
    if (!isfinite(p->p[a]))
      return reject(f, SE_NUMERIC);
    s->p[a] = p->p[a];
    s->bg[a] = f->gyro_sum[a] / f->stationary_count;
  }
  for (int a = 0; a < SE_N; a++)
    s->P[a][a] = a < 3    ? (se_real)0.0004
                 : a < 6  ? (se_real)0.04
                 : a < 9  ? (se_real)0.0025
                 : a < 12 ? (se_real)0.0001
                          : (se_real)0.25;
  s->time_us = i->time_us;
  f->history[0].state = *s;
  f->history[0].imu = *i;
  f->head = 0;
  f->count = 1;
  f->ready = true;
  f->last_pose_us = p->time_us;
  f->last_pose_sequence = p->sequence;
  f->last_fault = SE_OK;
  return true;
}
/* Right multiplicative body-frame attitude error; continuous F discretized
 * first order. Measurement noise: sigma_p=(se_real).02 m,
 * sigma_theta=(se_real).025 rad; IMU densities below. */
static bool propagate(se_filter *f, se_state *s, const se_imu *i,
                      uint64_t until) {
  if (until <= s->time_us)
    return false;
  se_real dt = (until - s->time_us) * (se_real)1e-6;
  if (dt > (se_real)0.015)
    return false;
  se_real w[3], a[3], R[3][3], A[3][3], W[3][3], aw[3];
  for (int j = 0; j < 3; j++) {
    w[j] = i->gyro[j] - s->bg[j];
    a[j] = i->accel[j] - s->ba[j];
  }
  rot(s->q, R);
  skew(a, A);
  skew(w, W);
  for (int j = 0; j < 3; j++) {
    aw[j] = dot(R[j], a, 3) - (j == 2 ? SE_GRAVITY : 0);
    s->p[j] += s->v[j] * dt + (se_real)0.5 * aw[j] * dt * dt;
    s->v[j] += aw[j] * dt;
  }
  se_real angle = se_sqrt(dot(w, w, 3)) * dt;
  se_real dq[4] = {se_cos(angle / 2), 0, 0, 0};
  se_real scale =
      angle < (se_real)1e-8 ? dt / 2 : se_sin(angle / 2) * dt / angle;
  for (int j = 0; j < 3; j++)
    dq[j + 1] = w[j] * scale;
  mulq(s->q, dq, s->q);
  if (!normq(s->q))
    return false;
  memset(f->F, 0, sizeof(f->F));
  for (int j = 0; j < SE_N; j++)
    f->F[j][j] = 1;
  for (int j = 0; j < 3; j++) {
    f->F[j][j + 3] = dt;
    f->F[j + 6][j + 9] = -dt;
    for (int k = 0; k < 3; k++) {
      se_real ra = 0;
      for (int l = 0; l < 3; l++)
        ra += R[j][l] * A[l][k];
      f->F[j + 3][k + 6] = -ra * dt;
      f->F[j + 3][k + 12] = -R[j][k] * dt;
      f->F[j + 6][k + 6] -= W[j][k] * dt;
    }
  }
  for (int j = 0; j < SE_N; j++)
    for (int k = 0; k < SE_N; k++) {
      se_real x = 0;
      for (int l = 0; l < SE_N; l++)
        x += f->F[j][l] * s->P[l][k];
      f->tmp[j][k] = x;
    }
  for (int j = 0; j < SE_N; j++)
    for (int k = 0; k < SE_N; k++) {
      se_real x = 0;
      for (int l = 0; l < SE_N; l++)
        x += f->tmp[j][l] * f->F[k][l];
      f->next[j][k] = x;
    }
  for (int j = 0; j < SE_N; j++) {
    se_real density = j < 3    ? (se_real)1e-8
                      : j < 6  ? (se_real)0.0064
                      : j < 9  ? (se_real)0.0001
                      : j < 12 ? (se_real)1e-8
                               : (se_real)1e-6;
    f->next[j][j] += density * dt;
  }
  for (int j = 0; j < SE_N; j++)
    for (int k = 0; k < SE_N; k++)
      s->P[j][k] = (f->next[j][k] + f->next[k][j]) / 2;
  s->time_us = until;
  return se_state_valid(s);
}
bool se_push_imu(se_filter *f, const se_imu *i) {
  if (!f->ready)
    return reject(f, SE_NOT_READY);
  if (!timing(f, i))
    return false;
  se_entry *last = &f->history[f->head];
  if (!newer(i->sequence, f->last_imu_sequence) ||
      i->time_us <= last->state.time_us)
    return reject(f, SE_ORDER);
  if (i->time_us - last->state.time_us > SE_MAX_GAP_US) {
    f->ready = false;
    f->stationary_count = 0;
    f->has_init_pose = false;
    memset(f->gyro_sum, 0, sizeof(f->gyro_sum));
    return reject(f, SE_GAP);
  }
  if (f->catching_up && f->count == SE_HISTORY &&
      ((f->head + 1) % SE_HISTORY) == f->cursor) {
    f->ready = false;
    return reject(f, SE_OVERFLOW);
  }
  unsigned next = (f->head + 1) % SE_HISTORY;
  f->history[next].state = last->state;
  if (!propagate(f, &f->history[next].state, i, i->time_us)) {
    f->ready = false;
    return reject(f, SE_NUMERIC);
  }
  f->history[next].imu = *i;
  f->head = next;
  if (f->count < SE_HISTORY)
    f->count++;
  if (f->catching_up)
    f->remaining++;
  else
    f->state = f->history[next].state;
  f->last_imu_sequence = i->sequence;
  f->propagated++;
  return true;
}
/* Cholesky solves, no matrix inverse. Joseph covariance update. */
static bool update(se_filter *f, se_state *s, const se_pose *p) {
  int idx[6] = {0, 1, 2, 6, 7, 8};
  se_real r[6], qc[4] = {s->q[0], -s->q[1], -s->q[2], -s->q[3]}, dq[4], pq[4];
  memcpy(pq, p->q, sizeof(pq));
  if (!normq(pq))
    return reject(f, SE_NUMERIC);
  mulq(qc, pq, dq);
  se_real sign = dq[0] < 0 ? -1 : 1;
  for (int j = 0; j < 3; j++) {
    r[j] = p->p[j] - s->p[j];
    r[j + 3] = 2 * sign * dq[j + 1];
    if (!isfinite(r[j]))
      return reject(f, SE_NUMERIC);
  }
  se_real L[6][6] = {{0}},
          noise[6] = {(se_real).0004,   (se_real).0004,   (se_real).0004,
                      (se_real).000625, (se_real).000625, (se_real).000625};
  for (int j = 0; j < 6; j++)
    for (int k = 0; k <= j; k++) {
      se_real x = s->P[idx[j]][idx[k]] + (j == k ? noise[j] : 0);
      for (int l = 0; l < k; l++)
        x -= L[j][l] * L[k][l];
      if (j == k) {
        if (x <= 0 || !isfinite(x))
          return reject(f, SE_NUMERIC);
        L[j][k] = se_sqrt(x);
      } else
        L[j][k] = x / L[k][k];
    }
  se_real y[6];
  for (int j = 0; j < 6; j++) {
    se_real x = r[j];
    for (int k = 0; k < j; k++)
      x -= L[j][k] * y[k];
    y[j] = x / L[j][j];
  }
  f->nis = dot(y, y, 6);
  memcpy(f->innovation, r, sizeof(r));
  if (!isfinite(f->nis) || f->nis > (se_real)22.458)
    return reject(f, SE_GATE);
  se_real K[15][6], dx[15] = {0};
  for (int row = 0; row < 15; row++) {
    for (int j = 0; j < 6; j++) {
      se_real x = s->P[row][idx[j]];
      for (int k = 0; k < j; k++)
        x -= L[j][k] * y[k];
      y[j] = x / L[j][j];
    }
    for (int j = 5; j >= 0; j--) {
      se_real x = y[j];
      for (int k = j + 1; k < 6; k++)
        x -= L[k][j] * K[row][k];
      K[row][j] = x / L[j][j];
    }
    dx[row] = dot(K[row], r, 6);
  }
  for (int j = 0; j < 3; j++) {
    s->p[j] += dx[j];
    s->v[j] += dx[j + 3];
    s->bg[j] += dx[j + 9];
    s->ba[j] += dx[j + 12];
  }
  se_real correction[4] = {1, dx[6] / 2, dx[7] / 2, dx[8] / 2};
  normq(correction);
  mulq(s->q, correction, s->q);
  if (!normq(s->q))
    return reject(f, SE_NUMERIC);
  memset(f->F, 0, sizeof(f->F));
  for (int j = 0; j < 15; j++) {
    f->F[j][j] = 1;
    for (int k = 0; k < 6; k++)
      f->F[j][idx[k]] -= K[j][k];
  }
  for (int j = 0; j < 15; j++)
    for (int k = 0; k < 15; k++) {
      se_real x = 0;
      for (int l = 0; l < 15; l++)
        x += f->F[j][l] * s->P[l][k];
      f->tmp[j][k] = x;
    }
  for (int j = 0; j < 15; j++)
    for (int k = 0; k < 15; k++) {
      se_real x = 0;
      for (int l = 0; l < 15; l++)
        x += f->tmp[j][l] * f->F[k][l];
      for (int l = 0; l < 6; l++)
        x += K[j][l] * noise[l] * K[k][l];
      f->next[j][k] = x;
    }
  /* Error reset Jacobian after injecting attitude correction. */
  se_real theta[3] = {dx[6], dx[7], dx[8]}, S[3][3];
  skew(theta, S);
  memset(f->F, 0, sizeof(f->F));
  for (int j = 0; j < 15; j++)
    f->F[j][j] = 1;
  for (int j = 0; j < 3; j++)
    for (int k = 0; k < 3; k++)
      f->F[j + 6][k + 6] -= (se_real).5 * S[j][k];
  for (int j = 0; j < 15; j++)
    for (int k = 0; k < 15; k++) {
      se_real x = 0;
      for (int l = 0; l < 15; l++)
        x += f->F[j][l] * f->next[l][k];
      f->tmp[j][k] = x;
    }
  for (int j = 0; j < 15; j++)
    for (int k = 0; k < 15; k++) {
      se_real x = 0;
      for (int l = 0; l < 15; l++)
        x += f->tmp[j][l] * f->F[k][l];
      s->P[j][k] = x;
    }
  for (int j = 0; j < 15; j++)
    for (int k = j + 1; k < 15; k++)
      s->P[j][k] = s->P[k][j] = (s->P[j][k] + s->P[k][j]) / 2;
  return se_state_valid(s);
}
bool se_submit_pose(se_filter *f, const se_pose *p) {
  if (!f->ready)
    return reject(f, SE_NOT_READY);
  if (p->epoch != f->epoch)
    return reject(f, SE_EPOCH);
  if (p->uncertainty_us > SE_MAX_CLOCK_US)
    return reject(f, SE_CLOCK);
  if (p->time_us <= f->last_pose_us ||
      !newer(p->sequence, f->last_pose_sequence))
    return reject(f, SE_ORDER);
  if (f->has_pending) {
    if (p->time_us <= f->pending.time_us)
      return reject(f, SE_ORDER);
    f->pending_replaced++;
  }
  f->pending = *p;
  f->has_pending = true;
  return true;
}
unsigned se_service(se_filter *f) {
  if (!f->ready)
    return 0;
  if (!f->catching_up && f->has_pending) {
    se_pose *p = &f->pending;
    uint64_t newest = f->history[f->head].state.time_us;
    if (p->time_us > newest)
      return 0;
    unsigned oldest = (f->head + SE_HISTORY + 1 - f->count) % SE_HISTORY;
    f->has_pending = false;
    if (p->time_us < f->history[oldest].state.time_us) {
      reject(f, SE_TOO_OLD);
      return 0;
    }
    unsigned at = oldest;
    while (at != f->head &&
           f->history[(at + 1) % SE_HISTORY].state.time_us <= p->time_us)
      at = (at + 1) % SE_HISTORY;
    f->work = f->history[at].state;
    if (p->time_us > f->work.time_us &&
        !propagate(f, &f->work, &f->history[(at + 1) % SE_HISTORY].imu,
                   p->time_us)) {
      reject(f, SE_NUMERIC);
      return 0;
    }
    if (!update(f, &f->work, p))
      return 0;
    f->history[at].state = f->work;
    f->cursor = (at + 1) % SE_HISTORY;
    f->remaining = (f->head + SE_HISTORY - at) % SE_HISTORY;
    f->last_pose_us = p->time_us;
    f->last_pose_sequence = p->sequence;
    f->fused++;
    f->catching_up = f->remaining > 0;
    if (!f->catching_up) {
      f->state = f->work;
      return 0;
    }
  }
  unsigned done = 0;
  while (f->catching_up && done < SE_REPLAY_BUDGET) {
    se_entry *e = &f->history[f->cursor];
    if (!propagate(f, &f->work, &e->imu, e->imu.time_us)) {
      f->ready = false;
      reject(f, SE_NUMERIC);
      break;
    }
    e->state = f->work;
    f->cursor = (f->cursor + 1) % SE_HISTORY;
    f->remaining--;
    done++;
    if (!f->remaining)
      f->catching_up = false;
  }
  if (done) {
    f->state = f->work;
    f->replayed += done;
    if (done > f->max_replay)
      f->max_replay = done;
  }
  return done;
}
