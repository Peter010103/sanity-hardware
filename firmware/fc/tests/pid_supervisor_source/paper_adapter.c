/* Selectable measured-speed adapter; reference command-force branch untouched.
 * MSP receive and PID are cooperative scheduler tasks. Raw decode uses bounded
 * seqlock publication; no force observer state is reused. BF aligned gyro is
 * FLU (installed estimator convention); FRD=diag(1,-1,-1)*FLU, det=+1.
 */
#include "platform.h"
#ifdef USE_SANITY_INDI
#include "common/maths.h"
#include "config/config.h"
#include "core.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/time.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/mixer_init.h"
#include "flight/pid.h"
#include "model.h"
#include "paper_adapter.h"
#include "paper_core.h"
#include "paper_rpm.h"
#include "pg/motor.h"
#include "pg/sanity_indi.h"
#include "pg/sanity_paper.h"
#include "pg/sanity_rate_ff.h"
#include "rate_feedforward.h"
#include "rx/rx.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
#include <math.h>
#include <string.h>
static spState_t paper;
static spNative_t native;
static spRpmHistory_t rpm;
static spRpmSelection_t selection;
static shState_t policy;
static sanityPaperConfig_t saved;
static sanityRateFfConfig_t savedRateFf;
static srFfState_t rateFf;
static sanityIndiConfig_t supervisorConfig;
static uint32_t configHash;
static uint32_t period, previous, stableAt, slewStarted, slewDuration, guardAt,
    badTakeoffAt;
static float applied[4];
static bool haveApplied, configured, configValid, armedLast, observationsValid,
    slewActive;
/* Engineering timing envelope, not a measured hardware qualification. One
 * anomalous tick per 100ms; both dt and previous-runtime flags share one event.
 * A second anomalous catchup tick is deliberately rejected. */
#define SP_TIMING_QUIET_US 100000U
#define SP_TIMING_SHORT 1U
#define SP_TIMING_LONG 2U
#define SP_TIMING_RUNTIME 4U
#define SP_TIMING_HARD 8U
#define SP_TIMING_REPEAT 16U
#define SP_TIMING_GRACE 32U
#define SP_TIMING_UNREADY 64U
static struct {
  uint32_t events, graces, rejects, reseeds, lastUs;
  bool haveLast;
  uint32_t first[6], rejected[6]; /* seq, now, dt, prevPID, flags, adapterUs */
} timing;
static uint8_t backend;
static bool solveCurrent, solveValid;
static uint16_t currentReason = 1;
static uint32_t voltageStamp;
static float deliveredVoltage;
static bool haveVoltage;
static int32_t trace[SP_TRACE_COUNT];
static float clip(float x, float l, float h) { return fminf(h, fmaxf(l, x)); }
static float toCount(float x) {
  return x < DSHOT_MIN_THROTTLE ? 1000
                                : scaleRangef(x, DSHOT_MIN_THROTTLE,
                                              DSHOT_MAX_THROTTLE, 1001, 2000);
}
static float fromCount(float x) {
  return x <= 1000 ? DSHOT_CMD_MOTOR_STOP
                   : scaleRangef(clip(x, 1001, 2000), 1001, 2000,
                                 DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
}
void sanityPaperHousekeeping(uint32_t now) {
  spNativePoll(&native, now);
  spRpmPoll(&rpm, now, 1000000U);
}
uint8_t sanityPaperBackend(bool armed) {
  if (!armed && backend != sanityPaperConfig()->backend) {
    backend = sanityPaperConfig()->backend;
    native.accepted.flags = 0;
    native.accepted.claimed = false;
    native.expired = true;
    native.accepted.expiryUs = micros();
    native.consumedSession = native.consumedSequence = native.consumedUs = 0;
  }
  return backend;
}
bool sanityPaperTraceSelected(void) { return backend == 1; }
bool sanityPaperConfigured(void) { return configured && configValid; }
bool sanityPaperInitialized(void) { return paper.initialized; }
void sanityPaperRecordVoltage(uint32_t stamp, float voltage) {
  voltageStamp = stamp;
  deliveredVoltage = voltage;
  haveVoltage = true;
}
void sanityPaperRecordErpm(unsigned m, uint32_t stamp, uint32_t raw) {
  spRpmRecord(&rpm, m, stamp, raw);
}
bool sanityPaperReceive(const uint8_t *p, unsigned n, uint32_t now) {
  if (n == SP_NATIVE_SIZE && (p[7] & 1) &&
      (backend != 1 || sanityPaperConfig()->inputSource != 0)) {
    native.rejected++;
    native.reason = 4;
    return false;
  }
  return spNativeReceive(&native, p, n, now, ARMING_FLAG(ARMED));
}
static bool authority(void) {
  return rxIsReceivingSignal() && !(flightModeFlags & ~FAILSAFE_MODE);
}
bool sanityPaperFallbackRate(unsigned a, float *out) {
  if (a >= 3 || backend != 1 || saved.inputSource != 0 || !authority() ||
      !native.accepted.claimed)
    return false;
  /* During controlled return retain the same bounded companion identity. */
  if (!spNativeValid(&native, micros()) && !(policy.weight > 0))
    return false;
  *out = native.accepted.rate[a] * (a ? -1.0f : 1.0f) * (180.0f / M_PIf);
  return true;
}
static bool configure(bool armed) {
  sanityPaperConfig_t c = *sanityPaperConfig();
  sanityRateFfConfig_t ff = *sanityRateFfConfig();
  sanityIndiConfig_t sup = *sanityIndiConfig();
  sup.traceEnabled = 0;
  if (configured && period == targetPidLooptime &&
      !memcmp(&c, &saved, sizeof(c)) &&
      !memcmp(&ff, &savedRateFf, sizeof(ff)) &&
      !memcmp(&sup, &supervisorConfig, sizeof(sup)) && (configValid || armed))
    return configValid;
  if (configured && armed) {
    configValid = false;
    return false;
  }
  if (configured && saved.inputSource != c.inputSource) {
    native.accepted.flags = 0;
    native.accepted.claimed = false;
    native.expired = true;
    native.accepted.expiryUs = micros();
    native.consumedSession = native.consumedSequence = native.consumedUs = 0;
  }
  saved = c;
  savedRateFf = ff;
  supervisorConfig = sup;
  // Hash the identical saved bytes only when a disarmed configuration is
  // adopted. Trace flag-only changes are already excluded from this copy.
  configHash = 2166136261U;
  const uint8_t *configBytes = (const uint8_t *)&saved;
  for (unsigned i = 0; i < sizeof(saved); i++)
    configHash = (configHash ^ configBytes[i]) * 16777619U;
  const uint8_t *supervisorBytes = (const uint8_t *)&supervisorConfig;
  for (unsigned i = 0; i < sizeof(supervisorConfig); i++)
    configHash = (configHash ^ supervisorBytes[i]) * 16777619U;
  const uint8_t *ffBytes = (const uint8_t *)&savedRateFf;
  for (unsigned i = 0; i < sizeof(savedRateFf); i++)
    configHash = (configHash ^ ffBytes[i]) * 16777619U;
  period = targetPidLooptime;
  configured = true;
  configValid = false;
  stableAt = 0;
  srFfConfig_t ffConfig = {.dt = period * 1e-6f, .cutoffHz = ff.cutoffHz};
  for (unsigned a = 0; a < 3; a++) {
    ffConfig.gain[a] = ff.gainMilli[a] * .001f;
    ffConfig.limit[a] = ff.alphaLimit[a];
  }
  if (!srFfInit(&rateFf, &ffConfig))
    return false;
  if ((period != 500 && period != 1000) || c.backend != 1 ||
      c.inputSource > 1 || !c.servoEnabled || c.cutoffHz < 1 ||
      c.cutoffHz > 100 || c.rpmMaxAgeUs < 100 || c.rpmMaxAgeUs > 20000 ||
      c.rpmMaxSkewUs > 20000 || c.voltageMaxAgeMs < 10 ||
      c.voltageMaxAgeMs > 1000 || c.omegaMin >= c.omegaMax ||
      c.omegaMax > 5883 || c.omegaMax < 100 || c.maxIterations < 1 ||
      c.maxIterations > 20 || c.warmupMs > 1000 || !c.countSlew)
    return false;
  if (sup.transitionMs < 50 || sup.transitionMs > 1000 || sup.requestAux < 1 ||
      sup.requestAux > 4 || sup.entryDwellMs < 100 || sup.entryDwellMs > 3000 ||
      sup.entryDeltaPercent < 1 || sup.entryDeltaPercent > 20 ||
      !sup.outputStepPermille || sup.outputStepPermille > 20 ||
      sup.entryRateDps < 5 || sup.entryRateDps > 100 || sup.entryTiltDeg < 2 ||
      sup.entryTiltDeg > 30 || sup.guardTimeMs < 50 || sup.guardTimeMs > 500 ||
      sup.guardWindowMs < 1000 || sup.guardWindowMs > 30000)
    return false;
  spConfig_t cfg = {0};
  cfg.dt = period * 1e-6f;
  cfg.cutoffHz = c.cutoffHz;
  cfg.inertia[0] = SI_IXX;
  cfg.inertia[1] = SI_IYY;
  cfg.inertia[2] = SI_IZZ;
  cfg.kp = c.kpMilli * .001f;
  cfg.ki = c.kiMilli * .001f;
  cfg.kaw = c.kaw;
  cfg.integralLimit = c.integralLimit;
  cfg.countSlewPerSec = c.countSlew;
  cfg.maxIterations = c.maxIterations;
  cfg.warmupTicks = (uint32_t)c.warmupMs * 1000 / period;
  const float x[4] = {-SI_ARM_X_M, SI_ARM_X_M, -SI_ARM_X_M, SI_ARM_X_M};
  const float y[4] = {SI_ARM_Y_M, SI_ARM_Y_M, -SI_ARM_Y_M, -SI_ARM_Y_M};
  const float yaw[4] = {-1, 1, 1, -1};
  const float bfYawSign = mixerConfig()->yaw_motors_reversed ? 1 : -1;
  for (unsigned m = 0; m < 4; m++) {
    cfg.g1[0][m] = SI_KT;
    cfg.g1[1][m] = -y[m] * SI_KT;
    cfg.g1[2][m] = x[m] * SI_KT;
    cfg.g1[3][m] = -bfYawSign * yaw[m] * SI_KQ;
    cfg.g2[3][m] = -bfYawSign * yaw[m] * (float)c.rotorInertiaNano[m] * 1e-9f;
    cfg.omegaMin[m] = c.omegaMin;
    cfg.omegaMax[m] = c.omegaMax;
  }
  cfg.rowScale[0] = 1;
  cfg.rowScale[1] = cfg.rowScale[2] = .02f;
  cfg.rowScale[3] = .002f;
  cfg.priority[0] = 4;
  cfg.priority[1] = cfg.priority[2] = 8;
  cfg.priority[3] = 1;
  configValid = spInit(&paper, &cfg);
  return configValid;
}
static void trackPid(float outMin, float outRange, float mixScale) {
  float target[3] = {0};
  const float yawSign = mixerConfig()->yaw_motors_reversed ? 1 : -1;
  for (unsigned m = 0; m < 4; m++) {
    const float out = (motor[m] - outMin) / outRange * .25f * 1000;
    target[0] += out * mixerRuntime.currentMixer[m].roll;
    target[1] += out * mixerRuntime.currentMixer[m].pitch;
    target[2] += out * mixerRuntime.currentMixer[m].yaw * yawSign;
  }
  for (unsigned a = 0; a < 3; a++) {
    target[a] /= mixScale;
  }
  pidTrackSanityOutput(target);
}
static void numericStop(void) {
  setArmingDisabled(ARMING_DISABLED_RUNAWAY_TAKEOFF);
  disarm(DISARM_REASON_RUNAWAY_TAKEOFF);
  stableAt = 0;
  policy.weight = 0;
  policy.mode = SH_PID;
  policy.fault = policy.lastFault = policy.reason = SH_NUMERIC;
  for (unsigned m = 0; m < 4; m++)
    motor[m] = mixerRuntime.disarmMotorOutput;
}
static int32_t quant(float x, float scale) {
  x *= scale;
  if (!isfinite(x)) {
    trace[5] |= 1 << 16;
    return 0;
  }
  if (x > 1e9f) {
    trace[5] |= 1 << 17;
    x = 1e9f;
  } else if (x < -1e9f) {
    trace[5] |= 1 << 17;
    x = -1e9f;
  }
  return lrintf(x);
}
void sanityPaperApply(timeUs_t now, float minimum, float maximum,
                      float collectiveOutput, float outputMinimum,
                      float outputRange, float pidMixScale, bool motorStop,
                      sanityIndiStatus_t *s) {
  const uint32_t start = micros();
  const bool armed = ARMING_FLAG(ARMED), enabled = sanityIndiConfig()->enabled;
  const uint32_t dt = previous ? now - previous : targetPidLooptime;
  previous = now;
  s->sequence++;
  s->enabled = enabled;
  s->armed = armed;
  s->eligible = false;
  s->readyUs = 0;
  const bool valid = configure(armed);
  const sanityIndiConfig_t *c = &supervisorConfig;
  s->supported = getMotorCount() == 4 &&
                 mixerConfig()->mixerMode == MIXER_QUADX &&
                 !mixerRuntime.feature3dEnabled && isMotorProtocolDshot() &&
                 motorConfig()->dev.motorPwmProtocol == PWM_TYPE_DSHOT300 &&
                 motorConfig()->motorPoleCount == 12 &&
                 motorConfig()->dev.useDshotTelemetry &&
                 currentPidProfile->thrustLinearization == 0 &&
                 !currentPidProfile->use_integrated_yaw &&
                 isfinite(pidMixScale) && pidMixScale > 0 && outputRange > 0;
  if (!haveApplied) {
    for (unsigned m = 0; m < 4; m++)
      applied[m] = motor[m];
    haveApplied = true;
  }
  if (armed != armedLast) {
    if (configured)
      spReset(&paper);
    stableAt = 0;
    guardAt = now;
    badTakeoffAt = 0;
    srFfReset(&rateFf);
    timing.haveLast = false;
    if (armed) {
      memset(timing.first, 0, sizeof(timing.first));
      memset(timing.rejected, 0, sizeof(timing.rejected));
    }
  }
  armedLast = armed;
  bool firstTimingEvent = false, firstTimingReject = false;
  unsigned timingFlags = 0;
  if (enabled && armed) {
    if (dt < targetPidLooptime / 2) timingFlags |= SP_TIMING_SHORT;
    if (dt > targetPidLooptime * 3 / 2) timingFlags |= SP_TIMING_LONG;
    if (s->cycleUs > targetPidLooptime) timingFlags |= SP_TIMING_RUNTIME;
  }
  bool timingGrace = false;
  if (timingFlags) {
    timing.events++;
    if (dt < targetPidLooptime / 4 || dt > targetPidLooptime * 5 / 2 ||
        s->cycleUs > targetPidLooptime * 7 / 4)
      timingFlags |= SP_TIMING_HARD;
    if (timing.haveLast && (uint32_t)(now - timing.lastUs) < SP_TIMING_QUIET_US)
      timingFlags |= SP_TIMING_REPEAT;
    if (!paper.observed || !paper.ready) timingFlags |= SP_TIMING_UNREADY;
    timingGrace = !(timingFlags & (SP_TIMING_HARD | SP_TIMING_REPEAT | SP_TIMING_UNREADY));
    timing.lastUs = now;
    timing.haveLast = true;
    if (timingGrace) { timing.graces++; timingFlags |= SP_TIMING_GRACE; }
    else timing.rejects++;
    const uint32_t event[6] = {s->sequence, now, dt, s->cycleUs, timingFlags, 0};
    if (!timing.first[0]) {
      memcpy(timing.first, event, sizeof(event)); firstTimingEvent = true;
    }
    if (!timingGrace && !timing.rejected[0]) {
      memcpy(timing.rejected, event, sizeof(event)); firstTimingReject = true;
    }
    /* Neither accepted nor rejected timing gaps may retain reference history. */
    srFfReset(&rateFf);
  }
  const bool nativeSource = saved.inputSource == 0;
  const bool demand = nativeSource ? spNativeValid(&native, now) : true;
  s->requestAux = c->requestAux;
  s->requested = demand && (!nativeSource || (native.accepted.flags & 2)) &&
                 c->requestAux >= 1 && c->requestAux <= 4 &&
                 rcData[4 + c->requestAux - 1] > 1700;
  float rate[3] = {0}, desiredRate[3] = {0}, desiredAlpha[3] = {0}, maxRate = 0,
        maxError = 0;
  float feedbackAlpha[3] = {0};
  bool numeric = true;
  bool observed = false, allocated = false, servo = false, ffActive = false;
  solveCurrent = false;
  solveValid = false;
  float desiredCollective = native.accepted.collective;
  float previousUsed[4] = {0};
  UNUSED(collectiveOutput);
  for (unsigned a = 0; a < 3; a++) {
    rate[a] = DEGREES_TO_RADIANS(gyro.sanityIndiGyro[a]) * (a ? -1 : 1);
    s->rate[a] = DEGREES_TO_RADIANS(
        gyro.gyroADCf[a]); /* Existing authority packet stays BF/FLU. */
    desiredRate[a] =
        nativeSource ? native.accepted.rate[a]
                     : DEGREES_TO_RADIANS(getSetpointRate(a)) * (a ? -1 : 1);
    desiredAlpha[a] = nativeSource ? native.accepted.alpha[a]
                                   : (float)c->rateGains[a] * .1f *
                                         (desiredRate[a] - rate[a]);
    if (!nativeSource)
      feedbackAlpha[a] = desiredAlpha[a];
    const float guardDemand = desiredRate[a];
    maxRate = fmaxf(maxRate, fabsf(gyro.gyroADCf[a]));
    maxError = fmaxf(maxError, fabsf(guardDemand - rate[a]) * 180 / M_PIf);
    numeric = numeric && isfinite(rate[a]) && isfinite(gyro.gyroADCf[a]);
  }
  for (unsigned m = 0; m < 4; m++)
    numeric = numeric && isfinite(motor[m]);
  const float low = toCount(minimum), high = toCount(maximum),
              voltage = deliveredVoltage;
  if (enabled && armed && !numeric) {
    srFfReset(&rateFf);
    numericStop();
    goto finish;
  }
  uint8_t reason = SH_OK;
  currentReason = 0;
  if (!enabled) {
    reason = SH_DISABLED;
    currentReason = 1;
  } else if (!valid || !s->supported || low < 1100 || high > 2000 ||
             low >= high) {
    reason = SH_CONFIG;
    currentReason = 4;
  } else if (!haveVoltage || (int32_t)(now - voltageStamp) < 0 ||
             now - voltageStamp > (uint32_t)saved.voltageMaxAgeMs * 1000 ||
             !isfinite(voltage) || voltage <= 0) {
    reason = SH_CONFIG;
    currentReason = 6;
  } else if (!authority()) {
    reason = SH_RX;
    currentReason = 3;
  } else if (isFlipOverAfterCrashActive() || isLaunchControlActive()) {
    reason = SH_MODE;
    currentReason = 3;
  } else if (timingFlags && !timingGrace) {
    reason = SH_TIMING;
    currentReason = 10;
  } else if (!demand) {
    reason = SH_RX;
    currentReason = native.accepted.claimed ? 2 : 1;
  } else if (motorStop || rcData[THROTTLE] < SI_RC_MIN) {
    reason = SH_IDLE;
    currentReason = 3;
  }
  const bool rpmValid = spRpmSelect(&rpm, now, saved.rpmMaxAgeUs,
                                    saved.rpmMaxSkewUs, 6, &selection);
  s->rpmMask = selection.validMask;
  if (!nativeSource)
    desiredCollective = siRequestedCollective(
        rcData[THROTTLE],
        (uint16_t)clip(voltage * 1000, 0,
                       65535)); /* Approximate old RC->physical collective
                                   interface; native bypasses this. */
  /* Invalid observation gaps never enter derivative/filter history. */
  if (valid && s->supported && numeric && rpmValid &&
      ((!timingFlags && dt >= targetPidLooptime / 2 &&
        dt <= targetPidLooptime * 3 / 2) || timingGrace)) {
    observed = timingGrace ? spReseedObservation(&paper, selection.omega, rate)
                           : spObserve(&paper, selection.omega, rate);
    if (observed && timingGrace) timing.reseeds++;
  } else if (configured)
    spReset(&paper);
  if (reason == SH_OK && !observed) {
    reason = SH_RPM;
    currentReason = 5;
  }
  if (reason == SH_OK && !paper.ready) {
    reason = SH_WARMUP;
    currentReason = 5;
  }
  /* Reference differentiation tracks healthy compatibility PID-shadow ticks
   * too. Invalid gaps and native T+alpha commands cannot retain FF history. */
  ffActive = enabled && armed && !nativeSource && reason == SH_OK &&
      observed && paper.ready && valid && s->supported;
  if (!srFfUpdate(&rateFf, desiredRate, ffActive) && ffActive) {
    numericStop();
    goto finish;
  }
  if (ffActive)
    for (unsigned a = 0; a < 3; a++)
      if (savedRateFf.gainMilli[a])
        desiredAlpha[a] += rateFf.alpha[a];
  if (reason == SH_OK && observed && paper.ready && demand && valid && s->supported) {
    if (policy.weight == 0)
      for (unsigned m = 0; m < 4; m++)
        paper.omegaPrevious[m] = selection.omega[m];
    memcpy(previousUsed, paper.omegaPrevious, sizeof(previousUsed));
    solveCurrent = true;
    allocated = spAllocate(&paper, desiredCollective, desiredAlpha);
    if (allocated) {
      float previousCount[4];
      for (unsigned m = 0; m < 4; m++)
        previousCount[m] = toCount(applied[m]);
      servo = spServo(&paper, selection.omega, voltage, low, high,
                      previousCount, policy.weight == 0);
    }
    if (reason == SH_OK && !allocated) {
      reason = SH_ALLOCATOR;
      currentReason = 7;
    }
    if (reason == SH_OK && allocated && !servo) {
      reason = SH_CONFIG;
      currentReason = 11; /* Hard servo/model rejection; finite support loss is degraded. */
    }
  }
  s->collectiveN = desiredCollective;
  s->requestedThrustN = s->collectiveN;
  s->shadowValid = allocated && servo;
  s->trackingError = 0;
  if (servo)
    for (unsigned m = 0; m < 4; m++)
      s->trackingError =
          fmaxf(s->trackingError,
                fabsf(toCount(motor[m]) - paper.servoCount[m]) / 900);
  if (enabled && armed && valid &&
      (uint32_t)(now - guardAt) < (uint32_t)c->guardWindowMs * 1000) {
    float mn = motor[0], mx = motor[0];
    for (unsigned m = 1; m < 4; m++) {
      mn = fminf(mn, motor[m]);
      mx = fmaxf(mx, motor[m]);
    }
    if (!(maxRate > c->guardRateDps && maxError > c->guardErrorDps &&
          (mx - mn) > .05f * outputRange && rcData[THROTTLE] >= SI_RC_MIN))
      badTakeoffAt = 0;
    else if (!badTakeoffAt)
      badTakeoffAt = now;
    else if ((uint32_t)(now - badTakeoffAt) >=
             (uint32_t)c->guardTimeMs * 1000) {
      numericStop();
      policy.fault = policy.lastFault = policy.reason = SH_TAKEOFF;
      goto finish;
    }
  }
  uint8_t entry = reason;
  // Readiness means consecutive healthy observations. Preserve only the
  // SH_WARMUP produced below by this entry dwell, never an underlying fault
  // or the observer's own warmup after an invalid observation gap.
  if (reason != SH_OK)
    stableAt = 0;
  if (entry == SH_OK && policy.weight == 0) {
    if (maxRate > c->entryRateDps ||
        getCosTiltAngle() < cosf(DEGREES_TO_RADIANS(c->entryTiltDeg)))
      entry = SH_MOTION;
    else if (s->trackingError > c->entryDeltaPercent * .01f)
      entry = SH_TRACKING;
    if (entry == SH_OK) {
      if (!stableAt)
        stableAt = now;
      if (now - stableAt < (uint32_t)c->entryDwellMs * 1000)
        entry = SH_WARMUP;
    } else
      stableAt = 0;
  }
  s->eligible = entry == SH_OK;
  s->readyUs = stableAt ? now - stableAt : 0;
  const float oldWeight = policy.weight;
  uint8_t fault = policy.weight > 0 && reason != SH_OK && reason != SH_MODE &&
                          reason != SH_IDLE && reason != SH_DISABLED
                      ? reason
                      : 0;
  const shInput_t in = {.armed = armed,
                        .enabled = enabled,
                        .request = s->requested,
                        .eligible = s->eligible,
                        .reason = entry,
                        .fault = fault,
                        .nowUs = now,
                        .dtUs = targetPidLooptime,
                        .transitionUs =
                            (uint32_t)(valid ? c->transitionMs : 200) * 1000};
  shUpdate(&policy, &in);
  if (oldWeight != policy.weight) {
    slewStarted = now;
    slewDuration = (uint32_t)c->transitionMs * 1000;
    slewActive = true;
  }
  if (slewActive && (uint32_t)(now - slewStarted) >= slewDuration)
    slewActive = false;
  if (!armed)
    slewActive = false;
  if (armed && !motorStop && policy.weight > 0 && s->supported) {
    for (unsigned m = 0; m < 4; m++) {
      const float pidCount = toCount(motor[m]);
      const float indiCount =
          reason == SH_OK && servo ? paper.servoCount[m] : toCount(applied[m]);
      const float count =
          (1 - policy.weight) * pidCount + policy.weight * indiCount;
      motor[m] = clip(fromCount(count), minimum, maximum);
    }
    if (reason == SH_OK && allocated && servo) {
      spCommit(&paper);
      if (nativeSource)
        spNativeConsume(&native, now);
      s->steps++;
    }
  }
  if (armed && !motorStop && valid && slewActive) {
    const float delta = c->outputStepPermille * .001f * outputRange *
                        ((float)targetPidLooptime / 500);
    for (unsigned m = 0; m < 4; m++) {
      const float previousBounded = clip(applied[m], minimum, maximum);
      motor[m] = clip(motor[m], fmaxf(minimum, previousBounded - delta),
                      fminf(maximum, previousBounded + delta));
    }
  }
  if (servo) {
    float finalCount[4];
    for (unsigned m = 0; m < 4; m++)
      finalCount[m] = toCount(motor[m]);
    spTrack(&paper, finalCount);
  }
  if (armed && (policy.weight > 0 || slewActive) && s->supported)
    trackPid(outputMinimum, outputRange, pidMixScale);
finish:
  observationsValid = observed && paper.ready;
  solveValid = allocated;
  for (unsigned m = 0; m < 4; m++) {
    applied[m] = motor[m];
  }
  s->active = policy.weight > 0 && ARMING_FLAG(ARMED);
  s->armed = ARMING_FLAG(ARMED);
  s->weight = policy.weight;
  s->mode = policy.mode;
  s->reason = policy.reason;
  s->fault = policy.fault;
  s->lastFault = policy.lastFault;
  s->transitions = policy.transitions;
  s->fallbacks = policy.fallbacks;
  s->solverExit = paper.solverCode;
  s->iterations = paper.iterations;
  s->saturatedMask = paper.saturatedMask;
  /* Distinct compact p[] schema, fixed at log start, all floats quantized
   * below. */
  memset(trace, 0, sizeof(trace));
  trace[0] = SP_TRACE_VERSION;
  trace[1] = s->sequence;
  trace[2] = now;
  trace[3] = targetPidLooptime;
  trace[4] = dt;
  trace[5] = (observed ? 1 : 0) | (allocated ? 2 : 0) | (servo ? 4 : 0) |
             (demand ? 8 : 0);
  trace[6] = s->mode;
  trace[7] = currentReason;
  trace[8] = paper.solverCode;
  trace[9] = paper.iterations;
  trace[10] = selection.validMask;
  trace[11] = selection.newMask;
  trace[12] = selection.skew;
  trace[13] = paper.saturatedMask;
  trace[14] = quant(s->weight, 1e6f);
  trace[15] = quant(voltage, 1000);
  trace[16] = paper.voltageSupport;
  trace[17] = SP_NATIVE_MODEL;
  trace[18] = native.accepted.session;
  trace[19] = native.accepted.sequence;
  trace[20] = native.consumedSession;
  trace[21] = native.consumedSequence;
  trace[22] = native.accepted.receiptUs;
  trace[23] = native.consumedUs;
  trace[24] = quant(desiredCollective, 1e6f);
  for (unsigned a = 0; a < 3; a++) {
    trace[25 + a] = quant(desiredAlpha[a], 1000);
    trace[28 + a] = quant(rate[a], 1e6f);
    trace[31 + a] = quant(paper.bodyFiltered[a], 1e6f);
    trace[34 + a] = quant(paper.alphaFiltered[a], 1000);
    trace[37 + a] = quant(paper.tauFiltered[a], 1e9f);
    trace[40 + a] = quant(paper.tauDesired[a], 1e9f);
    trace[43 + a] = quant(paper.g2Torque[a], 1e9f);
    trace[46 + a] = quant(desiredRate[a], 1e6f);
  }
  for (unsigned m = 0; m < 4; m++) {
    trace[49 + m] = selection.selected[m].raw;
    trace[53 + m] = selection.selected[m].stamp;
    trace[57 + m] = selection.age[m];
    trace[61 + m] = selection.selectedSequence[m];
    trace[65 + m] = quant(selection.omega[m], 1000);
    trace[69 + m] = quant(paper.omegaFiltered[m], 1000);
    trace[73 + m] = quant(paper.omegaCommand[m], 1000);
    trace[77 + m] = quant(previousUsed[m], 1000);
    trace[81 + m] = quant(paper.residual[m], m ? 1e9f : 1e6f);
    trace[85 + m] = quant(paper.servoError[m], 1000);
    trace[89 + m] = quant(paper.servoP[m], 1000);
    trace[93 + m] = quant(paper.servoI[m], 1000);
    trace[97 + m] = quant(paper.feedforward[m], 1000);
    trace[101 + m] = quant(paper.servoCount[m], 1000);
    trace[105 + m] = quant(toCount(applied[m]), 1000);
    trace[109] += rpm.motor[m].duplicates;
    trace[110] += rpm.motor[m].outOfOrder;
    trace[111] += rpm.motor[m].invalid;
  }
  trace[121] = native.rejected;
  trace[122] = selection.futureMask;
  trace[123] = selection.missingMask;
  trace[124] = s->cycleUs;
  trace[125] = s->maxCycleUs;
  /* v2 appends diagnostics; the original 126-field prefix retains its positions. */
  trace[126] = quant(low, 1000);
  trace[127] = quant(high, 1000);
  for (unsigned m = 0; m < 4; m++) {
    trace[128 + m] = quant(paper.feedforwardCandidate[m], 1000);
    trace[132 + m] = paper.ffReason[m];
    trace[137 + m] = paper.ffModelFlags[m];
    trace[141 + m] = paper.ffRecoveryTicks[m];
  }
  trace[136] = paper.ffValidMask | (paper.ffCandidateMask << 4) |
      (paper.ffDegradedMask << 8) | (paper.ffHeldMask << 12) |
      (paper.ffRecoveringMask << 16) | (paper.ffSeededMask << 20) |
      (paper.servoISaturatedMask << 24);
  trace[145] = paper.ffDegradedEntries;
  trace[146] = paper.ffRecoveries;
  trace[147] = paper.servoISaturationTicks;
  trace[148] = paper.servoFailure;
  trace[149] = SP_FF_ACCEPT_TRANSFER;
  trace[150] = spFeedforwardModelFingerprint();
  trace[151] = quant(SP_FF_RECOVERY_DWELL_S, 1e6f);
  /* v3 preserves all 152 v2 offsets, appending rate-reference FF diagnostics. */
  for (unsigned a = 0; a < 3; a++) {
    trace[152 + a] = quant(feedbackAlpha[a], 1000);
    trace[155 + a] = quant(rateFf.raw[a], 1000);
    trace[158 + a] = quant(rateFf.derivative[a], 1000);
    trace[161 + a] = quant(rateFf.alpha[a], 1000);
    trace[164 + a] = savedRateFf.gainMilli[a];
    trace[168 + a] = quant(savedRateFf.alphaLimit[a], 1000);
  }
  trace[167] = savedRateFf.cutoffHz;
  trace[171] = (rateFf.configured ? 1 : 0) | (rateFf.primed ? 2 : 0) |
      (rateFf.seeded ? 4 : 0) | (ffActive ? 8 : 0);
  trace[172] = rateFf.clippedMask;
  trace[173] = rateFf.resets;
  /* v4 event latches survive decimation; fields0..173 keep their offsets. */
  trace[174] = timing.events;
  trace[175] = timing.graces;
  trace[176] = timing.rejects;
  for (unsigned i = 0; i < 6; i++) {
    trace[177 + i] = timing.first[i];
    trace[183 + i] = timing.rejected[i];
  }
  trace[189] = timing.reseeds;
  int32_t referenceTiming[SI_TRACE_COUNT];
  sanityIndiTraceSnapshot(referenceTiming);
  trace[112] = referenceTiming[SI_TRACE_DEADLINES];
  trace[113] = referenceTiming[SI_TRACE_LATE_INTERVALS];
  trace[114] = referenceTiming[SI_TRACE_LOG_DROPS];
  trace[115] = configHash;
  trace[116] = 1 | (saved.inputSource << 8);
  for (unsigned m = 0; m < 4; m++)
    if (saved.rotorInertiaNano[m])
      trace[116] |= 1 << 16;
  trace[117] = (uint32_t)native.accepted.epoch;
  trace[118] = native.accepted.epoch >> 32;
  // Include trace quantization, snapshots and configuration hashing. The
  // timestamp precedes only its own two trace stores, avoiding self timing.
  const uint32_t elapsed = micros() - start;
  s->lastRuntimeUs = elapsed > UINT16_MAX ? UINT16_MAX : elapsed;
  if (s->lastRuntimeUs > s->maxRuntimeUs)
    s->maxRuntimeUs = s->lastRuntimeUs;
  trace[119] = s->lastRuntimeUs;
  trace[120] = s->maxRuntimeUs;
  if (firstTimingEvent) trace[182] = timing.first[5] = elapsed;
  if (firstTimingReject) trace[188] = timing.rejected[5] = elapsed;
}
void sanityPaperTraceSnapshot(int32_t fields[SP_TRACE_COUNT]) {
  memcpy(fields, trace, sizeof(trace));
}
void sanityPaperEncodeStatus(uint8_t out[80], uint32_t now) {
  memset(out, 0, 80);
  memcpy(out, "SIPS", 4);
  out[4] = 2;
  out[5] = backend;
  out[6] = 1;
  bool g2 = false;
  for (unsigned m = 0; m < 4; m++)
    g2 = g2 || saved.rotorInertiaNano[m] > 0;
  const bool demand = spNativeValid(&native, now);
  const bool currentObservation =
      observationsValid && period &&
      (uint32_t)(now - previous) <= period * 3U / 2U;
  out[7] = 1 | (demand ? 2 : 0) | (policy.weight > 0 ? 4 : 0) | (g2 ? 8 : 0) |
           (saved.servoEnabled ? 16 : 0) | (ARMING_FLAG(ARMED) ? 32 : 0) |
           (currentObservation ? 64 : 0);
  spPut32(out + 8, SP_NATIVE_MODEL);
  spPut32(out + 12, targetPidLooptime);
  spPut64(out + 16, native.accepted.epoch);
  spPut32(out + 24, native.accepted.session);
  spPut32(out + 28, native.accepted.sequence);
  spPut32(out + 32, native.consumedSession);
  spPut32(out + 36, native.consumedSequence);
  spPut32(out + 40, native.accepted.receiptUs);
  spPut32(out + 44, native.consumedUs);
  spPut16(out + 48, demand ? (native.accepted.expiryUs - now) / 1000 : 0);
  spPut16(out + 50, currentReason);
  out[52] = selection.validMask;
  out[53] = paper.saturatedMask;
  out[54] = currentObservation && solveCurrent ? paper.solverCode : 255;
  for (unsigned a = 0; a < 4; a++) {
    spPutFloat(out + 56 + 4 * a,
               currentObservation && solveValid && isfinite(paper.residual[a])
                   ? paper.residual[a]
                   : 0);
  }
  spPut32(out + 72, native.rejected);
  out[76] = paper.voltageSupport;
  out[77] = paper.ffDegradedMask;
  out[78] = paper.ffRecoveringMask;
  out[79] = paper.ffSeededMask;
}
#endif
