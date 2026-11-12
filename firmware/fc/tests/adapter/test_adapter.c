#include "adapter_env.h"
#include "drivers/dshot_command.h"
#include "flight/sanity_indi.h"
#include "flight/sanity_indi/paper_adapter.h"
#include "pg/sanity_indi.h"
#include "pg/sanity_paper.h"
#include "pg/sanity_rate_ff.h"
#include "flight/sanity_indi/paper_core.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
uint32_t testArmedFlags, flightModeFlags,
    targetPidLooptime = 500, testClock = 1000000, testArmingDisabled;
mixerRuntime_t mixerRuntime = {
    .disarmMotorOutput = 0,
    .currentMixer = {
        {1, -1, 1, -1}, {1, -1, -1, 1}, {1, 1, 1, 1}, {1, 1, -1, -1}}};
gyro_t gyro;
telemetry_t dshotTelemetryState;
float motor[4], rcData[8] = {1500, 1500, 1500, 1300, 1000, 2000, 1000, 1000};
static pidProfile_t pid;
const pidProfile_t *currentPidProfile = &pid;
static mixerConfig_t mixer = {MIXER_QUADX, true};
static motorConfig_t motors = {{PWM_TYPE_DSHOT300, 1}, 12};
static sanityIndiConfig_t cfg = {.rateGains = {100, 100, 80},
                                 .syncCutoffHz = 15,
                                 .motorTauMs = {20, 24, 20, 20},
                                 .maxIterations = 16,
                                 .requireRpm = 1,
                                 .rpmTimeoutMs = 25,
                                 .axisWeights = {50, 50, 50, 5},
                                 .requestAux = 3,
                                 .shadowDenom = 4,
                                 .transitionMs = 200,
                                 .entryDwellMs = 100,
                                 .entryRateDps = 40,
                                 .entryTiltDeg = 15,
                                 .entryDeltaPercent = 10,
                                 .outputStepPermille = 5,
                                 .guardWindowMs = 10000,
                                 .guardRateDps = 250,
                                 .guardErrorDps = 150,
                                 .guardTimeMs = 150};
static sanityPaperConfig_t paperCfg = {.backend = 0,
                                       .servoEnabled = 1,
                                       .maxIterations = 16,
                                       .cutoffHz = 12,
                                       .rpmMaxAgeUs = 5000,
                                       .rpmMaxSkewUs = 5000,
                                       .warmupMs = 100,
                                       .omegaMin = 100,
                                       .voltageMaxAgeMs = 500,
                                       .omegaMax = 5883,
                                       .kpMilli = 40,
                                       .kiMilli = 1000,
                                       .kaw = 10,
                                       .integralLimit = 200,
                                       .countSlew = 5000};
static sanityRateFfConfig_t rateFfCfg = {.gainMilli={0,0,0},.cutoffHz=20,.alphaLimit={20,20,20}};
const sanityRateFfConfig_t *sanityRateFfConfig(void) {return &rateFfCfg;}
const sanityPaperConfig_t *sanityPaperConfig(void) { return &paperCfg; }
static uint32_t tick = 1000000, tracked;
static float testMixScale = 1;
static float testLowCount = 1101, testHighCount = 2000;
static bool testVoltageFresh = true;
static float testVoltage = 8;
static float baseline, spread, tilt = 1, setpoint[3];
static bool signal = true;
const sanityIndiConfig_t *sanityIndiConfig(void) { return &cfg; }
static bool probeTraceReady, sawPreparedTrace;
static uint32_t probeSequence;
uint32_t micros(void) {
  if (probeTraceReady && !sawPreparedTrace) {
    int32_t fields[SP_TRACE_COUNT];
    sanityPaperTraceSnapshot(fields);
    if ((uint32_t)fields[1] == probeSequence) {
      assert(fields[115] != 0 && fields[119] == 0 && fields[120] == 0);
      sawPreparedTrace = true; // final runtime sample sees prepared trace/hash
    }
  }
  return testClock++;
}
unsigned getMotorCount(void) { return 4; }
bool isMotorProtocolDshot(void) { return true; }
bool isFlipOverAfterCrashActive(void) { return false; }
bool isLaunchControlActive(void) { return false; }
const mixerConfig_t *mixerConfig(void) { return &mixer; }
const motorConfig_t *motorConfig(void) { return &motors; }
float getSetpointRate(unsigned a) {
  float nativeRate;
  if (sanityPaperFallbackRate(a, &nativeRate))
    return nativeRate;
  return setpoint[a];
}
void setArmingDisabled(uint32_t f) { testArmingDisabled |= f; }
void unsetArmingDisabled(uint32_t f) { testArmingDisabled &= ~f; }
void disarm(int r) {
  (void)r;
  testArmedFlags = 0;
}
uint16_t getBatteryVoltage(void) { return 800; }

static struct {
  float P, I, D, F, Sum;
} pidData[3];
static struct {
  float itermLimit;
} pidRuntime = {400};
static bool featureIsEnabled(int feature) {
  (void)feature;
  return mixerRuntime.feature3dEnabled;
}
static int constrain(int x, int a, int b) { return x < a ? a : x > b ? b : x; }
#include "extracted_adapter_functions.c"
uint16_t motorConvertToExternal(float x) { return dshotConvertToExternal(x); }
float motorConvertFromExternal(uint16_t x) {
  return dshotConvertFromExternal(x);
}

uint16_t getDshotErpm(unsigned m) {
  (void)m;
  return 1000;
}
bool rxIsReceivingSignal(void) { return signal; }
float getCosTiltAngle(void) { return tilt; }
void pidTrackSanityOutput(const float target[3]) {
  actualPidTrackSanityOutput(target);
  for (unsigned a = 0; a < 3; a++) {
    assert(isfinite(pidData[a].I));
    if (fabsf(target[a] - pidData[a].P - pidData[a].D - pidData[a].F) <= 400)
      assert(fabsf(pidData[a].Sum - target[a]) < .001f);
    else
      assert(fabsf(pidData[a].I) == 400);
  }
  tracked++;
}
static uint32_t testInterval, testPidExtra;
static uint32_t testRaw[4]={1000,1000,1000,1000};
static void step(bool rpm) {
  tick += testInterval ? testInterval : targetPidLooptime;
  testInterval = 0;
  testClock = tick;
  if (testVoltageFresh)
    sanityPaperRecordVoltage(tick, testVoltage);
  if (rpm)
    for (unsigned m = 0; m < 4; ++m) {
      dshotTelemetryState.motorState[m].sanityErpmTimestampUs = tick - 10;
      sanityPaperRecordErpm(m, tick - 10, testRaw[m]);
    }
  for (unsigned m = 0; m < 4; ++m)
    motor[m] = testArmedFlags ? baseline + (m < 2 ? -spread : spread) : 0;
  sanityIndiCycleStart();
#if 1
  sanityIndiApply(tick, scaleRangef(testLowCount,1001,2000,DSHOT_MIN_THROTTLE,DSHOT_MAX_THROTTLE),
                  scaleRangef(testHighCount,1001,2000,DSHOT_MIN_THROTTLE,DSHOT_MAX_THROTTLE),
                  dshotConvertFromExternal(1300),
                  dshotConvertFromExternal(1101),
                  2047 - dshotConvertFromExternal(1101), testMixScale, false);
#else
  sanityIndiApply(tick, scaleRangef(testLowCount,1001,2000,DSHOT_MIN_THROTTLE,DSHOT_MAX_THROTTLE),
                  scaleRangef(testHighCount,1001,2000,DSHOT_MIN_THROTTLE,DSHOT_MAX_THROTTLE),
                  dshotConvertFromExternal(1300),
                  dshotConvertFromExternal(1101),
                  2047 - dshotConvertFromExternal(1101), false);
#endif
  testClock += testPidExtra; testPidExtra = 0;
  sanityIndiCycleEnd();
}
static void reset(void) {
  testArmedFlags = 0;
  cfg.enabled = 0;
  testMixScale = 1;
  flightModeFlags = 0;
  testArmingDisabled = 0;
  rcData[6] = 1000;
  baseline = dshotConvertFromExternal(1300);
  spread = 0;
  signal = true;
  tilt = 1;
  for (unsigned a = 0; a < 3; a++) {
    pidData[a].P = 7 + a;
    pidData[a].D = -2 - a;
    pidData[a].F = 3;
    pidData[a].I = -8;
    pidData[a].Sum = 0;
  }
  memset(&gyro, 0, sizeof(gyro));
  memset(setpoint, 0, sizeof(setpoint));
  memset(&dshotTelemetryState, 0, sizeof(dshotTelemetryState));
  step(true);
  sanityIndiUpdateArmingStatus();
}
static void engage(void) {
  reset();
  cfg.enabled = 1;
  testArmedFlags = ARMED;
  for (unsigned n = 0; n < 1000; ++n)
    step(true);
  assert(sanityIndiStatus()->eligible);
  rcData[6] = 2000;
  for (unsigned n = 0; n < 1000; ++n)
    step(true);
  assert(sanityIndiStatus()->mode == SH_INDI && sanityIndiStatus()->active);
  assert(tracked > 0 && testArmedFlags);
}
int regression_main(void) {
  reset();
  testArmedFlags = ARMED;
  for (unsigned n = 0; n < 500; ++n)
    step(true);
  assert(!sanityIndiActive());
  for (unsigned m = 0; m < 4; ++m)
    assert(motor[m] == dshotConvertFromExternal(1300));
  reset();
  cfg.enabled = 1;
  rcData[6] = 2000;
  testArmedFlags = ARMED;
  for (unsigned n = 0; n < 1000; ++n)
    step(true);
  assert(!sanityIndiActive() && sanityIndiStatus()->reason == SH_REQUEST_RESET);
  engage();
  rcData[6] = 1000;
  baseline = dshotConvertFromExternal(1400);
  float previous[4];
  memcpy(previous, motor, sizeof(previous));
  for (unsigned n = 0; n < 1200; ++n) {
    step(true);
    for (unsigned m = 0; m < 4; ++m) {
      assert(fabsf(motor[m] - previous[m]) <=
             (2047 - dshotConvertFromExternal(1101)) * .005f + .01f);
      previous[m] = motor[m];
    }
  }
  assert(!sanityIndiActive() && testArmedFlags);
  for (unsigned m = 0; m < 4; ++m)
    assert(motor[m] == dshotConvertFromExternal(1400));
  engage();
  flightModeFlags = 1;
  for (unsigned n = 0; n < 1200; ++n)
    step(true);
  assert(!sanityIndiActive() && testArmedFlags && !testArmingDisabled);
  engage();
  for (unsigned n = 0; n < 1300; ++n)
    step(false);
  assert(testArmedFlags && !sanityIndiActive() &&
         sanityIndiStatus()->fault == SH_RPM);
  assert(!testArmingDisabled);
  engage();
  tick += 4 * targetPidLooptime;
  step(true);
  for (unsigned n = 0; n < 1200; ++n)
    step(true);
  assert(testArmedFlags && !sanityIndiActive() &&
         sanityIndiStatus()->fault == SH_TIMING);
  engage();
  for (unsigned n = 0; n < 3; ++n) {
    testClock = tick;
    sanityIndiCycleStart();
    testClock += 480;
    sanityIndiCycleEnd();
    step(true);
  }
  for (unsigned n = 0; n < 1200; ++n)
    step(true);
  assert(testArmedFlags && !sanityIndiActive() &&
         sanityIndiStatus()->fault == SH_TIMING);
  engage();
  gyro.sanityIndiGyro[0] = NAN;
  for (unsigned n = 0; n < 1200; ++n)
    step(true);
  assert(testArmedFlags && !sanityIndiActive() &&
         sanityIndiStatus()->fault == SH_NUMERIC);
  engage();
  gyro.gyroADCf[0] = NAN;
  step(true);
  assert(!testArmedFlags &&
         (testArmingDisabled & ARMING_DISABLED_RUNAWAY_TAKEOFF));
  for (unsigned m = 0; m < 4; ++m)
    assert(motor[m] == 0);
  reset();
  cfg.enabled = 1;
  testArmedFlags = ARMED;
  spread = dshotConvertFromExternal(1400) - dshotConvertFromExternal(1300);
  gyro.gyroADCf[0] = 300;
  for (unsigned n = 0; n < 1000 && testArmedFlags; ++n)
    step(true);
  assert(!testArmedFlags && sanityIndiStatus()->guardTriggered &&
         sanityIndiStatus()->lastFault == SH_TAKEOFF);
  targetPidLooptime = 500;
  engage();
  rcData[6] = 1000;
  for (unsigned n = 0; n < 1000; ++n)
    step(true);
  assert(!sanityIndiActive() && testArmedFlags);
  reset();
  cfg.enabled = 1;
  motors.motorPoleCount = 14;
  testArmedFlags = ARMED;
  for (unsigned n = 0; n < 100; ++n)
    step(true);
  assert(testArmedFlags && !sanityIndiActive() && !testArmingDisabled);
  motors.motorPoleCount = 12;
  reset();
  cfg.enabled = 1;
  mixerRuntime.feature3dEnabled = true;
  testArmedFlags = ARMED;
  for (unsigned n = 0; n < 100; ++n)
    step(true);
  assert(!sanityIndiStatus()->supported && !sanityIndiActive());
  mixerRuntime.feature3dEnabled = false;
  reset();
  cfg.enabled = 1;
  motors.dev.motorPwmProtocol = 5;
  testArmedFlags = ARMED;
  for (unsigned n = 0; n < 100; ++n)
    step(true);
  assert(!sanityIndiStatus()->supported && !sanityIndiActive());
  motors.dev.motorPwmProtocol = PWM_TYPE_DSHOT300;

#if 1
  const float invalidScale[] = {0, NAN, INFINITY, -1};
  for (unsigned index = 0; index < 4; index++) {
    reset();
    cfg.enabled = 1;
    testArmedFlags = ARMED;
    testMixScale = invalidScale[index];
    for (unsigned n = 0; n < 1000; n++)
      step(true);
    rcData[6] = 2000;
    for (unsigned n = 0; n < 1000; n++)
      step(true);
    assert(!sanityIndiStatus()->supported && !sanityIndiActive());
    for (unsigned m = 0; m < 4; m++)
      assert(motor[m] == baseline);
  }
  reset();
  testArmedFlags = ARMED;
  testMixScale = NAN;
  for (unsigned n = 0; n < 1000; n++)
    step(true);
  for (unsigned m = 0; m < 4; m++)
    assert(motor[m] == baseline);
  reset();
  cfg.enabled = 1;
  testArmedFlags = ARMED;
  testMixScale = .75f;
  for (unsigned n = 0; n < 1200; n++)
    step(true);
  rcData[6] = 2000;
  for (unsigned n = 0; n < 1200; n++)
    step(true);
  assert(sanityIndiActive());
  rcData[6] = 1000;
  for (unsigned n = 0; n < 1200; n++)
    step(true);
  assert(!sanityIndiActive() && testArmedFlags);
  for (unsigned m = 0; m < 4; m++)
    assert(motor[m] == baseline);
#endif
  uint8_t packet[SANITY_INDI_PACKET_SIZE];
  sanityIndiEncode(packet);
  FILE *f = fopen("controller_fixture.bin", "wb");
  assert(f);
  assert(fwrite(packet, 1, sizeof(packet), f) == sizeof(packet));
  fclose(f);
  puts("PASS: actual adapter + real DShot conversion + actual nonzero P/D/F "
       "PIDtracking: PID baseline, no arm-time engagement, live "
       "handover/return, bounded motor change, mode/RPM/timing/allocator "
       "fallback, critical gyro and takeoff protection");
  return 0;
}

static uint32_t read32(const uint8_t *p) {
  return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
         (uint32_t)p[3] << 24;
}
static void save(const char *name, const void *data, size_t size) {
  FILE *f = fopen(name, "wb");
  assert(f);
  assert(fwrite(data, 1, size, f) == size);
  fclose(f);
}
static void test_trace_timing(void) {
  targetPidLooptime = 500;
  engage();
  uint8_t before[80], after[80];
  sanityIndiEncodeTiming(before);
  assert(!memcmp(before, "SIT1", 4) && before[4] == 1 &&
         read32(before + 16) == 500);
  assert(!sanityIndiTraceEnabled());
  cfg.traceEnabled = 1;
  step(true);
  assert(sanityIndiTraceEnabled());
  gyro.sanityIndiGyro[0] = 1.25f;
  gyro.gyroADCf[0] = .5f;
  setpoint[0] = 2.5f;
  step(true);
  int32_t fields[SI_TRACE_COUNT];
  sanityIndiTraceSnapshot(fields);
  assert(fields[SI_TRACE_FLAGS] & SI_TRACE_OBSERVED);
  assert(fields[SI_TRACE_FLAGS] & SI_TRACE_ALLOCATED);
  assert(fields[SI_TRACE_SETPOINT0] ==
         lrintf(DEGREES_TO_RADIANS(2.5f) * 100000));
  assert(fields[SI_TRACE_EARLY0] == lrintf(DEGREES_TO_RADIANS(1.25f) * 100000));
  assert(fields[SI_TRACE_FINAL0] == lrintf(DEGREES_TO_RADIANS(.5f) * 100000));
  assert(fields[SI_TRACE_NU0] == lrintf(10 * DEGREES_TO_RADIANS(2) * 1000));
  const float k = tanf(M_PIf * 15 * .0005f),
              b0 = k * k / (1 + sqrtf(2) * k + k * k);
  assert(labs(fields[SI_TRACE_ALPHA0] -
              lrintf(DEGREES_TO_RADIANS(1.25f) / .0005f * b0 * 1000)) <= 1);
  for (unsigned m = 0; m < 4; m++) {
    assert(fields[SI_TRACE_PREVIOUS_FORCE0 + m] > 0);
    assert(fields[SI_TRACE_MODEL_FORCE0 + m] > 0);
    assert(fields[SI_TRACE_FILTERED_FORCE0 + m] > 0);
    assert(fields[SI_TRACE_APPLIED0 + m] >= 1100000);
    assert(fields[SI_TRACE_RPM_AGE0 + m] < 200);
  }
  save("trace_fixture_state.bin", fields, sizeof(fields));
  // Per-log latch keeps capture alive when the requested flag changes, and
  // flag-only changes preserve filter dynamics instead of reseeding them.
  sanityIndiTraceSetRecording(true);
  cfg.traceEnabled = 0;
  setpoint[0] = 3.5f;
  step(true);
  sanityIndiTraceSnapshot(fields);
  assert(fields[SI_TRACE_SETPOINT0] ==
         lrintf(DEGREES_TO_RADIANS(3.5f) * 100000));
  assert(fields[SI_TRACE_ALPHA0] != 0); // continuing H history, no reset
  sanityIndiTraceSetRecording(false);
  save("trace_fixture_before.bin", before, sizeof(before));
  sanityIndiCycleStart();
  testClock += 500;
  sanityIndiCycleEnd();
  tick += 1000;
  step(true);
  sanityIndiRecordTaskTime(false, 510, 500);
  sanityIndiRecordTaskTime(true, 1100, 1000);
  sanityIndiRecordLogDrop();
  sanityIndiRecordLogDrop();
  sanityIndiEncodeTiming(after);
  assert(read32(after + 32) >= read32(before + 32) + 1);
  assert(read32(after + 36) >= read32(before + 36) + 1);
  assert(read32(after + 44) == read32(before + 44) + 2);
  assert(read32(after + 52) == 510 && read32(after + 60) == 1100);
  assert(read32(after + 68) >= 1 && read32(after + 72) >= 1);
  save("controller_timing_fixture.bin", after, sizeof(after));
  // Period changes while armed invalidate the controller rather than silently
  // resetting filters. A disarmed initialization must adopt the new period.
  targetPidLooptime = 1000;
  step(true);
  assert(!sanityIndiStatus()->shadowValid ||
         sanityIndiStatus()->reason != SH_OK);
  reset();
  cfg.enabled = 1;
  step(true);
  sanityIndiEncodeTiming(after);
  assert(read32(after + 16) == 1000 && (after[5] & 2));
  cfg.traceEnabled = 2;
  step(true);
  sanityIndiEncodeTiming(after);
  assert((after[5] & 2) && !sanityIndiTraceEnabled());
  cfg.traceEnabled = 0;
  targetPidLooptime = 0;
  reset();
  cfg.enabled = 1;
  step(true);
  sanityIndiEncodeTiming(after);
  assert(!(after[5] & 2));
  targetPidLooptime = 500;
  reset();
}
static unsigned standby_steps(unsigned period) {
  targetPidLooptime = period;
  reset();
  cfg.enabled = 1;
  step(true);
  unsigned before = sanityIndiStatus()->steps;
  for (unsigned n = 0; n < 200000 / period; n++)
    step(true);
  return sanityIndiStatus()->steps - before;
}
static void test_rate_profiles(void) {
  unsigned two = standby_steps(500), one = standby_steps(1000);
  assert(two == 100 && one == 100);
  // Verify reference-tick slew gives same output change over fixed 20 ms.
  for (unsigned i = 0; i < 2; i++) {
    targetPidLooptime = i ? 1000 : 500;
    engage();
    rcData[6] = 1000;
    baseline = dshotConvertFromExternal(1900);
    const float bound = (2047 - dshotConvertFromExternal(1101)) * .005f *
                        (targetPidLooptime / 500.f);
    for (unsigned n = 0; n < 100000 / targetPidLooptime; n++) {
      float old[4];
      memcpy(old, motor, sizeof(old));
      step(true);
      for (unsigned m = 0; m < 4; m++)
        assert(fabsf(motor[m] - old[m]) <= bound + .01f);
    }
  }
  printf("PASS: standby steps2k=%u 1k=%u over200ms; slew reference500us; "
         "period cache/invalidity\n",
         two, one);
}

static uint32_t be32(const uint8_t *p) {
  return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 |
         p[3];
}
static void native_packet(uint8_t p[64], uint32_t seq, uint8_t flags,
                          float thrust) {
  memset(p, 0, 64);
  memcpy(p, "SIP1", 4);
  p[4] = 1;
  p[5] = 3;
  p[6] = 15;
  p[7] = flags;
  spPut64(p + 8, 123);
  spPut32(p + 16, 456);
  spPut32(p + 20, seq);
  spPut32(p + 24, flags ? 1000 : 0);
  spPut16(p + 28, 1000);
  p[30] = 1;
  spPut32(p + 32, SP_NATIVE_MODEL);
  spPutFloat(p + 36, thrust);
}
static void test_native_state(void) {
  spNative_t st = {0};
  uint8_t p[64];
  native_packet(p, 0, 0, 0);
  assert(spNativeReceive(&st, p, 64, 100, false));
  native_packet(p, 10, 3, .4f);
  assert(spNativeReceive(&st, p, 64, 200, true));
  assert(spNativeValid(&st, 201));
  const uint32_t expiry = st.accepted.expiryUs;
  assert(spNativeReceive(&st, p, 64, 300, true));
  assert(st.accepted.expiryUs == expiry);
  spNativeConsume(&st, 301);
  spNativeConsume(&st, 302);
  assert(st.consumedUs == 301);
  native_packet(p, 9, 0, 0);
  assert(spNativeReceive(&st, p, 64, 400, true));
  assert(st.accepted.sequence == 10 && !spNativeValid(&st, 401));
  native_packet(p, 10, 3, .4f);
  assert(!spNativeReceive(&st, p, 64, 500, true));
  native_packet(p, 11, 3, .4f);
  assert(spNativeReceive(&st, p, 64, 600, true));
  native_packet(p, 11, 0, 0);
  assert(spNativeReceive(&st, p, 64, 700, true));
  assert(!spNativeValid(&st, 701));
  native_packet(p, 12, 3, .4f);
  p[30] = 2;
  assert(!spNativeReceive(&st, p, 64, 800, true));
  p[30] = 1;
  spPut32(p + 40, 0x7fc00000);
  assert(!spNativeReceive(&st, p, 64, 801, true));
  assert(!spNativeReceive(&st, p, 63, 802, true));
  native_packet(p, 12, 3, .4f);
  spPut64(p + 8, 124);
  assert(!spNativeReceive(&st, p, 64, 803, true));
  native_packet(p, 12, 0, 0);
  spPut64(p + 8, 124);
  assert(!spNativeReceive(&st, p, 64, 804, true));
  assert(spNativeReceive(&st, p, 64, 805, false));
  native_packet(p, 0xfffffffeU, 0, 0);
  spPut64(p + 8, 125);
  assert(spNativeReceive(&st, p, 64, 0xfffff000U, false));
  native_packet(p, 1, 3, .4f);
  spPut64(p + 8, 125);
  assert(spNativeReceive(&st, p, 64, 0xfffff100U, true));
  assert(spNativeValid(&st, 500));
  assert(!spNativeValid(&st, 1000000));
  spNative_t longIdle = {0};
  native_packet(p, 0, 0, 0);
  assert(spNativeReceive(&longIdle, p, 64, 100, false));
  native_packet(p, 1, 3, .4f);
  assert(spNativeReceive(&longIdle, p, 64, 200, true));
  spNativePoll(&longIdle, UINT32_C(0x80000100));
  assert(!spNativeValid(&longIdle, UINT32_C(0x80000100)));
  assert(!spNativeValid(
      &longIdle, 200)); /* Complete clock wrap cannot revive latched expiry. */
  puts("PASS: production SIP1 owner/STOP watermark,duplicate "
       "expiry,consumption,replay/rollover,numeric/frame/short validation");
}
static void test_paper_adapter(void) {
  uint8_t p[64], stat[80];
  uint32_t seq = 0;
  const float omega = 1000 * (100.f * 2 * M_PIf / 60) / 6;
  const float thrust = 4 * 3.275921725e-8f * omega * omega;
  targetPidLooptime = 1000;
  reset();
  paperCfg.backend = 1;
  cfg.enabled = 1;
  step(true);
  native_packet(p, seq++, 0, 0);
  assert(sanityPaperReceive(p, 64, tick));
  native_packet(p, seq++, 1, thrust);
  assert(sanityPaperReceive(p, 64, tick));
  assert(!testArmedFlags);
  testArmedFlags = ARMED;
  for (unsigned n = 0; n < 1000; n++) {
    if (n % 100 == 0) {
      native_packet(p, seq++, 1, thrust);
      assert(sanityPaperReceive(p, 64, tick));
    }
    step(true);
  }
  if (!sanityIndiStatus()->eligible) {
    fprintf(stderr, "paper entry reason %u track %f solver %d\n",
            sanityIndiStatus()->reason, sanityIndiStatus()->trackingError,
            sanityIndiStatus()->solverExit);
  }
  assert(sanityIndiStatus()->eligible);
  rcData[6] = 2000;
  for (unsigned n = 0; n < 1000; n++) {
    if (n % 100 == 0) {
      native_packet(p, seq++, 3, thrust);
      assert(sanityPaperReceive(p, 64, tick));
    }
    step(true);
  }
  assert(sanityIndiActive());
  sanityPaperEncodeStatus(stat, tick);
  assert(!memcmp(stat, "SIPS", 4) && stat[5] == 1 && (stat[7] & 4));
  assert(be32(stat + 32) == 456 && be32(stat + 36) > 0);
  rcData[6] = 1000;
  testMixScale = .37f;
  testLowCount = 1400;
  testHighCount = 1500;
  step(true);
  for (unsigned m = 0; m < 4; m++)
    assert(motor[m] >= dshotConvertFromExternal(1400) &&
           motor[m] <= dshotConvertFromExternal(1500));
  testLowCount = 1101;
  testHighCount = 2000;
  rcData[6] = 2000;
  testMixScale = 1;
  /* Same-packet FRD companion becomes exact BF FLU setpoint without RC shaping.
   */
  native_packet(p, seq++, 3, thrust);
  spPutFloat(p + 52, .01f);
  spPutFloat(p + 56, .02f);
  spPutFloat(p + 60, .03f);
  assert(sanityPaperReceive(p, 64, tick));
  assert(fabsf(getSetpointRate(0) - .01f * 180 / M_PIf) < 1e-5f);
  assert(fabsf(getSetpointRate(1) + .02f * 180 / M_PIf) < 1e-5f);
  assert(fabsf(getSetpointRate(2) + .03f * 180 / M_PIf) < 1e-5f);
  step(true);
  int32_t f[SP_TRACE_COUNT];
  sanityPaperTraceSnapshot(f);
  save("paper_trace_fixture_state.bin", f, sizeof(f));
  assert(f[17] == (int32_t)SP_NATIVE_MODEL && f[10] == 15 && f[116] == 1);
  /* No demand refresh: expire and controlled fade uses actual count output. */
  for (unsigned n = 0; n < 1400; n++)
    step(true);
  assert(!sanityIndiActive() && testArmedFlags);
  sanityPaperEncodeStatus(stat, tick);
  assert(!(stat[7] & 2));
  /* Disarmed source choice selects the local legacy rate adapter; no native
   * revival. */
  testArmedFlags = 0;
  paperCfg.inputSource = 1;
  rcData[6] = 1000;
  step(true);
  testArmedFlags = ARMED;
  for (unsigned n = 0; n < 1000; n++)
    step(true);
  rcData[6] = 2000;
  for (unsigned n = 0; n < 1000; n++)
    step(true);
  assert(sanityIndiActive());
  assert(!sanityPaperFallbackRate(0, &baseline));
  uint8_t timingBefore[80], timingAfter[80];
  sanityIndiEncodeTiming(timingBefore);
  tick += 4000;
  step(true);
  sanityIndiEncodeTiming(timingAfter);
  assert(read32(timingAfter + 36) == read32(timingBefore + 36) + 1 &&
         read32(timingAfter + 48) == 5000);
  /* Midair config/rate modifications invalidate rather than reseed. */
  paperCfg.cutoffHz = 13;
  for (unsigned n = 0; n < 300; n++)
    step(true);
  assert(!sanityIndiActive());
  testArmedFlags = 0;
  paperCfg.backend = 0;
  paperCfg.inputSource = 0;
  paperCfg.cutoffHz = 12;
  step(true);
  targetPidLooptime = 500;
  reset();
  /* No transition has an armed slew deadline after half clock wrap. */
  paperCfg.backend = 1;
  paperCfg.inputSource = 1;
  step(true);
  testArmedFlags = ARMED;
  cfg.enabled = 0;
  baseline = dshotConvertFromExternal(1900);
  tick = UINT32_C(0x80001000);
  step(true);
  for (unsigned m = 0; m < 4; m++)
    assert(motor[m] == baseline);
  testArmedFlags = 0;
  paperCfg.backend = 0;
  paperCfg.inputSource = 0;
  step(true);
  reset();
  puts("PASS: production measured-RPM/native+local-rate adapters, no native "
       "arming, paired FRD fallback, active count output, consumed "
       "identity,expiry fallback,disarmed config");
}

static void test_paper_loss_freeze(void) {
  const float omega = 1000 * (100.f * 2 * M_PIf / 60) / 6;
  const float thrust = 4 * 3.275921725e-8f * omega * omega;
  for (unsigned fault = 0; fault < 3; fault++) {
    targetPidLooptime = 1000;
    testVoltageFresh = true;
    paperCfg.backend = 0;
    reset();
    paperCfg.backend = 1;
    cfg.enabled = 1;
    step(true);
    uint8_t packet[64], status[80];
    uint32_t seq = 0;
    native_packet(packet, seq++, 0, 0);
    assert(sanityPaperReceive(packet, 64, tick));
    testArmedFlags = ARMED;
    for (unsigned n = 0; n < 1000; n++) {
      if (n % 100 == 0) {
        native_packet(packet, seq++, 1, thrust);
        assert(sanityPaperReceive(packet, 64, tick));
      }
      step(true);
    }
    int32_t before[SP_TRACE_COUNT], after[SP_TRACE_COUNT];
    sanityPaperTraceSnapshot(before);
    assert((before[5] & 6) == 6);
    if (fault == 0) {
      testVoltageFresh = false;
      sanityPaperRecordVoltage(tick - 1000000U, 8);
    } else if (fault == 1)
      signal = false;
    else
      cfg.enabled = 0;
    step(true);
    sanityPaperTraceSnapshot(after);
    sanityPaperEncodeStatus(status, tick);
    assert(!(after[5] & 6) && status[54] == 255);
    for (unsigned m = 0; m < 4; m++) {
      if (fault < 2) {
        assert(before[93 + m] == after[93 + m]);
        assert(before[73 + m] == after[73 + m]);
      } else
        assert(after[93 + m] == 0);
    }
    testArmedFlags = 0;
    testVoltageFresh = true;
    signal = true;
    paperCfg.backend = 0;
    step(true);
  }
  targetPidLooptime = 500;
  reset();
  puts("PASS: actual paper adapter freezes allocation/PI/history on stale "
       "voltage and RX loss; disabled resets; no current solve reported");
}

static void test_paper_consecutive_readiness(void) {
  for (unsigned fault = 0; fault < 6; fault++) {
    targetPidLooptime = 1000;
    paperCfg.backend = 0;
    reset();
    paperCfg.backend = 1;
    paperCfg.inputSource = 1;
    cfg.enabled = 1;
    step(true);
    testArmedFlags = ARMED;
    for (unsigned n = 0; n < 400; n++)
      step(true);
    assert(sanityIndiStatus()->eligible && sanityIndiStatus()->readyUs >= 100000);
    uint8_t timing[80];
    sanityIndiEncodeTiming(timing);
    assert((timing[5] & 6) == 6);
    probeSequence = sanityIndiStatus()->sequence + 1;
    sawPreparedTrace = false;
    probeTraceReady = true;
    step(true);
    probeTraceReady = false;
    assert(sawPreparedTrace);
    int32_t fields[SP_TRACE_COUNT];
    sanityPaperTraceSnapshot(fields);
    assert(fields[119] == sanityIndiStatus()->lastRuntimeUs &&
           fields[120] == sanityIndiStatus()->maxRuntimeUs);
    uint32_t hash = 2166136261U;
    const uint8_t *bytes = (const uint8_t *)&paperCfg;
    for (unsigned i = 0; i < sizeof(paperCfg); i++) hash = (hash ^ bytes[i]) * 16777619U;
    sanityIndiConfig_t supervisor = cfg;
    supervisor.traceEnabled = 0;
    bytes = (const uint8_t *)&supervisor;
    for (unsigned i = 0; i < sizeof(supervisor); i++) hash = (hash ^ bytes[i]) * 16777619U;
    bytes = (const uint8_t *)&rateFfCfg;
    for (unsigned i = 0; i < sizeof(rateFfCfg); i++) hash = (hash ^ bytes[i]) * 16777619U;
    assert((uint32_t)fields[115] == hash);
    cfg.traceEnabled = 1;
    step(true);
    sanityPaperTraceSnapshot(fields);
    assert((uint32_t)fields[115] == hash && sanityIndiStatus()->eligible);
    cfg.traceEnabled = 0;
    if (fault == 0) {
      for (unsigned n = 0; n < 7; n++) step(false);
    } else if (fault == 1) {
      tick += 3000;
      step(true);
    } else if (fault == 2) {
      gyro.sanityIndiGyro[0] = NAN;
      step(true);
      gyro.sanityIndiGyro[0] = 0;
    } else if (fault == 3) {
      testMixScale = NAN;
      step(true);
      testMixScale = 1;
    } else if (fault == 4) {
      testVoltageFresh = false;
      sanityPaperRecordVoltage(tick - 1000000, 8);
      step(true);
      testVoltageFresh = true;
    } else {
      signal = false;
      step(true);
      signal = true;
    }
    assert(!sanityIndiStatus()->eligible && sanityIndiStatus()->readyUs == 0);
    if (fault == 2) {
      assert(!testArmedFlags && (testArmingDisabled & ARMING_DISABLED_RUNAWAY_TAKEOFF));
      step(true); // actual disarm transition resets observer before a new arm
      testArmedFlags = ARMED;
    }
    for (unsigned n = 0; n < 99; n++) {
      step(true);
      assert(!sanityIndiStatus()->eligible);
      if (fault < 4) {
        int32_t trace[SP_TRACE_COUNT];
        sanityPaperTraceSnapshot(trace);
        assert(!(trace[5] & 2)); // intact 100 ms observer warmup after invalid gap
      }
    }
    for (unsigned n = 0; n < 250; n++) step(true);
    if (!sanityIndiStatus()->eligible) fprintf(stderr,"readiness fault=%u armed=%u reason=%u ready=%u track=%f\n",fault,testArmedFlags,sanityIndiStatus()->reason,sanityIndiStatus()->readyUs,sanityIndiStatus()->trackingError);
    assert(sanityIndiStatus()->eligible);
    // Repeated short healthy intervals never accumulate into the 100ms dwell.
    for (unsigned interruption = 0; interruption < 3; interruption++) {
      testVoltageFresh = false;
      sanityPaperRecordVoltage(tick - 1000000, 8);
      step(true);
      assert(sanityIndiStatus()->readyUs == 0);
      testVoltageFresh = true;
      for (unsigned n = 0; n < 50; n++) step(true);
      assert(!sanityIndiStatus()->eligible && sanityIndiStatus()->readyUs < 50000);
    }
    for (unsigned n = 0; n < 101; n++) step(true);
    assert(sanityIndiStatus()->eligible);
    // Invalid paper configuration must be reflected even if reference remains initialized.
    paperCfg.cutoffHz++;
    step(true);
    sanityIndiEncodeTiming(timing);
    assert(!(timing[5] & 2));
    testArmedFlags = 0;
    paperCfg.cutoffHz--;
    paperCfg.backend = 0;
    paperCfg.inputSource = 0;
    step(true);
  }
  targetPidLooptime = 500;
  reset();
  puts("PASS: paper readiness requires consecutive healthy dwell after RPM/timing/numeric/config/voltage/RX failures; observer warmup preserved; cached hash exact; runtime sample follows trace prep; selected SIT1 validity");
}

static void test_speed_model_adapter(void) {
  paperCfg.backend=0;testVoltage=8;testVoltageFresh=true;targetPidLooptime=1000;reset();
  cfg.enabled=1;paperCfg.backend=1;paperCfg.inputSource=1;paperCfg.warmupMs=100;
  step(true);testArmedFlags=ARMED;
  for(unsigned n=0;n<400;n++)step(true);
  assert(sanityIndiStatus()->eligible);rcData[6]=2000;
  for(unsigned n=0;n<400;n++)step(true);
  assert(sanityIndiActive());
  const float voltages[]={7.48f,7.04f,7.0f,6.95f,6.94f};
  const float highs[]={1900,1950,1999.99988f,2000};
  for(unsigned v=0;v<5;v++)for(unsigned h=0;h<4;h++) {
    testVoltage=voltages[v];testHighCount=highs[h];
    for(unsigned n=0;n<10;n++)step(true);
    int32_t trace[SP_TRACE_COUNT];uint8_t status[80];sanityPaperTraceSnapshot(trace);sanityPaperEncodeStatus(status,tick);
    assert(trace[0]==SP_TRACE_VERSION && trace[5]&4 && trace[7]!=6 && trace[7]!=11);
    if (!(trace[148]==0 && abs(trace[126]-1101000)<=1 && abs(trace[127]-(int)lrintf(testHighCount*1000))<=1)) fprintf(stderr,"bounds v=%f high=%f trace=%d,%d failure=%d\n",testVoltage,testHighCount,trace[126],trace[127],trace[148]);
    assert(trace[148]==0 && abs(trace[126]-1101000)<=1 && abs(trace[127]-(int)lrintf(testHighCount*1000))<=1);
    assert(status[4]==2 && !(status[76]&~0x8f) && !(status[77]&~15));
    assert(status[76]==trace[16] && status[77]==((trace[136]>>8)&15));
    assert(status[78]==((trace[136]>>16)&15) && status[79]==((trace[136]>>20)&15));
    for(unsigned m=0;m<4;m++)assert(motor[m]<=dshotConvertFromExternal((uint16_t)ceilf(testHighCount)));
    assert(sanityIndiActive());
    if(v==4 && h==1)save("sips_v2_low_voltage.bin",status,80);
  }
  testArmedFlags=0;step(true);uint8_t status[80];sanityPaperEncodeStatus(status,tick);
  assert(status[77]==0 && status[78]==0 && status[79]==0);
  testHighCount=2000;testVoltage=8;paperCfg.backend=0;paperCfg.inputSource=0;reset();
  puts("PASS actual adapter: SIPSv2/190-field trace, dynamic bounds, low-voltage support stays servo-valid, disarm clears FF history");
}

/* These tests exercise the production adapter as well as the pure helper.
 * All earlier regression checks above remain unchanged. */
static void rate_ff_fixture(unsigned gain, unsigned cutoff) {
  testArmedFlags=0;paperCfg.backend=0;targetPidLooptime=1000;
  testLowCount=1101;testHighCount=2000;testVoltage=8;testVoltageFresh=true;
  reset();
  for(unsigned a=0;a<3;a++) {
    rateFfCfg.gainMilli[a]=a==2?0:gain;
    rateFfCfg.alphaLimit[a]=20;
  }
  rateFfCfg.cutoffHz=cutoff;
  cfg.rateGains[0]=cfg.rateGains[1]=150;cfg.rateGains[2]=80;
  paperCfg.backend=1;paperCfg.inputSource=1;paperCfg.cutoffHz=12;
  paperCfg.warmupMs=100;cfg.enabled=1;rcData[THROTTLE]=1300;
  step(true);assert(sanityPaperConfigured());
  testArmedFlags=ARMED;
  for(unsigned n=0;n<450;n++)step(true);
  assert(sanityIndiStatus()->eligible && !sanityIndiActive());
}
static void rate_ff_ramp(unsigned n) {
  /* BF-positive pitch becomes FRD-negative pitch; yaw gain stays zero. */
  setpoint[0]=(.2f*(n+1)*.001f)*180/M_PIf;
  setpoint[1]=(.2f*(n+1)*.001f)*180/M_PIf;
  setpoint[2]=(.1f*(n+1)*.001f)*180/M_PIf;
  step(true);
}
static void assert_rate_ff_identity(const int32_t f[SP_TRACE_COUNT]) {
  assert(f[0]==4 && (f[171]&8));
  for(unsigned a=0;a<3;a++) {
    assert(abs(f[25+a]-f[152+a]-f[161+a])<=2);
    assert(abs(f[161+a])<=f[168+a]);
  }
}
static void assert_rate_ff_cleared(const int32_t f[SP_TRACE_COUNT]) {
  assert(!(f[171]&14));
  for(unsigned a=0;a<3;a++)
    assert(f[155+a]==0 && f[158+a]==0 && f[161+a]==0);
}
static void test_rate_ff_shadow_and_handover(void) {
  rate_ff_fixture(250,20);
  int32_t before[SP_TRACE_COUNT],after[SP_TRACE_COUNT];
  for(unsigned n=0;n<500;n++)rate_ff_ramp(n);
  sanityPaperTraceSnapshot(before);
  assert_rate_ff_identity(before);
  assert(abs(before[155]-200)<=1 && abs(before[156]+200)<=1);
  assert(abs(before[158]-200)<=1 && abs(before[159]+200)<=1);
  assert(abs(before[161]-50)<=1 && abs(before[162]+50)<=1 && before[163]==0);
  assert(before[6]==SH_PID && before[14]==0 && (before[171]&2));
  /* History is already primed in PID shadow: AUX entry must not reseed it. */
  rcData[6]=2000;rate_ff_ramp(500);sanityPaperTraceSnapshot(after);
  assert_rate_ff_identity(after);
  assert(after[6]==SH_TO_INDI && after[14]>0 && !(after[171]&4));
  assert(abs(after[161]-before[161])<=1 && abs(after[162]-before[162])<=1);
  for(unsigned n=501;n<850;n++)rate_ff_ramp(n);
  assert(sanityIndiActive() && sanityIndiStatus()->mode==SH_INDI);
  sanityPaperTraceSnapshot(after);assert_rate_ff_identity(after);
  assert(after[164]==250 && after[165]==250 && after[166]==0 && after[167]==20);
  puts("PASS rate FF adapter: beta250 FRD roll/pitch signs, feedback+FF identity, bounded contribution, healthy PID-shadow history and bumpless AUX handover");
}
static void test_rate_ff_gap_reseeding(void) {
  for(unsigned fault=0;fault<4;fault++) {
    rate_ff_fixture(250,20);
    for(unsigned n=0;n<100;n++)rate_ff_ramp(n);
    int32_t f[SP_TRACE_COUNT];sanityPaperTraceSnapshot(f);
    assert(f[161]>0 && f[162]<0);uint32_t resets=(uint32_t)f[173];
    if(fault==0){signal=false;step(true);signal=true;}
    else if(fault==1){for(unsigned n=0;n<8;n++)step(false);}
    else if(fault==2){tick+=3000;step(true);}
    else {rcData[THROTTLE]=1000;step(true);rcData[THROTTLE]=1300;}
    sanityPaperTraceSnapshot(f);assert_rate_ff_cleared(f);
    assert((uint32_t)f[173]==resets+1);
    /* A reference change during rejection must not be differentiated across
     * the stale interval. Wait for the first actually admitted computation. */
    setpoint[0]+=.1f*180/M_PIf;setpoint[1]+=.1f*180/M_PIf;
    bool seeded=false;
    for(unsigned n=0;n<300;n++) {
      step(true);sanityPaperTraceSnapshot(f);
      if(f[171]&8) {
        assert((f[171]&6)==6);
        for(unsigned a=0;a<3;a++)assert(f[161+a]==0 && f[155+a]==0);
        seeded=true;break;
      }
      assert_rate_ff_cleared(f);
    }
    assert(seeded);
    setpoint[0]+=.0002f*180/M_PIf;setpoint[1]+=.0002f*180/M_PIf;
    step(true);sanityPaperTraceSnapshot(f);assert_rate_ff_identity(f);
    assert(f[161]>0 && f[162]<0 && !(f[171]&4));
  }
  rate_ff_fixture(250,20);
  for(unsigned n=0;n<100;n++)rate_ff_ramp(n);
  testArmedFlags=0;step(true);int32_t f[SP_TRACE_COUNT];sanityPaperTraceSnapshot(f);
  assert_rate_ff_cleared(f);setpoint[0]=12;setpoint[1]=15;
  testArmedFlags=ARMED;bool seeded=false;
  for(unsigned n=0;n<300;n++) {
    step(true);sanityPaperTraceSnapshot(f);
    if(f[171]&8) {assert((f[171]&6)==6 && f[161]==0 && f[162]==0);seeded=true;break;}
  }
  assert(seeded && testArmedFlags);
  puts("PASS rate FF adapter: RX/RPM/timing/idle rejection resets and first healthy tick seeds zero; disarm/rearm cannot retain derivative history");
}
static void test_rate_ff_native_bypass(void) {
  rate_ff_fixture(250,20);testArmedFlags=0;step(true);
  paperCfg.inputSource=0;step(true);assert(sanityPaperConfigured());
  uint8_t packet[64];uint32_t seq=1000000;
  const float omega=1000*(100.f*2*M_PIf/60)/6;
  const float thrust=4*3.275921725e-8f*omega*omega;
  native_packet(packet,seq++,0,0);assert(sanityPaperReceive(packet,64,tick));
  testArmedFlags=ARMED;
  for(unsigned n=0;n<700;n++) {
    if(n%50==0) {
      native_packet(packet,seq++,1,thrust);
      spPutFloat(packet+40,.12f);spPutFloat(packet+44,-.21f);spPutFloat(packet+48,.31f);
      spPutFloat(packet+52,.001f*n);spPutFloat(packet+56,-.001f*n);
      assert(sanityPaperReceive(packet,64,tick));
    }
    step(true);int32_t f[SP_TRACE_COUNT];sanityPaperTraceSnapshot(f);
    assert_rate_ff_cleared(f);
    assert(abs(f[25]-120)<=1 && abs(f[26]+210)<=1 && abs(f[27]-310)<=1);
    assert(f[152]==0 && f[153]==0 && f[154]==0);
  }
  puts("PASS rate FF adapter: native FRD angular acceleration passes unchanged with beta nonzero; varying companion rates never enable duplicate feedforward");
}
static void test_rate_ff_configuration(void) {
  for(unsigned invalid=0;invalid<3;invalid++) {
    rate_ff_fixture(250,20);testArmedFlags=0;step(true);
    if(invalid==0)rateFfCfg.gainMilli[0]=1001;
    else if(invalid==1)rateFfCfg.cutoffHz=0;
    else rateFfCfg.alphaLimit[1]=0;
    step(true);assert(!sanityPaperConfigured());
    testArmedFlags=ARMED;
    for(unsigned n=0;n<10;n++)step(true);
    int32_t f[SP_TRACE_COUNT];sanityPaperTraceSnapshot(f);
    assert(!(f[5]&6));assert_rate_ff_cleared(f);
  }
  rate_ff_fixture(250,20);rcData[6]=2000;
  for(unsigned n=0;n<400;n++)step(true);
  assert(sanityIndiActive());int32_t before[SP_TRACE_COUNT],after[SP_TRACE_COUNT];
  sanityPaperTraceSnapshot(before);rateFfCfg.gainMilli[0]=300;step(true);
  sanityPaperTraceSnapshot(after);assert(!sanityPaperConfigured());
  assert(after[164]==250 && after[115]==before[115]);
  assert(!(after[5]&6));assert_rate_ff_cleared(after);
  for(unsigned n=0;n<300;n++)step(true);
  assert(!sanityIndiActive() && testArmedFlags);
  testArmedFlags=0;step(true);sanityPaperTraceSnapshot(after);
  assert(sanityPaperConfigured() && after[164]==300 && after[115]!=before[115]);
  puts("PASS rate FF adapter: malformed new PG rejects; midair FF settings invalidate and are adopted only after disarm with changed hash");
}
static void test_rate_ff_zero_gain_equivalence(void) {
  float outputs[2][500][4];int32_t alphas[2][500][3];
  for(unsigned pass=0;pass<2;pass++) {
    rate_ff_fixture(0,pass?80:20);rcData[6]=2000;
    for(unsigned n=0;n<400;n++)step(true);
    assert(sanityIndiActive());
    for(unsigned n=0;n<500;n++) {
      setpoint[0]=.05f*sinf(2*M_PIf*(float)n*.001f)*180/M_PIf;
      setpoint[1]=.03f*sinf(4*M_PIf*(float)n*.001f)*180/M_PIf;
      step(true);int32_t f[SP_TRACE_COUNT];sanityPaperTraceSnapshot(f);
      assert_rate_ff_identity(f);
      for(unsigned a=0;a<3;a++) {assert(f[161+a]==0 && f[25+a]==f[152+a]);alphas[pass][n][a]=f[25+a];}
      memcpy(outputs[pass][n],motor,sizeof(motor));
    }
  }
  assert(!memcmp(outputs[0],outputs[1],sizeof(outputs[0])));
  assert(!memcmp(alphas[0],alphas[1],sizeof(alphas[0])));
  puts("PASS rate FF adapter: zero beta leaves acceleration and all motor outputs bit-identical despite changing derivative cutoff20/80Hz");
}
static void test_rate_feedforward_adapter(void) {
  sanityIndiConfig_t oldCfg=cfg;sanityPaperConfig_t oldPaper=paperCfg;
  sanityRateFfConfig_t oldFf=rateFfCfg;
  test_rate_ff_shadow_and_handover();
  test_rate_ff_gap_reseeding();
  test_rate_ff_native_bypass();
  test_rate_ff_configuration();
  test_rate_ff_zero_gain_equivalence();
  testArmedFlags=0;step(true);cfg=oldCfg;paperCfg=oldPaper;rateFfCfg=oldFf;
  targetPidLooptime=500;reset();
}

static void timing_fixture(void) {
  for(unsigned m=0;m<4;m++)testRaw[m]=1000;
  rate_ff_fixture(250,12);
  cfg.rateGains[0]=cfg.rateGains[1]=200; cfg.rateGains[2]=80;
  testArmedFlags=0;step(true);testArmedFlags=ARMED;
  rcData[6]=1000;
  for(unsigned n=0;n<450;n++)step(true);
  rcData[6]=2000;
  for(unsigned n=0;n<400;n++)step(true);
  assert(sanityIndiActive() && sanityIndiStatus()->mode==SH_INDI);
}
static void timing_snapshot(int32_t f[SP_TRACE_COUNT]) {sanityPaperTraceSnapshot(f);assert(f[0]==4);}
static void test_timing_robustness(void) {
  int32_t f[SP_TRACE_COUNT],before[SP_TRACE_COUNT];
  timing_fixture();timing_snapshot(before);
  /* One observed-envelope event with BOTH dt/runtime triggers is ONE event.
   * Simulated previous completed PID cost is available on the following tick. */
  testPidExtra=1495;step(true);assert(sanityIndiStatus()->cycleUs>1000);
  testInterval=2045;setpoint[0]=20;gyro.sanityIndiGyro[0]=1;step(true);
  timing_snapshot(f);
  assert(f[174]==before[174]+1 && f[175]==before[175]+1 && f[176]==before[176]);
  assert(f[181]==(2|4|32) && f[179]==2045 && f[180]>1000 && f[180]<=1750);
  assert(f[182]>0 && f[189]==before[189]+1 && f[177]==f[1]);
  assert((f[5]&7)==7 && f[6]==SH_INDI && f[14]==1000000);
  assert(f[34]==0 && f[35]==0 && f[36]==0); /* no stale rate derivative */
  assert((f[171]&14)==14 && f[155]==0 && f[158]==0 && f[161]==0);
  const uint32_t firstSeq=f[177];
  for(unsigned n=0;n<4;n++)step(true);
  timing_snapshot(f);
  assert((uint32_t)f[177]==firstSeq && f[179]==2045 && f[182]>0);
  assert((f[5]&7)==7 && f[6]==SH_INDI); /* no warmup/stale hold */
  /* A second event at6ms is rejected even with nominal intervening ticks. */
  testInterval=1600;step(true);timing_snapshot(f);
  assert((f[187]&16) && !(f[187]&32) && f[185]==1600 && f[188]>0);
  assert(f[6]==SH_TO_PID && sanityIndiStatus()->lastFault==SH_TIMING);
  assert(!(f[5]&7));assert_rate_ff_cleared(f);
  /* An adjacent short catchup is intentionally not treated as same event. */
  timing_fixture();testInterval=2045;step(true);testInterval=400;step(true);timing_snapshot(f);
  assert((f[187]&(1|16))==(1|16) && sanityIndiStatus()->lastFault==SH_TIMING);
  /* Hard envelope boundaries, repeated tick, and reset-on-disarm. */
  const uint32_t intervals[]={249,2501,0,2000};
  for(unsigned k=0;k<4;k++) {
    timing_fixture();
    if(k==2){testPidExtra=1800;step(true);} else if(k==3){testInterval=2000;step(true);}
    testInterval=intervals[k];step(true);timing_snapshot(f);
    assert(sanityIndiStatus()->lastFault==SH_TIMING && f[183]);
    assert(f[187]&(k==3?16:8));
  }
  timing_fixture();testInterval=2500;step(true);timing_snapshot(f);assert(f[181]&32);
  for(unsigned n=0;n<100;n++)step(true);
  testInterval=1600;step(true);timing_snapshot(f);
  assert(!f[183] && f[6]==SH_INDI); /* >=100ms quiet allows another isolated event */
  testArmedFlags=0;step(true);timing_snapshot(f);assert(f[177]);
  testArmedFlags=ARMED;step(true);timing_snapshot(f);assert(!f[177] && !f[183]);
  testInterval=1600;step(true);timing_snapshot(f);
  assert((f[187]&64) && !(f[187]&32)); /* no grace to bypass initial warmup */
  /* Independent safety gates remain effective on a grace-sized interval. */
  for(unsigned fault=0;fault<4;fault++) {
    timing_fixture();
    if(fault==0) for(unsigned n=0;n<7;n++)step(false);
    if(fault==1)gyro.sanityIndiGyro[0]=NAN;
    if(fault==2)signal=false;
    if(fault==3)testVoltage=NAN;
    testInterval=1600;step(fault!=0);timing_snapshot(f);
    assert(!(f[5]&6));
    if(fault==1)assert(!testArmedFlags && sanityIndiStatus()->lastFault==SH_NUMERIC);
    else assert(sanityIndiStatus()->lastFault!=(unsigned)SH_OK);
  }
  puts("PASS timing: isolated2045us plus previous PID>1000us, coherent reseed/fresh solve, latched event survives decimation; repeated/bursty/catchup, hard gap/runtime, warmup, disarm and RPM/numeric/RX/voltage faults");
}
static void test_timing_output_perturbation(void) {
  float baselineCounts[500][4],peak=0;
  for(unsigned pass=0;pass<2;pass++) {
    timing_fixture();
    for(unsigned n=0;n<500;n++) {
      setpoint[0]=2*sinf(.01f*n);setpoint[1]=1*cosf(.01f*n);
      gyro.sanityIndiGyro[0]=.2f*sinf(.01f*n);
      gyro.sanityIndiGyro[1]=.1f*cosf(.01f*n);
      if(pass && n==200)testInterval=2045;
      step(true);int32_t f[SP_TRACE_COUNT];timing_snapshot(f);
      assert((f[5]&7)==7 && f[6]==SH_INDI);
      for(unsigned m=0;m<4;m++) {
        float count=dshotConvertToExternal(motor[m]);
        if(!pass)baselineCounts[n][m]=count;
        else peak=fmaxf(peak,fabsf(count-baselineCounts[n][m]));
        assert(count>=1100 && count<=2000);
      }
    }
  }
  assert(peak<=20);printf("PASS timing perturbation: peak clean-vs-isolated-event output difference %.3f external counts (<20 engineering test bound), fresh solve every tick\n",peak);
}
static void test_observer_reseed_invalid_history(void) {
  spState_t state;
  spConfig_t c={.dt=.001f,.cutoffHz=12,.inertia={1,1,1},.maxIterations=16,.warmupTicks=2};
  for(unsigned m=0;m<4;m++){c.omegaMax[m]=5883;c.rowScale[m]=c.priority[m]=1;c.g1[0][m]=1;c.g1[1][m]=.01f;}
  assert(spInit(&state,&c));float omega[4]={1000,1100,1200,1300},body[3]={.1f,.2f,.3f};
  assert(!spReseedObservation(&state,omega,body));
  assert(spObserve(&state,omega,body));assert(!spReseedObservation(&state,omega,body));
  assert(spObserve(&state,omega,body));assert(state.ready);
  state.servoI[0]=17;state.feedforward[0]=1350;state.ffValidMask=15;
  for(unsigned i=0;i<7;i++)state.filterZ1[i]=state.filterZ2[i]=NAN;
  for(unsigned i=0;i<3;i++)state.observedBodyPrevious[i]=state.alphaFiltered[i]=NAN;
  for(unsigned i=0;i<4;i++)state.observedOmegaPrevious[i]=NAN;
  assert(spReseedObservation(&state,omega,body));
  assert(state.ready && state.ticks==2 && state.servoI[0]==17 && state.feedforward[0]==1350 && state.ffValidMask==15);
  for(unsigned a=0;a<3;a++)assert(state.alphaFiltered[a]==0 && state.g2Torque[a]==0 && isfinite(state.tauFiltered[a]));
  assert(spObserve(&state,omega,body));
  for(unsigned a=0;a<3;a++)assert(fabsf(state.alphaFiltered[a])<.001f);
  omega[0]=NAN;assert(!spReseedObservation(&state,omega,body));assert(!state.ready && !state.observed);
  puts("PASS observer reseed: invalid derivative/filter history fully overwritten, synchronized torque and zero derivatives, preserves ready/servo state only for validated observations, invalid RPM resets");
}

#include "replay_native_samples.h"
static void test_recorded_input_replay(void) {
  const unsigned rows=sizeof(replayNative)/sizeof(replayNative[0]);
  float outputs[1000][4],peak=0;
  assert(rows*4<=1000);
  for(unsigned pass=0;pass<2;pass++) {
    timing_fixture();
    for(unsigned r=0;r<rows;r++)for(unsigned sub=0;sub<4;sub++) {
      const float *v=replayNative[r];unsigned n=r*4+sub;
      for(unsigned m=0;m<4;m++)testRaw[m]=(uint32_t)v[m];
      for(unsigned a=0;a<3;a++) {
        gyro.sanityIndiGyro[a]=gyro.gyroADCf[a]=v[4+a]*(a?-1:1)*180/M_PIf;
        setpoint[a]=v[7+a]*(a?-1:1)*180/M_PIf;
      }
      testVoltage=v[10];
      if(pass && n==rows*2)testInterval=2045;
      step(true);int32_t f[SP_TRACE_COUNT];timing_snapshot(f);
      if((f[5]&7)!=7 || f[6]!=SH_INDI)fprintf(stderr,"replay row%u sub%u reason%d flags%d mode%d\n",r,sub,f[7],f[5],f[6]);
      assert((f[5]&7)==7 && f[6]==SH_INDI);
      for(unsigned m=0;m<4;m++) {
        const float count=dshotConvertToExternal(motor[m]);
        if(!pass)outputs[n][m]=count;
        else peak=fmaxf(peak,fabsf(count-outputs[n][m]));
        assert(count>=1100 && count<=2000);
      }
    }
  }
  assert(peak<=50);
  printf("PASS recorded input replay: %u measured250Hz rows held for %u synthetic1kHz ticks,2045us injected event, fresh solve each tick, peak output change %.3f counts (<50 engineering bound); exact missing tick not reconstructed\n",rows,rows*4,peak);
}

int main(void) {
  regression_main();
  test_trace_timing();
  test_rate_profiles();
  test_native_state();
  test_paper_adapter();
  test_paper_loss_freeze();
  test_paper_consecutive_readiness();
  test_speed_model_adapter();
  test_rate_feedforward_adapter();
  test_observer_reseed_invalid_history();
  test_timing_output_perturbation();
  test_timing_robustness();
  test_recorded_input_replay();
  puts("PASS: native full-rate trace quantization/validity; all-cycle "
       "timing/gyro/filter/loss counters; actual80B encoder fixture");
  return 0;
}
