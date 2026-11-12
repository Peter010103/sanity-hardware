#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#define USE_SANITY_INDI
#define FAST_DATA_ZERO_INIT
#define UNUSED(x) (void)(x)
#define M_PIf 3.14159265358979323846f
#define DEGREES_TO_RADIANS(x) ((x)*M_PIf/180.0f)
#define ARMING_FLAG(x) (testArmedFlags & (x))
#define ARMED 1U
#define FAILSAFE_MODE (1U<<10)
#define ARMING_DISABLED_SANITY_INDI (1U<<26)
#define ARMING_DISABLED_RUNAWAY_TAKEOFF (1U<<11)
#define DISARM_REASON_RUNAWAY_TAKEOFF 1
#define MIXER_QUADX 3
#define PWM_TYPE_DSHOT300 6
#define THROTTLE 3
#define PG_DECLARE(type,name) const type *name(void)
typedef uint32_t timeUs_t;
typedef struct {float throttle, roll, pitch, yaw;} motorMixer_t;
typedef struct {uint8_t mixerMode;bool yaw_motors_reversed;} mixerConfig_t;
typedef struct {uint8_t motorPwmProtocol,useDshotTelemetry;} motorDevConfig_t;
typedef struct {motorDevConfig_t dev;uint8_t motorPoleCount;} motorConfig_t;
typedef struct {uint8_t thrustLinearization,use_integrated_yaw;} pidProfile_t;
typedef struct {bool feature3dEnabled;motorMixer_t currentMixer[4];float disarmMotorOutput;} mixerRuntime_t;
typedef struct {float sanityIndiGyro[3],gyroADCf[3];} gyro_t;
typedef struct {uint32_t sanityErpmTimestampUs;} telemetryMotor_t;
typedef struct {telemetryMotor_t motorState[4];} telemetry_t;
extern uint32_t testArmedFlags,flightModeFlags,targetPidLooptime,testClock,testArmingDisabled;
extern mixerRuntime_t mixerRuntime;
extern gyro_t gyro;
extern telemetry_t dshotTelemetryState;
extern float motor[4],rcData[8];
extern const pidProfile_t *currentPidProfile;
uint32_t micros(void);
unsigned getMotorCount(void);
bool isMotorProtocolDshot(void);
bool isFlipOverAfterCrashActive(void);
bool isLaunchControlActive(void);
const mixerConfig_t *mixerConfig(void);
const motorConfig_t *motorConfig(void);
float getSetpointRate(unsigned a);
void setArmingDisabled(uint32_t flag);
void unsetArmingDisabled(uint32_t flag);
void disarm(int reason);
uint16_t getBatteryVoltage(void);
uint16_t motorConvertToExternal(float x);
float motorConvertFromExternal(uint16_t x);
uint16_t getDshotErpm(unsigned m);
bool rxIsReceivingSignal(void);
float getCosTiltAngle(void);
void pidTrackSanityOutput(const float target[3]);
static inline float constrainf(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}

#define DSHOT_MIN_THROTTLE 48
#define DSHOT_MAX_THROTTLE 2047
#define DSHOT_3D_FORWARD_MIN_THROTTLE 1048
#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE_MIDDLE 1500
#define FEATURE_3D 1
float scaleRangef(float x,float srcFrom,float srcTo,float destFrom,float destTo);
