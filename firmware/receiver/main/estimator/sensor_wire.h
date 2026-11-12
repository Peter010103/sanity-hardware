#ifndef SANITY_SENSOR_WIRE_H
#define SANITY_SENSOR_WIRE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define SE_SENSOR_MSP 0x7f12
#define SE_CLOCK_MSP 0x7f13
#define SE_WIRE_SENSOR_SIZE 76
#define SE_WIRE_CLOCK_SIZE 48
#define SE_WIRE_VERSION 1
#define SE_FLAG_GYRO_VALID 1
#define SE_FLAG_ACC_VALID 2
#define SE_FLAG_CALIBRATED 4
#define SE_FLAG_DISARMED 8
/* Explicit little endian wire, IEEE754 binary32; CRC32C final 4 bytes.
 * gyro and acceleration timestamps remain independent through decimation. */
typedef struct {
  uint64_t epoch;
  uint32_t gyro_delay_us, accel_delay_us;
  uint32_t sequence, gyro_us, accel_us, gyro_uncertainty_us,
      accel_uncertainty_us, dropped;
  float gyro[3], accel[3];
  uint16_t flags;
  uint8_t gyro_source, accel_source;
} se_sensor_packet;
typedef struct {
  uint64_t epoch, nonce, t1, t2, t3;
  uint32_t sequence;
} se_clock_packet;
uint32_t se_crc32c(const uint8_t *b, size_t n);
bool se_sensor_decode(const uint8_t *b, size_t n, se_sensor_packet *p);
void se_sensor_encode(const se_sensor_packet *p,
                      uint8_t b[SE_WIRE_SENSOR_SIZE]);
bool se_clock_decode(const uint8_t *b, size_t n, se_clock_packet *p);
void se_clock_encode(const se_clock_packet *p, uint8_t b[SE_WIRE_CLOCK_SIZE]);
/* FC32 timestamp unwrap; ambiguous (>2^31us) backwards jumps fail, epoch resets
 * explicit. */
typedef struct {
  uint64_t epoch, extended;
  uint32_t raw;
  bool present;
} se_unwrap;
bool se_unwrap_time(se_unwrap *s, uint64_t epoch, uint32_t raw, uint64_t *out);
typedef struct {
  uint64_t epoch, remote_anchor, local_anchor, last_local, fit_remote,
      fit_local;
  double scale;
  uint32_t uncertainty_us;
  unsigned samples;
  bool valid;
} se_clock_map;
bool se_clock_observe(se_clock_map *m, uint64_t epoch, uint64_t t1, uint64_t t2,
                      uint64_t t3, uint64_t t4);
/* Dedicated 921600-baud, 57-byte request/reply profile; symmetric >=600 us
 * one-way wire lower bound. Generic transports must use se_clock_observe. */
bool se_clock_observe_uart(se_clock_map *m, uint64_t epoch, uint64_t t1, uint64_t t2,
                           uint64_t t3, uint64_t t4);
bool se_clock_convert(const se_clock_map *m, uint64_t epoch, uint64_t remote,
                      uint64_t now, uint64_t *local, uint32_t *uncertainty);
#endif
