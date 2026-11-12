#ifndef SANITY_MOCAP_STATE_H
#define SANITY_MOCAP_STATE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Network byte order. State: configured world Z-up, body FLU, q_world_body WXYZ.
 * This cache has no actuator/control authority. Callers serialize access. */
#define SZ_MOCAP_SIZE 88
#define SZ_MOCAP_CLOCK_REQUEST_SIZE 24
#define SZ_MOCAP_CLOCK_REPLY_SIZE 48
#define SZ_MOCAP_STATUS_SIZE 48
#define SZ_MOCAP_MAX_AGE_US 40000u
#define SZ_MOCAP_MAX_CLOCK_WIDTH_US 20000u
struct sz_mocap_sample {
    uint64_t epoch, source_ros_ns, capture_lower_us, capture_upper_us;
    uint32_t session, sequence;
    float position[3], quaternion[4], velocity[3];
};
struct sz_mocap_cache {
    struct sz_mocap_sample sample;
    uint64_t epoch, received_us;
    uint32_t accepted, rejected;
    uint8_t device;
    bool present;
};
void sz_mocap_init(struct sz_mocap_cache *cache, uint8_t device, uint64_t epoch);
bool sz_mocap_decode(const uint8_t *bytes, size_t n, uint8_t device, struct sz_mocap_sample *out);
bool sz_mocap_accept(struct sz_mocap_cache *cache, const struct sz_mocap_sample *sample, uint64_t now_us);
bool sz_mocap_usable(const struct sz_mocap_cache *cache, uint64_t now_us);
void sz_mocap_status(const struct sz_mocap_cache *cache, uint64_t now_us, uint8_t out[SZ_MOCAP_STATUS_SIZE]);
bool sz_mocap_clock_request(const uint8_t *bytes, size_t n, uint8_t device,
                            uint64_t *nonce, uint64_t *host_send_us);
void sz_mocap_clock_reply(uint8_t device, uint64_t epoch, uint64_t nonce, uint64_t host_send_us,
                          uint64_t received_us, uint64_t sent_us, uint8_t out[SZ_MOCAP_CLOCK_REPLY_SIZE]);
#endif
