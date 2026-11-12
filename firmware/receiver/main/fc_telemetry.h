#ifndef SANITY_FC_TELEMETRY_H
#define SANITY_FC_TELEMETRY_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define SZ_FC_HEADER_SIZE 12
#define SZ_FC_RECORD_HEADER_SIZE 44
#define SZ_FC_PAYLOAD_MAX 128
#define SZ_FC_BATCH_SIZE 1400
#define SZ_FC_OUTPUT 1
#define SZ_FC_RESPONSE 2
#define SZ_FC_ERROR 3
#define SZ_FC_TIMEOUT 4
#define SZ_FC_HEALTH 5
struct sz_fc_health {
    uint64_t boot_id,last_disconnect_us;
    uint32_t boot_count,reset_reason,free_heap,min_free_heap,disconnects,connects,disconnect_reason;
    int8_t rssi;
    uint8_t channel;
};
struct sz_fc_record {
    uint8_t type, command, flags, length;
    uint64_t epoch;
    uint32_t session, sequence;
    uint64_t command_rx_us, start_us, end_us;
    uint8_t payload[SZ_FC_PAYLOAD_MAX];
};
struct sz_fc_parser {
    uint8_t state, length, index, checksum, command, direction, payload[255];
    uint64_t first_us;
    uint64_t pending[7], retry_after[7];
    bool outstanding[7];
    uint32_t malformed, skipped;
};
void sz_fc_init(struct sz_fc_parser *p);
bool sz_fc_byte(struct sz_fc_parser *p, uint8_t byte, uint64_t now_us, struct sz_fc_record *record);
bool sz_fc_query(struct sz_fc_parser *p, uint8_t command, uint64_t now_us, struct sz_fc_record *timeout);
size_t sz_fc_encode(const struct sz_fc_record *record, uint8_t *out, size_t capacity);
void sz_fc_health_record(struct sz_fc_record *record, const struct sz_fc_health *health, uint64_t epoch, uint64_t now_us);
void sz_fc_batch_header(uint8_t *out, uint8_t device, uint16_t count, uint32_t dropped);
void sz_fc_controller_query_payload(uint8_t out[6]);
/*254 is an internal query-slot ID only; wire request remains MSP255/0x7f14. */
#define SZ_FC_TIMING_QUERY_SLOT 254
#define SZ_FC_PAPER_QUERY_SLOT 253
void sz_fc_timing_query_payload(uint8_t out[6]);
bool sz_fc_is_controller_record(const struct sz_fc_record *record);
#endif
