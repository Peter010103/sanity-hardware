#pragma once
#include "../mocap_state.h"
#include <stddef.h>
#include <stdint.h>
void se_runtime_start(void);
void se_runtime_epoch(uint64_t epoch);
void se_runtime_byte(uint8_t byte, uint64_t received_us);
size_t se_runtime_poll(uint64_t now, uint8_t out[64]);
size_t se_runtime_poll_in_slot(uint64_t now, uint8_t out[64], bool clock_slot);
void se_runtime_pose(const struct sz_mocap_sample *pose, uint64_t received_us);
size_t se_runtime_record(uint8_t out[256]);
