#pragma once
#include <stdint.h>
/* Optional read-only acquisition. Associates source clock with receiver health;
 * no FC state, command or control dependency. Acquisition boot ID stays distinct. */
void sz_hx711_start(uint64_t receiver_boot_id, uint32_t receiver_boot_count);
