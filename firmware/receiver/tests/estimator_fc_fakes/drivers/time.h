#include <stdint.h>
extern uint64_t fake_fc_now;
static inline uint32_t micros(void){return (uint32_t)fake_fc_now;}
