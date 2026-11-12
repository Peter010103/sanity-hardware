#include <stdint.h>
extern uint64_t fake_now;
static inline int64_t esp_timer_get_time(void){return fake_now;}
