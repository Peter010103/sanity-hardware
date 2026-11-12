#include <stdint.h>
static inline uint32_t esp_random(void){static uint32_t v=123456789;v=v*1664525u+1013904223u;return v;}
