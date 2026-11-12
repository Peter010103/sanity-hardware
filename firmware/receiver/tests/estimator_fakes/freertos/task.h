#pragma once
#include "FreeRTOS.h"
static inline void vTaskDelay(unsigned delay){(void)delay;}
static inline void*xTaskCreateStaticPinnedToCore(void(*task)(void*),const char*name,unsigned bytes,void*arg,unsigned priority,StackType_t*stack,StaticTask_t*buffer,unsigned core){(void)task;(void)name;(void)bytes;(void)arg;(void)priority;(void)stack;(void)core;return buffer;}
