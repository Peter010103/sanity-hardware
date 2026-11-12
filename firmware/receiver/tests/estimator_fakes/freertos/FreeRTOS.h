#pragma once
#include <stdint.h>
#include <stddef.h>
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
typedef uint32_t StackType_t;
typedef struct {int unused;} StaticTask_t;
typedef struct {uint8_t*bytes;unsigned capacity,size,head,count;} StaticQueue_t;
typedef StaticQueue_t* QueueHandle_t;

#define configTICK_RATE_HZ 1000
