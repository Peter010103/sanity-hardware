#pragma once
#include "FreeRTOS.h"
#include <string.h>
static inline QueueHandle_t xQueueCreateStatic(unsigned n,unsigned size,uint8_t*storage,StaticQueue_t*q){q->bytes=storage;q->capacity=n;q->size=size;q->head=q->count=0;return q;}
static inline int xQueueSend(QueueHandle_t q,const void*item,unsigned timeout){(void)timeout;if(q->count==q->capacity)return 0;memcpy(q->bytes+((q->head+q->count)%q->capacity)*q->size,item,q->size);q->count++;return pdPASS;}
static inline int xQueueReceive(QueueHandle_t q,void*item,unsigned timeout){(void)timeout;if(!q->count)return 0;memcpy(item,q->bytes+q->head*q->size,q->size);q->head=(q->head+1)%q->capacity;q->count--;return pdPASS;}

static inline void xQueueReset(QueueHandle_t q){q->head=q->count=0;}
