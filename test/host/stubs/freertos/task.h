#pragma once
#include "FreeRTOS.h"
typedef void* TaskHandle_t;
TickType_t xTaskGetTickCount();
void vTaskDelayUntil(TickType_t* prev, TickType_t inc);
BaseType_t xTaskCreatePinnedToCore(void (*fn)(void*), const char* name,
                                   uint32_t stack, void* arg, int prio,
                                   TaskHandle_t* out, int core);
