#pragma once
#include "FreeRTOS.h"
#include <stddef.h>
typedef void* QueueHandle_t;
QueueHandle_t xQueueCreate(int len, size_t item_size);
BaseType_t xQueueReceive(QueueHandle_t q, void* out, TickType_t wait);
BaseType_t xQueueSend(QueueHandle_t q, const void* item, TickType_t wait);
