// ---------------------------------------------------------------------------
// Host implementations of the RTOS and platform services controller.cpp needs.
// A real (if tiny) queue, so a controller-level test can push requests; the
// task creator does not spawn anything - the test drives ticks itself.
// ---------------------------------------------------------------------------
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_timer.h>
#include "../../src/logger.h"
#include "../../src/nvs_store.h"

#include <cstring>
#include <deque>
#include <vector>

// --- tasks -----------------------------------------------------------------
TickType_t xTaskGetTickCount() { return (TickType_t)(esp_timer_get_time() / 1000); }
void vTaskDelayUntil(TickType_t* prev, TickType_t inc) { if (prev) *prev += inc; }
BaseType_t xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t,
                                   void*, int, TaskHandle_t* out, int) {
  if (out) *out = nullptr;
  return pdPASS;   // the test calls the tick function directly
}

// --- queue -----------------------------------------------------------------
namespace {
struct Queue {
  size_t item_size;
  size_t cap;
  std::deque<std::vector<unsigned char>> items;
};
}

QueueHandle_t xQueueCreate(int len, size_t item_size) {
  Queue* q = new Queue{ item_size, (size_t)len, {} };
  return (QueueHandle_t)q;
}

BaseType_t xQueueReceive(QueueHandle_t h, void* out, TickType_t) {
  Queue* q = (Queue*)h;
  if (!q || q->items.empty()) return pdFALSE;
  std::memcpy(out, q->items.front().data(), q->item_size);
  q->items.pop_front();
  return pdTRUE;
}

BaseType_t xQueueSend(QueueHandle_t h, const void* item, TickType_t) {
  Queue* q = (Queue*)h;
  if (!q || q->items.size() >= q->cap) return pdFALSE;
  const unsigned char* p = (const unsigned char*)item;
  q->items.emplace_back(p, p + q->item_size);
  return pdTRUE;
}

// --- logger ----------------------------------------------------------------
// The ring itself is not under test here; keep the last sample so a test can
// assert on what the control loop recorded.
namespace logger {
LogSample last_sample{};
size_t    pushed = 0;

bool   begin(size_t)        { return true; }
void   push(const LogSample& s) { last_sample = s; pushed++; }
void   pause()              {}
void   resume()             {}
void   clear()              { pushed = 0; }
size_t count()              { return pushed; }
size_t capacity()           { return 30000; }
int    dumpToSd(char*, size_t)      { return -1; }
int    dumpToStream(Print&, size_t) { return -1; }
const char* csvHeader()     { return ""; }
bool   sdReady()            { return false; }
}

// --- settings --------------------------------------------------------------
namespace store {
Params saved{};
bool   saved_any = false;
void load(Params& p) { if (saved_any) p = saved; }
void save(const Params& p) { saved = p; saved_any = true; }
void clear() { saved_any = false; }
}

// --- M5 / Arduino singletons (compile-only surface) -------------------------
#include <M5Unified.h>
#include <SPI.h>
#include <SD.h>

namespace fonts { Font Font2{0}, Font4{0}; }
M5Unified  M5;
SerialStub Serial;
SPIClass   SPI;
SDClass    SD;
void delay(uint32_t ms) { g_fake_now_us += (int64_t)ms * 1000; }
