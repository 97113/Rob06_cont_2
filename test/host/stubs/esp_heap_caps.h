#pragma once
#include <stdlib.h>
#define MALLOC_CAP_SPIRAM (1 << 10)
#define MALLOC_CAP_8BIT   (1 << 2)
static inline void* heap_caps_malloc(size_t n, uint32_t caps) {
  // No PSRAM on the host: let the firmware take its documented fallback path.
  if (caps & MALLOC_CAP_SPIRAM) return nullptr;
  return malloc(n);
}
