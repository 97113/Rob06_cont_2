// Host stub: only what the firmware actually reaches for.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

static inline uint32_t millis() { return (uint32_t)(esp_timer_get_time() / 1000); }
static inline uint32_t micros() { return (uint32_t)esp_timer_get_time(); }
void delay(uint32_t ms);

// Arduino's output base class. logger::dumpToStream() takes a Print&.
class Print {
public:
  virtual ~Print() {}
  void print(const char*) {}
  void print(unsigned long) {}
  void print(int) {}
  void print(double, int) {}
  void println() {}
  void println(const char*) {}
  void println(unsigned long) {}
  void println(int) {}
  void println(double, int) {}
  void write(const uint8_t*, size_t) {}
};

struct SerialStub : public Print {
  void begin(unsigned long) {}
  int  available() { return 0; }
  int  read()      { return -1; }
};
extern SerialStub Serial;
