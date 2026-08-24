#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <driver/twai.h>   // gpio_num_t
#define FILE_WRITE "w"
struct File {
  explicit operator bool() const { return false; }
  void println(const char*) {}
  void write(const uint8_t*, size_t) {}
  void close() {}
};
struct SDClass {
  bool begin(gpio_num_t, SPIClass&, uint32_t) { return false; }
  bool exists(const char*) { return false; }
  bool mkdir(const char*)  { return false; }
  File open(const char*, const char*) { return File{}; }
};
extern SDClass SD;
