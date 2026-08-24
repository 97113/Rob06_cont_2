#pragma once
#include <Arduino.h>
class Preferences {
public:
  bool   begin(const char*, bool = false) { return false; }
  void   end() {}
  size_t getBytesLength(const char*) { return 0; }
  size_t getBytes(const char*, void*, size_t) { return 0; }
  size_t putBytes(const char*, const void*, size_t) { return 0; }
  bool   clear() { return true; }
};
