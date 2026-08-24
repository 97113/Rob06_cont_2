// ---------------------------------------------------------------------------
// logger.h : lock-free single-producer ring in PSRAM, dumped to microSD.
//
// Producer  : control task @1 kHz (push() must stay allocation free)
// Consumer  : UI task, only while the controller is idle - dumping shares the
//             SPI bus with the LCD, so it is never done while moving.
// ---------------------------------------------------------------------------
#pragma once
#include <stdint.h>
#include <stddef.h>

class Print;

struct LogSample {
  uint32_t t_us;
  float    p_cmd, p_act;
  float    v_cmd, v_act;
  float    t_cmd, t_act;
  float    iq, vbus, temp;
  uint16_t events;
  uint8_t  state;
  uint8_t  _pad;
};

namespace logger {
  bool   begin(size_t capacity);
  void   push(const LogSample& s);     // control task
  void   pause();
  void   resume();
  void   clear();
  size_t count();
  size_t capacity();
  // Writes the ring, oldest first, as CSV. Returns the number of rows or -1.
  int    dumpToSd(char* path_out, size_t path_len);
  // Same rows, to any Print (the USB serial link). max_samples == 0 means all.
  int    dumpToStream(Print& out, size_t max_samples);
  const char* csvHeader();
  bool   sdReady();
}
