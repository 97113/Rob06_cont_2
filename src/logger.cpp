#include "logger.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <esp_heap_caps.h>

namespace {
  LogSample*      buf_   = nullptr;
  size_t          cap_   = 0;
  volatile size_t head_  = 0;   // next write slot
  volatile size_t fill_  = 0;
  volatile bool   paused_= false;
  bool            sd_ok_ = false;
}

namespace logger {

bool begin(size_t capacity) {
  buf_ = (LogSample*)heap_caps_malloc(capacity * sizeof(LogSample), MALLOC_CAP_SPIRAM);
  if (!buf_) {
    // Fall back to a much smaller internal-RAM ring rather than losing logging.
    capacity = 2000;
    buf_ = (LogSample*)heap_caps_malloc(capacity * sizeof(LogSample), MALLOC_CAP_8BIT);
    if (!buf_) return false;
  }
  cap_ = capacity; head_ = 0; fill_ = 0;

  // Core2: SCK=18, MISO=38, MOSI=23, TF_CS=4 (shared with the LCD bus).
  SPI.begin(18, 38, 23, -1);   // SS handled by SD.begin
  sd_ok_ = SD.begin(GPIO_NUM_4, SPI, 20000000);
  return true;
}

void push(const LogSample& s) {
  if (!buf_ || paused_) return;
  buf_[head_] = s;
  head_ = (head_ + 1) % cap_;
  if (fill_ < cap_) fill_++;
}

void pause()  { paused_ = true;  }
void resume() { paused_ = false; }
void clear()  { head_ = 0; fill_ = 0; }
size_t count()    { return fill_; }
size_t capacity() { return cap_; }
bool   sdReady()  { return sd_ok_; }

const char* csvHeader() {
  return "t_us,p_cmd_deg,p_act_deg,v_cmd,v_act,t_cmd,t_act,iq,vbus,temp,state,events";
}

// Shared row formatter so the SD file and the serial dump can never drift.
static int formatRow(const LogSample& s, char* line, size_t n) {
  return snprintf(line, n,
    "%lu,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.3f,%.2f,%.1f,%u,%u\n",
    (unsigned long)s.t_us,
    s.p_cmd * 57.29578f, s.p_act * 57.29578f,
    s.v_cmd, s.v_act, s.t_cmd, s.t_act,
    s.iq, s.vbus, s.temp,
    (unsigned)s.state, (unsigned)s.events);
}

int dumpToStream(Print& out, size_t max_samples) {
  if (!buf_) return -1;
  pause();
  size_t n = fill_;
  if (max_samples && max_samples < n) n = max_samples;
  const size_t start = (head_ + cap_ - n) % cap_;

  out.print("D,"); out.println((unsigned long)n);
  out.println(csvHeader());
  char line[192];
  for (size_t i = 0; i < n; i++) {
    const int len = formatRow(buf_[(start + i) % cap_], line, sizeof(line));
    out.write((const uint8_t*)line, len);
  }
  out.println("DEND");
  resume();
  return (int)n;
}

int dumpToSd(char* path_out, size_t path_len) {
  if (!buf_ || !sd_ok_) return -1;
  pause();

  if (!SD.exists("/rs06")) SD.mkdir("/rs06");
  char path[48];
  for (int i = 0; i < 1000; i++) {
    snprintf(path, sizeof(path), "/rs06/log%03d.csv", i);
    if (!SD.exists(path)) break;
  }
  File f = SD.open(path, FILE_WRITE);
  if (!f) { resume(); return -1; }

  f.println(csvHeader());

  const size_t n     = fill_;
  const size_t start = (head_ + cap_ - n) % cap_;
  char line[192];
  for (size_t i = 0; i < n; i++) {
    const LogSample& s = buf_[(start + i) % cap_];
    int len = snprintf(line, sizeof(line),
      "%lu,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.3f,%.2f,%.1f,%u,%u\n",
      (unsigned long)s.t_us,
      s.p_cmd * 57.29578f, s.p_act * 57.29578f,
      s.v_cmd, s.v_act, s.t_cmd, s.t_act,
      s.iq, s.vbus, s.temp,
      (unsigned)s.state, (unsigned)s.events);
    f.write((const uint8_t*)line, len);
  }
  f.close();
  if (path_out) snprintf(path_out, path_len, "%s", path);
  resume();
  return (int)n;
}

} // namespace logger
