// ---------------------------------------------------------------------------
// command_source.h : where the torque set-point comes from.
//
// The real machine's upper controller is not decided yet, so the controller
// only ever talks to this interface. Swapping in an analog pedal, an external
// CAN master or a UART link is a matter of instantiating a different subclass
// in main.cpp - nothing in the control loop changes.
//
// healthy() going false is treated as a SOFT fault: torque ramps to zero and
// the actuator comes to a damped stop.
// ---------------------------------------------------------------------------
#pragma once
#include <stdint.h>
#include <math.h>
#include "config.h"

class CommandSource {
public:
  virtual ~CommandSource() {}
  virtual float       torqueNm(uint32_t now_ms) = 0;
  virtual bool        healthy(uint32_t now_ms)  = 0;
  virtual const char* name() const              = 0;
};

// --- the one we ship today: the CONTROL page ------------------------------
class UiTorqueSource : public CommandSource {
public:
  void set(float nm, uint32_t now_ms) { value_ = nm; stamp_ = now_ms; live_ = true; }
  void clear()                        { value_ = 0.0f; live_ = false; }

  float torqueNm(uint32_t) override { return value_; }
  // A locally-typed set-point cannot go stale - there is no link to lose.
  bool  healthy(uint32_t) override  { return true; }
  const char* name() const override { return "UI"; }

  uint32_t lastSetMs() const { return stamp_; }
  bool     touched()   const { return live_; }

private:
  float    value_ = 0.0f;
  uint32_t stamp_ = 0;
  bool     live_  = false;
};

// --- template for a link that can actually go stale ------------------------
// Kept compiled so the staleness path is never bit-rotted; not instantiated
// until a real upper controller exists.
class TimestampedSource : public CommandSource {
public:
  void feed(float nm, uint32_t now_ms) { value_ = nm; stamp_ = now_ms; seen_ = true; }
  float torqueNm(uint32_t) override    { return value_; }
  bool  healthy(uint32_t now_ms) override {
    return seen_ && (now_ms - stamp_) <= CMDSRC_STALE_MS;
  }
  const char* name() const override { return "EXT"; }
protected:
  float    value_ = 0.0f;
  uint32_t stamp_ = 0;
  bool     seen_  = false;
};
