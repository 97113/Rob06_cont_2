// ---------------------------------------------------------------------------
// ui.h : 320x240 touch UI, runs on core 0 at ~20 Hz.
// Never touches CAN directly - everything goes through ctrl::request().
// ---------------------------------------------------------------------------
#pragma once
#include "command_source.h"

namespace ui {
  void begin(UiTorqueSource* torque_src);
  void tick();          // call from loop()
}
