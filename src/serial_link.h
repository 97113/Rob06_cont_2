// ---------------------------------------------------------------------------
// serial_link.h : line based USB control + telemetry link.
//
// Runs on core 0 next to the UI. Never touches CAN directly - every command
// goes through ctrl::request(), so the safety state machine and the limiter
// chain apply exactly as they do for the touch screen.
//
// PC -> Core2 (one command per line, case insensitive)
//   PING                 -> OK PONG
//   ID                   -> banner
//   START | STOP | CLEARFAULT
//   MODE POS | MODE TRQ
//   MOVE <deg>           absolute target angle
//   TRQ  <Nm>            torque set-point (torque mode)
//   ZERO MOTOR | ZERO SOFT
//   CALIB START | CALIB ABORT
//   PARAMS               -> P <key>=<value> ... one line per parameter
//   SET <key> <value>    -> OK SET <key>=<value>
//   STREAM <hz>          0 disables. Clamped to 200 Hz.
//   DUMP [seconds]       high resolution 1 kHz capture, default 3 s
//   SDDUMP               write the ring to microSD instead
//
// Core2 -> PC
//   #RS06 ...            banner / comments
//   OK <text> | ERR <text>
//   T,<fields...>        streamed telemetry (header announced by STREAM).
//                        Two event words: `events` is latched until the fault
//                        is cleared, `active` is rebuilt every control tick
//                        and says what is limiting the axis right now.
//   D,<n> then CSV rows then DEND
// ---------------------------------------------------------------------------
#pragma once
#include "command_source.h"

namespace serial_link {
  void begin(UiTorqueSource* torque_src);
  void tick();     // call from loop()
}
