// ---------------------------------------------------------------------------
// Arduino IDE entry point.
//
// This file is intentionally empty. The Arduino build compiles every source
// under the sketch's src/ folder recursively, and src/main.cpp provides
// setup() and loop(). Keeping the .ino empty means the PlatformIO build
// (which ignores .ino files) and the Arduino build compile exactly the same
// code, with no duplicated entry point.
//
// Board   : M5Core2  (or "M5Stack-Core2" from the esp32 package)
// PSRAM   : enabled (required - the 30 s log ring lives there)
// Upload  : 921600 baud, fall back to 115200 if it fails
// ---------------------------------------------------------------------------
