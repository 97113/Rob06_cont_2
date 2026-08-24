// ---------------------------------------------------------------------------
// RS06 electric brake actuator controller
//   M5Stack Core2  +  Module13.2 PwrCAN  +  RobStride RS06
//
//   core 1 : 1 kHz control task (controller.cpp)
//   core 0 : UI + logging (this file -> ui.cpp)
//
// USB: 921600 baud line protocol for PC control and telemetry streaming.
// Run tools/rs06_console.py for the plotting GUI. See serial_link.h.
//
// HOLD BtnA (left touch pad) WHILE POWERING UP to run the CAN scanner: it
// sweeps every PwrCAN DIP pin pair, every bit rate and every motor CAN_ID,
// then stores whatever answered. Use it whenever the bus goes quiet.
//
// WIRING - read before powering up:
//   * PwrCAN XT30(2+2) power pins are tied straight to its 9-24 V DC jack.
//     The RS06 XT30 carries 48 V. Wire CANH/CANL ONLY; feed the PwrCAN from a
//     separate 9-24 V supply on the DC jack.
//   * PwrCAN "CAN Select" DIP -> Core2 GPIO:
//        CH1 TX G14   CH2 TX G2    CH3 TX G27   CH4 TX G0
//        CH5 RX G13   CH6 RX G19   CH7 RX G34   CH8 RX G35
//     This build is wired CH3 (TX G27) + CH6 (RX G19); the CAN SETUP page
//     and the boot scanner can change it without a rebuild.
//     The RS485 Select DIP must not claim the same pins.
//   * 120 ohm termination at both ends of the bus (about 60 ohm measured
//     across CANH/CANL with everything powered down).
// ---------------------------------------------------------------------------
#include <M5Unified.h>
#include "config.h"
#include "types.h"
#include "controller.h"
#include "can_scan.h"
#include "nvs_store.h"
#include "ui.h"
#include "serial_link.h"
#include "logger.h"
#include "command_source.h"

// The command source the controller reads its torque set-point from. Swap this
// for an AnalogSource / ExternalCanSource / UartSource when the real upper
// controller exists - nothing else changes.
static UiTorqueSource g_torque_src;

static int g_y = 60;
static void splash(const char* line, uint16_t col) {
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextColor(col, TFT_BLACK);
  M5.Display.setTextDatum(top_left);
  M5.Display.drawString(line, 10, g_y);
  g_y += 18;
}

void setup() {
  auto cfg = M5.config();
  cfg.internal_imu = false;
  cfg.internal_mic = false;
  cfg.internal_spk = false;
  M5.begin(cfg);

  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(top_center);
  M5.Display.drawString("RS06 BRAKE ACTUATOR", 160, 20);

  Serial.begin(921600);   // telemetry stream + PC control link

  // --- optional CAN discovery -------------------------------------------
  // Sample the touch pads a few times; BtnA held at power-up starts the scan.
  bool want_scan = false;
  for (int i = 0; i < 20 && !want_scan; i++) { M5.update(); want_scan = M5.BtnA.isPressed(); delay(10); }

  Params par;
  store::load(par);

  if (want_scan) {
    const can_scan::Result r = can_scan::run();
    if (r.verdict == can_scan::SCAN_FOUND) {
      par.motor_id = r.id;
      par.can_tx   = r.tx;
      par.can_rx   = r.rx;
      par.can_baud = r.baud;
      store::save(par);
    }
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setFont(&fonts::Font4);
    M5.Display.setTextDatum(top_center);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.drawString("RS06 BRAKE ACTUATOR", 160, 20);
    g_y = 60;
    char s[64];
    can_scan::describe(r, s, sizeof(s));
    splash(s, r.verdict == can_scan::SCAN_FOUND ? TFT_GREEN : TFT_ORANGE);
  }

  char s[64];
  snprintf(s, sizeof(s), "CAN TX G%u  RX G%u  %luk  id 0x%02X",
           par.can_tx, par.can_rx, (unsigned long)(par.can_baud / 1000), par.motor_id);
  splash(s, TFT_CYAN);

  if (logger::begin(LOG_CAPACITY)) {
    snprintf(s, sizeof(s), "log ring %u samples, SD %s",
             (unsigned)logger::capacity(), logger::sdReady() ? "ok" : "absent");
    splash(s, logger::sdReady() ? TFT_GREEN : TFT_ORANGE);
  } else {
    splash("log ring FAILED", TFT_RED);
  }

  if (!ctrl::begin(&g_torque_src)) {
    splash("CAN / control task FAILED", TFT_RED);
    splash("hold BtnA at power-up to run CAN scan", TFT_ORANGE);
    for (;;) { M5.update(); delay(100); }
  }
  splash("1 kHz control task up, motion-control mode", TFT_GREEN);

  delay(1200);
  ui::begin(&g_torque_src);
  serial_link::begin(&g_torque_src);
}

void loop() {
  serial_link::tick();   // cheap unless a stream is running
  ui::tick();
  delay(2);
}
