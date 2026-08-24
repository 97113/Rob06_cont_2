// ---------------------------------------------------------------------------
// can_scan.h : boot-time CAN discovery.
//
// Sweeps every pin pair the PwrCAN DIP can select, every supported bit rate,
// and every motor CAN_ID, and reports what actually answers. Runs before the
// control task exists, so it owns the TWAI driver outright.
//
// PwrCAN "CAN Select" DIP -> Core2 GPIO (from the board silkscreen crossed
// with the M5-Bus pin numbering):
//     CH1 TX G14 (bus 16)   CH5 RX G13 (bus 15)
//     CH2 TX G2  (bus 23)   CH6 RX G19 (bus 22)
//     CH3 TX G27 (bus 21)   CH7 RX G34 (bus 26)
//     CH4 TX G0  (bus 24)   CH8 RX G35 (bus 2)
// ---------------------------------------------------------------------------
#pragma once
#include <stdint.h>
#include <stddef.h>   // size_t (describe())

namespace can_scan {

enum Verdict : uint8_t {
  SCAN_FOUND = 0,   // a motor answered: tx/rx/baud/id are all valid
  SCAN_ACK_ONLY,    // some node ACKs our frames, but no motor ID replied
  SCAN_NOTHING,     // nothing ACKs anything - wiring, power or termination
};

struct Result {
  Verdict  verdict = SCAN_NOTHING;
  uint8_t  tx      = 0;      // GPIO number
  uint8_t  rx      = 0;
  uint32_t baud    = 0;      // bit/s
  uint8_t  id      = 0;      // motor CAN_ID
  uint8_t  dip_tx  = 0;      // 1..4  -> CH1..CH4
  uint8_t  dip_rx  = 0;      // 5..8  -> CH5..CH8
};

// Draws its own progress on M5.Display. Blocks for a few seconds.
Result run();

// Human readable one-liner for the splash screen.
void describe(const Result& r, char* out, size_t n);

} // namespace can_scan
