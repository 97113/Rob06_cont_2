// ---------------------------------------------------------------------------
// controller.h : the 1 kHz control task.
//
// Owns the CAN driver, the trajectory generator, the calibration machine and
// the safety state machine. The UI only ever talks to it through request() /
// snapshot() / setParams(), never by touching the motor directly.
// ---------------------------------------------------------------------------
#pragma once
#include "types.h"
#include "command_source.h"

namespace ctrl {

bool begin(CommandSource* src);

// Non-blocking; queued and serviced one per control tick.
void request(const CtrlRequest& r);

void snapshot(Telemetry& out);
void getParams(Params& out);
void setParams(const Params& p);   // also re-pushes limits to the motor

// True while the axis is enabled or calibrating - the SD dump and the
// mechanical-zero write both refuse to run in that condition.
bool busy();

const char* stateName(uint8_t s);

} // namespace ctrl
