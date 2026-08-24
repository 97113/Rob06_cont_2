// ---------------------------------------------------------------------------
// types.h : structures shared between the control task and the UI task.
// ---------------------------------------------------------------------------
#pragma once
#include <stdint.h>
#include "config.h"

// --- persisted settings ----------------------------------------------------
struct Params {
  uint32_t magic       = PARAMS_MAGIC;   // must stay the first member
  uint8_t motor_id     = DEFAULT_MOTOR_ID;

  // CAN wiring. Defaults match PwrCAN DIP CH3(TX G27) + CH6(RX G19), which is
  // how this machine is actually strapped. The CAN SETUP page and the
  // boot-time scanner both overwrite these.
  uint8_t  can_tx      = 27;   // DIP CH3
  uint8_t  can_rx      = 19;   // DIP CH6
  uint32_t can_baud    = 1000000;

  // limiters
  float lim_torque     = DEF_LIM_TORQUE;
  float lim_torque_hold= DEF_LIM_TORQUE_HOLD;
  float pos_deadband   = DEF_POS_DEADBAND;   // deg
  float lim_speed      = DEF_LIM_SPEED;
  float lim_acc        = DEF_LIM_ACC;
  float lim_dec        = DEF_LIM_DEC;
  float pos_min        = DEF_POS_MIN;
  float pos_max        = DEF_POS_MAX;

  // gains
  float kp             = DEF_KP;
  float kd             = DEF_KD;
  float kd_torque      = DEF_KD_TORQUE;

  // identified model
  float j_hat          = DEF_J_HAT;
  float fric_c         = DEF_FRIC_C;
  float fric_v         = DEF_FRIC_V;
  float kt             = DEF_KT_NM_PER_ARMS;

  // safety
  float temp_derate    = DEF_TEMP_DERATE_C;
  float temp_trip      = DEF_TEMP_TRIP_C;
  float vbus_margin    = DEF_VBUS_MARGIN_V;
  float vbus_trip_max  = DEF_VBUS_TRIP_MAX;
  float vbus_min       = DEF_VBUS_MIN_V;
  float regen_w_max    = DEF_REGEN_W_MAX;

  // software zero (second stage on top of the motor's mechanical zero)
  float zero_offset    = 0.0f;

  // measured capability, filled in by the calibration mode
  bool     calibrated  = false;
  float    meas_vmax   = 0.0f;   // rad/s actually reached
  float    meas_amax   = 0.0f;   // rad/s^2 actually reached
  uint32_t meas_reach_ms = 0;    // ms to first reach 200 deg
  float    meas_vbus_ref = 0.0f; // bus voltage at the time of calibration
  float    meas_vbus_sag = 0.0f; // worst sag during the capability run
  float    meas_rtt_us   = 0.0f;
  float    meas_drop_pct = 0.0f;
};

// --- controller state machine ---------------------------------------------
enum CtrlState : uint8_t {
  ST_INIT = 0,
  ST_IDLE,        // motor disabled
  ST_READY,       // enabled, zero torque hold
  ST_RUN_POS,     // position mode, following a trajectory
  ST_RUN_TRQ,     // torque mode, following the command source
  ST_CALIB,
  ST_FAULT_SOFT,  // ramping torque down, damped stop
  ST_FAULT_HARD,  // disabled by a severe fault
};

// --- event / fault bookkeeping --------------------------------------------
//
// Two words carry these, and which word a bit lands in is the whole point:
//
//   Telemetry::events  LATCHED. Something went wrong; it stays set until the
//                      operator clears the fault or restarts. Reading it
//                      answers "what happened?".
//   Telemetry::active  INSTANTANEOUS. Recomputed from scratch every tick.
//                      Reading it answers "what is limiting me right now?".
//
// Mixing the two is what made the old single word useless: EV_REGEN_LIMIT
// fires for one millisecond on every fast move (600 W / 31.5 rad/s = 19 Nm,
// just under the 20 Nm torque limit), and because nothing ever cleared it,
// ten of the twelve captures in tools/ show REGEN-LIM lit permanently - with
// any real fault hidden behind it.
enum EventBits : uint32_t {
  EV_CAN_MISS      = 1u << 0,
  EV_CAN_BUS_OFF   = 1u << 1,
  EV_TEMP_DERATE   = 1u << 2,
  EV_TEMP_TRIP     = 1u << 3,
  EV_OVERVOLT      = 1u << 4,
  EV_UNDERVOLT     = 1u << 5,
  EV_OVERCURRENT   = 1u << 6,
  EV_ENCODER       = 1u << 7,
  EV_STALL         = 1u << 8,
  EV_CMDSRC_STALE  = 1u << 9,
  EV_POS_LIMIT     = 1u << 10,
  EV_REGEN_LIMIT   = 1u << 11,
  EV_UNCALIBRATED  = 1u << 12,
  // Distinct causes that used to be flattened into EV_OVERCURRENT, so the
  // screen said "OVERCURRENT" when the gate driver had failed.
  EV_DRIVER_CHIP   = 1u << 13,   // Type21 bit1
  EV_POS_INIT      = 1u << 14,   // Type21 bit9,  position initialisation
  EV_HW_ID         = 1u << 15,   // Type21 bit8,  hardware id
  EV_AUX_STALE     = 1u << 16,   // VBUS / current readings have gone quiet
};

enum CtrlMode : uint8_t { CM_POSITION = 0, CM_TORQUE = 1 };

// --- live telemetry snapshot ----------------------------------------------
struct Telemetry {
  // measured
  float pos       = 0;   // rad, unwrapped, software zero applied
  float pos_raw   = 0;   // rad, straight from Type2 (wraps at +-4pi)
  float vel       = 0;   // rad/s
  float torque    = 0;   // N.m
  float temp      = 0;   // degC
  float iq        = 0;   // A   (Type17 0x701A)
  float vbus      = 0;   // V   (Type17 0x701C)
  uint32_t aux_age_ms = 0;      // age of the VBUS reading, ms
  bool     aux_ok     = false;  // VBUS fresh enough for the safety layer

  // commanded
  float p_cmd     = 0;
  float v_cmd     = 0;
  float a_cmd     = 0;
  float t_cmd     = 0;   // N.m actually sent as t_ff
  float i_cmd     = 0;   // A, t_cmd / kt (display only)
  float t_limit   = 0;   // effective torque cap after all limiters

  // status
  uint8_t  state       = ST_INIT;
  uint8_t  mode        = CM_POSITION;
  uint8_t  fb_fault    = 0;
  uint8_t  mode_state  = 0;
  uint32_t fault_word  = 0;
  uint32_t warn_word   = 0;
  uint32_t events      = 0;   // latched  - what went wrong
  uint32_t active      = 0;   // this tick - what is limiting right now
  bool     enabled     = false;

  // motion bookkeeping
  uint32_t reach_ms    = 0;
  float    peak_vel    = 0;

  // link quality
  uint32_t rtt_us      = 0;
  uint32_t tx          = 0;   // every frame sent, including aux requests
  uint32_t mtx         = 0;   // Type1 frames only - the drop-rate denominator
  uint32_t rx          = 0;
  uint32_t miss        = 0;
  float    vbus_trip   = 0;

  // TWAI controller internals (link bring-up diagnostics)
  uint8_t  twai_state  = 0;   // 0 stopped 1 running 2 bus-off 3 recovering
  uint16_t twai_tec    = 0;
  uint16_t twai_rec    = 0;
  uint32_t twai_buserr = 0;
  uint32_t twai_queued = 0;

  // calibration progress
  uint8_t  calib_step  = 0;
  uint8_t  calib_pct   = 0;
  char     calib_msg[32] = {0};
};

// --- one-shot requests from the UI to the control task --------------------
enum ReqType : uint8_t {
  REQ_NONE = 0,
  REQ_START,
  REQ_STOP,
  REQ_ZERO_MOTOR,     // Type6 mechanical zero + clear software offset
  REQ_ZERO_SOFT,      // software offset only
  REQ_CLEAR_FAULT,
  REQ_SET_MODE,       // arg_u8 = CtrlMode
  REQ_SET_TARGET_POS, // arg_f  = rad (absolute, after zero)
  REQ_SET_TORQUE,     // arg_f  = N.m
  REQ_CALIB_START,
  REQ_CALIB_ABORT,
  REQ_CAN_REINIT,     // tear down and restart TWAI with the stored pins/baud
  REQ_WRITE_MOTOR_CANID, // Type7: rewrite the motor's OWN CAN_ID (arg_u8)
  REQ_PARAMS_CHANGED, // re-push limits/gains to the motor
  REQ_LOG_DUMP,
};

struct CtrlRequest {
  uint8_t type   = REQ_NONE;
  uint8_t arg_u8 = 0;
  float   arg_f  = 0.0f;
};
