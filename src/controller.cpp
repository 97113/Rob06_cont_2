#include "controller.h"
#include "rs06_driver.h"
#include "traj.h"
#include "calib.h"
#include "logger.h"
#include "nvs_store.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_timer.h>

using namespace rs06;

namespace {

constexpr float DEG        = 0.017453293f;
constexpr float REACH_BAND = 5.0f * DEG;          // the +-5 deg acceptance band
constexpr float DT         = CTRL_PERIOD_US * 1e-6f;
// Type1 can only express +-4*pi, so the whole working range must stay inside
// one revolution window of the mechanical zero. The soft window is checked
// against the unwrapped position; crossing it is a soft fault.
constexpr float PSET_CLAMP = P_LIM - 0.05f;
constexpr float POS_WRAP_GUARD = 12.4f;

Rs06           g_can;
Trajectory     g_traj;
Calibration    g_cal;
// Calibration writes its results incrementally, so it needs a Params copy that
// survives between ticks - the per-tick snapshot of g_par would discard them.
Params         g_calP;
CommandSource* g_src = nullptr;

Params         g_par;      // authoritative settings, guarded by g_mux
Telemetry      g_pub;      // what the UI reads,      guarded by g_mux
Telemetry      g_tm;       // control task working copy, NOT guarded
portMUX_TYPE   g_mux = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t  g_q   = nullptr;

// --- control task private state -------------------------------------------
CtrlState g_state      = ST_INIT;
CtrlMode  g_mode       = CM_POSITION;
bool      g_enabled    = false;
// Latched: something went wrong, held until CLEAR FAULT / START.
uint32_t  g_events     = 0;
// Instantaneous: rebuilt from nothing every tick. See EventBits in types.h.
uint32_t  g_active     = 0;
uint32_t  g_miss_run   = 0;         // consecutive missed replies
uint32_t  g_aux        = 0;
uint32_t  g_soft_t0    = 0;
float     g_soft_t_ref = 0.0f;
float     g_iq         = 0.0f;
float     g_vbus       = 0.0f;
bool      g_vbus_ok    = false;     // reading fresh enough to act on
uint32_t  g_vbus_age   = 0;
uint32_t  g_fault_word = 0;
uint32_t  g_warn_word  = 0;

// --- VBUS rest-voltage reference ------------------------------------------
// The overvoltage trip is relative to the supply actually fitted, so it needs
// a trustworthy rest voltage. The first good sample seeds it immediately (no
// window without protection), then the median of VBUS_REF_SAMPLES readings
// taken while the motor is disabled replaces that provisional value.
float     g_vbus_boot  = 0.0f;
float     g_vbus_trip  = DEF_VBUS_TRIP_MAX;
float     g_vbus_ref_buf[VBUS_REF_SAMPLES] = {0};
int       g_vbus_ref_n    = 0;
bool      g_vbus_ref_done = false;
// Age of the cached VBUS reading last tick. The cache is read every tick but
// only refilled when a reply lands, so "the age went down" is how a genuinely
// new measurement is told apart from the same one read again.
uint32_t  g_vbus_prev_age = 0xFFFFFFFFu;

// position unwrapping
float     g_prev_raw   = 0.0f;
int32_t   g_turns      = 0;
bool      g_first_fb   = true;
float     g_pos        = 0.0f;      // unwrapped, software zero applied
float     g_pos_unwrap = 0.0f;      // unwrapped, motor frame

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float sgnf(float v)                       { return v >= 0.0f ? 1.0f : -1.0f; }

// ---------------------------------------------------------------------------
// Drop everything known about the supply voltage. Called when the CAN wiring
// changes underneath us: a reference measured through the previous pin pair
// (or from a motor that is no longer answering) must not keep arming the
// overvoltage trip.
// ---------------------------------------------------------------------------
void resetVbusRef() {
  g_vbus_boot    = 0.0f;
  g_vbus_trip    = DEF_VBUS_TRIP_MAX;
  g_vbus_ref_n   = 0;
  g_vbus_ref_done= false;
  g_vbus_prev_age= 0xFFFFFFFFu;
  g_vbus         = 0.0f;
  g_vbus_ok      = false;
  g_vbus_age     = 0;
  g_iq           = 0.0f;
}

// Median of the collected rest samples. Nine elements, called once - an
// insertion sort on a copy is the right amount of machinery here.
float vbusRefMedian() {
  float s[VBUS_REF_SAMPLES];
  for (int i = 0; i < g_vbus_ref_n; i++) s[i] = g_vbus_ref_buf[i];
  for (int i = 1; i < g_vbus_ref_n; i++) {
    const float key = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > key) { s[j + 1] = s[j]; j--; }
    s[j + 1] = key;
  }
  return s[g_vbus_ref_n / 2];
}

// ---------------------------------------------------------------------------
// Push the settings that live inside the motor. Only called while disabled -
// it burns several CAN round trips and the manual forbids changing the run
// mode while the joint is moving.
// ---------------------------------------------------------------------------
void pushMotorConfig(const Params& P) {
  g_can.setMotorId(P.motor_id);
  g_can.writeParamU8 (P_RUN_MODE, MODE_MOTION);         // stay in motion control
  g_can.writeParamF  (P_LIMIT_TORQUE, P.lim_torque);
  g_can.writeParamU32(P_CAN_TMO, MOTOR_CAN_TIMEOUT);    // motor-side watchdog
  g_can.setActiveReport(false);                         // Type2 comes with Type1
}

// ---------------------------------------------------------------------------
void goSoftFault(uint32_t ev, float last_torque) {
  if (g_state == ST_FAULT_HARD) return;
  g_events |= ev;
  if (g_state == ST_FAULT_SOFT) return;
  g_state      = ST_FAULT_SOFT;
  g_soft_t0    = millis();
  g_soft_t_ref = last_torque;
  g_cal.abort();
}

void goHardFault(uint32_t ev) {
  g_events |= ev;
  if (g_state == ST_FAULT_HARD) return;
  g_state = ST_FAULT_HARD;
  g_cal.abort();
  Feedback fb;
  g_can.stop(false, fb);
  g_enabled = false;
}

void doStop() {
  Feedback fb;
  g_can.stop(false, fb);
  g_enabled = false;
  g_state   = ST_IDLE;
  g_cal.abort();
  g_traj.reset(g_pos);
}

// ---------------------------------------------------------------------------
// Returns true when the request consumed this tick's CAN slot.
// ---------------------------------------------------------------------------
bool handleRequest(const CtrlRequest& r, Params& P) {
  switch (r.type) {

  case REQ_START:
    if (g_state == ST_FAULT_HARD) return false;
    if (!g_enabled) {
      pushMotorConfig(P);
      Feedback fb;
      if (!g_can.enable(fb)) return true;
      g_enabled = true;
    }
    g_traj.configure(P.lim_speed, P.lim_acc, P.lim_dec);
    g_traj.commitDecelCfg();
    g_traj.reset(g_pos);
    g_state  = (g_mode == CM_POSITION) ? ST_RUN_POS : ST_RUN_TRQ;
    g_events = 0;
    return true;

  case REQ_STOP:
    doStop();
    return true;

  case REQ_CLEAR_FAULT: {
    Feedback fb;
    g_can.stop(true, fb);                  // Byte0 = 1 clears the fault
    g_enabled    = false;
    g_fault_word = 0;
    g_warn_word  = 0;
    g_events     = 0;
    g_miss_run   = 0;
    g_state      = ST_IDLE;
    return true;
  }

  case REQ_ZERO_MOTOR: {
    // Only while stopped. In motion-control mode the new firmware also snaps
    // the internal target to 0, so the axis does not jump.
    if (g_state != ST_IDLE && g_state != ST_READY) return false;
    Feedback fb;
    if (!g_can.setZero(fb)) return true;
    g_turns = 0; g_first_fb = true; g_pos = 0; g_pos_unwrap = 0;
    P.zero_offset = 0.0f;
    portENTER_CRITICAL(&g_mux); g_par.zero_offset = 0.0f; portEXIT_CRITICAL(&g_mux);
    g_traj.reset(0.0f);
    return true;
  }

  case REQ_ZERO_SOFT:
    P.zero_offset += g_pos;                // current reading becomes 0
    portENTER_CRITICAL(&g_mux); g_par.zero_offset = P.zero_offset; portEXIT_CRITICAL(&g_mux);
    g_pos = 0.0f;
    g_traj.reset(0.0f);
    return false;

  case REQ_SET_MODE:
    if (g_state == ST_RUN_POS || g_state == ST_RUN_TRQ) {
      g_mode  = (CtrlMode)r.arg_u8;
      g_state = (g_mode == CM_POSITION) ? ST_RUN_POS : ST_RUN_TRQ;
      g_traj.reset(g_pos);
    } else {
      g_mode = (CtrlMode)r.arg_u8;
    }
    return false;

  case REQ_SET_TARGET_POS: {
    const float tgt = clampf(r.arg_f, P.pos_min, P.pos_max);
    if (g_state == ST_RUN_POS) {
      g_traj.configure(P.lim_speed, P.lim_acc, P.lim_dec);
      g_traj.commitDecelCfg();
      g_traj.moveTo(tgt, g_pos, g_tm.vel);
    }
    return false;
  }

  case REQ_SET_TORQUE:
    return false;                          // the command source already has it

  case REQ_CALIB_START:
    if (g_state == ST_FAULT_HARD) return false;
    if (!g_enabled) {
      pushMotorConfig(P);
      Feedback fb;
      if (!g_can.enable(fb)) return true;
      g_enabled = true;
    }
    g_calP  = P;
    g_cal.start(g_calP, g_pos);
    g_state = ST_CALIB;
    return true;

  case REQ_CALIB_ABORT:
    g_cal.abort();
    doStop();
    return true;

  case REQ_CAN_REINIT: {
    // Changing the CAN pins or bit rate means reinstalling the TWAI driver,
    // which is only safe with the motor disabled.
    if (g_enabled || g_state == ST_CALIB) return false;
    g_can.end();
    if (!g_can.begin(P.motor_id, P.can_tx, P.can_rx, P.can_baud)) {
      g_events |= EV_CAN_BUS_OFF;
      return true;
    }
    g_can.resetStats();
    g_can.resetCache();
    // The supply reference was measured through the old wiring; keeping it
    // would arm the overvoltage trip against a number that no longer means
    // anything. Re-measure from scratch.
    resetVbusRef();
    g_miss_run   = 0;
    g_fault_word = 0;
    g_warn_word  = 0;
    g_events     = 0;
    g_first_fb   = true;
    g_state      = ST_IDLE;
    return true;
  }

  case REQ_WRITE_MOTOR_CANID: {
    // Refused while enabled: the manual forbids reconfiguring a running joint,
    // and a mid-move id change would orphan the control loop.
    if (g_enabled || g_state == ST_CALIB) return false;
    if (!g_can.setCanId(r.arg_u8)) return true;
    g_can.save();                          // Type22, persist inside the motor
    P.motor_id = r.arg_u8;
    portENTER_CRITICAL(&g_mux); g_par.motor_id = r.arg_u8; portEXIT_CRITICAL(&g_mux);
    return true;
  }

  case REQ_PARAMS_CHANGED:
    if (!g_enabled) { pushMotorConfig(P); return true; }
    g_can.writeParamF(P_LIMIT_TORQUE, P.lim_torque);   // safe while running
    g_traj.configure(P.lim_speed, P.lim_acc, P.lim_dec);
    g_traj.commitDecelCfg();
    return true;

  default:
    return false;
  }
}

// ---------------------------------------------------------------------------
// The limiter chain. Runs on every command, in every state.
// ---------------------------------------------------------------------------
void applyLimits(const Params& P, MotionCmd& c, bool holding) {
  // 1. thermal derate. Derating is a live condition, not an incident: it goes
  // in the instantaneous word so it clears itself once the motor cools.
  float tscale = 1.0f;
  if (g_tm.temp > P.temp_derate) {
    tscale = clampf((P.temp_trip - g_tm.temp) / (P.temp_trip - P.temp_derate), 0.0f, 1.0f);
    g_active |= EV_TEMP_DERATE;
  }
  float tcap = P.lim_torque * tscale;

  // 1b. parked on a set-point: a brake actuator does not need move-level
  // torque to hold, and capping it here is what keeps the standstill current
  // (and the audible buzz) down.
  if (holding && P.lim_torque_hold > 0.0f) {
    tcap = fminf(tcap, P.lim_torque_hold * tscale);
  }

  // 2. regen power ceiling while braking (torque opposing motion)
  const float w = fabsf(g_tm.vel);
  const bool braking = (c.t_ff * g_tm.vel < 0.0f) && (w > 0.5f);
  if (braking) {
    const float tcap_regen = P.regen_w_max / w;
    if (tcap_regen < tcap) { tcap = tcap_regen; g_active |= EV_REGEN_LIMIT; }
    // Tell the profiler too, so it stops planning a deceleration it is not
    // allowed to execute (otherwise it would overshoot the target).
    g_traj.setDecelCap(tcap_regen / fmaxf(P.j_hat, 1e-4f));
  } else {
    g_traj.setDecelCap(P.lim_dec);
  }

  // 3. bus overvoltage: stop feeding energy back immediately. Only ever acted
  // on with a fresh reading - cutting braking torque on a value that is one
  // second old is as likely to be wrong as right.
  if (g_vbus_ok && g_vbus > g_vbus_trip && braking) {
    c.t_ff = 0.0f;
    g_active |= EV_OVERVOLT;
    g_events |= EV_OVERVOLT;   // worth keeping a record of, unlike the caps
  }

  // 4. hard caps
  c.t_ff  = clampf(c.t_ff,  -tcap, tcap);
  c.v_set = clampf(c.v_set, -P.lim_speed, P.lim_speed);
  c.kp    = clampf(c.kp,    0.0f, KP_LIM);
  c.kd    = clampf(c.kd,    0.0f, KD_LIM);
  g_tm.t_limit = tcap;
}

// ---------------------------------------------------------------------------
void evaluateFaults(const Params& P, uint32_t now_ms, float last_torque) {
  // --- severe, from the Type2 status word --------------------------------
  const uint8_t f2 = g_tm.fb_fault;
  if (f2 & FB2_OVERCURRENT) goHardFault(EV_OVERCURRENT);
  if (f2 & FB2_OVERTEMP)    goHardFault(EV_TEMP_TRIP);
  if (f2 & FB2_ENCODER)     goHardFault(EV_ENCODER);
  if (f2 & FB2_STALL)       goHardFault(EV_STALL);
  if (f2 & FB2_UNCALIBRATED) g_active |= EV_UNCALIBRATED;

  // --- severe, from the Type21 fault frame (overvoltage lives only here) --
  // Mapped bit by bit. Collapsing six unrelated causes into EV_OVERCURRENT,
  // as this used to, made the screen report a phase overcurrent when what had
  // actually failed was the gate driver or the position initialisation.
  if (g_fault_word & FLT_OVERVOLTAGE)  goHardFault(EV_OVERVOLT);
  if (g_fault_word & FLT_UNDERVOLTAGE) goHardFault(EV_UNDERVOLT);
  if (g_fault_word & (FLT_IA_OVERCUR | FLT_IB_OVERCUR | FLT_IC_OVERCUR))
                                       goHardFault(EV_OVERCURRENT);
  if (g_fault_word & FLT_OVERTEMP)     goHardFault(EV_TEMP_TRIP);
  if (g_fault_word & FLT_DRIVER_CHIP)  goHardFault(EV_DRIVER_CHIP);
  if (g_fault_word & FLT_STALL_ALGO)   goHardFault(EV_STALL);
  if (g_fault_word & FLT_POS_INIT)     goHardFault(EV_POS_INIT);
  if (g_fault_word & FLT_HW_ID)        goHardFault(EV_HW_ID);
  if (g_fault_word & FLT_ENC_UNCAL)    goHardFault(EV_ENCODER);

  // --- severe, measured locally ------------------------------------------
  // The undervoltage floor is the higher of the fixed limit and a fraction of
  // the measured rest voltage, so the same build guards a 32 V bench supply
  // and a 13S pack without anyone remembering to change a number.
  const float vfloor = fmaxf(P.vbus_min,
                             g_vbus_ref_done ? g_vbus_boot * VBUS_SAG_TRIP_FRAC : 0.0f);
  if (g_tm.temp >= P.temp_trip)                       goHardFault(EV_TEMP_TRIP);
  if (g_vbus_ok && g_vbus > g_vbus_trip + 2.0f)       goHardFault(EV_OVERVOLT);
  if (g_vbus_ok && g_vbus < vfloor)                   goHardFault(EV_UNDERVOLT);
  if (g_miss_run >= DEF_CAN_MISS_HARD)                goHardFault(EV_CAN_MISS);
  if (!g_can.busHealthy())                            goHardFault(EV_CAN_BUS_OFF);

  // --- mild ---------------------------------------------------------------
  if (f2 & FB2_UNDERVOLTAGE)                 goSoftFault(EV_UNDERVOLT, last_torque);
  if (g_warn_word & WRN_OVERTEMP)            goSoftFault(EV_TEMP_DERATE, last_torque);
  if (g_miss_run >= DEF_CAN_MISS_SOFT)       goSoftFault(EV_CAN_MISS, last_torque);
  if (g_src && !g_src->healthy(now_ms) && g_state == ST_RUN_TRQ)
                                             goSoftFault(EV_CMDSRC_STALE, last_torque);
  if (fabsf(g_pos_unwrap) > POS_WRAP_GUARD ||
      g_pos < P.pos_min - REACH_BAND || g_pos > P.pos_max + REACH_BAND)
                                             goSoftFault(EV_POS_LIMIT, last_torque);
}

// ---------------------------------------------------------------------------
// Auxiliary telemetry: VBUS, phase current and the Type21 fault frame.
//
// Requests are fire-and-forget - one CAN frame, no wait. Replies are filed by
// Rs06::pump() whenever they turn up and are read back out of the cache here,
// with an age attached. That decoupling is the fix for the failure this
// replaces: the old code waited 300 us inside the slot, and any reply slower
// than that was swallowed and dropped by the next tick's feedback wait. VBUS
// updated 0.7 times a second against a design rate of 200 Hz, and the current
// reading never updated at all - so every protection built on VBUS was
// effectively running blind.
// ---------------------------------------------------------------------------
void auxPoll(const Params& P, bool braking) {
  g_aux++;

  // --- schedule ------------------------------------------------------------
  const uint32_t vdiv = braking ? AUX_VBUS_DIV_DECEL : AUX_VBUS_DIV;
  if      (g_aux % vdiv == 0)           g_can.requestParam(P_VBUS);
  else if (g_aux % AUX_FAULT_DIV == 3)  g_can.requestFault();
  else if (g_aux % AUX_IQF_DIV  == 7)   g_can.requestParam(P_IQF);

  // --- harvest -------------------------------------------------------------
  float v; uint32_t age;

  if (g_can.paramValue(P_VBUS, v, age) && v > 1.0f && v < 100.0f) {
    g_vbus     = v;
    g_vbus_age = age;
    g_vbus_ok  = (age <= AUX_STALE_MS);
    const bool new_sample = (age < g_vbus_prev_age);
    g_vbus_prev_age = age;

    if (g_vbus_ok) {
      // Seed the trip from the first good reading so there is never a window
      // with no overvoltage protection at all...
      if (g_vbus_boot <= 0.0f) {
        g_vbus_boot = v;
        g_vbus_trip = fminf(DEF_VBUS_TRIP_MAX, v + P.vbus_margin);
      }
      // ...then replace that provisional value with a median taken while the
      // motor is disabled, which is the only time the rail is guaranteed to
      // be at rest (regen cannot lift a rail the motor is not driving into).
      if (new_sample && !g_vbus_ref_done && !g_enabled) {
        g_vbus_ref_buf[g_vbus_ref_n++] = v;
        if (g_vbus_ref_n >= VBUS_REF_SAMPLES) {
          g_vbus_boot     = vbusRefMedian();
          g_vbus_trip     = fminf(DEF_VBUS_TRIP_MAX, g_vbus_boot + P.vbus_margin);
          g_vbus_ref_done = true;
        }
      }
    }
  } else {
    g_vbus_ok       = false;
    g_vbus_age      = AUX_STALE_MS + 1;
    g_vbus_prev_age = 0xFFFFFFFFu;
  }

  if (g_can.paramValue(P_IQF, v, age) && fabsf(v) < 100.0f && age <= AUX_STALE_MS) {
    g_iq = v;
  }

  uint32_t f, w;
  if (g_can.faultWords(f, w, age) && age <= AUX_STALE_MS) {
    g_fault_word = f;
    g_warn_word  = w;
  }

  // A silent auxiliary channel is itself worth showing: it means every VBUS
  // guard below is inactive, which is exactly the condition that went
  // unnoticed for the whole of the captured test history.
  if (!g_vbus_ok && g_enabled) g_active |= EV_AUX_STALE;
}

// ---------------------------------------------------------------------------
void tick() {
  const uint32_t now_ms = millis();

  // Collect whatever the motor sent since the last tick before deciding
  // anything: late auxiliary replies land in the cache here rather than being
  // discarded by the feedback wait further down.
  g_can.pump();

  // The instantaneous word is rebuilt from nothing every tick. Anything that
  // belongs in it must be re-asserted below or it disappears, which is the
  // point - "regen is capping me" is only true while it is true.
  g_active = 0;

  Params P;
  portENTER_CRITICAL(&g_mux); P = g_par; portEXIT_CRITICAL(&g_mux);

  // --- one queued request per tick ---------------------------------------
  bool slot_used = false;
  CtrlRequest r;
  if (g_q && xQueueReceive(g_q, &r, 0) == pdTRUE) {
    slot_used = handleRequest(r, P);
    if (r.type == REQ_ZERO_SOFT || r.type == REQ_SET_TARGET_POS ||
        r.type == REQ_SET_MODE) {
      portENTER_CRITICAL(&g_mux); g_par.zero_offset = P.zero_offset; portEXIT_CRITICAL(&g_mux);
    }
  }

  // --- build the command --------------------------------------------------
  MotionCmd cmd{ g_pos, 0.0f, 0.0f, 0.0f, 0.0f };
  bool holding = false;   // parked on a position set-point, not moving

  switch (g_state) {
  case ST_INIT:
  case ST_IDLE:
  case ST_FAULT_HARD:
    cmd = MotionCmd{ g_pos, 0.0f, 0.0f, 0.0f, 0.0f };
    break;

  case ST_READY:
    cmd = MotionCmd{ g_pos, 0.0f, 0.0f, P.kd_torque, 0.0f };
    holding = true;
    break;

  case ST_RUN_POS: {
    g_traj.step(DT, REACH_BAND);
    holding = g_traj.done() && fabsf(g_tm.vel) < 0.3f;

    cmd.p_set = g_traj.pos();
    cmd.v_set = g_traj.vel();
    cmd.kp    = P.kp;
    cmd.kd    = P.kd;

    if (holding) {
      // No profile left to follow, so no inertia or friction feed-forward -
      // pushing J*a here is exactly what made the axis buzz at standstill.
      cmd.t_ff = 0.0f;
      // Fade the position gain out inside the dead band. A hard switch would
      // chatter on the boundary, so ramp from 0 at |err| = db to full at 2*db.
      const float db = P.pos_deadband * DEG;
      if (db > 1e-6f) {
        const float err = fabsf(g_traj.target() - g_pos);
        cmd.kp *= clampf((err - db) / db, 0.0f, 1.0f);
      }
    } else {
      cmd.t_ff = P.j_hat * g_traj.acc()
               + P.fric_c * sgnf(g_traj.vel()) * (fabsf(g_traj.vel()) > 0.05f ? 1.0f : 0.0f)
               + P.fric_v * g_traj.vel();
    }
    break;
  }

  case ST_RUN_TRQ: {
    // Kp = 0 so position never fights the torque command; a small Kd is kept
    // as a runaway brake (it costs Kd*|v| N.m of the commanded torque).
    const float tq = g_src ? g_src->torqueNm(now_ms) : 0.0f;
    cmd.p_set = g_pos;
    cmd.v_set = 0.0f;
    cmd.kp    = 0.0f;
    cmd.kd    = P.kd_torque;
    cmd.t_ff  = tq;
    break;
  }

  case ST_CALIB:
    g_cal.tick(DT, g_tm, g_calP, cmd);
    if (g_cal.finished()) {
      portENTER_CRITICAL(&g_mux); g_par = g_calP; portEXIT_CRITICAL(&g_mux);
      store::save(g_calP);
      doStop();
    } else if (g_cal.aborted()) {
      doStop();
    }
    break;

  case ST_FAULT_SOFT: {
    const uint32_t el = now_ms - g_soft_t0;
    const float k = 1.0f - clampf((float)el / (float)SOFT_RAMP_MS, 0.0f, 1.0f);
    cmd.p_set = g_pos;
    cmd.v_set = 0.0f;
    cmd.kp    = 0.0f;
    cmd.kd    = DEF_KD_DAMP_STOP;
    cmd.t_ff  = g_soft_t_ref * k;
    if (el > SOFT_RAMP_MS && fabsf(g_tm.vel) < 0.2f) doStop();
    break;
  }
  }

  applyLimits(P, cmd, holding);

  // --- translate to the motor frame and guard the +-4*pi window ----------
  const float p_set_user = cmd.p_set;
  cmd.p_set = clampf(cmd.p_set + P.zero_offset, -PSET_CLAMP, PSET_CLAMP);

  // --- CAN exchange -------------------------------------------------------
  Feedback fb;
  bool ok = false;
  if (!slot_used) {
    ok = g_can.sendMotion(cmd, fb);
    if (ok) {
      g_miss_run = 0;

      if (g_first_fb) { g_prev_raw = fb.pos; g_turns = 0; g_first_fb = false; }
      const float d = fb.pos - g_prev_raw;
      if      (d >  P_LIM) g_turns--;          // wrapped -4pi -> +4pi
      else if (d < -P_LIM) g_turns++;          // wrapped +4pi -> -4pi
      g_prev_raw   = fb.pos;
      g_pos_unwrap = fb.pos + (float)g_turns * (2.0f * P_LIM);
      g_pos        = g_pos_unwrap - P.zero_offset;

      g_tm.pos_raw    = fb.pos;
      g_tm.vel        = fb.vel;
      g_tm.torque     = fb.torque;
      g_tm.temp       = fb.temp;
      g_tm.fb_fault   = fb.fault;
      g_tm.mode_state = fb.mode_state;
    } else {
      g_miss_run++;
    }
  }

  const bool braking = (cmd.t_ff * g_tm.vel < 0.0f) && (fabsf(g_tm.vel) > 1.0f);
  // Runs on every tick now, including the ones a UI request consumed: sending
  // a request costs one frame and no wait, and harvesting replies must not
  // stall just because the operator touched the screen.
  auxPoll(P, braking);

  evaluateFaults(P, now_ms, cmd.t_ff);

  // --- publish ------------------------------------------------------------
  g_tm.pos        = g_pos;
  g_tm.p_cmd      = p_set_user;
  g_tm.v_cmd      = cmd.v_set;
  g_tm.a_cmd      = g_traj.acc();
  g_tm.t_cmd      = cmd.t_ff;
  g_tm.i_cmd      = (P.kt > 1e-3f) ? (cmd.t_ff / P.kt) : 0.0f;
  g_tm.iq         = g_iq;
  g_tm.vbus       = g_vbus;
  g_tm.aux_age_ms = g_vbus_age;
  g_tm.aux_ok     = g_vbus_ok;
  g_tm.vbus_trip  = g_vbus_trip;
  g_tm.state      = g_state;
  g_tm.mode       = g_mode;
  g_tm.enabled    = g_enabled;
  g_tm.events     = g_events;
  g_tm.active     = g_active;
  g_tm.fault_word = g_fault_word;
  g_tm.warn_word  = g_warn_word;
  g_tm.reach_ms   = g_traj.reachMs();
  g_tm.peak_vel   = g_traj.peakVel();
  g_tm.rtt_us     = g_can.lastRttUs();
  g_tm.tx         = g_can.txCount();
  g_tm.mtx        = g_can.motionTxCount();
  g_tm.rx         = g_can.rxCount();
  g_tm.miss       = g_can.missCount();
  g_can.busStats(g_tm.twai_state, g_tm.twai_tec, g_tm.twai_rec,
                 g_tm.twai_buserr, g_tm.twai_queued);
  g_tm.calib_step = g_cal.step();
  g_tm.calib_pct  = g_cal.pct();
  snprintf(g_tm.calib_msg, sizeof(g_tm.calib_msg), "%s", g_cal.msg());

  portENTER_CRITICAL(&g_mux); g_pub = g_tm; portEXIT_CRITICAL(&g_mux);

  // --- log ----------------------------------------------------------------
  LogSample s;
  s.t_us   = (uint32_t)esp_timer_get_time();
  s.p_cmd  = p_set_user;  s.p_act = g_pos;
  s.v_cmd  = cmd.v_set;   s.v_act = g_tm.vel;
  s.t_cmd  = cmd.t_ff;    s.t_act = g_tm.torque;
  s.iq     = g_iq;        s.vbus  = g_vbus;  s.temp = g_tm.temp;
  s.events = g_events;    s.active = g_active; s.state = g_state;
  logger::push(s);
}

void ctrlTask(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    tick();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CTRL_PERIOD_US / 1000));
  }
}

} // namespace

#ifdef CTRL_HOST_TEST
// Test-only hook. The host build does not spawn the control task, so the
// tests advance the loop one slot at a time through here. Never compiled into
// firmware - see test/host/.
extern "C" void ctrl_host_tick() { tick(); }
#endif

// ===========================================================================
namespace ctrl {

bool begin(CommandSource* src) {
  g_src = src;
  store::load(g_par);

  g_q = xQueueCreate(16, sizeof(CtrlRequest));
  if (!g_q) return false;

  if (!g_can.begin(g_par.motor_id, g_par.can_tx, g_par.can_rx, g_par.can_baud)) return false;

  // Start from a defined state rather than whatever the globals happen to
  // hold. Nothing should be carried in here, and the supply reference in
  // particular must be measured, never assumed.
  g_events = 0; g_active = 0; g_miss_run = 0; g_aux = 0;
  g_enabled = false; g_first_fb = true; g_turns = 0;
  g_pos = 0.0f; g_pos_unwrap = 0.0f; g_prev_raw = 0.0f;
  g_fault_word = 0; g_warn_word = 0;
  g_tm = Telemetry{}; g_pub = Telemetry{};
  resetVbusRef();

  Feedback fb;
  g_can.stop(false, fb);          // known state: disabled
  pushMotorConfig(g_par);
  g_state = ST_IDLE;

  g_traj.configure(g_par.lim_speed, g_par.lim_acc, g_par.lim_dec);
  g_traj.commitDecelCfg();

  return xTaskCreatePinnedToCore(ctrlTask, "rs06_ctrl", 8192, nullptr,
                                 CTRL_PRIO, nullptr, CTRL_CORE) == pdPASS;
}

void request(const CtrlRequest& r) {
  if (g_q) xQueueSend(g_q, &r, 0);
}

void snapshot(Telemetry& out) {
  portENTER_CRITICAL(&g_mux); out = g_pub; portEXIT_CRITICAL(&g_mux);
}

void getParams(Params& out) {
  portENTER_CRITICAL(&g_mux); out = g_par; portEXIT_CRITICAL(&g_mux);
}

void setParams(const Params& p) {
  portENTER_CRITICAL(&g_mux); g_par = p; portEXIT_CRITICAL(&g_mux);
  store::save(p);
  CtrlRequest r{ REQ_PARAMS_CHANGED, 0, 0.0f };
  request(r);
}

bool busy() {
  Telemetry t; snapshot(t);
  return t.enabled || t.state == ST_CALIB;
}

const char* stateName(uint8_t s) {
  switch (s) {
    case ST_INIT:       return "INIT";
    case ST_IDLE:       return "IDLE";
    case ST_READY:      return "READY";
    case ST_RUN_POS:    return "POS";
    case ST_RUN_TRQ:    return "TRQ";
    case ST_CALIB:      return "CALIB";
    case ST_FAULT_SOFT: return "SOFT-FLT";
    case ST_FAULT_HARD: return "HARD-FLT";
  }
  return "?";
}

} // namespace ctrl
