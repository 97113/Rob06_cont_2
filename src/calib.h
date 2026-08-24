// ---------------------------------------------------------------------------
// calib.h : identification + capability measurement + gain search.
//
// Runs inside the 1 kHz control task as a state machine; every tick it hands
// back a MotionCmd which the controller still passes through the full limiter
// chain. Abortable at any point (STOP button).
//
// Steps
//   1 ZERO       mechanical zero (done by the controller) + software offset
//   2 FRICTION   velocity sweep      -> fric_c, fric_v
//   3 INERTIA    torque pulse        -> j_hat
//   4 CAPABILITY full-effort 200 deg -> meas_vmax, meas_amax, meas_reach_ms,
//                                       meas_vbus_sag
//   5 GAINS      30 deg step search  -> kp, kd
//   6 COMM       link quality        -> meas_rtt_us, meas_drop_pct
// ---------------------------------------------------------------------------
#pragma once
#include "types.h"
#include "traj.h"
#include "rs06_proto.h"

class Calibration {
public:
  enum Step : uint8_t {
    CS_IDLE = 0, CS_SETTLE, CS_FRICTION, CS_INERTIA,
    CS_CAPABILITY, CS_GAINS, CS_COMM, CS_DONE, CS_ABORT
  };

  void start(const Params& p, float pos_now);
  void abort() { step_ = CS_ABORT; }

  bool active()   const { return step_ != CS_IDLE && step_ != CS_DONE && step_ != CS_ABORT; }
  bool finished() const { return step_ == CS_DONE; }
  bool aborted()  const { return step_ == CS_ABORT; }

  // dt seconds. Fills cmd. `p` is updated in place as results land.
  void tick(float dt, const Telemetry& tm, Params& p, rs06::MotionCmd& cmd);

  uint8_t     step() const { return step_; }
  uint8_t     pct()  const { return pct_; }
  const char* msg()  const { return msg_; }

private:
  void enter(Step s, const char* m);
  void hold(rs06::MotionCmd& cmd, float pos);

  Step     step_    = CS_IDLE;
  uint32_t t_ms_    = 0;      // ms inside the current step
  uint8_t  pct_     = 0;
  char     msg_[32] = {0};
  float    p0_      = 0.0f;   // reference position at start of calibration

  // --- friction sweep -----------------------------------------------------
  static constexpr int   N_FRIC = 8;
  static constexpr float FRIC_V[N_FRIC] = { 0.5f, 1.0f, 2.0f, 4.0f, -0.5f, -1.0f, -2.0f, -4.0f };
  int    fi_ = 0;
  float  fsum_ = 0.0f; int fn_ = 0;
  float  fv_[N_FRIC] = {0}, ft_[N_FRIC] = {0};

  // --- inertia pulse ------------------------------------------------------
  int    ii_ = 0;                 // 0 = forward pulse, 1 = reverse pulse
  float  ipulse_ = 4.0f;          // N.m
  float  iv0_ = 0.0f, iv1_ = 0.0f;
  float  j_acc_ = 0.0f; int j_n_ = 0;

  // --- capability / gain trials ------------------------------------------
  Trajectory traj_;
  float    trial_start_ = 0.0f;
  float    overshoot_   = 0.0f;
  uint32_t settle_ms_   = 0;
  bool     settled_     = false;
  float    vbus_min_    = 1e9f;
  float    vpeak_       = 0.0f;
  float    apeak_       = 0.0f;
  float    vprev_       = 0.0f;

  int      gi_ = 0;               // gain trial index
  float    best_cost_ = 1e9f, best_kp_ = 0, best_kd_ = 0;
  static constexpr int   N_KP = 4, N_KD = 4;
  static constexpr float KP_SET[N_KP] = { 15.0f, 30.0f, 60.0f, 120.0f };
  static constexpr float KD_SET[N_KD] = { 0.5f, 1.0f, 2.0f, 4.0f };

  // --- comm ---------------------------------------------------------------
  uint32_t comm_tx0_ = 0, comm_miss0_ = 0;
  double   rtt_sum_  = 0.0; uint32_t rtt_n_ = 0;
};
