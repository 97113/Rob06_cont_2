#include "calib.h"
#include <string.h>
#include <stdio.h>

static constexpr float DEG          = 0.017453293f;
static constexpr float TARGET_200DEG = 200.0f * DEG;   // 3.4907 rad
static constexpr float STEP_30DEG    = 30.0f  * DEG;
static constexpr float SETTLE_BAND   = 0.5f   * DEG;
static constexpr float REACH_BAND    = 5.0f   * DEG;   // the +-5 deg spec band

void Calibration::enter(Step s, const char* m) {
  step_ = s;
  t_ms_ = 0;
  snprintf(msg_, sizeof(msg_), "%s", m);
}

void Calibration::hold(rs06::MotionCmd& cmd, float pos) {
  cmd.p_set = pos; cmd.v_set = 0.0f; cmd.kp = 20.0f; cmd.kd = 2.0f; cmd.t_ff = 0.0f;
}

void Calibration::start(const Params& p, float pos_now) {
  p0_ = pos_now;
  fi_ = 0; fsum_ = 0; fn_ = 0;
  ii_ = 0; j_acc_ = 0; j_n_ = 0;
  gi_ = 0; best_cost_ = 1e9f; best_kp_ = p.kp; best_kd_ = p.kd;
  vbus_min_ = 1e9f; vpeak_ = 0; apeak_ = 0; vprev_ = 0;
  rtt_sum_ = 0; rtt_n_ = 0;
  pct_ = 0;
  enter(CS_SETTLE, "settling");
}

void Calibration::tick(float dt, const Telemetry& tm, Params& p, rs06::MotionCmd& cmd) {
  t_ms_++;
  cmd = rs06::MotionCmd{ tm.pos, 0.0f, 0.0f, 1.0f, 0.0f };   // safe default

  switch (step_) {

  // -----------------------------------------------------------------------
  case CS_SETTLE:
    pct_ = 2;
    hold(cmd, p0_);
    if (t_ms_ > 300 && fabsf(tm.vel) < 0.05f) {
      p0_ = tm.pos;
      enter(CS_FRICTION, "friction 1/8");
    }
    break;

  // --- constant velocity plateaus, average the reported torque ------------
  case CS_FRICTION: {
    pct_ = 2 + (uint8_t)(18 * fi_ / N_FRIC);
    const float v = FRIC_V[fi_];
    cmd.p_set = tm.pos; cmd.v_set = v; cmd.kp = 0.0f; cmd.kd = 2.0f; cmd.t_ff = 0.0f;

    if (t_ms_ > 200) { fsum_ += tm.torque; fn_++; }     // last 200 ms of 400
    if (t_ms_ >= 400 || fabsf(tm.pos - p0_) > 8.0f) {
      fv_[fi_] = v;
      ft_[fi_] = (fn_ > 0) ? (fsum_ / fn_) : 0.0f;
      fsum_ = 0; fn_ = 0;
      fi_++;
      if (fi_ >= N_FRIC) {
        // Least squares fit of  tau = Fc*sign(v) + Fv*v
        float s11 = 0, s12 = 0, s22 = 0, b1 = 0, b2 = 0;
        for (int i = 0; i < N_FRIC; i++) {
          const float x1 = (fv_[i] >= 0) ? 1.0f : -1.0f;
          const float x2 = fv_[i];
          s11 += x1 * x1; s12 += x1 * x2; s22 += x2 * x2;
          b1  += x1 * ft_[i]; b2 += x2 * ft_[i];
        }
        const float det = s11 * s22 - s12 * s12;
        if (fabsf(det) > 1e-6f) {
          p.fric_c = fabsf(( b1 * s22 - b2 * s12) / det);
          p.fric_v = fabsf((-b1 * s12 + b2 * s11) / det);
        }
        if (p.fric_c > 5.0f) p.fric_c = 5.0f;      // sanity clamps
        if (p.fric_v > 1.0f) p.fric_v = 1.0f;
        enter(CS_INERTIA, "inertia 1/2");
      } else {
        char m[32]; snprintf(m, sizeof(m), "friction %d/%d", fi_ + 1, N_FRIC);
        enter(CS_FRICTION, m);
      }
    }
    break;
  }

  // --- open loop torque pulse, read back the acceleration -----------------
  case CS_INERTIA: {
    pct_ = 20 + (uint8_t)(15 * ii_ / 2);
    const float sgn = (ii_ == 0) ? 1.0f : -1.0f;

    if (t_ms_ <= 60) {                       // the pulse itself
      cmd.p_set = tm.pos; cmd.v_set = 0.0f; cmd.kp = 0.0f; cmd.kd = 0.0f;
      cmd.t_ff  = sgn * ipulse_;
      if (t_ms_ == 20) iv0_ = tm.vel;        // skip the current rise
      if (t_ms_ == 60) iv1_ = tm.vel;
    } else if (t_ms_ <= 400) {               // damped recovery to the start
      cmd.p_set = p0_; cmd.v_set = 0.0f; cmd.kp = 20.0f; cmd.kd = 3.0f; cmd.t_ff = 0.0f;
    } else {
      const float a = (iv1_ - iv0_) / 0.040f;
      if (fabsf(a) > 1.0f) {
        const float vmid = 0.5f * (iv0_ + iv1_);
        const float tf   = p.fric_c * sgn + p.fric_v * vmid;
        const float j    = (sgn * ipulse_ - tf) / a;
        if (j > 1e-4f && j < 1.0f) { j_acc_ += j; j_n_++; }
      }
      ii_++;
      if (ii_ >= 2) {
        if (j_n_ > 0) p.j_hat = j_acc_ / j_n_;
        traj_.configure(p.lim_speed, p.lim_acc, p.lim_dec);
        traj_.commitDecelCfg();
        traj_.moveTo(p0_ + TARGET_200DEG, tm.pos, tm.vel);
        vbus_min_ = 1e9f; vpeak_ = 0; apeak_ = 0; vprev_ = tm.vel;
        enter(CS_CAPABILITY, "capability");
      } else {
        enter(CS_INERTIA, "inertia 2/2");
      }
    }
    break;
  }

  // --- full effort 200 deg: what can the 48 V rail actually deliver -------
  case CS_CAPABILITY: {
    pct_ = 35 + (uint8_t)((t_ms_ < 1200) ? (10 * t_ms_ / 1200) : 10);
    traj_.step(dt, REACH_BAND);
    cmd.p_set = traj_.pos();
    cmd.v_set = traj_.vel();
    cmd.kp    = p.kp;
    cmd.kd    = p.kd;
    cmd.t_ff  = p.j_hat * traj_.acc()
              + p.fric_c * ((traj_.vel() >= 0) ? 1.0f : -1.0f)
              + p.fric_v * traj_.vel();

    if (tm.vbus > 1.0f && tm.vbus < vbus_min_) vbus_min_ = tm.vbus;
    const float av = fabsf(tm.vel);
    if (av > vpeak_) vpeak_ = av;
    const float acc = fabsf(tm.vel - vprev_) / dt;
    if (acc < 5000.0f && acc > apeak_) apeak_ = acc;
    vprev_ = tm.vel;

    if (t_ms_ >= 1500) {
      p.meas_vmax     = vpeak_;
      p.meas_amax     = apeak_;
      p.meas_reach_ms = traj_.reachMs();
      p.meas_vbus_ref = tm.vbus;
      p.meas_vbus_sag = (vbus_min_ < 1e8f) ? (tm.vbus - vbus_min_) : 0.0f;
      traj_.moveTo(p0_, tm.pos, tm.vel);
      gi_ = -1;                              // -1 == "drive back to start" phase
      enter(CS_GAINS, "gains 1/16");
    }
    break;
  }

  // --- grid search over Kp/Kd using a 30 deg step -------------------------
  case CS_GAINS: {
    pct_ = 45 + (uint8_t)(45 * (gi_ < 0 ? 0 : gi_) / (N_KP * N_KD));

    if (gi_ < 0) {                                   // returning to start
      traj_.step(dt, REACH_BAND);
      cmd.p_set = traj_.pos(); cmd.v_set = traj_.vel();
      cmd.kp = p.kp; cmd.kd = p.kd;
      cmd.t_ff = p.j_hat * traj_.acc();
      if (traj_.done() && fabsf(tm.vel) < 0.05f) {
        gi_ = 0; p0_ = tm.pos;
        traj_.moveTo(p0_ + STEP_30DEG, tm.pos, tm.vel);
        trial_start_ = tm.pos; overshoot_ = 0; settled_ = false; settle_ms_ = 0;
        t_ms_ = 0;
      }
      break;
    }

    const float kp  = KP_SET[gi_ / N_KD];
    const float kd  = KD_SET[gi_ % N_KD];
    const float tgt = traj_.target();

    traj_.step(dt, REACH_BAND);
    cmd.p_set = traj_.pos(); cmd.v_set = traj_.vel();
    cmd.kp = kp; cmd.kd = kd;
    cmd.t_ff = p.j_hat * traj_.acc()
             + p.fric_c * ((traj_.vel() >= 0) ? 1.0f : -1.0f)
             + p.fric_v * traj_.vel();

    const float err = tgt - tm.pos;
    const float dir_sign = (tgt >= trial_start_) ? 1.0f : -1.0f;
    const float ovr = -err * dir_sign;               // positive == past target
    if (ovr > overshoot_) overshoot_ = ovr;
    if (!settled_ && fabsf(err) <= SETTLE_BAND && fabsf(tm.vel) < 0.2f) {
      settled_ = true; settle_ms_ = t_ms_;
    }

    if (t_ms_ >= 600) {
      const float cost = (settled_ ? (float)settle_ms_ : 900.0f)
                       + 20.0f * (overshoot_ / DEG);
      if (cost < best_cost_) { best_cost_ = cost; best_kp_ = kp; best_kd_ = kd; }

      gi_++;
      if (gi_ >= N_KP * N_KD) {
        p.kp = best_kp_; p.kd = best_kd_;
        comm_tx0_ = tm.tx; comm_miss0_ = tm.miss; rtt_sum_ = 0; rtt_n_ = 0;
        traj_.moveTo(p0_, tm.pos, tm.vel);
        enter(CS_COMM, "link quality");
      } else {
        // Alternate direction so the axis does not walk away from p0_.
        const float dir = (gi_ % 2 == 0) ? 1.0f : -1.0f;
        trial_start_ = tm.pos;
        traj_.moveTo(tm.pos + dir * STEP_30DEG, tm.pos, tm.vel);
        overshoot_ = 0; settled_ = false; settle_ms_ = 0;
        char m[32]; snprintf(m, sizeof(m), "gains %d/%d", gi_ + 1, N_KP * N_KD);
        enter(CS_GAINS, m);
      }
    }
    break;
  }

  // --- 1000 ticks of plain holding: round trip time and drop rate ---------
  case CS_COMM: {
    pct_ = 92;
    traj_.step(dt, REACH_BAND);
    cmd.p_set = traj_.pos(); cmd.v_set = traj_.vel();
    cmd.kp = p.kp; cmd.kd = p.kd;
    cmd.t_ff = p.j_hat * traj_.acc();

    if (tm.rtt_us > 0) { rtt_sum_ += tm.rtt_us; rtt_n_++; }

    if (t_ms_ >= 1000) {
      const uint32_t dtx   = tm.tx   - comm_tx0_;
      const uint32_t dmiss = tm.miss - comm_miss0_;
      p.meas_rtt_us   = (rtt_n_ > 0) ? (float)(rtt_sum_ / rtt_n_) : 0.0f;
      p.meas_drop_pct = (dtx > 0) ? (100.0f * dmiss / dtx) : 0.0f;
      p.calibrated    = true;
      pct_ = 100;
      enter(CS_DONE, "done");
    }
    break;
  }

  default:
    hold(cmd, tm.pos);
    break;
  }
}
