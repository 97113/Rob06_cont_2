// ---------------------------------------------------------------------------
// traj.h : online trapezoidal profile generator.
//
// State-feedback form (not time-parameterised) so that vmax / amax / dmax can
// be changed *during* a move - which the regen power limiter and the thermal
// derate both need to do.
// ---------------------------------------------------------------------------
#pragma once
#include <math.h>
#include <stdint.h>

class Trajectory {
public:
  // Settle thresholds. POS_EPS is well below the 0.022 deg quantisation of the
  // Type1 position field, so snapping here costs nothing in accuracy.
  static constexpr float POS_EPS = 1e-4f;    // rad
  static constexpr float VEL_EPS = 1e-3f;    // rad/s

  void configure(float vmax, float amax, float dmax) {
    vmax_ = fabsf(vmax); amax_ = fabsf(amax); dmax_ = fabsf(dmax);
    if (vmax_ < 1e-3f) vmax_ = 1e-3f;
    if (amax_ < 1e-3f) amax_ = 1e-3f;
    if (dmax_ < 1e-3f) dmax_ = 1e-3f;
  }
  // Momentary override of the deceleration cap (regen limiter). Never raises
  // it above the configured value.
  void setDecelCap(float dmax) { dmax_ = fmaxf(1e-3f, fminf(dmax, dmax_cfg_)); }
  void commitDecelCfg()        { dmax_cfg_ = dmax_; }

  void reset(float pos, float vel = 0.0f) {
    p_ = pos; v_ = vel; a_ = 0.0f; target_ = pos;
    done_ = true; reached_ = true; t_ms_ = 0; reach_ms_ = 0; vpeak_ = 0.0f;
  }
  void moveTo(float target, float pos_now, float vel_now) {
    target_ = target; p_ = pos_now; v_ = vel_now; a_ = 0.0f;
    done_ = false; reached_ = false; t_ms_ = 0; reach_ms_ = 0; vpeak_ = 0.0f;
  }

  // dt in seconds. reach_band: |err| considered "first reached".
  void step(float dt, float reach_band) {
    // Once the move is over the generator must FREEZE. Integrating on at the
    // set-point makes |d| <= stop trivially true (both are zero), which asks
    // for full deceleration, flips the sign of v on the next tick, and limit
    // cycles at the tick rate - the feed-forward term J*a then swings between
    // +-dmax*J and the axis buzzes at standstill.
    if (done_) { p_ = target_; v_ = 0.0f; a_ = 0.0f; return; }
    t_ms_++;

    // Settled: snap and latch rather than hunting around the set-point.
    if (fabsf(target_ - p_) <= POS_EPS && fabsf(v_) <= VEL_EPS) {
      p_ = target_; v_ = 0.0f; a_ = 0.0f; done_ = true;
      if (!reached_) { reached_ = true; reach_ms_ = t_ms_; }
      return;
    }

    const float d    = target_ - p_;
    const float dirn = (d >= 0.0f) ? 1.0f : -1.0f;
    const bool  toward = (v_ * dirn) >= 0.0f;
    const float stop = (v_ * v_) / (2.0f * dmax_);

    if (!toward)                        a_ =  dirn * amax_;   // kill wrong-way motion
    else if (fabsf(d) <= stop)          a_ = -dirn * dmax_;   // brake
    else if (fabsf(v_) < vmax_)         a_ =  dirn * amax_;   // accelerate
    else                                a_ =  0.0f;           // cruise

    v_ += a_ * dt;
    if (v_ >  vmax_) { v_ =  vmax_; a_ = 0.0f; }
    if (v_ < -vmax_) { v_ = -vmax_; a_ = 0.0f; }

    const float p_new = p_ + v_ * dt;
    // Do not step past the target inside one tick.
    if ((target_ - p_new) * dirn < 0.0f) { p_ = target_; v_ = 0.0f; a_ = 0.0f; }
    else                                  p_ = p_new;

    if (fabsf(v_) > vpeak_) vpeak_ = fabsf(v_);
    if (!reached_ && fabsf(target_ - p_) <= reach_band) { reached_ = true; reach_ms_ = t_ms_; }
    if (fabsf(target_ - p_) <= POS_EPS && fabsf(v_) <= VEL_EPS) {
      p_ = target_; v_ = 0.0f; a_ = 0.0f; done_ = true;
    }
  }

  float pos()    const { return p_; }
  float vel()    const { return v_; }
  float acc()    const { return a_; }
  float target() const { return target_; }
  bool  done()   const { return done_; }
  bool  reached()const { return reached_; }
  uint32_t elapsedMs()   const { return t_ms_; }
  uint32_t reachMs()     const { return reach_ms_; }
  float    peakVel()     const { return vpeak_; }
  float    vmax()        const { return vmax_; }

private:
  float p_ = 0, v_ = 0, a_ = 0, target_ = 0;
  float vmax_ = 1, amax_ = 1, dmax_ = 1, dmax_cfg_ = 1;
  bool  done_ = true, reached_ = true;
  uint32_t t_ms_ = 0, reach_ms_ = 0;
  float vpeak_ = 0;
};
