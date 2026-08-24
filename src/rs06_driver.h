// ---------------------------------------------------------------------------
// rs06_driver.h : TWAI transport for the RS06 private protocol.
// All methods are called from the 1 kHz control task only (single threaded).
//
// RECEIVE MODEL - read this before changing anything here.
//
// Every frame the motor sends is filed by pump(), which drains the RX queue
// and dispatches by communication type. Nothing is ever discarded because it
// "was not what we were waiting for":
//
//   Type2  (feedback)      -> last_fb_, and bumps the Type2 sequence number
//   Type17 (param read)    -> the parameter cache, keyed by index
//   Type21 (fault frame)   -> fault_/warn_
//   anything else          -> its type sequence number, for waitType()
//
// That matters because the motor's answer to an auxiliary read routinely
// arrives after the slot that asked for it has ended. The previous design
// waited a fixed 300 us inside the slot and let the next slot's feedback wait
// swallow and drop the late reply, so once a read slipped by one slot it
// never recovered: VBUS updated at 0.7 Hz instead of 200 Hz and the current
// reading never updated at all. Requests are now fire-and-forget
// (requestParam / requestFault) and the answer is picked up by paramValue() /
// faultWords() whenever it lands, with an age so the caller can tell fresh
// data from stale.
// ---------------------------------------------------------------------------
#pragma once
#include "rs06_proto.h"
#include "config.h"
#include <driver/twai.h>

class Rs06 {
public:
  bool begin(uint8_t motor_id, uint8_t tx_gpio, uint8_t rx_gpio, uint32_t baud);
  void end();
  void setMotorId(uint8_t id) { motor_id_ = id; }
  uint8_t motorId() const     { return motor_id_; }

  // --- receive -------------------------------------------------------------
  // Drains the RX queue and files everything it finds. Non-blocking and cheap;
  // call it at the top of every control tick and inside every wait loop.
  void pump();

  // --- one-shot exchanges (each waits for its reply inside the slot) -------
  bool sendMotion(const rs06::MotionCmd& c, rs06::Feedback& fb);
  bool enable(rs06::Feedback& fb);
  bool stop(bool clear_fault, rs06::Feedback& fb);
  bool setZero(rs06::Feedback& fb);
  bool save();
  // Type7. Rewrites the CAN_ID stored inside the motor and retargets this
  // driver at it. Only meaningful with a single motor on the bus.
  bool setCanId(uint8_t new_id);

  // --- asynchronous parameter access --------------------------------------
  // Send the request and return immediately. The reply is filed by pump().
  bool requestParam(uint16_t idx);
  bool requestFault();
  // Read what has been filed. age_ms is how long ago the value arrived, so a
  // caller that must not act on stale data can say so. False means the value
  // has never been seen since begin().
  bool paramValue(uint16_t idx, float& v, uint32_t& age_ms) const;
  bool faultWords(uint32_t& fault, uint32_t& warn, uint32_t& age_ms) const;

  // Blocking convenience wrapper, built on the same cache: sends the request
  // and pumps until THIS index answers or the deadline passes. Used for setup
  // reads where a round trip inside the call is acceptable.
  bool readParamF(uint16_t idx, float& v, uint32_t timeout_us = SYNC_RESP_TIMEOUT_US);

  bool writeParamF (uint16_t idx, float v,    bool wait_reply = true);
  bool writeParamU8(uint16_t idx, uint8_t v,  bool wait_reply = true);
  bool writeParamU16(uint16_t idx, uint16_t v, bool wait_reply = true);
  bool writeParamU32(uint16_t idx, uint32_t v, bool wait_reply = true);
  // Type24. Active reporting is left OFF: its floor is 10 ms (EPScan_time),
  // far too coarse for a 100 ms move, and the Type1 exchange already returns
  // a Type2 every millisecond.
  bool setActiveReport(bool on);

  // --- health -------------------------------------------------------------
  bool     busHealthy();          // clears bus-off / recovers if needed
  // Raw controller counters. Reading these is how you tell "transmitting but
  // nobody answers" (bus_err / tec climbing) from "never got to transmit"
  // (queue backing up with tec == 0, i.e. the bus looks permanently busy).
  void     busStats(uint8_t& state, uint16_t& tec, uint16_t& rec,
                    uint32_t& bus_err, uint32_t& queued);
  uint32_t txCount()      const { return tx_; }
  uint32_t motionTxCount()const { return mtx_; }   // Type1 only, for drop rate
  uint32_t rxCount()      const { return rx_; }
  uint32_t missCount()    const { return miss_; }
  uint32_t lastRttUs()    const { return rtt_us_; }
  const rs06::Feedback& lastFeedback() const { return last_fb_; }
  bool     lastFeedbackValid() const { return fb_valid_; }
  void     resetStats() { tx_ = mtx_ = rx_ = miss_ = 0; }
  // Forgets every cached reply. Call after re-installing the driver so a
  // value from the previous wiring cannot be mistaken for a live one.
  void     resetCache();

private:
  static constexpr int N_PCACHE = 8;   // distinct parameter indices cached

  struct PSlot {
    uint16_t idx  = 0;
    uint32_t raw  = 0;
    uint32_t t_ms = 0;
    uint16_t seq  = 0;      // bumped on every reply filed here
    bool     used = false;  // index claimed
    bool     seen = false;  // a reply has actually landed
  };

  bool   xmit(uint32_t id, const uint8_t d[8]);
  // Pumps until the sequence counter for `type` moves, i.e. until a NEW frame
  // of that type arrives. Returns false (and counts a miss) on timeout.
  bool   waitType(uint8_t type, uint32_t timeout_us);
  bool   paramWrite(uint16_t idx, const uint8_t data4[4], bool wait_reply);
  PSlot* pslot(uint16_t idx);              // find, or claim a free/oldest slot
  const PSlot* pslotConst(uint16_t idx) const;
  static uint32_t nowMs();

  uint8_t        motor_id_  = DEFAULT_MOTOR_ID;
  bool           started_   = false;
  uint32_t       tx_ = 0, mtx_ = 0, rx_ = 0, miss_ = 0, rtt_us_ = 0;
  rs06::Feedback last_fb_{};
  bool           fb_valid_  = false;

  // One counter per communication type (5 bits of the ID), bumped by pump().
  // A wait is "note the counter, pump, wait for it to change", which cannot
  // be satisfied by a frame that was already sitting in the queue.
  uint16_t       type_seq_[32] = {0};

  PSlot          pc_[N_PCACHE];
  uint32_t       fault_ = 0, warn_ = 0, fault_t_ms_ = 0;
  bool           fault_seen_ = false;
};
