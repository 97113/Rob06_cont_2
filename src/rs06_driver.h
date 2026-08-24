// ---------------------------------------------------------------------------
// rs06_driver.h : TWAI transport for the RS06 private protocol.
// All methods are called from the 1 kHz control task only (single threaded).
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

  // --- one-shot exchanges (each waits for its reply inside the slot) -------
  bool sendMotion(const rs06::MotionCmd& c, rs06::Feedback& fb);
  bool enable(rs06::Feedback& fb);
  bool stop(bool clear_fault, rs06::Feedback& fb);
  bool setZero(rs06::Feedback& fb);
  bool save();
  // Type7. Rewrites the CAN_ID stored inside the motor and retargets this
  // driver at it. Only meaningful with a single motor on the bus.
  bool setCanId(uint8_t new_id);

  bool readParamRaw(uint16_t idx, uint32_t& raw);
  bool readParamF (uint16_t idx, float& v);
  bool writeParamF (uint16_t idx, float v,    bool wait_reply = true);
  bool writeParamU8(uint16_t idx, uint8_t v,  bool wait_reply = true);
  bool writeParamU16(uint16_t idx, uint16_t v, bool wait_reply = true);
  bool writeParamU32(uint16_t idx, uint32_t v, bool wait_reply = true);
  bool readFault(uint32_t& fault, uint32_t& warn);
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
  uint32_t txCount()   const { return tx_; }
  uint32_t rxCount()   const { return rx_; }
  uint32_t missCount() const { return miss_; }
  uint32_t lastRttUs() const { return rtt_us_; }
  const rs06::Feedback& lastFeedback() const { return last_fb_; }
  bool     lastFeedbackValid() const { return fb_valid_; }
  void     resetStats() { tx_ = rx_ = miss_ = 0; }

private:
  bool xmit(uint32_t id, const uint8_t d[8]);
  // Busy-polls the RX queue until a frame of `type` arrives or the deadline
  // expires. Any Type2 seen along the way is latched as telemetry.
  bool waitFor(uint8_t type, uint32_t timeout_us, twai_message_t& out);
  bool paramWrite(uint16_t idx, const uint8_t data4[4], bool wait_reply);

  uint8_t        motor_id_  = DEFAULT_MOTOR_ID;
  bool           started_   = false;
  uint32_t       tx_ = 0, rx_ = 0, miss_ = 0, rtt_us_ = 0;
  rs06::Feedback last_fb_{};
  bool           fb_valid_  = false;
};
