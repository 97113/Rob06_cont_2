#include "rs06_driver.h"
#include <esp_timer.h>

using namespace rs06;

bool Rs06::begin(uint8_t motor_id, uint8_t tx_gpio, uint8_t rx_gpio, uint32_t baud) {
  motor_id_ = motor_id;
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)tx_gpio, (gpio_num_t)rx_gpio, TWAI_MODE_NORMAL);
  g.tx_queue_len = 8;
  g.rx_queue_len = 48;
  g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  if      (baud <= 125000) { twai_timing_config_t x = TWAI_TIMING_CONFIG_125KBITS(); t = x; }
  else if (baud <= 250000) { twai_timing_config_t x = TWAI_TIMING_CONFIG_250KBITS(); t = x; }
  else if (baud <= 500000) { twai_timing_config_t x = TWAI_TIMING_CONFIG_500KBITS(); t = x; }
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  started_ = true;
  return true;
}

void Rs06::end() {
  if (!started_) return;
  twai_stop();
  twai_driver_uninstall();
  started_ = false;
}

bool Rs06::xmit(uint32_t id, const uint8_t d[8]) {
  twai_message_t m{};
  m.identifier       = id;
  m.extd             = 1;
  m.data_length_code = 8;
  memcpy(m.data, d, 8);
  if (twai_transmit(&m, 0) != ESP_OK) return false;
  tx_++;
  return true;
}

bool Rs06::waitFor(uint8_t type, uint32_t timeout_us, twai_message_t& out) {
  const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_us;
  twai_message_t m;
  do {
    if (twai_receive(&m, 0) == ESP_OK) {
      rx_++;
      const uint8_t ty = id_type(m.identifier);
      if (ty == CT_FEEDBACK) {
        // Latch telemetry regardless of what we were waiting for.
        if (parse_feedback(m.identifier, m.data, last_fb_)) fb_valid_ = true;
      }
      if (ty == type) { out = m; return true; }
    }
  } while (esp_timer_get_time() < deadline);
  miss_++;
  return false;
}

bool Rs06::sendMotion(const MotionCmd& c, Feedback& fb) {
  uint32_t id; uint8_t d[8];
  build_motion(c, motor_id_, id, d);
  const int64_t t0 = esp_timer_get_time();
  if (!xmit(id, d)) { miss_++; return false; }
  twai_message_t m;
  if (!waitFor(CT_FEEDBACK, RESP_TIMEOUT_US, m)) return false;
  rtt_us_ = (uint32_t)(esp_timer_get_time() - t0);
  fb = last_fb_;
  return true;
}

// Type3 / Type4 / Type6 / Type22 all answer with a Type2 feedback frame.
static inline void simple_frame(uint8_t type, uint8_t motor_id, uint8_t b0,
                                uint32_t& id, uint8_t d[8]) {
  id = make_id(type, (uint16_t)HOST_ID, motor_id);
  memset(d, 0, 8);
  d[0] = b0;
}

bool Rs06::enable(Feedback& fb) {
  uint32_t id; uint8_t d[8];
  simple_frame(CT_ENABLE, motor_id_, 0, id, d);
  if (!xmit(id, d)) return false;
  twai_message_t m;
  if (!waitFor(CT_FEEDBACK, RESP_TIMEOUT_US, m)) return false;
  fb = last_fb_;
  return true;
}

bool Rs06::stop(bool clear_fault, Feedback& fb) {
  uint32_t id; uint8_t d[8];
  simple_frame(CT_STOP, motor_id_, clear_fault ? 1 : 0, id, d);
  if (!xmit(id, d)) return false;
  twai_message_t m;
  if (!waitFor(CT_FEEDBACK, RESP_TIMEOUT_US, m)) return false;
  fb = last_fb_;
  return true;
}

bool Rs06::setZero(Feedback& fb) {
  uint32_t id; uint8_t d[8];
  simple_frame(CT_SET_ZERO, motor_id_, 1, id, d);
  if (!xmit(id, d)) return false;
  twai_message_t m;
  if (!waitFor(CT_FEEDBACK, RESP_TIMEOUT_US, m)) return false;
  fb = last_fb_;
  return true;
}

bool Rs06::save() {
  uint32_t id = make_id(CT_SAVE, (uint16_t)HOST_ID, motor_id_);
  uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  if (!xmit(id, d)) return false;
  twai_message_t m;
  return waitFor(CT_FEEDBACK, RESP_TIMEOUT_US, m);
}

// ID layout for Type7: data area 2 holds the host id in its low byte and the
// new (preset) CAN_ID in its high byte; the destination byte is the motor's
// CURRENT id. The reply is a Type0 broadcast frame.
bool Rs06::setCanId(uint8_t new_id) {
  uint32_t id = make_id(CT_SET_CANID,
                        (uint16_t)(((uint16_t)new_id << 8) | (uint16_t)HOST_ID),
                        motor_id_);
  uint8_t d[8]{};
  if (!xmit(id, d)) return false;
  twai_message_t m;
  if (!waitFor(CT_GET_ID, RESP_TIMEOUT_US, m)) return false;
  motor_id_ = new_id;
  return true;
}

bool Rs06::readParamRaw(uint16_t idx, uint32_t& raw) {
  uint32_t id = make_id(CT_PARAM_READ, (uint16_t)HOST_ID, motor_id_);
  uint8_t d[8]{};
  le_put16(&d[0], idx);            // index: little endian
  if (!xmit(id, d)) return false;
  twai_message_t m;
  if (!waitFor(CT_PARAM_READ, AUX_RESP_TIMEOUT_US, m)) return false;
  if (((uint16_t)m.data[0] | ((uint16_t)m.data[1] << 8)) != idx) return false;
  raw = le_get32(&m.data[4]);      // payload: little endian
  return true;
}

bool Rs06::readParamF(uint16_t idx, float& v) {
  uint32_t raw;
  if (!readParamRaw(idx, raw)) return false;
  memcpy(&v, &raw, 4);
  return true;
}

bool Rs06::paramWrite(uint16_t idx, const uint8_t data4[4], bool wait_reply) {
  uint32_t id = make_id(CT_PARAM_WRITE, (uint16_t)HOST_ID, motor_id_);
  uint8_t d[8]{};
  le_put16(&d[0], idx);
  memcpy(&d[4], data4, 4);
  if (!xmit(id, d)) return false;
  if (!wait_reply) return true;
  twai_message_t m;
  return waitFor(CT_FEEDBACK, AUX_RESP_TIMEOUT_US, m);
}

bool Rs06::writeParamF(uint16_t idx, float v, bool wait_reply) {
  uint8_t b[4]; le_put_f32(b, v);
  return paramWrite(idx, b, wait_reply);
}
bool Rs06::writeParamU8(uint16_t idx, uint8_t v, bool wait_reply) {
  uint8_t b[4]{}; b[0] = v;
  return paramWrite(idx, b, wait_reply);
}
bool Rs06::writeParamU16(uint16_t idx, uint16_t v, bool wait_reply) {
  uint8_t b[4]{}; le_put16(b, v);
  return paramWrite(idx, b, wait_reply);
}
bool Rs06::writeParamU32(uint16_t idx, uint32_t v, bool wait_reply) {
  uint8_t b[4]{}; le_put32(b, v);
  return paramWrite(idx, b, wait_reply);
}

bool Rs06::readFault(uint32_t& fault, uint32_t& warn) {
  uint32_t id = make_id(CT_FAULT_FB, (uint16_t)HOST_ID, motor_id_);
  uint8_t d[8]{};
  if (!xmit(id, d)) return false;
  twai_message_t m;
  if (!waitFor(CT_FAULT_FB, AUX_RESP_TIMEOUT_US, m)) return false;
  fault = le_get32(&m.data[0]);
  warn  = le_get32(&m.data[4]);
  return true;
}

bool Rs06::setActiveReport(bool on) {
  uint32_t id = make_id(CT_ACTIVE_REP, (uint16_t)HOST_ID, motor_id_);
  uint8_t d[8] = {1, 2, 3, 4, 5, 6, (uint8_t)(on ? 1 : 0), 0};
  if (!xmit(id, d)) return false;
  twai_message_t m;
  return waitFor(CT_ACTIVE_REP, AUX_RESP_TIMEOUT_US, m);
}

void Rs06::busStats(uint8_t& state, uint16_t& tec, uint16_t& rec,
                    uint32_t& bus_err, uint32_t& queued) {
  twai_status_info_t st{};
  if (twai_get_status_info(&st) != ESP_OK) { state = 0xFF; return; }
  state   = (uint8_t)st.state;
  tec     = (uint16_t)st.tx_error_counter;
  rec     = (uint16_t)st.rx_error_counter;
  bus_err = st.bus_error_count;
  queued  = st.msgs_to_tx;
}

bool Rs06::busHealthy() {
  uint32_t alerts = 0;
  twai_read_alerts(&alerts, 0);
  if (alerts & TWAI_ALERT_BUS_OFF) {
    twai_initiate_recovery();
    return false;
  }
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    if (st.state == TWAI_STATE_BUS_OFF)   { twai_initiate_recovery(); return false; }
    if (st.state == TWAI_STATE_STOPPED)   { twai_start(); return false; }
    if (st.state == TWAI_STATE_RECOVERING) return false;
  }
  return true;
}
