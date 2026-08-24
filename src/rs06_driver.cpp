#include "rs06_driver.h"
#include <esp_timer.h>

using namespace rs06;

uint32_t Rs06::nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

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
  resetCache();
  return true;
}

void Rs06::end() {
  if (!started_) return;
  twai_stop();
  twai_driver_uninstall();
  started_ = false;
}

void Rs06::resetCache() {
  for (int i = 0; i < N_PCACHE; i++) pc_[i] = PSlot{};
  fault_ = warn_ = 0;
  fault_seen_ = false;
  fb_valid_   = false;
}

// ---------------------------------------------------------------------------
// Slot lookup. The table is tiny (two indices in normal operation), so a
// linear scan beats anything cleverer. A full table recycles the least
// recently updated entry rather than failing.
// ---------------------------------------------------------------------------
Rs06::PSlot* Rs06::pslot(uint16_t idx) {
  for (int i = 0; i < N_PCACHE; i++)
    if (pc_[i].used && pc_[i].idx == idx) return &pc_[i];
  for (int i = 0; i < N_PCACHE; i++)
    if (!pc_[i].used) { pc_[i].used = true; pc_[i].idx = idx; return &pc_[i]; }
  int oldest = 0;
  for (int i = 1; i < N_PCACHE; i++)
    if ((int32_t)(pc_[i].t_ms - pc_[oldest].t_ms) < 0) oldest = i;
  pc_[oldest] = PSlot{};
  pc_[oldest].used = true;
  pc_[oldest].idx  = idx;
  return &pc_[oldest];
}

const Rs06::PSlot* Rs06::pslotConst(uint16_t idx) const {
  for (int i = 0; i < N_PCACHE; i++)
    if (pc_[i].used && pc_[i].seen && pc_[i].idx == idx) return &pc_[i];
  return nullptr;
}

// ---------------------------------------------------------------------------
// The one place frames are taken off the queue. Every reply the motor sends
// is filed here, whatever the caller happens to be waiting for.
// ---------------------------------------------------------------------------
void Rs06::pump() {
  twai_message_t m;
  // Bounded so a flooded bus cannot stall the control tick inside this loop.
  for (int guard = 0; guard < 48; guard++) {
    if (twai_receive(&m, 0) != ESP_OK) return;
    rx_++;
    const uint8_t ty = id_type(m.identifier);

    switch (ty) {
    case CT_FEEDBACK:
      if (parse_feedback(m.identifier, m.data, last_fb_)) fb_valid_ = true;
      break;

    case CT_PARAM_READ: {
      const uint16_t idx = (uint16_t)m.data[0] | ((uint16_t)m.data[1] << 8);
      PSlot* s = pslot(idx);
      s->raw  = le_get32(&m.data[4]);
      s->t_ms = nowMs();
      s->seq++;
      s->seen = true;
      break;
    }

    case CT_FAULT_FB:
      fault_      = le_get32(&m.data[0]);
      warn_       = le_get32(&m.data[4]);
      fault_t_ms_ = nowMs();
      fault_seen_ = true;
      break;

    default:
      break;
    }
    type_seq_[ty & 31]++;
  }
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

bool Rs06::waitType(uint8_t type, uint32_t timeout_us) {
  const uint8_t  k  = type & 31;
  const uint16_t s0 = type_seq_[k];
  const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_us;
  do {
    pump();
    if (type_seq_[k] != s0) return true;
  } while (esp_timer_get_time() < deadline);
  miss_++;
  return false;
}

// ---------------------------------------------------------------------------
bool Rs06::sendMotion(const MotionCmd& c, Feedback& fb) {
  uint32_t id; uint8_t d[8];
  build_motion(c, motor_id_, id, d);
  // Clear anything already queued first, so the wait below cannot be satisfied
  // by the previous tick's feedback and report a stale position as fresh.
  pump();
  const int64_t t0 = esp_timer_get_time();
  if (!xmit(id, d)) { miss_++; return false; }
  mtx_++;
  if (!waitType(CT_FEEDBACK, RESP_TIMEOUT_US)) return false;
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
  pump();
  if (!xmit(id, d)) return false;
  if (!waitType(CT_FEEDBACK, RESP_TIMEOUT_US)) return false;
  fb = last_fb_;
  return true;
}

bool Rs06::stop(bool clear_fault, Feedback& fb) {
  uint32_t id; uint8_t d[8];
  simple_frame(CT_STOP, motor_id_, clear_fault ? 1 : 0, id, d);
  pump();
  if (!xmit(id, d)) return false;
  if (!waitType(CT_FEEDBACK, RESP_TIMEOUT_US)) return false;
  fb = last_fb_;
  return true;
}

bool Rs06::setZero(Feedback& fb) {
  uint32_t id; uint8_t d[8];
  simple_frame(CT_SET_ZERO, motor_id_, 1, id, d);
  pump();
  if (!xmit(id, d)) return false;
  if (!waitType(CT_FEEDBACK, RESP_TIMEOUT_US)) return false;
  fb = last_fb_;
  return true;
}

bool Rs06::save() {
  uint32_t id = make_id(CT_SAVE, (uint16_t)HOST_ID, motor_id_);
  uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  pump();
  if (!xmit(id, d)) return false;
  return waitType(CT_FEEDBACK, RESP_TIMEOUT_US);
}

// ID layout for Type7: data area 2 holds the host id in its low byte and the
// new (preset) CAN_ID in its high byte; the destination byte is the motor's
// CURRENT id. The reply is a Type0 broadcast frame.
bool Rs06::setCanId(uint8_t new_id) {
  uint32_t id = make_id(CT_SET_CANID,
                        (uint16_t)(((uint16_t)new_id << 8) | (uint16_t)HOST_ID),
                        motor_id_);
  uint8_t d[8]{};
  pump();
  if (!xmit(id, d)) return false;
  if (!waitType(CT_GET_ID, RESP_TIMEOUT_US)) return false;
  motor_id_ = new_id;
  return true;
}

// ---------------------------------------------------------------------------
// Asynchronous parameter access
// ---------------------------------------------------------------------------
bool Rs06::requestParam(uint16_t idx) {
  uint32_t id = make_id(CT_PARAM_READ, (uint16_t)HOST_ID, motor_id_);
  uint8_t d[8]{};
  le_put16(&d[0], idx);            // index: little endian
  return xmit(id, d);
}

bool Rs06::requestFault() {
  uint32_t id = make_id(CT_FAULT_FB, (uint16_t)HOST_ID, motor_id_);
  uint8_t d[8]{};
  return xmit(id, d);
}

bool Rs06::paramValue(uint16_t idx, float& v, uint32_t& age_ms) const {
  const PSlot* s = pslotConst(idx);
  if (!s) return false;
  memcpy(&v, &s->raw, 4);          // payload: little endian, already unpacked
  age_ms = nowMs() - s->t_ms;
  return true;
}

bool Rs06::faultWords(uint32_t& fault, uint32_t& warn, uint32_t& age_ms) const {
  if (!fault_seen_) return false;
  fault  = fault_;
  warn   = warn_;
  age_ms = nowMs() - fault_t_ms_;
  return true;
}

bool Rs06::readParamF(uint16_t idx, float& v, uint32_t timeout_us) {
  PSlot* s = pslot(idx);
  const uint16_t seq0 = s->seq;
  pump();
  if (!requestParam(idx)) return false;
  const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_us;
  do {
    pump();
    // Only this index counts, and only a reply that arrived AFTER the request.
    // The old code accepted the first Type17 frame it saw and then failed the
    // index check, which is why a reply meant for a different index silently
    // killed the read.
    if (s->seq != seq0) { memcpy(&v, &s->raw, 4); return true; }
  } while (esp_timer_get_time() < deadline);
  miss_++;
  return false;
}

bool Rs06::paramWrite(uint16_t idx, const uint8_t data4[4], bool wait_reply) {
  uint32_t id = make_id(CT_PARAM_WRITE, (uint16_t)HOST_ID, motor_id_);
  uint8_t d[8]{};
  le_put16(&d[0], idx);
  memcpy(&d[4], data4, 4);
  if (wait_reply) pump();
  if (!xmit(id, d)) return false;
  if (!wait_reply) return true;
  return waitType(CT_FEEDBACK, SYNC_RESP_TIMEOUT_US);
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

bool Rs06::setActiveReport(bool on) {
  uint32_t id = make_id(CT_ACTIVE_REP, (uint16_t)HOST_ID, motor_id_);
  uint8_t d[8] = {1, 2, 3, 4, 5, 6, (uint8_t)(on ? 1 : 0), 0};
  pump();
  if (!xmit(id, d)) return false;
  return waitType(CT_ACTIVE_REP, SYNC_RESP_TIMEOUT_US);
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
