// ---------------------------------------------------------------------------
// A scriptable RS06 on a virtual CAN bus.
//
// The point of this fake is latency. The real motor answers a Type1 inside the
// slot that asked, but answers an auxiliary Type17 read whenever it feels like
// it - and the captured test history in tools/ shows that "whenever" is
// routinely later than the slot. Every reply here is therefore scheduled at an
// explicit due time, so a test can reproduce exactly that and check what the
// driver does with an answer that arrives two slots late.
//
// Time advances only inside twai_receive(): polling the bus is what costs
// microseconds, which keeps every test deterministic and fast.
// ---------------------------------------------------------------------------
#include "fake_bus.h"
#include <driver/twai.h>
#include <esp_timer.h>
#include <deque>
#include <vector>

int64_t g_fake_now_us = 0;

namespace {

struct Pending {
  int64_t        due_us;
  twai_message_t msg;
};

bool                       g_installed = false;
bool                       g_running   = false;
std::vector<Pending>       g_inflight;   // scheduled, not yet deliverable
std::deque<twai_message_t> g_rxq;        // deliverable now
twai_status_info_t         g_status{};
uint32_t                   g_alerts = 0;

// Microseconds of virtual time burned by one poll of the RX queue.
constexpr int64_t RX_POLL_US = 5;

void promote() {
  for (size_t i = 0; i < g_inflight.size();) {
    if (g_inflight[i].due_us <= g_fake_now_us) {
      g_rxq.push_back(g_inflight[i].msg);
      g_inflight.erase(g_inflight.begin() + i);
    } else {
      i++;
    }
  }
}

} // namespace

namespace fake {

void reset() {
  g_installed = false;
  g_running   = false;
  g_fake_now_us = 0;
  g_inflight.clear();
  g_rxq.clear();
  g_status = twai_status_info_t{};
  g_status.state = TWAI_STATE_RUNNING;
  g_alerts = 0;
  tx_log.clear();
  auto_reply = nullptr;
}

std::vector<twai_message_t> tx_log;
ReplyFn                     auto_reply = nullptr;

void schedule(const twai_message_t& m, int64_t delay_us) {
  g_inflight.push_back(Pending{ g_fake_now_us + delay_us, m });
}

void queueNow(const twai_message_t& m) { g_rxq.push_back(m); }

int64_t nowUs() { return g_fake_now_us; }
void    advance(int64_t us) { g_fake_now_us += us; }

void setState(int s) { g_status.state = (twai_state_t)s; }

} // namespace fake

// ---------------------------------------------------------------------------
esp_err_t twai_driver_install(const twai_general_config_t*, const twai_timing_config_t*,
                              const twai_filter_config_t*) {
  if (g_installed) return ESP_FAIL;
  g_installed = true;
  return ESP_OK;
}
esp_err_t twai_start()            { g_running = true;  g_status.state = TWAI_STATE_RUNNING; return ESP_OK; }
esp_err_t twai_stop()             { g_running = false; g_status.state = TWAI_STATE_STOPPED; return ESP_OK; }
esp_err_t twai_driver_uninstall() { g_installed = false; return ESP_OK; }

esp_err_t twai_transmit(const twai_message_t* m, uint32_t) {
  if (!g_running) return ESP_FAIL;
  fake::tx_log.push_back(*m);
  if (fake::auto_reply) fake::auto_reply(*m);
  return ESP_OK;
}

esp_err_t twai_receive(twai_message_t* m, uint32_t) {
  g_fake_now_us += RX_POLL_US;
  promote();
  if (g_rxq.empty()) return ESP_FAIL;
  *m = g_rxq.front();
  g_rxq.pop_front();
  return ESP_OK;
}

esp_err_t twai_get_status_info(twai_status_info_t* st) { *st = g_status; return ESP_OK; }
esp_err_t twai_read_alerts(uint32_t* a, uint32_t)      { *a = g_alerts; g_alerts = 0; return ESP_OK; }
esp_err_t twai_initiate_recovery()                     { g_status.state = TWAI_STATE_RECOVERING; return ESP_OK; }
