#include "can_scan.h"
#include "rs06_proto.h"
#include "config.h"

#include <M5Unified.h>
#include <driver/twai.h>
#include <esp_timer.h>
#include <stdio.h>

using namespace rs06;

namespace {

struct PinOpt { gpio_num_t gpio; uint8_t ch; };

// CH1..CH4 drive the transceiver input, CH5..CH8 read its output.
const PinOpt TXP[] = {
  { GPIO_NUM_14, 1 }, { GPIO_NUM_2, 2 }, { GPIO_NUM_27, 3 }, { GPIO_NUM_0, 4 },
};
const PinOpt RXP[] = {
  { GPIO_NUM_13, 5 }, { GPIO_NUM_19, 6 }, { GPIO_NUM_34, 7 }, { GPIO_NUM_35, 8 },
};
constexpr int N_TX = sizeof(TXP) / sizeof(TXP[0]);
constexpr int N_RX = sizeof(RXP) / sizeof(RXP[0]);

struct BaudOpt { uint32_t baud; twai_timing_config_t timing; };

bool install(gpio_num_t tx, gpio_num_t rx, const twai_timing_config_t& t) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
  g.tx_queue_len = 4;
  g.rx_queue_len = 16;
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_timing_config_t tt = t;
  if (twai_driver_install(&g, &tt, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  return true;
}

void teardown() {
  twai_stop();
  twai_driver_uninstall();
}

void drain() {
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {}
}

// Type 0 (get device ID) is the cheapest probe: it needs no prior state in the
// motor and answers with the 64-bit MCU UID.
bool probe(uint8_t motor_id, uint32_t wait_us) {
  twai_message_t m{};
  m.identifier       = make_id(CT_GET_ID, (uint16_t)HOST_ID, motor_id);
  m.extd             = 1;
  m.data_length_code = 8;
  if (twai_transmit(&m, pdMS_TO_TICKS(2)) != ESP_OK) return false;

  const int64_t deadline = esp_timer_get_time() + (int64_t)wait_us;
  twai_message_t r;
  do {
    if (twai_receive(&r, 0) == ESP_OK) {
      // Any extended frame carrying our host id in the low byte is a reply.
      if (r.extd && id_dest(r.identifier) == HOST_ID) return true;
    }
  } while (esp_timer_get_time() < deadline);
  return false;
}

// If nothing on the bus acknowledges, the TX error counter runs away. That
// separates "wrong pins / no motor power / no termination" from "right bus,
// wrong motor ID".
bool busAcks() {
  twai_status_info_t before{}, after{};
  twai_get_status_info(&before);
  for (int i = 0; i < 6; i++) probe(0x7F, 300);
  twai_get_status_info(&after);
  return after.tx_error_counter <= before.tx_error_counter + 8;
}

void say(const char* line, int y, uint16_t col = TFT_WHITE) {
  M5.Display.fillRect(0, y, 320, 18, TFT_BLACK);
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextColor(col, TFT_BLACK);
  M5.Display.setTextDatum(top_left);
  M5.Display.drawString(line, 6, y);
}

} // namespace

namespace can_scan {

Result run() {
  Result best;
  char line[64];

  BaudOpt bauds[4] = {
    { 1000000, TWAI_TIMING_CONFIG_1MBITS()   },
    {  500000, TWAI_TIMING_CONFIG_500KBITS() },
    {  250000, TWAI_TIMING_CONFIG_250KBITS() },
    {  125000, TWAI_TIMING_CONFIG_125KBITS() },
  };

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextDatum(top_center);
  M5.Display.drawString("CAN SCAN", 160, 6);

  const int total = N_TX * N_RX * 4;
  int done = 0;

  for (int bi = 0; bi < 4; bi++) {
    for (int ti = 0; ti < N_TX; ti++) {
      for (int ri = 0; ri < N_RX; ri++) {
        done++;
        snprintf(line, sizeof(line), "TX G%d / RX G%d @ %lu k   [%d/%d]",
                 (int)TXP[ti].gpio, (int)RXP[ri].gpio,
                 (unsigned long)(bauds[bi].baud / 1000), done, total);
        say(line, 40);
        M5.Display.fillRect(10, 62, 300, 10, TFT_BLACK);
        M5.Display.fillRect(10, 62, 300 * done / total, 10, TFT_DARKGREEN);

        if (!install(TXP[ti].gpio, RXP[ri].gpio, bauds[bi].timing)) continue;
        drain();

        if (!busAcks()) { teardown(); continue; }   // fast reject

        // Something out there is ACKing. Record it even if no ID answers.
        if (best.verdict == SCAN_NOTHING) {
          best.verdict = SCAN_ACK_ONLY;
          best.tx = (uint8_t)TXP[ti].gpio; best.rx = (uint8_t)RXP[ri].gpio;
          best.baud = bauds[bi].baud;
          best.dip_tx = TXP[ti].ch; best.dip_rx = RXP[ri].ch;
        }
        say("bus ACK - sweeping motor IDs...", 84, TFT_YELLOW);

        for (int id = 0; id <= 0x7F; id++) {
          drain();
          if (!probe((uint8_t)id, 2000)) continue;
          best.verdict = SCAN_FOUND;
          best.tx = (uint8_t)TXP[ti].gpio; best.rx = (uint8_t)RXP[ri].gpio;
          best.baud = bauds[bi].baud; best.id = (uint8_t)id;
          best.dip_tx = TXP[ti].ch;   best.dip_rx = RXP[ri].ch;
          break;
        }
        teardown();
        if (best.verdict == SCAN_FOUND) goto finished;
      }
    }
  }

finished:
  M5.Display.fillRect(0, 30, 320, 100, TFT_BLACK);
  describe(best, line, sizeof(line));
  say(line, 40, best.verdict == SCAN_FOUND ? TFT_GREEN : TFT_ORANGE);
  if (best.verdict == SCAN_FOUND) {
    snprintf(line, sizeof(line), "DIP: CH%u (TX) + CH%u (RX) ON, rest OFF",
             best.dip_tx, best.dip_rx);
    say(line, 60, TFT_LIGHTGREY);
  } else if (best.verdict == SCAN_ACK_ONLY) {
    say("bus is alive but no motor ID replied", 60, TFT_LIGHTGREY);
  } else {
    say("check: DIP, CANH/L swap, 120ohm x2,", 60, TFT_LIGHTGREY);
    say("motor 48V power, PwrCAN 9-24V power", 78, TFT_LIGHTGREY);
  }
  delay(2500);
  return best;
}

void describe(const Result& r, char* out, size_t n) {
  switch (r.verdict) {
    case SCAN_FOUND:
      snprintf(out, n, "FOUND id 0x%02X  TX G%u RX G%u  %luk",
               r.id, r.tx, r.rx, (unsigned long)(r.baud / 1000));
      break;
    case SCAN_ACK_ONLY:
      snprintf(out, n, "ACK only  TX G%u RX G%u  %luk",
               r.tx, r.rx, (unsigned long)(r.baud / 1000));
      break;
    default:
      snprintf(out, n, "no CAN response on any pin/baud");
      break;
  }
}

} // namespace can_scan
