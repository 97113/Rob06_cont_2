// Host stub of the ESP-IDF TWAI driver, backed by a scriptable fake motor.
// Only the surface rs06_driver.cpp actually uses is modelled.
#pragma once
#include <stdint.h>
#include <string.h>

typedef int esp_err_t;
#define ESP_OK   0
#define ESP_FAIL -1

typedef int gpio_num_t;
#define GPIO_NUM_0  0
#define GPIO_NUM_4  4
#define GPIO_NUM_2  2
#define GPIO_NUM_13 13
#define GPIO_NUM_14 14
#define GPIO_NUM_19 19
#define GPIO_NUM_27 27
#define GPIO_NUM_34 34
#define GPIO_NUM_35 35

typedef enum { TWAI_MODE_NORMAL = 0, TWAI_MODE_NO_ACK, TWAI_MODE_LISTEN_ONLY } twai_mode_t;

typedef struct {
  uint32_t identifier;
  uint32_t extd : 1;
  uint32_t rtr  : 1;
  uint8_t  data_length_code;
  uint8_t  data[8];
} twai_message_t;

typedef struct {
  twai_mode_t mode;
  gpio_num_t  tx_io, rx_io;
  int         tx_queue_len, rx_queue_len;
  uint32_t    alerts_enabled;
} twai_general_config_t;

typedef struct { int brp; } twai_timing_config_t;
typedef struct { uint32_t acceptance_code, acceptance_mask; bool single_filter; } twai_filter_config_t;

#define TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, op) \
  twai_general_config_t{ (op), (tx), (rx), 5, 5, 0 }
#define TWAI_TIMING_CONFIG_1MBITS()   twai_timing_config_t{ 4 }
#define TWAI_TIMING_CONFIG_500KBITS() twai_timing_config_t{ 8 }
#define TWAI_TIMING_CONFIG_250KBITS() twai_timing_config_t{ 16 }
#define TWAI_TIMING_CONFIG_125KBITS() twai_timing_config_t{ 32 }
#define TWAI_FILTER_CONFIG_ACCEPT_ALL() twai_filter_config_t{ 0, 0xFFFFFFFF, true }

#define TWAI_ALERT_BUS_OFF       (1u << 0)
#define TWAI_ALERT_ERR_PASS      (1u << 1)
#define TWAI_ALERT_RX_QUEUE_FULL (1u << 2)

typedef enum {
  TWAI_STATE_STOPPED = 0, TWAI_STATE_RUNNING, TWAI_STATE_BUS_OFF, TWAI_STATE_RECOVERING
} twai_state_t;

typedef struct {
  twai_state_t state;
  uint32_t msgs_to_tx, msgs_to_rx;
  uint32_t tx_error_counter, rx_error_counter;
  uint32_t tx_failed_count, rx_missed_count, rx_overrun_count;
  uint32_t arb_lost_count, bus_error_count;
} twai_status_info_t;

// --- fake bus, defined in fake_bus.cpp -------------------------------------
esp_err_t twai_driver_install(const twai_general_config_t*, const twai_timing_config_t*,
                              const twai_filter_config_t*);
esp_err_t twai_start();
esp_err_t twai_stop();
esp_err_t twai_driver_uninstall();
esp_err_t twai_transmit(const twai_message_t* m, uint32_t ticks);
esp_err_t twai_receive(twai_message_t* m, uint32_t ticks);
esp_err_t twai_get_status_info(twai_status_info_t* st);
esp_err_t twai_read_alerts(uint32_t* alerts, uint32_t ticks);
esp_err_t twai_initiate_recovery();
