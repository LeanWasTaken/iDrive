#include "twai_driver.h"

#include <Arduino.h>
#include "driver/twai.h"

namespace {

constexpr gpio_num_t TX_GPIO     = GPIO_NUM_3;
constexpr gpio_num_t RX_GPIO     = GPIO_NUM_2;
constexpr gpio_num_t SILENT_GPIO = GPIO_NUM_20;

bool initialized = false;

}  // namespace

bool twai_init() {
    // Configure silent mode pin (TJA1441A/B: HIGH = silent, LOW = normal)
    pinMode(SILENT_GPIO, OUTPUT);
    twai_set_silent_mode(false);

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO, RX_GPIO, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 10;
    g_config.tx_queue_len = 5;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        Serial.printf("TWAI driver install failed: %s\n", esp_err_to_name(result));
        return false;
    }

    result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("TWAI start failed: %s\n", esp_err_to_name(result));
        twai_driver_uninstall();
        return false;
    }

    initialized = true;
    return true;
}

bool twai_send(uint32_t id, uint8_t len, const uint8_t *data) {
    if (!initialized) return false;

    twai_message_t message;
    message.identifier = id;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = len;

    for (int i = 0; i < len && i < 8; i++) {
        message.data[i] = data[i];
    }

    return twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK;
}

bool twai_receive(uint32_t *id, uint8_t *len, uint8_t *data) {
    if (!initialized) return false;

    twai_message_t message;
    if (twai_receive(&message, 0) != ESP_OK) return false;

    *id = message.identifier;
    *len = message.data_length_code;

    for (int i = 0; i < message.data_length_code && i < 8; i++) {
        data[i] = message.data[i];
    }

    return true;
}

void twai_set_silent_mode(bool silent) {
    // TJA1441A/B: HIGH = silent mode
    digitalWrite(SILENT_GPIO, silent ? HIGH : LOW);
}
