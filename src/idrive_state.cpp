#include "idrive_state.h"
#include "config.h"

// Global variables
// Debug mode: 0 = Normal (state changes only), 1 = Debug (known packets), 2 = Raw (all packets)
uint8_t debugMode = 0;
unsigned long startMillis;

// State instances
iDriveState current;
iDriveState previous;

// TWAI driver implementation
static bool twai_initialized = false;

bool twai_init() {
    // Configure silent mode pin (TJA1441A/B: HIGH = normal mode, LOW = silent)
    pinMode(TWAI_SILENT_GPIO, OUTPUT);
    twai_set_silent_mode(false);  // Start in normal mode

    // TWAI general configuration
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 10;
    g_config.tx_queue_len = 5;

    // TWAI timing configuration for 500kbps
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    // TWAI filter configuration - accept all messages
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        Serial.printf("TWAI driver install failed: %s\n", esp_err_to_name(result));
        return false;
    }

    // Start TWAI driver
    result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("TWAI start failed: %s\n", esp_err_to_name(result));
        twai_driver_uninstall();
        return false;
    }

    twai_initialized = true;
    return true;
}

bool twai_send(uint32_t id, uint8_t len, const uint8_t *data) {
    if (!twai_initialized) return false;

    twai_message_t message;
    message.identifier = id;
    message.extd = 0;  // Standard frame (11-bit ID)
    message.rtr = 0;   // Data frame
    message.data_length_code = len;

    for (int i = 0; i < len && i < 8; i++) {
        message.data[i] = data[i];
    }

    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(10));
    return result == ESP_OK;
}

bool twai_receive(uint32_t *id, uint8_t *len, uint8_t *data) {
    if (!twai_initialized) return false;

    twai_message_t message;
    esp_err_t result = twai_receive(&message, 0);  // Non-blocking

    if (result != ESP_OK) {
        return false;
    }

    *id = message.identifier;
    *len = message.data_length_code;

    for (int i = 0; i < message.data_length_code && i < 8; i++) {
        data[i] = message.data[i];
    }

    return true;
}

void twai_set_silent_mode(bool silent) {
    // TJA1441A/B: HIGH = silent mode, so we invert for normal operation
    digitalWrite(TWAI_SILENT_GPIO, silent ? HIGH : LOW);
}
