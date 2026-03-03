#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "driver/twai.h"

// TWAI (CAN) GPIO pin definitions for ESP32-C3
#define TWAI_TX_GPIO GPIO_NUM_3
#define TWAI_RX_GPIO GPIO_NUM_2
#define TWAI_SILENT_GPIO GPIO_NUM_20  // Silent mode control for TJA1441A/B (active-HIGH)

// CAN message IDs
#define IDRIVE_UNKNOWN_567 0x567
#define IDRIVE_CONTROLLER_ID 0x25B
#define IDRIVE_UNKNOWN_5E7 0x5E7
#define IDRIVE_DATA_STREAM_ID 0x0BF
#define IDRIVE_STATUS_BURST_ID 0x3C
#define IDRIVE_GEAR_INDICATION_ID 0x3FD
#define IDRIVE_KEEPALIVE_ID 0x510
#define IDRIVE_TERMINAL_STATUS_ID 0x12F

// Keep-alive timing (milliseconds)
#define KEEPALIVE_INTERVAL 500  // Send 0x510 every 500ms
#define STATUS_BURST_INTERVAL 2  // Send 0x3C every 2ms

extern uint8_t debugMode;  // 0=Normal, 1=Debug, 2=Raw
extern unsigned long startMillis;

// TWAI driver functions
bool twai_init();
bool twai_send(uint32_t id, uint8_t len, const uint8_t *data);
bool twai_receive(uint32_t *id, uint8_t *len, uint8_t *data);
void twai_set_silent_mode(bool silent);

#endif
