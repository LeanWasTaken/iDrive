#ifndef CONFIG_H
#define CONFIG_H

#include <mcp_can.h>

// Pin definitions are provided via build flags in platformio.ini
#ifndef CAN_INT
#define CAN_INT 14  // Default for ESP32-S3
#endif

#ifndef CAN_CS
#define CAN_CS 10   // Default for ESP32-S3
#endif

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
extern MCP_CAN CAN;

#endif