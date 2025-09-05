#ifndef CONFIG_H
#define CONFIG_H

#include <mcp_can.h>

#define CAN_INT 14
#define CAN_CS 10

#define IDRIVE_UNKNOWN_567 0x567
#define IDRIVE_CONTROLLER_ID 0x25B
#define IDRIVE_UNKNOWN_5E7 0x5E7
#define IDRIVE_DATA_STREAM_ID 0x0BF

extern bool rawDebugMode;
extern bool statusDebugMode;
extern bool filterUnknown;
extern unsigned long startMillis;
extern MCP_CAN CAN;

#endif