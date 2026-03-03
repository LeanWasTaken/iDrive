#pragma once

#include <cstdint>

// CAN message IDs
constexpr uint16_t ID_CONTROLLER    = 0x25B;
constexpr uint16_t ID_HEARTBEAT_567 = 0x567;
constexpr uint16_t ID_HEARTBEAT_5E7 = 0x5E7;
constexpr uint16_t ID_DATA_STREAM   = 0x0BF;
constexpr uint16_t ID_GEAR          = 0x3FD;
constexpr uint16_t ID_KEEPALIVE     = 0x510;
constexpr uint16_t ID_BRIGHTNESS    = 0x202;

// Timing intervals (milliseconds)
constexpr uint32_t KEEPALIVE_INTERVAL_MS = 500;

// Brightness constants
constexpr uint8_t MAX_BRIGHTNESS       = 0xFD;
constexpr uint8_t BRIGHTNESS_OFF       = 0xFE;
constexpr uint8_t BRIGHTNESS_STEP      = 0x20;
constexpr uint8_t BRIGHTNESS_BASE      = 0x20;
constexpr uint8_t BRIGHTNESS_INCREMENT = 0x18;

// Keepalive payload
constexpr uint8_t KEEPALIVE_FRAME[8] = {0x40, 0x10, 0x00, 0x02, 0x03, 0x92, 0x01, 0x00};
