#include "communication.h"

namespace {

constexpr uint16_t BRIGHTNESS_CAN_ID = 0x202;
constexpr uint8_t MAX_BRIGHTNESS = 0xFD;
constexpr uint8_t BRIGHTNESS_STEP = 0x20;

uint8_t clampBrightness(uint8_t level) {
    return (level > MAX_BRIGHTNESS) ? MAX_BRIGHTNESS : level;
}

void sendSingleByte(uint16_t id, uint8_t value) {
    uint8_t payload[1] = {value};
    twai_send(id, 1, payload);
}

void sendLightsOff() {
    sendSingleByte(BRIGHTNESS_CAN_ID, 0xFE);
}

void sendBrightnessValue(uint8_t level) {
    sendSingleByte(BRIGHTNESS_CAN_ID, level);
}

void logBrightness(uint8_t level) {
    Serial.print("Brightness: 0x");
    Serial.print(level, HEX);
    Serial.print(" (");
    Serial.print(level ? (level * 100) / MAX_BRIGHTNESS : 0);
    Serial.println("%)");
}

uint8_t computeAdjustedBrightness(int8_t delta) {
    if (delta > 0) {
        if (current.brightnessLevel == 0x00) {
            return BRIGHTNESS_STEP;
        }
        uint16_t candidate = current.brightnessLevel + BRIGHTNESS_STEP;
        return (candidate > MAX_BRIGHTNESS) ? MAX_BRIGHTNESS : static_cast<uint8_t>(candidate);
    }

    if (current.brightnessLevel <= BRIGHTNESS_STEP) {
        return 0x00;
    }
    return current.brightnessLevel - BRIGHTNESS_STEP;
}

}  // namespace

void setBrightness(uint8_t level) {
    uint8_t normalized = clampBrightness(level);

    if (normalized == 0x00) {
        sendLightsOff();
    } else {
        sendBrightnessValue(normalized);
    }

    current.brightnessLevel = normalized;
    current.iDriveLightOn = (normalized != 0x00);
    delay(30);
}

void adjustBrightness(int8_t delta) {
    uint8_t newLevel = computeAdjustedBrightness(delta);
    setBrightness(newLevel);
    logBrightness(newLevel);
}

void sendKeepAlive() {
    uint8_t keepalive[8] = {0x40, 0x10, 0x00, 0x02, 0x03, 0x92, 0x01, 0x00};
    twai_send(IDRIVE_KEEPALIVE_ID, 8, keepalive);
    current.lastKeepAliveTime = millis();
}

void sendStatusBurst() {
    // Status burst frames for 0x3C - cycles through different payloads
    static const uint8_t STATUS_FRAMES[][8] = {
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    };
    constexpr uint8_t NUM_FRAMES = sizeof(STATUS_FRAMES) / sizeof(STATUS_FRAMES[0]);

    twai_send(IDRIVE_STATUS_BURST_ID, 8, STATUS_FRAMES[current.statusBurstIndex]);
    current.statusBurstIndex = (current.statusBurstIndex + 1) % NUM_FRAMES;
    current.lastStatusBurstTime = millis();
}
