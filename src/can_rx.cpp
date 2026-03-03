#include "can_rx.h"
#include "can_protocol.h"
#include "idrive_controller.h"
#include "twai_driver.h"

#include <Arduino.h>

namespace {

void printRawMessage(const char *type, unsigned long id, uint8_t len, uint8_t *data, unsigned long timestamp) {
    Serial.print("[");
    if (timestamp < 100000) Serial.print(" ");
    if (timestamp < 10000) Serial.print(" ");
    if (timestamp < 1000) Serial.print(" ");
    if (timestamp < 100) Serial.print(" ");
    if (timestamp < 10) Serial.print(" ");
    Serial.print(timestamp);
    Serial.print("ms] [");
    Serial.print(type);
    Serial.print("] 0x");
    Serial.print(id, HEX);
    Serial.print(":");

    for (int i = 0; i < len; i++) {
        Serial.print(" ");
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
    }
    Serial.println();
}

void handleHeartbeat567(uint8_t *data, unsigned long timestamp) {
    if (debugMode >= 1) {
        printRawMessage("ID_567", ID_HEARTBEAT_567, 8, data, timestamp);
    }
    state.last567Time = timestamp;
}

void handleController(uint8_t *data) {
    updateKnobStates(data[3]);
    updateButtonStates(data);
    updateRotation(data[0], data[1]);
}

void handleHeartbeat5E7(uint8_t *data, unsigned long timestamp) {
    if (debugMode >= 1) {
        printRawMessage("ID_5E7", ID_HEARTBEAT_5E7, 8, data, timestamp);
    }
}

void handleGearIndication(uint8_t *data, unsigned long timestamp) {
    if (debugMode >= 1) {
        printRawMessage("GEAR", ID_GEAR, 8, data, timestamp);
    }
}

}  // namespace

void processCanMessages() {
    uint32_t rxId;
    uint8_t len;
    uint8_t rxBuf[8];

    if (!twai_receive(&rxId, &len, rxBuf)) return;

    unsigned long now = millis();

    if (debugMode == 2 && rxId != ID_DATA_STREAM) {
        printRawMessage("RAW", rxId, len, rxBuf, now);
    }

    switch (rxId) {
        case ID_HEARTBEAT_567:
            handleHeartbeat567(rxBuf, now);
            break;
        case ID_CONTROLLER:
            handleController(rxBuf);
            break;
        case ID_HEARTBEAT_5E7:
            handleHeartbeat5E7(rxBuf, now);
            break;
        case ID_GEAR:
            handleGearIndication(rxBuf, now);
            break;
        default:
            if (debugMode == 2 && rxId != ID_DATA_STREAM) {
                printRawMessage("UNKNOWN", rxId, len, rxBuf, now);
            }
            break;
    }
}
