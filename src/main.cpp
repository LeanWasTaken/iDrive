#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include "idrive_state.h"
#include "can_handlers.h"
#include "communication.h"
#include "serial_interface.h"

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    startMillis = millis();
    pinMode(CAN_INT, INPUT);

    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN Bus OK");
    } else {
        Serial.println("CAN Bus FAIL");
        while (1) delay(1000);
    }

    CAN.setMode(MCP_NORMAL);
    attachInterrupt(digitalPinToInterrupt(CAN_INT), []() {}, FALLING);
    current.lastKeepAliveTime = millis();

    Serial.println("iDrive Controller Ready");
    Serial.println("Press 'h' for help");
    Serial.println();
}

void loop() {
    handleSerialCommands();
    processCanMessages();
}
