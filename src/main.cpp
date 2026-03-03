#include <Arduino.h>
#include "twai_driver.h"
#include "can_protocol.h"
#include "idrive_controller.h"
#include "can_rx.h"
#include "can_tx.h"
#include "serial_commands.h"

void setup() {
    Serial.begin(115200);
    unsigned long serialTimeout = millis();
    while (!Serial && (millis() - serialTimeout < 3000)) delay(10);

    Serial.println("Starting iDrive Controller...");

    if (twai_init()) {
        Serial.println("TWAI (CAN) Bus OK");
    } else {
        Serial.println("TWAI (CAN) Bus FAIL");
        while (1) delay(1000);
    }

    state.lastKeepAliveTime = millis();

    Serial.println("iDrive Controller Ready");
    Serial.println("Press 'h' for help");
    Serial.println();
}

void loop() {
    handleSerialCommands();
    processCanMessages();

    unsigned long now = millis();

    if (now - state.lastKeepAliveTime >= KEEPALIVE_INTERVAL_MS) {
        sendKeepAlive();
    }
}
