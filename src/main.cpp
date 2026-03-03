#include <Arduino.h>
#include "config.h"
#include "idrive_state.h"
#include "can_handlers.h"
#include "communication.h"
#include "serial_interface.h"

void setup() {
    Serial.begin(115200);
    // Wait for serial with timeout (useful for USB CDC on ESP32-C3)
    unsigned long serialTimeout = millis();
    while (!Serial && (millis() - serialTimeout < 3000)) delay(10);

    Serial.println("Starting iDrive Controller...");

    startMillis = millis();

    // Initialize TWAI (CAN) driver
    if (twai_init()) {
        Serial.println("TWAI (CAN) Bus OK");
    } else {
        Serial.println("TWAI (CAN) Bus FAIL");
        while (1) delay(1000);
    }

    current.lastKeepAliveTime = millis();

    Serial.println("iDrive Controller Ready");
    Serial.println("Press 'h' for help");
    Serial.println();
}

void loop() {
    handleSerialCommands();
    processCanMessages();

    unsigned long currentTime = millis();
    // Automatic keepalive
    if (currentTime - current.lastKeepAliveTime >= KEEPALIVE_INTERVAL) {
        sendKeepAlive();
    }

    // Continuous status burst: Cycle through 0x3C frames every 2ms, aint workin yet
    if (currentTime - current.lastStatusBurstTime >= STATUS_BURST_INTERVAL) {
        sendStatusBurst();
    }
}
