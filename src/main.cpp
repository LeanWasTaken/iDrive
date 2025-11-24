#include <Arduino.h>
#include <SPI.h>
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
    pinMode(CAN_INT, INPUT);

    // Initialize SPI with explicit pins for ESP32-C3 compatibility
    // QT Py ESP32-C3: SCK=GPIO2, MISO=GPIO3, MOSI=GPIO4
    // ESP32-S3: Uses default SPI pins
    #ifdef CONFIG_IDF_TARGET_ESP32C3
    SPI.begin(2, 3, 4, -1);  // SCK, MISO, MOSI
    #else
    SPI.begin();
    #endif

    Serial.println("SPI initialized");

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
