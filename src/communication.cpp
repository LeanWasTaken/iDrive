#include "communication.h"

void setBrightness(uint8_t level) {
    if (level > 0xFD) level = 0xFD;
    
    if (level == 0x00) {
        unsigned char off[1] = {0xFE};
        CAN.sendMsgBuf(0x202, 0, 1, off);
        current.iDriveLightOn = false;
    } else {
        unsigned char data[1] = {level};
        CAN.sendMsgBuf(0x202, 0, 1, data);
        current.brightnessLevel = level;
        current.iDriveLightOn = true;
    }
    delay(30);
}

void adjustBrightness(int8_t delta) {
    uint8_t newLevel;
    
    if (delta > 0) {
        newLevel = min(0xFD, current.brightnessLevel + 0x20);
        if (current.brightnessLevel == 0x00) newLevel = 0x20;
    } else {
        newLevel = (current.brightnessLevel <= 0x20) ? 0x00 : current.brightnessLevel - 0x20;
    }
    
    setBrightness(newLevel);
    
    Serial.print("Brightness: 0x");
    Serial.print(newLevel, HEX);
    Serial.print(" (");
    Serial.print(newLevel ? (newLevel * 100) / 0xFD : 0);
    Serial.println("%)");
}

void sendWakeUp() {
    unsigned char data[8] = {0x1D, 0xE1, 0x00, 0xF0, 0xFF, 0x7F, 0xDE, 0x04};
    CAN.sendMsgBuf(0x273, 0, 8, data);
    if (statusDebugMode) Serial.println("Wake-up sent");
}

void sendKeepAlive() {
    unsigned char data[8] = {0x40, 0x67, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00};
    CAN.sendMsgBuf(0x567, 0, 8, data);
    if (statusDebugMode) Serial.println("Keep-alive sent");
}