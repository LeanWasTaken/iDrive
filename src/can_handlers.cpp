#include "can_handlers.h"

void processCanMessages() {
    if (digitalRead(CAN_INT)) return;
    
    unsigned long rxId;
    unsigned char len;
    unsigned char rxBuf[8];

    if (CAN.readMsgBuf(&rxId, &len, rxBuf) != CAN_OK) return;
    
    unsigned long currentTime = millis();
    previous = current;

    if (rawDebugMode && rxId != IDRIVE_DATA_STREAM_ID)
        printRawMessage("RAW", rxId, len, rxBuf);

    switch (rxId) {
        case IDRIVE_UNKNOWN_567:
            handleUnknown567(rxBuf, currentTime);
            break;
        case IDRIVE_CONTROLLER_ID:
            handleController(rxBuf, currentTime);
            break;
        case IDRIVE_UNKNOWN_5E7:
            handleUnknown5E7(rxBuf);
            break;
        default:
            if (rawDebugMode && rxId != IDRIVE_DATA_STREAM_ID)
                printRawMessage("UNKNOWN", rxId, len, rxBuf);
            break;
    }
}

void handleUnknown567(unsigned char *data, unsigned long timestamp) {
    if (rawDebugMode && !filterUnknown) {
        printRawMessage("ID_567", IDRIVE_UNKNOWN_567, 8, data);
        Serial.println("Crown contact detected - no clue what ts does");
    }
    current.last567Time = timestamp;
}

void handleController(unsigned char *data, unsigned long timestamp) {
    uint8_t newSequence = data[0];
    uint8_t newEncoder = data[1]; 
    
    // Handle knob press with release detection
    uint8_t knobState = data[3];
    bool newKnobCenter = (knobState == 0x01);
    bool newKnobLeft = (knobState == 0xA0);
    bool newKnobUp = (knobState == 0x10);
    bool newKnobRight = (knobState == 0x40);
    bool newKnobDown = (knobState == 0x70);
    
    if (current.knobPressedCenter != newKnobCenter) {
        current.knobPressedCenter = newKnobCenter;
        if (statusDebugMode) {
            Serial.print("Knob ");
            Serial.println(newKnobCenter ? "CENTER" : "RELEASED");
        }
    }
    
    if (current.knobPressedLeft != newKnobLeft) {
        current.knobPressedLeft = newKnobLeft;
        if (statusDebugMode) {
            Serial.print("Knob ");
            Serial.println(newKnobLeft ? "LEFT" : "RELEASED");
        }
    }
    
    if (current.knobPressedUp != newKnobUp) {
        current.knobPressedUp = newKnobUp;
        if (statusDebugMode) {
            Serial.print("Knob ");
            Serial.println(newKnobUp ? "UP" : "RELEASED");
        }
    }
    
    if (current.knobPressedRight != newKnobRight) {
        current.knobPressedRight = newKnobRight;
        if (statusDebugMode) {
            Serial.print("Knob ");
            Serial.println(newKnobRight ? "RIGHT" : "RELEASED");
        }
    }
    
    if (current.knobPressedDown != newKnobDown) {
        current.knobPressedDown = newKnobDown;
        if (statusDebugMode) {
            Serial.print("Knob ");
            Serial.println(newKnobDown ? "DOWN" : "RELEASED");
        }
    }
    
    // Handle button state changes with touch/press detection
    bool newBackPressed = (data[4] == 0x20);
    bool newBackTouched = (data[4] == 0x80);
    bool newHomePressed = (data[4] == 0x04);
    bool newHomeTouched = (data[4] == 0x10);
    
    if (current.backButtonPressed != newBackPressed || current.backButtonTouched != newBackTouched) {
        current.backButtonPressed = newBackPressed;
        current.backButtonTouched = newBackTouched;
        if (statusDebugMode) {
            Serial.print("BACK ");
            Serial.println(data[4] == 0x00 ? "RELEASED" : (data[4] == 0x20 ? "PRESSED" : "TOUCHED"));
        }
    }
    
    if (current.homeButtonPressed != newHomePressed || current.homeButtonTouched != newHomeTouched) {
        current.homeButtonPressed = newHomePressed;
        current.homeButtonTouched = newHomeTouched;
        if (statusDebugMode) {
            Serial.print("HOME ");
            Serial.println(data[4] == 0x00 ? "RELEASED" : (data[4] == 0x04 ? "PRESSED" : "TOUCHED"));
        }
    }
    
    bool newComPressed = (data[5] == 0x08);
    bool newComTouched = (data[5] == 0x20);
    bool newOptionPressed = (data[5] == 0x01);
    bool newOptionTouched = (data[5] == 0x04);
    
    if (current.comButtonPressed != newComPressed || current.comButtonTouched != newComTouched) {
        current.comButtonPressed = newComPressed;
        current.comButtonTouched = newComTouched;
        if (statusDebugMode) {
            Serial.print("COM ");
            Serial.println(data[5] == 0x00 ? "RELEASED" : (data[5] == 0x08 ? "PRESSED" : "TOUCHED"));
        }
    }
    
    if (current.optionButtonPressed != newOptionPressed || current.optionButtonTouched != newOptionTouched) {
        current.optionButtonPressed = newOptionPressed;
        current.optionButtonTouched = newOptionTouched;
        if (statusDebugMode) {
            Serial.print("OPTION ");
            Serial.println(data[5] == 0x00 ? "RELEASED" : (data[5] == 0x01 ? "PRESSED" : "TOUCHED"));
        }
    }
    
    bool newMediaPressed = (data[6] == 0xC1);
    bool newMediaTouched = (data[6] == 0xC4);
    bool newNavPressed = (data[6] == 0xC8);
    bool newNavTouched = (data[6] == 0xE0);
    
    if (current.mediaButtonPressed != newMediaPressed || current.mediaButtonTouched != newMediaTouched) {
        current.mediaButtonPressed = newMediaPressed;
        current.mediaButtonTouched = newMediaTouched;
        if (statusDebugMode) {
            Serial.print("MEDIA ");
            Serial.println(data[6] == 0xC0 ? "RELEASED" : (data[6] == 0xC1 ? "PRESSED" : "TOUCHED"));
        }
    }
    
    if (current.navButtonPressed != newNavPressed || current.navButtonTouched != newNavTouched) {
        current.navButtonPressed = newNavPressed;
        current.navButtonTouched = newNavTouched;
        if (statusDebugMode) {
            Serial.print("NAV ");
            Serial.println(data[6] == 0xC0 ? "RELEASED" : (data[6] == 0xC8 ? "PRESSED" : "TOUCHED"));
        }
    }
    
    bool newMapPressed = (data[7] == 0xC1);
    bool newMapTouched = (data[7] == 0xC4);
    bool newGlobePressed = (data[7] == 0xC8);
    bool newGlobeTouched = (data[7] == 0xE0);
    
    if (current.mapButtonPressed != newMapPressed || current.mapButtonTouched != newMapTouched) {
        current.mapButtonPressed = newMapPressed;
        current.mapButtonTouched = newMapTouched;
        if (statusDebugMode) {
            Serial.print("MAP ");
            Serial.println(data[7] == 0xC0 ? "RELEASED" : (data[7] == 0xC1 ? "PRESSED" : "TOUCHED"));
        }
    }
    
    if (current.globeButtonPressed != newGlobePressed || current.globeButtonTouched != newGlobeTouched) {
        current.globeButtonPressed = newGlobePressed;
        current.globeButtonTouched = newGlobeTouched;
        if (statusDebugMode) {
            Serial.print("GLOBE ");
            Serial.println(data[7] == 0xC0 ? "RELEASED" : (data[7] == 0xC8 ? "PRESSED" : "TOUCHED"));
        }
    }
    
    // Handle rotation
    if (newSequence != current.sequenceCounter) {
        if (!current.firstRotationMessage) {
            int16_t encoderDiff = (int16_t)newEncoder - (int16_t)current.lastEncoderValue;
            
            if (encoderDiff > 127) encoderDiff -= 256;
            else if (encoderDiff < -127) encoderDiff += 256;
            
            if (encoderDiff != 0) {
                current.rotationDirection = (encoderDiff > 0) ? 1 : -1;
                current.stepPosition += current.rotationDirection;
                
                if (statusDebugMode) {
                    Serial.print("Rotation ");
                    Serial.print(current.rotationDirection == 1 ? "CW" : "CCW");
                    Serial.print(" (");
                    Serial.print(current.stepPosition);
                    Serial.println(")");
                }
            } else {
                current.rotationDirection = 0;
            }
        } else {
            current.firstRotationMessage = false;
            current.rotationDirection = 0;
        }
        
        current.sequenceCounter = newSequence;
        current.lastEncoderValue = newEncoder;
    } else {
        current.rotationDirection = 0;
    }
}

void handleUnknown5E7(unsigned char *data) {
    if (rawDebugMode && !filterUnknown) {
        printRawMessage("ID_5E7", IDRIVE_UNKNOWN_5E7, 8, data);
        if (data[0] == 0x05 && data[1] == 0x67 && data[2] == 0x04 && data[3] == 0x02)
            Serial.println("Crown contact pattern detected - no clue what ts does");
    }
}

void printRawMessage(const char *type, unsigned long id, unsigned char len, unsigned char *data) {
    Serial.print("[");
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