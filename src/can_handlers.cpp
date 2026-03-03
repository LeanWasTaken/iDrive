#include "can_handlers.h"

namespace {

struct KnobMapping {
    const char *label;
    uint8_t matchValue;
    bool iDriveState::*pressedField;
};

constexpr KnobMapping KNOB_MAPPINGS[] = {
    {"CENTER", 0x01, &iDriveState::knobPressedCenter},
    {"LEFT",   0xA0, &iDriveState::knobPressedLeft},
    {"UP",     0x10, &iDriveState::knobPressedUp},
    {"RIGHT",  0x40, &iDriveState::knobPressedRight},
    {"DOWN",   0x70, &iDriveState::knobPressedDown},
};

enum class ButtonState : uint8_t {
    Released,
    Pressed,
    Touched,
};

struct ButtonDescriptor {
    const char *label;
    uint8_t byteIndex;
    uint8_t pressedValue;
    uint8_t touchedValue;
    bool iDriveState::*pressedField;
    bool iDriveState::*touchedField;
};

constexpr ButtonDescriptor BUTTON_MAPPINGS[] = {
    {"BACK",   4, 0x20, 0x80, &iDriveState::backButtonPressed,  &iDriveState::backButtonTouched},
    {"HOME",   4, 0x04, 0x10, &iDriveState::homeButtonPressed,  &iDriveState::homeButtonTouched},
    {"COM",    5, 0x08, 0x20, &iDriveState::comButtonPressed,   &iDriveState::comButtonTouched},
    {"OPTION", 5, 0x01, 0x04, &iDriveState::optionButtonPressed,&iDriveState::optionButtonTouched},
    {"MEDIA",  6, 0xC1, 0xC4, &iDriveState::mediaButtonPressed, &iDriveState::mediaButtonTouched},
    {"NAV",    6, 0xC8, 0xE0, &iDriveState::navButtonPressed,   &iDriveState::navButtonTouched},
    {"MAP",    7, 0xC1, 0xC4, &iDriveState::mapButtonPressed,   &iDriveState::mapButtonTouched},
    {"GLOBE",  7, 0xC8, 0xE0, &iDriveState::globeButtonPressed, &iDriveState::globeButtonTouched},
};

bool shouldLogStateChanges() {
    return debugMode == 0 || debugMode == 1;
}

const char *toStateString(ButtonState state) {
    switch (state) {
        case ButtonState::Pressed: return "PRESSED";
        case ButtonState::Touched: return "TOUCHED";
        default: return "RELEASED";
    }
}

void logKnobChange(const char *label, bool isPressed) {
    if (!shouldLogStateChanges()) return;
    Serial.print("Knob ");
    Serial.println(isPressed ? label : "RELEASED");
}

void logButtonChange(const char *label, ButtonState state) {
    if (!shouldLogStateChanges()) return;
    Serial.print(label);
    Serial.print(" ");
    Serial.println(toStateString(state));
}

ButtonState decodeButtonState(uint8_t raw, const ButtonDescriptor &descriptor) {
    if (raw == descriptor.pressedValue) return ButtonState::Pressed;
    if (raw == descriptor.touchedValue) return ButtonState::Touched;
    return ButtonState::Released;
}

void updateKnobStates(uint8_t knobState) {
    for (const auto &mapping : KNOB_MAPPINGS) {
        bool pressed = knobState == mapping.matchValue;
        bool &stateField = current.*(mapping.pressedField);
        if (stateField != pressed) {
            stateField = pressed;
            logKnobChange(mapping.label, pressed);
        }
    }
}

void updateButtonStates(const unsigned char *data) {
    for (const auto &descriptor : BUTTON_MAPPINGS) {
        ButtonState state = decodeButtonState(data[descriptor.byteIndex], descriptor);
        bool pressed = state == ButtonState::Pressed;
        bool touched = state == ButtonState::Touched;
        bool &pressedField = current.*(descriptor.pressedField);
        bool &touchedField = current.*(descriptor.touchedField);

        if (pressedField != pressed || touchedField != touched) {
            pressedField = pressed;
            touchedField = touched;
            logButtonChange(descriptor.label, state);
        }
    }
}

void updateRotation(uint8_t newSequence, uint8_t newEncoder) {
    if (newSequence == current.sequenceCounter) {
        current.rotationDirection = 0;
        return;
    }

    if (!current.firstRotationMessage) {
        int16_t encoderDiff = static_cast<int16_t>(newEncoder) - static_cast<int16_t>(current.lastEncoderValue);

        if (encoderDiff > 127) {
            encoderDiff -= 256;
        } else if (encoderDiff < -127) {
            encoderDiff += 256;
        }

        if (encoderDiff != 0) {
            current.rotationDirection = (encoderDiff > 0) ? 1 : -1;
            current.stepPosition += current.rotationDirection;

            if (shouldLogStateChanges()) {
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
}

}  // namespace

void processCanMessages() {
    uint32_t rxId;
    uint8_t len;
    uint8_t rxBuf[8];

    if (!twai_receive(&rxId, &len, rxBuf)) return;

    unsigned long currentTime = millis();
    previous = current;

    if (debugMode == 2 && rxId != IDRIVE_DATA_STREAM_ID) {
        printRawMessage("RAW", rxId, len, rxBuf, currentTime);
    }

    switch (rxId) {
        case IDRIVE_UNKNOWN_567:
            handleUnknown567(rxBuf, currentTime);
            break;
        case IDRIVE_CONTROLLER_ID:
            handleController(rxBuf, currentTime);
            break;
        case IDRIVE_UNKNOWN_5E7:
            handleUnknown5E7(rxBuf, currentTime);
            break;
        case IDRIVE_GEAR_INDICATION_ID:
            handleGearIndication(rxBuf, currentTime);
            break;
        default:
            if (debugMode == 2 && rxId != IDRIVE_DATA_STREAM_ID) {
                printRawMessage("UNKNOWN", rxId, len, rxBuf, currentTime);
            }
            break;
    }
}

void handleUnknown567(unsigned char *data, unsigned long timestamp) {
    if (debugMode >= 1) {
        printRawMessage("ID_567", IDRIVE_UNKNOWN_567, 8, data, timestamp);
    }
    current.last567Time = timestamp;
}

void handleController(unsigned char *data, unsigned long /*timestamp*/) {
    updateKnobStates(data[3]);
    updateButtonStates(data);
    updateRotation(data[0], data[1]);
}

void handleUnknown5E7(unsigned char *data, unsigned long timestamp) {
    if (debugMode >= 1) {
        printRawMessage("ID_5E7", IDRIVE_UNKNOWN_5E7, 8, data, timestamp);
    }
}

void handleGearIndication(unsigned char *data, unsigned long timestamp) {
    // 0x3FD is used for gear indication - no processing needed, yet...
    if (debugMode >= 1) {
        printRawMessage("GEAR", IDRIVE_GEAR_INDICATION_ID, 8, data, timestamp);
    }
}

void printRawMessage(const char *type, unsigned long id, unsigned char len, unsigned char *data, unsigned long timestamp) {
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
