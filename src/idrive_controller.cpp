#include "idrive_controller.h"
#include "can_protocol.h"
#include "twai_driver.h"

#include <Arduino.h>

// Global state
iDriveState state;
uint8_t debugMode = 0;

namespace {

// --- Knob mapping table ---

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

// --- Button mapping table ---

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
    {"BACK",   4, 0x20, 0x80, &iDriveState::backButtonPressed,   &iDriveState::backButtonTouched},
    {"HOME",   4, 0x04, 0x10, &iDriveState::homeButtonPressed,   &iDriveState::homeButtonTouched},
    {"COM",    5, 0x08, 0x20, &iDriveState::comButtonPressed,    &iDriveState::comButtonTouched},
    {"OPTION", 5, 0x01, 0x04, &iDriveState::optionButtonPressed, &iDriveState::optionButtonTouched},
    {"MEDIA",  6, 0xC1, 0xC4, &iDriveState::mediaButtonPressed,  &iDriveState::mediaButtonTouched},
    {"NAV",    6, 0xC8, 0xE0, &iDriveState::navButtonPressed,    &iDriveState::navButtonTouched},
    {"MAP",    7, 0xC1, 0xC4, &iDriveState::mapButtonPressed,    &iDriveState::mapButtonTouched},
    {"GLOBE",  7, 0xC8, 0xE0, &iDriveState::globeButtonPressed,  &iDriveState::globeButtonTouched},
};

// --- Logging helpers ---

bool shouldLogStateChanges() {
    return debugMode == 0 || debugMode == 1;
}

const char *toStateString(ButtonState bs) {
    switch (bs) {
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

void logButtonChange(const char *label, ButtonState bs) {
    if (!shouldLogStateChanges()) return;
    Serial.print(label);
    Serial.print(" ");
    Serial.println(toStateString(bs));
}

ButtonState decodeButtonState(uint8_t raw, const ButtonDescriptor &desc) {
    if (raw == desc.pressedValue) return ButtonState::Pressed;
    if (raw == desc.touchedValue) return ButtonState::Touched;
    return ButtonState::Released;
}

// --- Brightness helpers ---

uint8_t clampBrightness(uint8_t level) {
    return (level > MAX_BRIGHTNESS) ? MAX_BRIGHTNESS : level;
}

uint8_t computeAdjustedBrightness(int8_t delta) {
    if (delta > 0) {
        if (state.brightnessLevel == 0x00) return BRIGHTNESS_STEP;
        uint16_t candidate = state.brightnessLevel + BRIGHTNESS_STEP;
        return (candidate > MAX_BRIGHTNESS) ? MAX_BRIGHTNESS : static_cast<uint8_t>(candidate);
    }
    if (state.brightnessLevel <= BRIGHTNESS_STEP) return 0x00;
    return state.brightnessLevel - BRIGHTNESS_STEP;
}

void logBrightness(uint8_t level) {
    Serial.print("Brightness: 0x");
    Serial.print(level, HEX);
    Serial.print(" (");
    Serial.print(level ? (level * 100) / MAX_BRIGHTNESS : 0);
    Serial.println("%)");
}

void sendBrightnessFrame(uint8_t value) {
    uint8_t payload[1] = {value};
    twai_send(ID_BRIGHTNESS, 1, payload);
}

}  // namespace

// --- Public mutation functions ---

void updateKnobStates(uint8_t knobByte) {
    for (const auto &mapping : KNOB_MAPPINGS) {
        bool pressed = knobByte == mapping.matchValue;
        bool &field = state.*(mapping.pressedField);
        if (field != pressed) {
            field = pressed;
            logKnobChange(mapping.label, pressed);
        }
    }
}

void updateButtonStates(const uint8_t *frameData) {
    for (const auto &desc : BUTTON_MAPPINGS) {
        ButtonState bs = decodeButtonState(frameData[desc.byteIndex], desc);
        bool pressed = bs == ButtonState::Pressed;
        bool touched = bs == ButtonState::Touched;
        bool &pressedField = state.*(desc.pressedField);
        bool &touchedField = state.*(desc.touchedField);

        if (pressedField != pressed || touchedField != touched) {
            pressedField = pressed;
            touchedField = touched;
            logButtonChange(desc.label, bs);
        }
    }
}

void updateRotation(uint8_t newSequence, uint8_t newEncoder) {
    if (newSequence == state.sequenceCounter) {
        state.rotationDirection = 0;
        return;
    }

    if (!state.firstRotationMessage) {
        int16_t diff = static_cast<int16_t>(newEncoder) - static_cast<int16_t>(state.lastEncoderValue);

        if (diff > 127) diff -= 256;
        else if (diff < -127) diff += 256;

        if (diff != 0) {
            state.rotationDirection = (diff > 0) ? 1 : -1;
            state.stepPosition += state.rotationDirection;

            if (shouldLogStateChanges()) {
                Serial.print("Rotation ");
                Serial.print(state.rotationDirection == 1 ? "CW" : "CCW");
                Serial.print(" (");
                Serial.print(state.stepPosition);
                Serial.println(")");
            }
        } else {
            state.rotationDirection = 0;
        }
    } else {
        state.firstRotationMessage = false;
        state.rotationDirection = 0;
    }

    state.sequenceCounter = newSequence;
    state.lastEncoderValue = newEncoder;
}

void setBrightness(uint8_t level) {
    uint8_t normalized = clampBrightness(level);

    if (normalized == 0x00) {
        sendBrightnessFrame(BRIGHTNESS_OFF);
    } else {
        sendBrightnessFrame(normalized);
    }

    state.brightnessLevel = normalized;
    state.iDriveLightOn = (normalized != 0x00);
    delay(30);
}

void adjustBrightness(int8_t delta) {
    uint8_t newLevel = computeAdjustedBrightness(delta);
    setBrightness(newLevel);
    logBrightness(newLevel);
}
