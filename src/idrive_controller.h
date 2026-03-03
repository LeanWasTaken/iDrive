#pragma once

#include <cstdint>

struct iDriveState {
    // Knob directional presses
    bool knobPressedCenter = false;
    bool knobPressedLeft   = false;
    bool knobPressedUp     = false;
    bool knobPressedRight  = false;
    bool knobPressedDown   = false;

    // Button press/touch states
    bool backButtonPressed   = false;
    bool backButtonTouched   = false;
    bool comButtonPressed    = false;
    bool comButtonTouched    = false;
    bool optionButtonPressed = false;
    bool optionButtonTouched = false;
    bool homeButtonPressed   = false;
    bool homeButtonTouched   = false;
    bool mediaButtonPressed  = false;
    bool mediaButtonTouched  = false;
    bool navButtonPressed    = false;
    bool navButtonTouched    = false;
    bool mapButtonPressed    = false;
    bool mapButtonTouched    = false;
    bool globeButtonPressed  = false;
    bool globeButtonTouched  = false;

    // Encoder / rotation
    int     rotationDirection    = 0;
    int     stepPosition         = 0;
    uint8_t sequenceCounter      = 0;
    uint8_t lastEncoderValue     = 0;
    bool    firstRotationMessage = true;

    // Brightness / light
    bool    iDriveLightOn   = false;
    uint8_t brightnessLevel = 0xFD;

    // Timing
    unsigned long last567Time         = 0;
    unsigned long last25BTime         = 0;
    unsigned long lastKeepAliveTime = 0;
};

extern iDriveState state;
extern uint8_t debugMode;

// State mutation functions
void updateKnobStates(uint8_t knobByte);
void updateButtonStates(const uint8_t *frameData);
void updateRotation(uint8_t newSequence, uint8_t newEncoder);
void setBrightness(uint8_t level);
void adjustBrightness(int8_t delta);
