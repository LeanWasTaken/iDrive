#ifndef IDRIVE_STATE_H
#define IDRIVE_STATE_H

#include <Arduino.h>

struct iDriveState
{
  bool knobPressedCenter = false;
  bool knobPressedLeft = false;
  bool knobPressedUp = false;
  bool knobPressedRight = false;
  bool knobPressedDown = false;
  bool backButtonPressed = false;
  bool backButtonTouched = false;
  bool comButtonPressed = false;
  bool comButtonTouched = false;
  bool optionButtonPressed = false;
  bool optionButtonTouched = false;
  bool homeButtonPressed = false;
  bool homeButtonTouched = false;
  bool mediaButtonPressed = false;
  bool mediaButtonTouched = false;
  bool navButtonPressed = false;
  bool navButtonTouched = false;
  bool mapButtonPressed = false;
  bool mapButtonTouched = false;
  bool globeButtonPressed = false;
  bool globeButtonTouched = false;
  int rotationDirection = 0;
  int stepPosition = 0;

  uint8_t sequenceCounter = 0;
  uint8_t lastEncoderValue = 0;
  bool firstRotationMessage = true;
  bool iDriveLightOn = false;
  uint8_t brightnessLevel = 0xFD;

  unsigned long last567Time = 0;
  unsigned long last25BTime = 0;
  unsigned long lastKeepAliveTime = 0;
  unsigned long lastStatusBurstTime = 0;
  uint8_t statusBurstIndex = 0;
  bool wakeUpSequenceSent = false;
};

extern iDriveState current;
extern iDriveState previous;

#endif