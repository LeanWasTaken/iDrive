#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "config.h"
#include "idrive_state.h"

void sendKeepAlive();
void sendStatusBurst();
void setBrightness(uint8_t level);
void adjustBrightness(int8_t delta);

#endif