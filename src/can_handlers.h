#ifndef CAN_HANDLERS_H
#define CAN_HANDLERS_H

#include <Arduino.h>
#include "config.h"
#include "idrive_state.h"

void processCanMessages();
void handleUnknown567(unsigned char *data, unsigned long timestamp);
void handleController(unsigned char *data, unsigned long timestamp);  
void handleUnknown5E7(unsigned char *data);
void printRawMessage(const char *type, unsigned long id, unsigned char len, unsigned char *data);

#endif