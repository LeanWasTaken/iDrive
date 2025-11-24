#include "idrive_state.h"
#include "config.h"

// Global variables
// Debug mode: 0 = Normal (state changes only), 1 = Debug (known packets), 2 = Raw (all packets)
uint8_t debugMode = 0;
unsigned long startMillis;
MCP_CAN CAN(CAN_CS);

// State instances
iDriveState current;
iDriveState previous;