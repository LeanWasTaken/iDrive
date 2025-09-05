#include "idrive_state.h"
#include "config.h"

// Global variables
bool rawDebugMode = true;
bool statusDebugMode = true; 
bool filterUnknown = true;
unsigned long startMillis;
MCP_CAN CAN(CAN_CS);

// State instances
iDriveState current;
iDriveState previous;