#include "serial_interface.h"

void handleSerialCommands() {
    if (!Serial.available()) return;
    
    char cmd = Serial.read();
    
    switch (cmd) {
        case 'd': case 'D':
            debugMode = (debugMode + 1) % 3;
            Serial.print("Debug mode: ");
            switch(debugMode) {
                case 0:
                    Serial.println("NORMAL (state changes only)");
                    break;
                case 1:
                    Serial.println("DEBUG (known packets + state changes)");
                    break;
                case 2:
                    Serial.println("RAW (all packets)");
                    break;
            }
            break;

        case 'k': case 'K':
            sendKeepAlive();
            break;
            
        case '+': case '=':
            adjustBrightness(1);
            break;
            
        case '-': case '_':
            adjustBrightness(-1);
            break;
            
        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9': {
            uint8_t level = (cmd == '0') ? 0x00 : 0x20 + ((cmd - '1') * 0x18);
            if (level > 0xFD) level = 0xFD;
            setBrightness(level);
            Serial.print("Level ");
            Serial.print(cmd);
            Serial.print(" (");
            Serial.print(level ? (level * 100) / 0xFD : 0);
            Serial.println("%)");
            break;
        }
        
        case 'h': case 'H': case '?':
            Serial.println("\nCommands:");
            Serial.println("  d     - Cycle debug mode (Normal/Debug/Raw)");
            Serial.println("  k     - Send keep-alive (0x510) - automatic every 500ms");
            Serial.println("  +/-   - Adjust brightness");
            Serial.println("  0-9   - Set brightness level");
            Serial.println("  h     - Help");
            Serial.println("\nDebug Modes:");
            Serial.println("  Normal: State changes only (buttons, knob, rotation)");
            Serial.println("  Debug:  Known CAN packets + state changes");
            Serial.println("  Raw:    All CAN packets");
            break;
            
        default:
            Serial.print("Unknown: '");
            Serial.print(cmd);
            Serial.println("'");
            break;
    }
}