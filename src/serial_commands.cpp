#include "serial_commands.h"
#include "can_protocol.h"
#include "idrive_controller.h"
#include "can_tx.h"
#include "twai_driver.h"

#include <Arduino.h>

namespace {

void cycleDebugMode() {
    debugMode = (debugMode + 1) % 3;
    static const char *kDescriptions[] = {
        "NORMAL (state changes only)",
        "DEBUG (known packets + state changes)",
        "RAW (all packets)",
    };

    Serial.print("Debug mode: ");
    Serial.println(kDescriptions[debugMode]);
}

void sendWakeSequence() {
    // TODO: ZBE wake-up via CAN not yet solved.
    // Tried: 0x510, 0x130, 0x440, 0x563, 0x12F — none wake the ZBE.
    // Currently requires physical center knob press.
    // Need to sniff real F44 K-CAN at ignition to find the wake frame.
    Serial.println("Wake not implemented — press center knob to wake ZBE");
}

uint8_t brightnessLevelFromKey(char cmd) {
    if (cmd == '0') return 0x00;
    uint8_t offset = static_cast<uint8_t>(cmd - '1');
    uint16_t level = BRIGHTNESS_BASE + (offset * BRIGHTNESS_INCREMENT);
    return (level > MAX_BRIGHTNESS) ? MAX_BRIGHTNESS : static_cast<uint8_t>(level);
}

void reportBrightnessShortcut(char cmd, uint8_t level) {
    Serial.print("Level ");
    Serial.print(cmd);
    Serial.print(" (");
    Serial.print(level ? (level * 100) / MAX_BRIGHTNESS : 0);
    Serial.println("%)");
}

void applyNumericBrightness(char cmd) {
    uint8_t level = brightnessLevelFromKey(cmd);
    setBrightness(level);
    reportBrightnessShortcut(cmd, level);
}

void printHelp() {
    Serial.println("\nCommands:");
    Serial.println("  d     - Cycle debug mode (Normal/Debug/Raw)");
    Serial.println("  k     - Send keep-alive (0x510) - automatic every 500ms");
    Serial.println("  w     - Wake ZBE (not yet working via CAN)");
    Serial.println("  +/-   - Adjust brightness");
    Serial.println("  0-9   - Set brightness level");
    Serial.println("  h     - Help");
    Serial.println("\nDebug Modes:");
    Serial.println("  Normal: State changes only (buttons, knob, rotation)");
    Serial.println("  Debug:  Known CAN packets + state changes");
    Serial.println("  Raw:    All CAN packets");
}

void reportUnknownCommand(char cmd) {
    Serial.print("Unknown: '");
    Serial.print(cmd);
    Serial.println("'");
}

}  // namespace

void handleSerialCommands() {
    if (!Serial.available()) return;

    char cmd = Serial.read();

    switch (cmd) {
        case 'd': case 'D':
            cycleDebugMode();
            break;

        case 'k': case 'K':
            sendKeepAlive();
            break;

        case 'w': case 'W':
            sendWakeSequence();
            break;

        case '+': case '=':
            adjustBrightness(1);
            break;

        case '-': case '_':
            adjustBrightness(-1);
            break;

        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9':
            applyNumericBrightness(cmd);
            break;

        case 'h': case 'H': case '?':
            printHelp();
            break;

        default:
            reportUnknownCommand(cmd);
            break;
    }
}
