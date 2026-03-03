#include "serial_interface.h"

namespace {

constexpr uint8_t MAX_BRIGHTNESS = 0xFD;
constexpr uint8_t BRIGHTNESS_BASE = 0x20;
constexpr uint8_t BRIGHTNESS_INCREMENT = 0x18;

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
    Serial.println("Sending wake sequence (0x202 FD x5)...");
    uint8_t payload[2] = {0xFD, 0x00};
    for (int i = 0; i < 5; i++) {
        twai_send(0x202, 1, payload);
        Serial.print("  Sent ");
        Serial.print(i + 1);
        Serial.println("/5");
        if (i < 4) delay(100);
    }
    Serial.println("Wake sequence complete");
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
    Serial.println("  w     - Send wake sequence (0x202 FD x5)");
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
