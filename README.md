# BMW iDrive Controller CAN Bus Interpreter

A CAN bus interface for a BMW F44 iDrive controller using ESP32-S3 and MCP2515.

## Hardware

- **Microcontroller:** ESP32-S3 or ESP32-C3
- **CAN Interface:** MCP2515 CAN controller breakout
- **iDrive Controller:** ZBE_07_HI_AT_RL_MATT (P/N. 9423197), pretty likely compatible with other models such as G20, F40, etc. as well.
  - Hardware Version: 05.01.00
  - Software Version: 05.08.00
- **CAN Bus Speed:** 500 kbps

## Wiring

### ESP32-S3

| ESP32-S3 | MCP2515 | iDrive Controller |
| -------- | ------- | ----------------- |
| GPIO 10  | CS      | -                 |
| GPIO 14  | INT     | -                 |
| 3.3V     | VCC     | -                 |
| GND      | GND     | GND               |
| GPIO 23  | SI      | -                 |
| GPIO 19  | SO      | -                 |
| GPIO 18  | SCK     | -                 |
| -        | -       | +12V              |
| -        | -       | CAN H             |
| -        | -       | CAN L             |

### ESP32-C3 (Adafruit QT Py)

| ESP32-C3 | MCP2515 | iDrive Controller |
| -------- | ------- | ----------------- |
| GPIO 10  | CS      | -                 |
| GPIO 14  | INT     | -                 |
| GPIO 2   | SCK     | -                 |
| GPIO 3   | MISO    | -                 |
| GPIO 4   | MOSI    | -                 |
| 3.3V     | VCC     | -                 |
| GND      | GND     | GND               |
| -        | -       | +12V              |
| -        | -       | CAN H             |
| -        | -       | CAN L             |

## Features

### Working

- **Rotation Detection:** Clockwise/counter-clockwise with step counting
- **Knob Press:** 5-directional joystick (center, up, down, left, right)
- **Button Detection:** All 8 buttons with press/touch states
  - BACK, HOME, COM, OPTION, MEDIA, NAV, MAP, GLOBE
- **Brightness Control:** Full range adjustment (0x00-0xFD)
- **Light Toggle:** On/off control via CAN bus
- **Real-time Monitoring:** Live CAN message interpretation

### Not Working

- **Touchpad:** Not implemented yet
- **Wake-up via CAN:** Knob has to be pressed to wake it up. (feel free to send a PR if you figure this out)

## Usage

### Serial Commands

```
d - Toggle debug modes (raw/known/status)
+/-   - Increase/decrease brightness  
0-9   - Set brightness level (0=off, 9=max)
h     - Show help menu
```
