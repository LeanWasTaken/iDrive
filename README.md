# BMW iDrive Controller CAN Bus Interpreter

A CAN bus interface for a BMW F44 iDrive controller using an ESP32-C3 with the native TWAI (CAN) peripheral and a TJA1441A/B transceiver.

## Hardware

- **Microcontroller:** Adafruit QT Py ESP32-C3
- **CAN Transceiver:** TJA1441A/B
- **iDrive Controller:** ZBE_07_HI_AT_RL_MATT (P/N. 9423197), pretty likely compatible with other models such as G20, F40, etc. as well.
  - Hardware Version: 05.01.00
  - Software Version: 05.08.00
- **CAN Bus Speed:** 500 kbps

## Wiring

| ESP32-C3 | TJA1441A/B | iDrive Controller |
| -------- | ---------- | ----------------- |
| GPIO 3   | TXD        | -                 |
| GPIO 2   | RXD        | -                 |
| GPIO 20  | S (silent) | -                 |
| 3.3V     | VCC        | -                 |
| GND      | GND        | GND               |
| -        | CANH       | CAN H             |
| -        | CANL       | CAN L             |
| -        | -          | +12V              |

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
d     - Cycle debug mode (Normal/Debug/Raw)
+/-   - Increase/decrease brightness
0-9   - Set brightness level (0=off, 9=max)
h     - Show help menu
```
