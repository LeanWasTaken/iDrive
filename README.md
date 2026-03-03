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

- **Touchpad:** Data stream (0x0BF) is received but not decoded yet
- **Wake-up via CAN:** Center knob press required to wake the ZBE. Tested NM frames (0x510, 0x130, 0x440, 0x563, 0x12F) — none trigger wake. Likely needs a specific frame from the MGU/BDC that hasn't been identified yet. Sniffing a real F44 K-CAN at ignition would reveal it.

## Usage

### Serial Commands

```
d     - Cycle debug mode (Normal/Debug/Raw)
k     - Send keep-alive (0x510) — automatic every 500ms
w     - Wake ZBE (not yet working via CAN)
+/-   - Increase/decrease brightness
0-9   - Set brightness level (0=off, 9=max)
h     - Show help menu
```
