/*
 * iDrive Controller CAN Bus Interpreter - Reworked Version
 * ========================================================
 *
 * This program interprets CAN messages from a BMW iDrive controller knob.
 * Based on confirmed analysis of captured CAN data:
 *
 * CONFIRMED WORKING:
 * CAN ID 0x567: Knob status messages
 *   - Pattern: 0x40 0x67 0x00 0x00 0x00 0xXX 0x00 0x00
 *   - data[5]: 0x02 = normal/touch, 0x00 = knob pressed down
 *   - Used for: Press detection and touch detection (when no rotation)
 *
 * CAN ID 0x25B: Combined rotation and button messages
 *   - data[0]: sequence counter (increments with each event)
 *   - data[1]: encoder position value (changes for rotation)
 *   - data[4]: BACK button state (0x00 = released, 0x20 = pressed, 0x80 = touched)
 *   - data[5]: COM/HOME button state (COM: 0x08=pressed, 0x20=touched; HOME: 0x04=pressed, 0x10=touched; 0x00=released)
 *   - data[6]: MEDIA button state (0xC0 = released, 0xC1 = pressed, 0xC4 = touched)
 *   - Used for: Rotation detection/direction AND BACK/COM/HOME/MEDIA button press/touch/release
 *
 * UNDER INVESTIGATION (raw messages printed):
 * CAN ID 0x5E7: Button press messages
 * CAN ID 0x0BF: Continuous data stream
 *
 * Hardware: ESP32-S3 with MCP2515 CAN controller
 * CS pin: 10, INT pin: 14, Speed: 500kbps
 */

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

// CAN bus configuration
#define CAN_INT 14 // Interrupt pin
#define CAN_CS 10  // CS pin

MCP_CAN CAN(CAN_CS);

// iDrive controller CAN IDs
#define IDRIVE_KNOB_STATUS_ID 0x567 // Knob press/touch status (CONFIRMED)
#define IDRIVE_ROTATION_ID 0x25B    // Rotation messages AND BACK button (CONFIRMED)
#define IDRIVE_BUTTONS_ID 0x5E7     // Other button presses (INVESTIGATING)
#define IDRIVE_DATA_STREAM_ID 0x0BF // Data stream (INVESTIGATING)

// Controller state - only confirmed functionality
struct iDriveState
{
  // CONFIRMED working states
  bool knobPressed = false;        // Physical knob press (0x567 data[5] = 0x00)
  bool knobTouched = false;        // Surface touch without rotation
  bool backButtonPressed = false;  // BACK button pressed (0x25B data[4] = 0x20)
  bool backButtonTouched = false;  // BACK button touched (0x25B data[4] = 0x80)
  bool comButtonPressed = false;   // COM button pressed (0x25B data[5] = 0x08)
  bool comButtonTouched = false;   // COM button touched (0x25B data[5] = 0x20)
  bool homeButtonPressed = false;  // HOME button pressed (0x25B data[5] = 0x04)
  bool homeButtonTouched = false;  // HOME button touched (0x25B data[5] = 0x10)
  bool mediaButtonPressed = false; // MEDIA button pressed (0x25B data[6] = 0xC1)
  bool mediaButtonTouched = false; // MEDIA button touched (0x25B data[6] = 0xC4)
  int rotationDirection = 0;       // -1 = CCW, 0 = none, 1 = CW
  int stepPosition = 0;            // Cumulative rotation steps

  // Sequence tracking for 0x25B messages
  uint8_t sequenceCounter = 0;
  uint8_t lastEncoderValue = 0;
  bool firstRotationMessage = true;
  bool iDriveLightOn = false; // Light state for 0x202 messages

  // Touch detection timing
  unsigned long last567Time = 0;
  unsigned long last25BTime = 0;
};

unsigned long int currentMillis;
unsigned long int startMillis;

iDriveState current;
iDriveState previous;

// Debug configuration
bool rawDebugMode = false;   // Show all unknown/investigating messages
bool statusDebugMode = true; // Show confirmed status changes

// Function declarations
void handleSerialCommands();
void processCanMessages();
void handleKnobStatus(unsigned char *data, unsigned long timestamp);
void handleRotation(unsigned char *data, unsigned long timestamp);
void handleButtonsInvestigation(unsigned char *data);
void handleDataStreamInvestigation(unsigned char *data);
void updateTouchDetection();
void reportStateChanges();
void printRawMessage(const char *type, unsigned long id, unsigned char len, unsigned char *data);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  startMillis = millis();

  // Initialize CAN bus
  pinMode(CAN_INT, INPUT);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
  {
    Serial.println("✓ CAN Bus initialized successfully");
  }
  else
  {
    Serial.println("✗ CAN Bus initialization failed");
    while (1)
      delay(1000);
  }

  CAN.setMode(MCP_NORMAL);
  attachInterrupt(digitalPinToInterrupt(CAN_INT), []() {}, FALLING);

  Serial.println("Commands:");
  Serial.println("  'r' - Toggle raw debug (investigating messages)");
  Serial.println("  's' - Toggle status debug (confirmed functionality)");
  Serial.println();
}

void do_iDriveLight()
{
  // ID: 202, Data: 2 FD 0 == Light ON, D: 202, Data: 2 FE 0 == Light OFF
  unsigned char buf[2] = {0x00, 0x0};
  const int msglength = 2;
  if (!current.iDriveLightOn)
  {
    buf[0] = 0xFD;
  }
  else
  {
    buf[0] = 0xFE;
  }
  CAN.sendMsgBuf(0x202, 0, msglength, buf);
}

void loop()
{
  handleSerialCommands();
  processCanMessages();
}

void handleSerialCommands()
{
  if (Serial.available())
  {
    char cmd = Serial.read();
    switch (cmd)
    {
    case 'r':
    case 'R':
      rawDebugMode = !rawDebugMode;
      Serial.print("Raw debug mode: ");
      Serial.println(rawDebugMode ? "ON" : "OFF");
      break;

    case 's':
    case 'S':
      statusDebugMode = !statusDebugMode;
      Serial.print("Status debug mode: ");
      Serial.println(statusDebugMode ? "ON" : "OFF");
      break;
    }
  }
}

void processCanMessages()
{
  if (!digitalRead(CAN_INT))
  {
    unsigned long rxId;
    unsigned char len;
    unsigned char rxBuf[8];

    if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK)
    {
      unsigned long currentTime = millis();
      previous = current;

      if (rawDebugMode)
      {
        if (rxId != IDRIVE_DATA_STREAM_ID)
          printRawMessage("RAW", rxId, len, rxBuf);
      }

      switch (rxId)
      {
      case IDRIVE_KNOB_STATUS_ID:
        handleKnobStatus(rxBuf, currentTime);
        break;

      case IDRIVE_ROTATION_ID:
        handleRotation(rxBuf, currentTime);
        break;

      case IDRIVE_BUTTONS_ID:
        handleButtonsInvestigation(rxBuf);
        break;

        /*case IDRIVE_DATA_STREAM_ID:
          handleDataStreamInvestigation(rxBuf);
          break;*/

      default:
        if (rawDebugMode && rxId != IDRIVE_DATA_STREAM_ID)
        {
          printRawMessage("UNKNOWN", rxId, len, rxBuf);
        }
        break;
      }
    }
  }
}

void handleKnobStatus(unsigned char *data, unsigned long timestamp)
{
  return;
  printRawMessage("KNOB", IDRIVE_KNOB_STATUS_ID, 8, data);
  if (data[0] == 0x40 && data[1] == 0x67)
  {
    current.last567Time = timestamp;

    // data[5] reliably indicates knob press state
    bool newPressed = (data[5] == 0x00);

    if (current.knobPressed != newPressed)
    {
      current.knobPressed = newPressed;

      if (statusDebugMode)
      {
        Serial.print("0x567: Knob ");
        Serial.print(newPressed ? "PRESSED" : "RELEASED");
        Serial.print(" (data[5]=0x");
        Serial.print(data[5], HEX);
        Serial.println(")");
      }
    }
  }
}

void handleRotation(unsigned char *data, unsigned long timestamp)
{
  // 0x25B: Handles rotation, BACK button, COM button, and MEDIA button messages
  uint8_t newSequence = data[0];
  uint8_t newEncoder = data[1];
  uint8_t backButtonState = data[4];
  uint8_t comButtonState = data[5];
  uint8_t mediaButtonState = data[6];

  // Handle BACK button press/touch state (data[4])
  bool newBackPressed = (backButtonState == 0x20);
  bool newBackTouched = (backButtonState == 0x80);

  // Check for BACK button press state change
  if (current.backButtonPressed != newBackPressed)
  {
    current.backButtonPressed = newBackPressed;

    if (statusDebugMode)
    {
      Serial.print("0x25B: BACK button ");
      Serial.print(newBackPressed ? "PRESSED" : "UNPRESSED");
      Serial.print(" (data[4]=0x");
      Serial.print(backButtonState, HEX);
      Serial.println(")");
    }
  }

  // Check for BACK button touch state change
  if (current.backButtonTouched != newBackTouched)
  {
    current.backButtonTouched = newBackTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: BACK button ");
      Serial.print(newBackTouched ? "TOUCHED" : "UNTOUCHED");
      Serial.print(" (data[4]=0x");
      Serial.print(backButtonState, HEX);
      Serial.println(")");
    }
  }

  // Handle COM button press/touch state (data[5])
  bool newComPressed = (comButtonState == 0x08);
  bool newComTouched = (comButtonState == 0x20);

  // Check for COM button press state change
  if (current.comButtonPressed != newComPressed)
  {
    current.comButtonPressed = newComPressed;

    if (statusDebugMode)
    {
      Serial.print("0x25B: COM button ");
      Serial.print(newComPressed ? "PRESSED" : "UNPRESSED");
      Serial.print(" (data[5]=0x");
      Serial.print(comButtonState, HEX);
      Serial.println(")");
    }
  }

  // Check for COM button touch state change
  if (current.comButtonTouched != newComTouched)
  {
    current.comButtonTouched = newComTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: COM button ");
      Serial.print(newComTouched ? "TOUCHED" : "UNTOUCHED");
      Serial.print(" (data[5]=0x");
      Serial.print(comButtonState, HEX);
      Serial.println(")");
    }
  }

  // Handle HOME button press/touch state (data[5] - different values)
  bool newHomePressed = (comButtonState == 0x04);
  bool newHomeTouched = (comButtonState == 0x10);

  // Check for HOME button press state change
  if (current.homeButtonPressed != newHomePressed)
  {
    current.homeButtonPressed = newHomePressed;

    if (statusDebugMode)
    {
      Serial.print("0x25B: HOME button ");
      Serial.print(newHomePressed ? "PRESSED" : "UNPRESSED");
      Serial.print(" (data[5]=0x");
      Serial.print(comButtonState, HEX);
      Serial.println(")");
    }
  }

  // Check for HOME button touch state change
  if (current.homeButtonTouched != newHomeTouched)
  {
    current.homeButtonTouched = newHomeTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: HOME button ");
      Serial.print(newHomeTouched ? "TOUCHED" : "UNTOUCHED");
      Serial.print(" (data[5]=0x");
      Serial.print(comButtonState, HEX);
      Serial.println(")");
    }
  }

  // Handle MEDIA button press/touch state (data[6])
  bool newMediaPressed = (mediaButtonState == 0xC1);
  bool newMediaTouched = (mediaButtonState == 0xC4);

  // Check for MEDIA button press state change
  if (current.mediaButtonPressed != newMediaPressed)
  {
    current.mediaButtonPressed = newMediaPressed;

    if (statusDebugMode)
    {
      Serial.print("0x25B: MEDIA button ");
      Serial.print(newMediaPressed ? "PRESSED" : "UNPRESSED");
      Serial.print(" (data[6]=0x");
      Serial.print(mediaButtonState, HEX);
      Serial.println(")");
    }
  }

  // Check for MEDIA button touch state change
  if (current.mediaButtonTouched != newMediaTouched)
  {
    current.mediaButtonTouched = newMediaTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: MEDIA button ");
      Serial.print(newMediaTouched ? "TOUCHED" : "UNTOUCHED");
      Serial.print(" (data[6]=0x");
      Serial.print(mediaButtonState, HEX);
      Serial.println(")");
    }
  }

  // Report combined button states for clarity
  if (statusDebugMode && (current.backButtonPressed != previous.backButtonPressed || current.backButtonTouched != previous.backButtonTouched))
  {
    Serial.print("0x25B: BACK button state: ");
    if (backButtonState == 0x00)
      Serial.println("RELEASED");
    else if (backButtonState == 0x20)
      Serial.println("PRESSED");
    else if (backButtonState == 0x80)
      Serial.println("TOUCHED");
    else
      Serial.println("UNKNOWN (0x" + String(backButtonState, HEX) + ")");
  }

  if (statusDebugMode && (current.comButtonPressed != previous.comButtonPressed || current.comButtonTouched != previous.comButtonTouched))
  {
    Serial.print("0x25B: COM button state: ");
    if (comButtonState == 0x00)
      Serial.println("RELEASED");
    else if (comButtonState == 0x08)
      Serial.println("PRESSED");
    else if (comButtonState == 0x20)
      Serial.println("TOUCHED");
    else
      Serial.println("UNKNOWN (0x" + String(comButtonState, HEX) + ")");
  }

  if (statusDebugMode && (current.homeButtonPressed != previous.homeButtonPressed || current.homeButtonTouched != previous.homeButtonTouched))
  {
    Serial.print("0x25B: HOME button state: ");
    if (comButtonState == 0x00)
      Serial.println("RELEASED");
    else if (comButtonState == 0x04)
      Serial.println("PRESSED");
    else if (comButtonState == 0x10)
      Serial.println("TOUCHED");
    else
      Serial.println("UNKNOWN (0x" + String(comButtonState, HEX) + ")");
  }

  if (statusDebugMode && (current.mediaButtonPressed != previous.mediaButtonPressed || current.mediaButtonTouched != previous.mediaButtonTouched))
  {
    Serial.print("0x25B: MEDIA button state: ");
    if (mediaButtonState == 0xC0)
      Serial.println("RELEASED");
    else if (mediaButtonState == 0xC1)
      Serial.println("PRESSED");
    else if (mediaButtonState == 0xC4)
      Serial.println("TOUCHED");
    else
      Serial.println("UNKNOWN (0x" + String(mediaButtonState, HEX) + ")");
  }

  // Handle rotation detection (data[0] and data[1])
  // Only process rotation if sequence counter changed
  if (newSequence != current.sequenceCounter)
  {
    if (!current.firstRotationMessage)
    {
      // Calculate rotation direction from encoder value change
      int16_t encoderDiff = (int16_t)newEncoder - (int16_t)current.lastEncoderValue;

      // Handle wrap-around
      if (encoderDiff > 127)
        encoderDiff -= 256;
      else if (encoderDiff < -127)
        encoderDiff += 256;

      // Only report rotation if there's actually encoder movement
      // (to distinguish from BACK button sequence increments)
      if (encoderDiff != 0)
      {
        if (encoderDiff > 0)
        {
          current.rotationDirection = 1; // Clockwise
          current.stepPosition++;
        }
        else if (encoderDiff < 0)
        {
          current.rotationDirection = -1; // Counter-clockwise
          current.stepPosition--;
        }

        if (statusDebugMode)
        {
          Serial.print("0x25B: Rotation ");
          Serial.print(current.rotationDirection == 1 ? "CLOCKWISE" : "COUNTER-CLOCKWISE");
          Serial.print(" (Step: ");
          Serial.print(current.stepPosition);
          Serial.print(", Encoder: 0x");
          Serial.print(current.lastEncoderValue, HEX);
          Serial.print("→0x");
          Serial.print(newEncoder, HEX);
          Serial.println(")");
        }
      }
      else
      {
        current.rotationDirection = 0; // No rotation movement
      }
    }
    else
    {
      current.firstRotationMessage = false;
      current.rotationDirection = 0;
    }

    current.sequenceCounter = newSequence;
    current.lastEncoderValue = newEncoder;
  }
  else
  {
    current.rotationDirection = 0; // No sequence change, no rotation
  }
}

void handleButtonsInvestigation(unsigned char *data)
{
  // 0x5E7: Button messages - UNDER INVESTIGATION
  if (rawDebugMode)
  {
    printRawMessage("BUTTONS", IDRIVE_BUTTONS_ID, 8, data);
  }

  // Look for patterns we've seen before
  if (data[0] == 0x27 && data[1] == 0x67 && data[2] == 0x2D)
  {
    Serial.println("0x5E7: Possible button press pattern detected!");
  }
}

void printRawMessage(const char *type, unsigned long id, unsigned char len, unsigned char *data)
{
  Serial.print("[");
  Serial.print(type);
  Serial.print("] ID:0x");
  Serial.print(id, HEX);
  Serial.print(" Data:");

  for (int i = 0; i < len; i++)
  {
    Serial.print(" ");
    if (data[i] < 0x10)
      Serial.print("0");
    Serial.print(data[i], HEX);
  }
  Serial.println();
}
