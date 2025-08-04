/*
 * iDrive * CAN ID 0x25B: Combined rotation, knob press, and button messages
 *   - data[0]: sequence counter (increments with each event)
 *   - data[1]: encoder position value (changes for rotation)
 *   - data[3]: knob press state (0x00 = released, 0x01 = center, 0x10 = up, 0x40 = right, 0x70 = down, 0xA0 = left)
 *   - data[4]: BACK button state (0x00 = released, 0x20 = pressed, 0x80 = touched)
 *   - data[5]: COM/OPTION button state (COM: 0x08=pressed, 0x20=touched; OPTION: 0x01=pressed, 0x04=touched; 0x00=released)
 *   - data[6]: MEDIA/NAV button state (MEDIA: 0xC1=pressed, 0xC4=touched; NAV: 0xC8=pressed, 0xE0=touched; 0xC0=released)
 *   - data[7]: MAP/GLOBE button state (MAP: 0xC1=pressed, 0xC4=touched; GLOBE: 0xC8=pressed, 0xE0=touched; 0xC0=released)
 *   - Used for: Rotation detection/direction AND knob press AND BACK/COM/OPTION/HOME/MEDIA/NAV/MAP/GLOBE button press/touch/releaseler CAN Bus Interpreter - Reworked Version
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
 *   - data[5]: COM/OPTION button state (COM: 0x08=pressed, 0x20=touched; OPTION: 0x01=pressed, 0x04=touched; 0x00=released)
 *   - data[6]: MEDIA/NAV button state (MEDIA: 0xC1=pressed, 0xC4=touched; NAV: 0xC8=pressed, 0xE0=touched; 0xC0=released)
 *   - data[7]: MAP/GLOBE button state (MAP: 0xC1=pressed, 0xC4=touched; GLOBE: 0xC8=pressed, 0xE0=touched; 0xC0=released)
 *   - Used for: Rotation detection/direction AND BACK/COM/OPTION/HOME/MEDIA/NAV/MAP/GLOBE button press/touch/release
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
  bool knobPressed = false;         // Physical knob press (0x567 data[5] = 0x00)
  bool knobTouched = false;         // Surface touch without rotation
  bool knobPressedLeft = false;     // Knob pressed left (0x25B data[3] = 0xA0)
  bool knobPressedUp = false;       // Knob pressed up (0x25B data[3] = 0x10)
  bool knobPressedRight = false;    // Knob pressed right (0x25B data[3] = 0x40)
  bool knobPressedDown = false;     // Knob pressed down (0x25B data[3] = 0x70)
  bool backButtonPressed = false;   // BACK button pressed (0x25B data[4] = 0x20)
  bool backButtonTouched = false;   // BACK button touched (0x25B data[4] = 0x80)
  bool comButtonPressed = false;    // COM button pressed (0x25B data[5] = 0x08)
  bool comButtonTouched = false;    // COM button touched (0x25B data[5] = 0x20)
  bool optionButtonPressed = false; // OPTION button pressed (0x25B data[5] = 0x01)
  bool optionButtonTouched = false; // OPTION button touched (0x25B data[5] = 0x04)
  bool homeButtonPressed = false;   // HOME button pressed (0x25B data[5] = 0x04)
  bool homeButtonTouched = false;   // HOME button touched (0x25B data[5] = 0x10)
  bool mediaButtonPressed = false;  // MEDIA button pressed (0x25B data[6] = 0xC1)
  bool mediaButtonTouched = false;  // MEDIA button touched (0x25B data[6] = 0xC4)
  bool navButtonPressed = false;    // NAV button pressed (0x25B data[6] = 0xC8)
  bool navButtonTouched = false;    // NAV button touched (0x25B data[6] = 0xE0)
  bool mapButtonPressed = false;    // MAP button pressed (0x25B data[7] = 0xC1)
  bool mapButtonTouched = false;    // MAP button touched (0x25B data[7] = 0xC4)
  bool globeButtonPressed = false;  // GLOBE button pressed (0x25B data[7] = 0xC8)
  bool globeButtonTouched = false;  // GLOBE button touched (0x25B data[7] = 0xE0)
  int rotationDirection = 0;        // -1 = CCW, 0 = none, 1 = CW
  int stepPosition = 0;             // Cumulative rotation steps

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
  // 0x25B: Handles rotation, knob press, and all button messages
  uint8_t newSequence = data[0];
  uint8_t newEncoder = data[1];
  uint8_t knobPressState = data[3];
  uint8_t backButtonState = data[4];
  uint8_t comButtonState = data[5];
  uint8_t mediaButtonState = data[6];
  uint8_t mapButtonState = data[7];

  // Handle knob press (data[3])
  bool newKnobPressed = (knobPressState == 0x01);
  bool newKnobPressedLeft = (knobPressState == 0xA0);
  bool newKnobPressedUp = (knobPressState == 0x10);
  bool newKnobPressedRight = (knobPressState == 0x40);
  bool newKnobPressedDown = (knobPressState == 0x70);

  if (current.knobPressed != newKnobPressed)
  {
    current.knobPressed = newKnobPressed;

    if (statusDebugMode)
    {
      Serial.print("0x25B: Knob ");
      if (knobPressState == 0x00)
        Serial.println("RELEASED");
      else if (knobPressState == 0x01)
        Serial.println("PRESSED CENTER");
    }
  }

  if (current.knobPressedLeft != newKnobPressedLeft)
  {
    current.knobPressedLeft = newKnobPressedLeft;

    if (statusDebugMode)
    {
      Serial.print("0x25B: Knob ");
      if (knobPressState == 0x00)
        Serial.println("RELEASED");
      else if (knobPressState == 0xA0)
        Serial.println("PRESSED LEFT");
    }
  }

  if (current.knobPressedUp != newKnobPressedUp)
  {
    current.knobPressedUp = newKnobPressedUp;

    if (statusDebugMode)
    {
      Serial.print("0x25B: Knob ");
      if (knobPressState == 0x00)
        Serial.println("RELEASED");
      else if (knobPressState == 0x10)
        Serial.println("PRESSED UP");
    }
  }

  if (current.knobPressedRight != newKnobPressedRight)
  {
    current.knobPressedRight = newKnobPressedRight;

    if (statusDebugMode)
    {
      Serial.print("0x25B: Knob ");
      if (knobPressState == 0x00)
        Serial.println("RELEASED");
      else if (knobPressState == 0x40)
        Serial.println("PRESSED RIGHT");
    }
  }

  if (current.knobPressedDown != newKnobPressedDown)
  {
    current.knobPressedDown = newKnobPressedDown;

    if (statusDebugMode)
    {
      Serial.print("0x25B: Knob ");
      if (knobPressState == 0x00)
        Serial.println("RELEASED");
      else if (knobPressState == 0x70)
        Serial.println("PRESSED DOWN");
    }
  } // Handle BACK button (data[4])
  bool newBackPressed = (backButtonState == 0x20);
  bool newBackTouched = (backButtonState == 0x80);
  if (current.backButtonPressed != newBackPressed || current.backButtonTouched != newBackTouched)
  {
    current.backButtonPressed = newBackPressed;
    current.backButtonTouched = newBackTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: BACK button ");
      if (backButtonState == 0x00)
        Serial.println("RELEASED");
      else if (backButtonState == 0x20)
        Serial.println("PRESSED");
      else if (backButtonState == 0x80)
        Serial.println("TOUCHED");
    }
  }

  // Handle COM button (data[5])
  bool newComPressed = (comButtonState == 0x08);
  bool newComTouched = (comButtonState == 0x20);

  // Handle OPTION button (data[5] - different values)
  bool newOptionPressed = (comButtonState == 0x01);
  bool newOptionTouched = (comButtonState == 0x04);

  // Handle HOME button (data[4] - different values)
  bool newHomePressed = (backButtonState == 0x04);
  bool newHomeTouched = (backButtonState == 0x10);

  // Check for COM button state changes
  if (current.comButtonPressed != newComPressed || current.comButtonTouched != newComTouched)
  {
    current.comButtonPressed = newComPressed;
    current.comButtonTouched = newComTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: COM button ");
      if (comButtonState == 0x00)
        Serial.println("RELEASED");
      else if (comButtonState == 0x08)
        Serial.println("PRESSED");
      else if (comButtonState == 0x20)
        Serial.println("TOUCHED");
    }
  }

  // Check for OPTION button state changes
  if (current.optionButtonPressed != newOptionPressed || current.optionButtonTouched != newOptionTouched)
  {
    current.optionButtonPressed = newOptionPressed;
    current.optionButtonTouched = newOptionTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: OPTION button ");
      if (comButtonState == 0x00)
        Serial.println("RELEASED");
      else if (comButtonState == 0x01)
        Serial.println("PRESSED");
      else if (comButtonState == 0x04)
        Serial.println("TOUCHED");
    }
  }

  // Check for HOME button state changes
  if (current.homeButtonPressed != newHomePressed || current.homeButtonTouched != newHomeTouched)
  {
    current.homeButtonPressed = newHomePressed;
    current.homeButtonTouched = newHomeTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: HOME button ");
      if (backButtonState == 0x00)
        Serial.println("RELEASED");
      else if (backButtonState == 0x04)
        Serial.println("PRESSED");
      else if (backButtonState == 0x10)
        Serial.println("TOUCHED");
      else
      {
        Serial.print("UNKNOWN STATE (data[5]=0x");
        Serial.print(backButtonState, HEX);
        Serial.println(")");
      }
    }
  }

  // Handle MEDIA button (data[6])
  bool newMediaPressed = (mediaButtonState == 0xC1);
  bool newMediaTouched = (mediaButtonState == 0xC4);

  // Handle NAV button (data[6] - different values)
  bool newNavPressed = (mediaButtonState == 0xC8);
  bool newNavTouched = (mediaButtonState == 0xE0);

  if (current.mediaButtonPressed != newMediaPressed || current.mediaButtonTouched != newMediaTouched)
  {
    current.mediaButtonPressed = newMediaPressed;
    current.mediaButtonTouched = newMediaTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: MEDIA button ");
      if (mediaButtonState == 0xC0)
        Serial.println("RELEASED");
      else if (mediaButtonState == 0xC1)
        Serial.println("PRESSED");
      else if (mediaButtonState == 0xC4)
        Serial.println("TOUCHED");
    }
  }

  if (current.navButtonPressed != newNavPressed || current.navButtonTouched != newNavTouched)
  {
    current.navButtonPressed = newNavPressed;
    current.navButtonTouched = newNavTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: NAV button ");
      if (mediaButtonState == 0xC0)
        Serial.println("RELEASED");
      else if (mediaButtonState == 0xC8)
        Serial.println("PRESSED");
      else if (mediaButtonState == 0xE0)
        Serial.println("TOUCHED");
    }
  }

  // Handle MAP button (data[7])
  bool newMapPressed = (mapButtonState == 0xC1);
  bool newMapTouched = (mapButtonState == 0xC4);

  // Handle GLOBE button (data[7] - different values)
  bool newGlobePressed = (mapButtonState == 0xC8);
  bool newGlobeTouched = (mapButtonState == 0xE0);

  if (current.mapButtonPressed != newMapPressed || current.mapButtonTouched != newMapTouched)
  {
    current.mapButtonPressed = newMapPressed;
    current.mapButtonTouched = newMapTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: MAP button ");
      if (mapButtonState == 0xC0)
        Serial.println("RELEASED");
      else if (mapButtonState == 0xC1)
        Serial.println("PRESSED");
      else if (mapButtonState == 0xC4)
        Serial.println("TOUCHED");
    }
  }

  if (current.globeButtonPressed != newGlobePressed || current.globeButtonTouched != newGlobeTouched)
  {
    current.globeButtonPressed = newGlobePressed;
    current.globeButtonTouched = newGlobeTouched;

    if (statusDebugMode)
    {
      Serial.print("0x25B: GLOBE button ");
      if (mapButtonState == 0xC0)
        Serial.println("RELEASED");
      else if (mapButtonState == 0xC8)
        Serial.println("PRESSED");
      else if (mapButtonState == 0xE0)
        Serial.println("TOUCHED");
    }
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
