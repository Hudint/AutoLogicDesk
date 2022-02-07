#include <Arduino.h>
#include <SoftwareSerial.h> //including the SoftwareSerial library will allow you to use the pin no. 2,3 as Rx, Tx.
#include "WiFiEsp.h"

// DEFINES //////////////////////////

// From motor controller to Arduino
#define PIN_SIGNAL_IN 3 // Must be interrupt pin, depends on board! (YELLOW)
#define PIN_HS1_IN 4    // (GREEN)
#define PIN_HS4_IN 5    // (BLUE)
#define PIN_HS3_IN 6
#define PIN_HS2_IN 7

// From hand switch to Arduino
#define PIN_SIGNAL_OUT 8 // (YELLOW)
#define PIN_HS1_OUT 9    // (GREEN)
#define PIN_HS4_OUT 10   // (BLUE)
#define PIN_HS3_OUT 11
#define PIN_HS2_OUT 12

// Interrupts
#define INTERRUPT_SIGNAL_IN 3 // Must match pin PIN_SIGNAL_IN above!

// Directions
#define DIR_NONE 0
#define DIR_UP 1
#define DIR_DOWN 2

// Constants
#define MAX_ULONG 4294967295 // Maximum value an unsigned long can have
#define CYCLE_TIME_US 1000   // 1 ms
#define ARRAY_SIZE 32        // Size of the array that stores received bits
#define MATCH_ARRAY_SIZE 23  // Size of the array that contains the bit pattern to compare to
#define BIT_INDEX_HEIGHT 23  // Index at which the height value starts

#define SERIAL_ENABLED false

// Messages
#define MSG_ID_SET_HEIGHT_REQUEST 0x01
#define MSG_ID_STOP_REQUEST 0x02
#define MSG_ID_MOVE_UP 0x03
#define MSG_ID_MOVE_DOWN 0x04
#define MSG_ID_MOVE_PRE 0x05

// Length (not including the CRC byte!)
#define MESSAGE_LENGTH 2

// Device ID (should be unique!)
#define DEVICE_ID 0xAA

// VARIABLES //////////////////////////

byte gCurBit = 0;
byte gCurHeight = 0;
byte gNumMatchingBits = 0;
byte gTargetHeight = 0;
byte gCurStatus = 0;
byte gCurDirection = DIR_NONE;

bool gIsAutoHeightMode = true;
bool gIsAutoNumMode = false;
bool gIsSwitchOverride = false;
bool btnPressed = false;
volatile unsigned long lastChange = 0;
volatile unsigned long gCurTimeUs = 0;
volatile unsigned long gLastTimeUs = 0;

volatile unsigned long lastWemosSerial = 0;

SoftwareSerial wemos(2, 13);

// ARRAYS //////////////////////////

// Array for serial messages
byte gMessageBytes[MESSAGE_LENGTH + 1];

// Array that stores received bits
byte gBitArray[ARRAY_SIZE];

// Array that contains the bit pattern to compare to
byte gMatchArray[MATCH_ARRAY_SIZE] = {1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1};

// FUNCTIONS //////////////////////////

void handleCurrentSignalBit();
void handleSerialMessaging();
void handleReceivedBit();
void handleSwitchInputs();
void controlTableMovement();
void extractHeightFromBitArray();
void onEdgeEvent();
void clearMessageArray();
void moveTableUp();
void moveTableDown();
void stopTable();
bool checkTime();
int8_t toState(bool high);
void sendSerial(const String &s);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Init serial connection
  if (SERIAL_ENABLED)
    Serial.begin(9600);
  wemos.begin(9600);

  // Setup pins
  pinMode(PIN_SIGNAL_IN, INPUT);
  pinMode(PIN_HS1_IN, INPUT);
  pinMode(PIN_HS4_IN, INPUT);
  pinMode(PIN_HS2_IN, INPUT);
  pinMode(PIN_HS3_IN, INPUT);

  pinMode(PIN_SIGNAL_OUT, OUTPUT);
  pinMode(PIN_HS1_OUT, OUTPUT);
  pinMode(PIN_HS4_OUT, OUTPUT);
  pinMode(PIN_HS3_OUT, OUTPUT);
  pinMode(PIN_HS2_OUT, OUTPUT);

  // Set initial values
  digitalWrite(PIN_HS1_OUT, LOW);
  digitalWrite(PIN_HS4_OUT, LOW);
  digitalWrite(PIN_HS3_OUT, LOW);
  digitalWrite(PIN_HS2_OUT, LOW);

  // Init interrupt
  attachInterrupt(INTERRUPT_SIGNAL_IN, onEdgeEvent, CHANGE);

  sendSerial("Started");

  gLastTimeUs = micros();
}

void loop()
{

  // Make sure loop is only executed every 1ms
  if (!checkTime())
    return;

  // Receive and execute messages from the PC
  handleSerialMessaging();

  // Read the current height signal bit and interpret it
  handleCurrentSignalBit();

  // Read input from the real switches (can override PC)
  handleSwitchInputs();

  // Move the table up, down or stop it
  controlTableMovement();
}

// Read input from the real switches (can override PC)
void handleSwitchInputs()
{
  int HS1Pressed = digitalRead(PIN_HS1_IN);
  int HS4Pressed = digitalRead(PIN_HS4_IN);
  int HS2Pressed = digitalRead(PIN_HS2_IN);
  int HS3Pressed = digitalRead(PIN_HS3_IN);

  int pressed = 0;
  if (HS1Pressed)
  {
    pressed++;
    // sendSerial("HS1");
  }
  if (HS4Pressed)
  {
    pressed++;
    // sendSerial("HS4");
  }
  if (HS2Pressed)
  {
    pressed++;
    // sendSerial("HS2");
  }
  if (HS3Pressed)
  {
    pressed++;
    // sendSerial("HS3");
  }

  btnPressed = (pressed > 0);

  if (btnPressed)
    lastChange = millis();

  if (gIsSwitchOverride)
  {
    if (!HS1Pressed && !HS4Pressed)
    {
      stopTable();
      gIsSwitchOverride = false;
      gIsAutoNumMode = false;
    }
  }
  else
  {
    if (HS1Pressed && pressed == 1)
    {
      //sendSerial("Up");
      moveTableUp();
      gIsSwitchOverride = true;
      gIsAutoHeightMode = false;
      gIsAutoNumMode = false;
    }
    else if (HS4Pressed && pressed == 1)
    {
      moveTableDown();
      gIsSwitchOverride = true;
      gIsAutoHeightMode = false;
      gIsAutoNumMode = false;
    }

    if (pressed == 2 && (HS1Pressed && HS4Pressed))
    {
      digitalWrite(PIN_HS1_OUT, HIGH);
      digitalWrite(PIN_HS4_OUT, HIGH);

      gIsSwitchOverride = true;
      gIsAutoHeightMode = false;
      gIsAutoNumMode = false;
    }

    if ((pressed == 2 && ((HS2Pressed && HS4Pressed) || (HS3Pressed && HS4Pressed))) || (pressed == 1 && (HS3Pressed || HS2Pressed)))
    {
      digitalWrite(PIN_HS2_OUT, toState(HS2Pressed));
      digitalWrite(PIN_HS3_OUT, toState(HS3Pressed));
      digitalWrite(PIN_HS4_OUT, toState(HS4Pressed));

      lastChange = millis();
      gIsAutoNumMode = true;
      gIsAutoHeightMode = false;
      gIsSwitchOverride = false;
    }
  }
}

// Move the table up, down or stop it
void controlTableMovement()
{
  // Don't interfere with the switch override
  if (gIsSwitchOverride)
    return;

  if (gIsAutoHeightMode)
  {
    if (gCurDirection == DIR_UP && gCurHeight >= gTargetHeight)
      stopTable();
    if (gCurDirection == DIR_DOWN && gCurHeight <= gTargetHeight)
      stopTable();
  }
  else if (gIsAutoNumMode && !btnPressed)
  {
    if ((millis() - lastChange) >= 1500)
    {
      stopTable();
      sendSerial("auto stop " + String(millis()) + " - " + String(lastChange));
    }
  }
}

int8_t toState(bool high)
{
  if (high)
    return HIGH;
  else
    return LOW;
}

bool checkTime()
{
  gCurTimeUs = micros();
  if (gCurTimeUs - gLastTimeUs >= CYCLE_TIME_US)
  {
    gLastTimeUs = gCurTimeUs;
    return true;
  }
  return false;
}

void handleCurrentSignalBit()
{
  // Grab the current sample and immediately inform the real switch
  gCurBit = digitalRead(PIN_SIGNAL_IN);
  digitalWrite(PIN_SIGNAL_OUT, gCurBit);
  handleReceivedBit();
}

void handleSerialMessaging()
{

  if (wemos.available())
  {
    sendSerial("getting data");
    if ((millis() - lastWemosSerial) >= 1000){
      sendSerial("clear");
      clearMessageArray();
    }

    while (wemos.available())
    {
      lastWemosSerial = millis();

      int current = 0;
      while (!(gMessageBytes[current] == 0x00))
        current++;

      // Insert new byte

      gMessageBytes[current] = wemos.read();

      sendSerial("0 " + String(gMessageBytes[0]));
      sendSerial("1 " + String(gMessageBytes[1]));

      if (!(current == (MESSAGE_LENGTH - 1)))
        return;

      if (gMessageBytes[0] == MSG_ID_SET_HEIGHT_REQUEST) // Set height request
      {
        // Move table to height
        if (!gIsSwitchOverride)
        {
          gIsAutoHeightMode = true;

          int change = 1;

          if ((gMessageBytes[1] - gCurHeight) > 1)
            change = change*-1;
          else
            change = change*1;

          gTargetHeight = gMessageBytes[1] + change;
          if (gCurHeight < gTargetHeight)
            moveTableUp();
          else if (gCurHeight > gTargetHeight)
            moveTableDown();
        }
      }

      else if (gMessageBytes[0] == MSG_ID_MOVE_PRE) // Set height request
      {
        // Move table to height
        if (!gIsSwitchOverride)
        {

          lastChange = millis();
          gIsAutoHeightMode = false;
          gIsAutoNumMode = true;

          int pre = gMessageBytes[1] - 1;

          int8_t presets[4][2] = {
              {PIN_HS3_OUT},
              {PIN_HS2_OUT},
              {PIN_HS2_OUT, PIN_HS4_OUT},
              {PIN_HS3_OUT, PIN_HS4_OUT},
          };

          if (pre >= 0 && pre < 4)
          {
            for (size_t i = 0; i < 2; i++)
            {
              if (presets[pre][i] != 0)
                digitalWrite(presets[pre][i], HIGH);
            }
          }
        }
      }

      else if (gMessageBytes[0] == MSG_ID_STOP_REQUEST) // Stop request
      {
        if (!gIsSwitchOverride)
          stopTable();
      }

      else if (gMessageBytes[0] == MSG_ID_MOVE_UP) // Move request
      {
        gIsAutoHeightMode = false;
        gIsAutoNumMode = false;
        moveTableUp();
      }

      else if (gMessageBytes[0] == MSG_ID_MOVE_DOWN) // Move request
      {
        gIsAutoHeightMode = false;
        gIsAutoNumMode = false;
        moveTableDown();
      }

      clearMessageArray();
    }
  }
}

void clearMessageArray()
{
  for (size_t i = 0; i < MESSAGE_LENGTH; i++)
  {
    gMessageBytes[i] = 0x00;
  }
}

void handleReceivedBit()
{
  // Shift the array
  for (byte i = 0; i < ARRAY_SIZE - 1; ++i)
    gBitArray[i] = gBitArray[i + 1];

  // Insert the new value
  gBitArray[ARRAY_SIZE - 1] = gCurBit;

  // Count the number of matching bits
  for (gNumMatchingBits = 0; gNumMatchingBits < MATCH_ARRAY_SIZE; gNumMatchingBits++)
    if (gBitArray[gNumMatchingBits] != gMatchArray[gNumMatchingBits])
      break;

  // Update the height if we have a match
  if (gNumMatchingBits >= MATCH_ARRAY_SIZE)
    extractHeightFromBitArray();
}

void extractHeightFromBitArray()
{
  // Extract the height value from the bit array
  byte newHeight = 0;
  for (byte i = 0; i < 8; ++i)
    newHeight |= (gBitArray[(BIT_INDEX_HEIGHT + i) % ARRAY_SIZE] << i);
  newHeight = ~newHeight;

  // Plausibility check 1: height must be in a valid range

  if (newHeight < 60 || newHeight > 120)
    return;

  sendSerial("Height " + String(newHeight));

  // Plausibility check 2: height must not differ more than 5cm from last value
  // if (gCurHeight == 0 || abs(gCurHeight - newHeight) < 5)
  // {

  // }
  if (!(gCurHeight == newHeight))
    lastChange = millis();
  gCurHeight = newHeight;
  wemos.write(newHeight);
}

void onEdgeEvent()
{
  // Synchronize wait time with edge
  gLastTimeUs = micros() - (CYCLE_TIME_US / 2);
}

// Moves the table
void moveTableUp()
{
  gCurDirection = DIR_UP;

  digitalWrite(PIN_HS4_OUT, LOW);
  digitalWrite(PIN_HS1_OUT, HIGH);
}

// Moves the table
void moveTableDown()
{
  gCurDirection = DIR_DOWN;

  digitalWrite(PIN_HS1_OUT, LOW);
  digitalWrite(PIN_HS4_OUT, HIGH);
}

// Stops the table
void stopTable()
{
  gCurDirection = DIR_NONE;

  digitalWrite(PIN_HS4_OUT, LOW);
  digitalWrite(PIN_HS1_OUT, LOW);
  digitalWrite(PIN_HS2_OUT, LOW);
  digitalWrite(PIN_HS3_OUT, LOW);

  gIsAutoHeightMode = false;
  gIsAutoNumMode = false;

  gTargetHeight = gCurHeight;
}

void sendSerial(const String &s)
{
  if (SERIAL_ENABLED)
    Serial.println(s);
}