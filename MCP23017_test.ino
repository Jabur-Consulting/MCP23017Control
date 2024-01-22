#include "MCP23017Control.h"

// Create two instances of MCP23017Control for each MCP23017 device
MCP23017Control mcp(0x20);  // Device 1: controls pins 0-15

void setup() {
  // Initialize both MCP23017 devices
  mcp.begin();

  // Set pin 0 as an output and pin 8 as an input with pull-up
  mcp.pinMode(0, OUTPUT);
  mcp.pinMode(8, INPUT_PULLUP);
}

void loop() {
  // Write HIGH to pin 0 (e.g., to turn on an LED)
  mcp.digitalWrite(0, HIGH);

  // Read the state of pin 8
  byte state = mcp.digitalRead(8);
  if (state == HIGH) {
    // Pin 8 is HIGH
  }

  // Write a byte value to Bank A
  mcp.writeBank(0, 0xFF);  // Set all pins in Bank A to HIGH

  // Read the current state of Bank B
  byte bankBState = mcp.readBank(1);

  uint16_t allOutputs = mcp.readOutputs();

  // Set pin 3 to HIGH
  mcp.setPinHigh(3);

  // Set the first 8 pins to HIGH and the rest to LOW
  mcp.writeOutputs(0x00FF);

  // Get a bitmask with only the 5th bit set
  uint16_t bitmask = MCP23017Control::getBitmask(5);
}