// MCP23017Control.h

#ifndef MCP23017CONTROL_H
#define MCP23017CONTROL_H

#include <Arduino.h>
#include <Wire.h>

struct MCP23017Registers {
  static const byte IODIRA = 0x00;
  static const byte IODIRB = 0x01;
  static const byte GPIOA = 0x12;
  static const byte GPIOB = 0x13;
  static const byte GPPUA = 0x0C;
  static const byte GPPUB = 0x0D;
  // ... other registers ...
};

class MCP23017Control {
public:
  MCP23017Control(byte address = 0x20)
    : _address(address) {}

  void begin(bool initWire = true) {
    if (initWire) {
      Wire.begin();
    }
    writeRegister(MCP23017Registers::IODIRA, 0xFF);  // Set all Bank A pins as inputs
    writeRegister(MCP23017Registers::IODIRB, 0xFF);  // Set all Bank B pins as inputs
  }

  void pinMode(byte pinNumber, byte mode) {
    byte iodirReg = (pinNumber < 8) ? MCP23017Registers::IODIRA : MCP23017Registers::IODIRB;
    byte gppuReg = (pinNumber < 8) ? MCP23017Registers::GPPUA : MCP23017Registers::GPPUB;
    if (pinNumber >= 8) pinNumber -= 8;  // Adjust for Bank B

    byte iodirVal = readRegister(iodirReg);
    byte gppuVal = readRegister(gppuReg);

    if (mode == INPUT || mode == INPUT_PULLUP) {
      iodirVal |= (1 << pinNumber);  // Set pin as input
      gppuVal = (mode == INPUT_PULLUP) ? (gppuVal | (1 << pinNumber)) : (gppuVal & ~(1 << pinNumber));
    } else {                          // OUTPUT
      iodirVal &= ~(1 << pinNumber);  // Set pin as output
    }

    writeRegister(iodirReg, iodirVal);
    writeRegister(gppuReg, gppuVal);
  }

  void digitalWrite(byte pinNumber, byte state) {
    byte iodirReg = (pinNumber < 8) ? MCP23017Registers::IODIRA : MCP23017Registers::IODIRB;
    byte gpioReg = (pinNumber < 8) ? MCP23017Registers::GPIOA : MCP23017Registers::GPIOB;
    if (pinNumber >= 8) pinNumber -= 8;  // Adjust for Bank B

    byte iodirVal = readRegister(iodirReg);
    byte gpioVal = readRegister(gpioReg);

    if (iodirVal & (1 << pinNumber)) {  // If pin is an input
      byte gppuReg = (pinNumber < 8) ? MCP23017Registers::GPPUA : MCP23017Registers::GPPUB;
      byte gppuVal = readRegister(gppuReg);
      if (state == HIGH) {
        gppuVal |= (1 << pinNumber);  // Enable pull-up resistor
      } else {
        gppuVal &= ~(1 << pinNumber);  // Disable pull-up resistor
      }
      writeRegister(gppuReg, gppuVal);
    } else {  // If pin is an output
      if (state == HIGH) {
        gpioVal |= (1 << pinNumber);
      } else {
        gpioVal &= ~(1 << pinNumber);
      }
      writeRegister(gpioReg, gpioVal);
    }
  }

  byte digitalRead(byte pinNumber) {
    byte regAddress = (pinNumber < 8) ? MCP23017Registers::GPIOA : MCP23017Registers::GPIOB;
    if (pinNumber >= 8) pinNumber -= 8;  // Adjust for Bank B
    byte regValue = readRegister(regAddress);
    return (regValue & (1 << pinNumber)) ? HIGH : LOW;
  }

  void writeBank(byte bank, byte value) {
    byte regAddress = (bank == 0) ? MCP23017Registers::GPIOA : MCP23017Registers::GPIOB;
    writeRegister(regAddress, value);
  }

  byte readBank(byte bank) {
    byte regAddress = (bank == 0) ? MCP23017Registers::GPIOA : MCP23017Registers::GPIOB;
    return readRegister(regAddress);
  }

  uint16_t readOutputs() {
    uint8_t gpioAValue = readRegister(MCP23017Registers::GPIOA);
    uint8_t gpioBValue = readRegister(MCP23017Registers::GPIOB);
    return ((uint16_t)gpioBValue << 8) | gpioAValue;
  }

  void setPinHigh(byte outputNumber) {
    digitalWrite(outputNumber, HIGH);
  }

  void writeOutputs(uint16_t value) {
    byte gpioAValue = value & 0xFF;
    byte gpioBValue = (value >> 8) & 0xFF;
    writeRegister(MCP23017Registers::GPIOA, gpioAValue);
    writeRegister(MCP23017Registers::GPIOB, gpioBValue);
  }

  static uint16_t getBitmask(byte bitPosition) {
    return (bitPosition < 16) ? (1 << bitPosition) : 0;
  }

private:
  byte _address;

  void writeRegister(byte regAddress, byte regValue) {
    Wire.beginTransmission(_address);
    Wire.write(regAddress);
    Wire.write(regValue);
    Wire.endTransmission();
  }

  byte readRegister(byte regAddress) {
    Wire.beginTransmission(_address);
    Wire.write(regAddress);
    Wire.endTransmission();
    Wire.requestFrom((int)_address, 1);
    return Wire.read();
  }
};

#endif
