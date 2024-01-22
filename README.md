# MCP23017Control Class

The `MCP23017Control` class provides an interface for interacting with the MCP23017 I/O expander chip over I²C on Arduino platforms. This class includes methods for initializing the chip, configuring pin modes, and reading/writing digital values to individual pins or entire banks.

## Constructor

- **MCP23017Control(byte address = 0x20)**
  - `address`: I²C address of the MCP23017. Default is `0x20`.

## Methods

### begin(bool initWire = true)

Initializes the MCP23017 I/O expander. This method sets up the I²C communication and configures all pins on the MCP23017 as inputs for safety.

- `initWire`: A boolean parameter that determines whether the Wire (I²C) library should be initialized within this method. Defaults to `true`.
  - If `true`, `Wire.begin()` will be called to initialize the I²C communication.
  - If `false`, `Wire.begin()` will not be called, allowing for manual initialization of the I²C bus elsewhere in your code.

### pinMode(byte pinNumber, byte mode)

Sets the mode of a specific pin.

- `pinNumber`: Pin number (0-15).
- `mode`: Mode of the pin (`INPUT`, `OUTPUT`, `INPUT_PULLUP`).

### digitalWrite(byte pinNumber, byte state)

Writes a digital value (HIGH or LOW) to a specified output pin. If the pin is set as an input, it automatically changes the pin mode to `INPUT` or `INPUT_PULLUP`.

- `pinNumber`: Pin number (0-15).
- `state`: State to write (HIGH or LOW).

### digitalRead(byte pinNumber)

Reads and returns the state (HIGH or LOW) of a specified input pin.

- `pinNumber`: Pin number (0-15).

### setPinHigh(byte outputNumber)

Sets a specified output pin to high.

- `outputNumber`: Pin number (0-15).

### setBankMode(byte bank, byte mode)

Sets the mode for all pins in a specified bank (Bank A or Bank B) of the MCP23017.

- `bank`: Bank number (0 for Bank A, 1 for Bank B).
- `mode`: Mode to set for all pins in the specified bank (`INPUT`, `OUTPUT`, `INPUT_PULLUP`).

### writeBank(byte bank, byte value)

Writes a byte value to all pins of either Bank A or Bank B.

- `bank`: Bank number (0 for Bank A, 1 for Bank B).
- `value`: Byte value to write to the bank.

### readBank(byte bank)

Reads and returns the state of all pins in either Bank A or Bank B.

- `bank`: Bank number (0 for Bank A, 1 for Bank B).

### setAllPinsMode(byte mode)

Sets the mode for all pins across both Bank A and Bank B of the MCP23017.

- `mode`: Mode to set for all pins (`INPUT`, `OUTPUT`, `INPUT_PULLUP`).
  
### readAllPins()

Returns the combined state of all 16 outputs as a 16-bit value.

### writeAllPins(uint16_t value)

Writes a 16-bit value to set the states of all 16 outputs.

- `value`: 16-bit value representing the state of each pin.

### static uint16_t getBitmask(byte bitPosition)

Returns a 16-bit number with only one bit set, based on the specified position.

- `bitPosition`: Position of the bit to set (0-15).

## Usage Examples

### Initializing the MCP23017

```cpp
#include "MCP23017Control.h"

MCP23017Control mcp(0x20); // Replace 0x20 with your MCP23017's I2C address

void setup() {
    mcp.begin(); // Initialize the MCP23017 with default Wire.begin()
    // or
    mcp.begin(false); // Skip Wire.begin() if I²C has been initialized elsewhere
}
```

### Configuring Pin Modes

```cpp
// Set pin 0 as an output and pin 8 as an input with pull-up
mcp.pinMode(0, OUTPUT);
mcp.pinMode(8, INPUT_PULLUP);
```

### Writing to and Reading from Pins

```cpp
// Write HIGH to pin 0 (e.g., to turn on an LED)
mcp.digitalWrite(0, HIGH);

// Read the state of pin 8
byte state = mcp.digitalRead(8);
if (state == HIGH) {
    // Pin 8 is HIGH
}
```

### Setting a Specific Pin High

```cpp
// Set pin 3 to HIGH
mcp.setPinHigh(3);
```

### Configuring a Bank Mode

```cpp
// Set all pins in Bank A to OUTPUT
mcp.setBankMode(0, OUTPUT);

// Set all pins in Bank B to INPUT_PULLUP
mcp.setBankMode(1, INPUT_PULLUP);
```

### Writing to and Reading from Banks

```cpp
// Write a byte value to Bank A
mcp.writeBank(0, 0xFF); // Set all pins in Bank A to HIGH

// Read the current state of Bank B
byte bankBState = mcp.readBank(1);
```

### Configuring All Pins Mode 

```cpp
// Set all pins to be inputs with pull-up resistors
mcp.setAllPinsMode(INPUT_PULLUP);
```

### Reading the State of All Pins

```cpp
uint16_t allAllPins = mcp.readAllPins();
```

### Writing to All Pins

```cpp
// Set the first 8 pins to HIGH and the rest to LOW
mcp.writeAllPins(0x00FF

);
```

### Using the getBitmask Utility

```cpp
// Get a bitmask with only the 5th bit set
uint16_t bitmask = MCP23017Control::getBitmask(5);
