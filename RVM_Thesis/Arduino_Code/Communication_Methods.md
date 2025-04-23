# RVM Project - Communication Methods

This document explains the two methods for communication between the Raspberry Pi and Arduino Mega in the RVM project.

## Available Communication Methods

### 1. GPIO Communication
Uses direct GPIO pins for digital signaling between the devices.

### 2. Serial (USB) Communication
Uses the Arduino's USB connection for serial data transfer.

## GPIO Communication

### Setup
- **Files**: `model_inference3_gpio.py` on Raspberry Pi, `RVM_Controller.ino` on Arduino
- **Connections**:
  - Raspberry Pi GPIO 27 → Arduino Digital Pin 4 (Classification signal)
  - Raspberry Pi GPIO 22 ← Arduino Digital Pin 5 (Ready signal)
  - Raspberry Pi GND ↔ Arduino GND (Common ground)
- **Signal Logic**:
  - HIGH (1) signal = Plastic detected
  - LOW (0) signal = Waste detected
  - HIGH on ready pin = Arduino is ready for next item

### Pros
- Simple and robust implementation
- Less processing overhead
- More direct hardware control
- Minimal latency
- Doesn't interfere with serial debugging on Arduino

### Cons
- Limited to binary signals (HIGH/LOW)
- Requires physical wiring between pins
- Uses up GPIO pins on both devices
- Needs common ground reference

## Serial Communication

### Setup
- **Files**: `model_inference3_serial.py` on Raspberry Pi, `RVM_Controller_Serial.ino` on Arduino
- **Connections**:
  - USB cable from Raspberry Pi to Arduino Mega
- **Command Protocol**:
  - Commands sent as text strings ending with newline character (`\n`)
  - Arduino responses begin with status tags (READY, BUSY, INFO, ERROR, WARNING)
  - Communication Format: `COMMAND\n` → `STATUS: Response message`
  - Example: `PLASTIC\n` → `BUSY: Processing plastic item`

### Pros
- Single-cable connection
- Can transmit complex data and commands
- Arduino receives power through USB connection
- Easier debugging and monitoring
- Extensible for future command additions

### Cons
- More complex implementation
- Potential for communication errors or buffer issues
- Serial port might vary (`/dev/ttyACM0`, `/dev/ttyACM1`, etc.)
- Serial monitor in Arduino IDE cannot be used simultaneously

## Command Reference for Serial Communication

| Command | Description | Arduino Response |
|---------|-------------|------------------|
| `HELLO` | Initial handshake | `READY: Hello from Arduino RVM Controller` |
| `STATUS` | Check Arduino status | `READY: System is ready for next command` or `BUSY: System is currently processing` |
| `PLASTIC` | Process a plastic item | `BUSY: Processing plastic item` followed by status updates and finally `READY: Plastic processing complete` |
| `WASTE` | Process a waste item | `BUSY: Processing waste item` followed by status updates and finally `READY: Waste processing complete` |
| `SHUTDOWN` | Prepare for system shutdown | `INFO: Shutting down...` |

## How to Choose

### Use GPIO Communication if:
- You want a simpler, more direct implementation
- You need minimal latency
- You want to maintain serial debugging capability on Arduino
- You're comfortable with physical wiring between the devices

### Use Serial Communication if:
- You prefer a single-cable connection
- You want easier setup and debugging
- You anticipate adding more complex commands in the future
- You want to power the Arduino through the same connection
- You need to send more detailed data between devices

## Switching Between Methods

To switch between communication methods:
1. Upload the corresponding Arduino sketch based on your chosen method
2. Run the corresponding Python script on the Raspberry Pi
3. Make sure the appropriate connections are in place
4. For serial communication, you may need to identify the correct serial port (try `ls /dev/tty*` on Raspberry Pi) 