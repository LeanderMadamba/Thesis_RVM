# Raspberry Pi RVM Wiring Diagram

This document outlines the connections between the Raspberry Pi and the peripherals used in the Reverse Vending Machine (RVM) system.

## Components Connected to Raspberry Pi

1. **Push Button** - For user input to trigger image capture
2. **I2C LCD Display** - For displaying system status and classification results
3. **Camera Module** - For capturing images of recyclable items

## Pin Connections

### Push Button
| Raspberry Pi Pin | Button Pin | Description |
|------------------|------------|-------------|
| GPIO 17 (Pin 11) | Pin 1      | Input signal |
| GND (Pin 6)      | Pin 2      | Ground |

### I2C LCD Display (Address: 0x27)
| Raspberry Pi Pin | LCD Pin | Description |
|------------------|---------|-------------|
| GPIO 2 (Pin 3)   | SDA     | I2C Serial Data |
| GPIO 3 (Pin 5)   | SCL     | I2C Serial Clock |
| 5V (Pin 2 or 4)  | VCC     | Power supply |
| GND (Pin 6)      | GND     | Ground |

### Camera Module
The Raspberry Pi Camera Module connects directly to the Camera Serial Interface (CSI) connector on the Raspberry Pi board. This is a ribbon cable that connects to the dedicated camera port.

### Serial Connection to Arduino
The Raspberry Pi communicates with the Arduino through a USB connection. The Arduino USB cable connects directly to one of the USB ports on the Raspberry Pi.

| Connection | Description |
|------------|-------------|
| Raspberry Pi USB Port | Arduino USB Port via USB cable |

This USB connection provides both power to the Arduino and establishes a serial communication channel between the two devices.

## Notes
- The LCD display uses the I2C protocol, which requires only 4 pins (including power and ground)
- The push button requires a pull-up/down resistor configuration, which is handled internally by the Raspberry Pi's GPIO settings
- The Arduino handles all hardware control for servos, ultrasonic sensors, DC motors, and the load cell

## Workflow
1. Raspberry Pi initializes all connected components
2. LCD displays "Ready for item" message
3. User presses the button to capture an image
4. Image is captured and processed by the machine learning model
5. Classification result is shown on the LCD display
6. Classification command ("PLASTIC" or "WASTE") is sent to Arduino
7. Arduino controls hardware components based on classification
8. System returns to ready state for next item 