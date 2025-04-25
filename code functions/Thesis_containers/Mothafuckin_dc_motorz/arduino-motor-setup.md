# Arduino Mega 2560 + L298N + DC Motor Setup Guide

This guide provides instructions for connecting and testing a DC worm gear motor (SGM-370 12V 6rpm) with an Arduino Mega 2560 using an L298N motor driver.

## Hardware Requirements

- Arduino Mega 2560
- L298N Motor Driver Module
- DC Worm Gear Motor SGM-370 (12V, 6rpm)
- 12V DC Power Supply for the motor
- Jumper wires
- USB cable for Arduino

## Wiring Connections

### Power Connections
- Connect the positive terminal (+) of your 12V power supply to the VCC/+12V terminal on the L298N
- Connect the negative terminal (-) of your power supply to the GND terminal on the L298N
- Connect a wire from any GND pin on the Arduino Mega to the GND on the L298N

### Motor Connections
- Connect one wire from your DC motor to OUT1 on the L298N
- Connect the other wire from your DC motor to OUT2 on the L298N

### Control Connections
- Connect Arduino pin 9 to ENA on the L298N (speed control via PWM)
- Connect Arduino pin 8 to IN1 on the L298N (direction control)
- Connect Arduino pin 7 to IN2 on the L298N (direction control)

## Wiring Diagram

```
Arduino Mega 2560                L298N                      DC Motor
+---------------+            +----------+                 +----------+
|               |            |          |                 |          |
| Pin 9 (PWM) -----> ENA     |          |                 |          |
|               |            |          |                 |          |
| Pin 8 ---------> IN1       |          |                 |          |
|               |            |          |    OUT1 --------|          |
| Pin 7 ---------> IN2       |          |                 |          |
|               |            |          |    OUT2 --------|          |
| GND -----------> GND       |          |                 |          |
|               |            |          |                 |          |
+---------------+            +----------+                 +----------+
                                 ^
                                 |
                            12V Power Supply
                                 |
                                 v
                            +----------+
                            |   12V    |
                            | DC Power |
                            +----------+
```

## Important Notes

1. The L298N module may have a 5V output which can power your Arduino if needed
2. Make sure to remove the jumper on the L298N if you're using an external 12V power supply
3. If your motor runs in the opposite direction than expected, swap the wires connected to OUT1 and OUT2

## Using the Test Code

1. **Upload the Code:**
   - Connect your Arduino Mega to your computer via USB
   - Open the provided test script in the Arduino IDE
   - Select the correct board (Arduino Mega 2560) and port
   - Upload the code to your Arduino

2. **Monitor Operation:**
   - Open the Serial Monitor in the Arduino IDE (set to 9600 baud)
   - You'll see status messages as the motor runs through its test sequence

3. **Test Sequence:**
   - Forward rotation at increasing speeds
   - Brief stop
   - Backward rotation at increasing speeds  
   - Brief stop
   - Direction changes at medium speed
   - Complete stop before repeating the sequence

## Troubleshooting

- **Motor doesn't move:**
  - Check all connections
  - Verify the 12V power supply is connected correctly and turned on
  - Ensure the L298N enable jumper is set correctly for your power setup

- **Motor only moves in one direction:**
  - Check the IN1 and IN2 connections
  - Verify the code is properly setting the direction pins

- **Motor runs but speed control doesn't work:**
  - Ensure ENA is connected to a PWM-capable pin (pin 9)
  - Check if the L298N has any required jumpers for PWM operation

- **Arduino doesn't respond or upload fails:**
  - Check USB connection
  - Verify correct board and port selection in Arduino IDE
  - Reset the Arduino and try again

## Code Explanation

The test code cycles the motor through different speeds and directions:

1. **Setup phase:**
   - Initializes all pins and serial communication
   - Stops the motor initially for safety

2. **Main loop:**
   - Runs the motor forward at gradually increasing speeds
   - Stops the motor briefly
   - Runs the motor backward at gradually increasing speeds
   - Stops the motor briefly
   - Tests direction changes at a moderate speed
   - Stops completely before repeating the test

This allows you to verify that speed control and direction control are both functioning properly.
