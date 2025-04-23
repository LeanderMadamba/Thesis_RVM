# RVM Project - Arduino Components Test Suite

This folder contains test sketches for the Arduino components used in the RVM (Reverse Vending Machine) project.

## Hardware Requirements

- Arduino Mega 2560
- SIM800L v2 GSM Module
- 6V DC Motor
- MG995R Servo Motor (360 degrees)
- MG996R Servo Motor (160 degrees)
- 2x HC-SR04 Ultrasonic Sensors
- 10kg Load Cell with HX711 Amplifier
- L298N Motor Driver (for DC motor)
- Power supplies:
  - 3.7-4.2V for SIM800L module
  - 5-6V for servo motors
  - 6V for DC motor
  - 5V for ultrasonic sensors and HX711

## Installation

1. Make sure you have the Arduino IDE installed
2. Install required libraries:
   - HX711 library by Bogdan Necula (for load cell)
   - Servo library (included with Arduino IDE)
   - SoftwareSerial library (included with Arduino IDE)
3. Connect the components according to the wiring diagram in `Arduino_Wiring_Diagram.md`
4. Load and run the test sketches individually to verify each component

## Test Sketches

### 1. GSM_Module_Test.ino
Tests the SIM800L v2 GSM module by sending an SMS message to a specified phone number.
- **Key Function:** `sendSMS()`
- **Requirements:** Replace the dummy phone number with your actual phone number
- **Output:** Serial monitor displays the module's response and confirmation when message is sent

### 2. Servo_Motors_Test.ino
Tests both the 360-degree continuous rotation servo (MG995R) and the standard 160-degree servo (MG996R).
- **Key Functions:** `testContinuousServo()` and `testStandardServo()`
- **Output:** Serial monitor displays progress through the test sequence

### 3. Ultrasonic_Sensors_Test.ino
Tests two HC-SR04 ultrasonic sensors by measuring distances and displaying the values.
- **Key Function:** `measureDistance()`
- **Output:** Serial monitor shows distance readings from both sensors

### 4. Load_Cell_Test.ino
Tests the 10kg load cell with HX711 amplifier for weight measurement.
- **Requirements:** Adjust the `CALIBRATION_FACTOR` based on calibration with a known weight
- **Output:** Serial monitor displays weight readings and raw values

### 5. DC_Motor_Test.ino
Tests a 6V DC motor using an L298N motor driver by controlling speed and direction.
- **Key Functions:** `testForward()`, `testReverse()`, `testAcceleration()`, `testDirectionChange()`
- **Output:** Serial monitor shows the current operation being performed

## Wiring

See `Arduino_Wiring_Diagram.md` for detailed wiring instructions.

## Power Supply Considerations

- Each component has specific power requirements. Review the wiring diagram for details.
- Always ensure common ground connections between the Arduino and all power supplies.
- Use separate power supplies for high-current components (motors, GSM module) to prevent voltage drops.

## Troubleshooting

### GSM Module
- If the module doesn't respond, check power supply voltage (must be 3.7-4.2V)
- Make sure you have a SIM card installed with no PIN code protection
- Check the serial connections (RX/TX might need to be swapped)

### Servo Motors
- If servos jitter, check your power supply - servos need a stable 5-6V
- For the continuous rotation servo, 90 degrees should stop rotation

### Ultrasonic Sensors
- Keep sensors away from soft or sound-absorbing surfaces
- The HC-SR04 has a range of approximately 2cm to 400cm

### Load Cell
- Follow the calibration procedure in the sketch comments
- Make sure the load cell is properly mounted with all force directed to the sensor

### DC Motor
- If the motor doesn't run, check all connections and the external power supply
- The L298N module may have a jumper to enable/disable the onboard 5V regulator 