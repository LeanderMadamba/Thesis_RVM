# RVM Project - Arduino Components Test Suite

This folder contains test sketches for the Arduino components used in the RVM (Reverse Vending Machine) project, along with the main integrated controller.

## Hardware Requirements

- Arduino Mega 2560
- SIM800L v2 GSM Module
- L298N Motor Driver + DC Worm Gear Motor SGM-370
- MG995R Servo Motor (360 degrees)
- MG996R Servo Motor (160 degrees)
- 2x HC-SR04 Ultrasonic Sensors
- 10kg Load Cell with HX711 Amplifier
- Power supplies:
  - 3.7-4.2V for SIM800L module
  - 5-6V for servo motors
  - 12V for L298N motor driver
  - 5V for ultrasonic sensors and HX711

## Installation

1. Make sure you have the Arduino IDE installed
2. Install required libraries:
   - HX711 library by Bogdan Necula (for load cell)
   - Servo library (included with Arduino IDE)
   - SoftwareSerial library (included with Arduino IDE)
3. Connect the components according to the wiring diagram in `Arduino_Wiring_Diagram_Updated.md`
4. Load and run the individual test sketches to verify each component or use the integrated RVM_Controller_Serial.ino file

## Main Controller

### RVM_Controller_Serial.ino
The main integrated controller that:
1. Receives commands from Raspberry Pi via USB serial
2. Processes "PLASTIC" or "WASTE" commands
3. Measures weight using load cell (for plastic items)
4. Controls servo mechanisms based on weight
5. Controls DC motor using L298N driver
6. Monitors ultrasonic sensors for container fullness and obstructions
7. Sends SMS notification if obstructions detected

**Supported Commands:**
- `HELLO` - Handshake to establish connection
- `STATUS` - Check if the system is ready or busy
- `PLASTIC` - Process a plastic item (includes weight measurement)
- `WASTE` - Process a waste item (bypasses weight measurement)
- `SHUTDOWN` - Prepare for system shutdown
- `CHECK_CONTAINERS` - Check container fullness with ultrasonic sensors
- `CHECK_OBSTRUCTIONS` - Check for obstructions in the system
- `MOTOR_TEST` - Run a test sequence on the DC motor
- `WEIGHT_TEST` - Test the load cell weight sensor

## Component Test Sketches

### 1. GSM_Module_Test.ino
Tests the SIM800L v2 GSM module by sending an SMS message to a specified phone number.
- **Key Function:** `sendSMS()`
- **Requirements:** Replace the dummy phone number with your actual phone number
- **Output:** Serial monitor displays the module's response and confirmation when message is sent

### 2. Servo_Motors_Test.ino
Tests both the 360-degree continuous rotation servo (MG995R) and the standard 160-degree servo (MG996R).
- **Key Functions:** `testContinuousServo()` and `testStandardServo()`
- **Output:** Serial monitor displays progress through the test sequence

### 3. Ultrasonic_Sensors_Test.ino (Thesis_containers.ino)
Tests two HC-SR04 ultrasonic sensors by measuring distances and displaying the values.
- **Key Function:** `getUltrasonicDistance()`, `checkContainerFullness()`
- **Output:** Serial monitor shows distance readings and container fullness percentage
- **Special Features:** Detects if containers remain full for consecutive seconds to verify status

### 4. Load_Cell_Test.ino
Tests the 10kg load cell with HX711 amplifier for weight measurement.
- **Requirements:** Calibration factor set to 750, may need adjustment based on your load cell
- **Output:** Serial monitor displays weight readings
- **Special Features:** Sends 't' in serial monitor to tare the scale

### 5. DC_Motor_Test.ino (dc_motorz.ino)
Tests the DC motor using an L298N motor driver by controlling speed and direction.
- **Key Functions:** Forward/Reverse control, speed adjustment
- **Connections:** Uses pins 6 (ENA), 8 (IN1), 7 (IN2) for motor control
- **Output:** Serial monitor shows the current operation and speed

## Wiring

See `Arduino_Wiring_Diagram_Updated.md` for detailed wiring instructions.

## Power Supply Considerations

- Each component has specific power requirements. Review the wiring diagram for details.
- Always ensure common ground connections between the Arduino and all power supplies.
- Use separate power supplies for high-current components (motors, GSM module) to prevent voltage drops.
- For the L298N motor driver, remove the 5V jumper if using an external power supply.

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
- For container fullness detection, ensure sensors are mounted securely at the top of containers

### Load Cell
- If weight readings are inconsistent, try increasing the number of samples
- Current calibration factor is 750, adjust as needed for your specific load cell
- Make sure the load cell is properly mounted with all force directed to the sensor

### L298N Motor Driver
- If the motor doesn't run, check all connections and the external power supply
- The L298N module has a jumper to enable/disable the onboard 5V regulator (remove if using external power)
- Check that the IN1/IN2 pins are correctly connected for proper direction control 