# L298N Motor Driver Wiring Diagram

## Components Needed:
- Arduino Mega 2560
- L298N Motor Driver Module
- DC Motor (6V)
- Power Supply (6-12V)
- Jumper Wires
- Optional: 5V power source for Arduino

## Wiring Instructions:

### 1. Power Connections:
```
L298N Module          Arduino Mega
+12V Input  --------->  External Power Supply (6-12V)
GND         --------->  Arduino GND
+5V Output  --------->  Arduino 5V (optional)
```

### 2. Control Signal Connections:
```
L298N Module          Arduino Mega
ENA (Enable A)  ----->  Digital Pin 6 (PWM)
IN1            ----->  Digital Pin 7
IN2            ----->  Digital Pin 8
```

### 3. Motor Connections:
```
L298N Module          DC Motor
OUT1           ----->  Motor Terminal 1
OUT2           ----->  Motor Terminal 2
```

## Complete Wiring Diagram:
```
                    +------------------+
                    |                  |
                    |  Arduino Mega    |
                    |                  |
                    +------------------+
                         |  |  |
                         |  |  |
                         |  |  |
                    +------------------+
                    |                  |
                    |  L298N Module    |
                    |                  |
                    +------------------+
                         |  |  |
                         |  |  |
                         |  |  |
                    +------------------+
                    |                  |
                    |  DC Motor        |
                    |                  |
                    +------------------+
```

## Power Supply Options:

### Option 1: Separate Power Supplies
```
External Power Supply (6-12V) -----> L298N +12V Input
                                    L298N GND -----> Arduino GND
Arduino USB Power -----------------> Arduino
```

### Option 2: Single Power Supply
```
External Power Supply (6-12V) -----> L298N +12V Input
                                    L298N +5V Output -----> Arduino 5V
                                    L298N GND -----> Arduino GND
```

## Important Notes:

1. **Power Requirements:**
   - L298N input voltage: 6V to 12V
   - Motor voltage should match your DC motor rating (6V in your case)
   - Current rating should be sufficient for your motor

2. **Enable Jumper:**
   - The L298N module has a jumper on the ENA pin
   - Remove this jumper when using PWM control
   - Keep it in place if you want the motor to run at full speed

3. **Heat Sink:**
   - The L298N module comes with a heat sink
   - Ensure it's properly attached
   - Consider adding thermal paste for better heat dissipation

4. **Protection:**
   - The module has built-in protection diodes
   - Still, avoid sudden direction changes
   - Use the rampSpeed() function in the code for smooth operation

5. **Troubleshooting:**
   - If motor doesn't move, check power connections
   - If motor runs at full speed only, check ENA jumper
   - If motor runs in wrong direction, swap OUT1 and OUT2 connections

## Safety Precautions:

1. Always disconnect power when making connections
2. Double-check all connections before powering up
3. Start with low speeds when testing
4. Monitor motor temperature during operation
5. Use appropriate wire gauge for power connections
6. Keep the L298N module away from moisture

## Testing the Setup:

1. Upload the L298N_Motor_Test.ino sketch
2. Open Serial Monitor (9600 baud)
3. Send commands:
   - 'f' for forward
   - 'r' for reverse
   - 's' to stop
   - 't' for test sequence
   - 'h' for help

Remember to start with low speeds and gradually increase to ensure everything is working properly. 