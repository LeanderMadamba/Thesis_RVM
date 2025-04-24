# RVM Project Wiring Diagram - Updated

This document provides the complete wiring connections for all components in the RVM project, including the communication between Raspberry Pi and Arduino.

## Components
- Raspberry Pi 4 Model B
- Arduino Mega 2560
- OV5647 Camera Module
- 20x4 I2C LCD Display Module
- Pushbutton
- SIM800L v2 GSM Module
- 6V DC Motor
- MG995R Servo Motor (360 degrees)
- MG996R Servo Motor (160 degrees)
- 2x HC-SR04 Ultrasonic Sensors
- 10kg Load Cell with HX711 Amplifier
- NPN Transistor (e.g., TIP120, 2N2222)
- 1K resistor
- 1N4001 diode (for motor protection)

## Power Supply Requirements
- Raspberry Pi: 5V/3A via USB-C
- Arduino Mega: 7-12V DC via barrel jack
- SIM800L v2: 3.7-4.2V DC (requires separate power supply)
- Servo Motors: 5-6V DC (separate power supply recommended)
- DC Motor: 6V DC
- HC-SR04 & HX711: 5V DC (can be powered from Arduino)

## Wiring Connections

### Raspberry Pi to Arduino Communication
| Raspberry Pi Pin | Arduino Mega Pin | Function |
|------------------|------------------|----------|
| GPIO 27 (Pin 13) | Digital Pin 4    | Classification signal (HIGH = plastic, LOW = waste) |
| GPIO 22 (Pin 15) | Digital Pin 5    | Ready signal from Arduino to Raspberry Pi |
| GND (Pin 6)      | GND              | Common ground |

### Raspberry Pi Components

#### OV5647 Camera Module
1. Connect to the Camera Serial Interface (CSI) connector on the Raspberry Pi
2. Blue side of the ribbon cable faces the Ethernet port

#### 20x4 I2C LCD Display
| LCD Pin | Raspberry Pi Pin |
|---------|------------------|
| VCC     | 5V (Pin 2 or 4)  |
| GND     | GND (Pin 6)      |
| SDA     | GPIO 2 (Pin 3)   |
| SCL     | GPIO 3 (Pin 5)   |

#### Pushbutton
| Button Pin | Raspberry Pi Pin |
|------------|------------------|
| Pin 1      | GPIO 17 (Pin 11) |
| Pin 2      | GND (Pin 9)      |

### Arduino Components

#### SIM800L v2 GSM Module
| SIM800L Pin | Arduino Mega Pin / Power Supply |
|-------------|--------------------------------|
| VCC         | External 3.7-4.2V power supply |
| GND         | GND                            |
| RXD         | Pin 18 (TX1)                   |
| TXD         | Pin 19 (RX1)                   |
| RST         | Pin 7                          |

#### DC Motor with Transistor
| Component      | Connection                           |
|----------------|--------------------------------------|
| Arduino Pin 6  | 1K resistor → Transistor Base        |
| Transistor Emitter | GND                              |
| Transistor Collector | Motor negative terminal        |
| Motor positive | External 6V power supply positive    |
| Power supply GND | GND (common with Arduino GND)      |
| 1N4001 diode   | Across motor terminals (cathode to positive) |

#### MG995R Servo Motor (360 degrees)
| Servo Pin     | Arduino Mega Pin / Power Supply |
|---------------|--------------------------------|
| Signal (Orange) | Pin 9 (PWM)                  |
| VCC (Red)     | External 5-6V Power Supply     |
| GND (Brown)   | GND                            |

#### MG996R Servo Motor (160 degrees)
| Servo Pin     | Arduino Mega Pin / Power Supply |
|---------------|--------------------------------|
| Signal (Orange) | Pin 10 (PWM)                 |
| VCC (Red)     | External 5-6V Power Supply     |
| GND (Brown)   | GND                            |

#### HC-SR04 Ultrasonic Sensor #1
| HC-SR04 Pin | Arduino Mega Pin |
|-------------|------------------|
| VCC         | 5V               |
| Trig        | Pin 24           |
| Echo        | Pin 25           |
| GND         | GND              |

#### HC-SR04 Ultrasonic Sensor #2
| HC-SR04 Pin | Arduino Mega Pin |
|-------------|------------------|
| VCC         | 5V               |
| Trig        | Pin 26           |
| Echo        | Pin 27           |
| GND         | GND              |

#### HX711 Load Cell Amplifier
| HX711 Pin     | Arduino Mega Pin / Load Cell |
|---------------|------------------------------|
| VCC           | 5V                           |
| GND           | GND                          |
| DT (Data)     | Pin 3                        |
| SCK (Clock)   | Pin 2                        |
| E+/E- & A+/A- | Connect to load cell (see below) |

#### 10kg Load Cell to HX711 Connections
| Load Cell Wire | HX711 Connection |
|----------------|------------------|
| Red            | E+               |
| Black          | E-               |
| White          | A+               |
| Green          | A-               |

## System Connection Diagram

```
                       +------------------+
                       |                  |
         +-------------+ Raspberry Pi 4   |
         |             |                  |
         |             +------------------+
         |                    |   |
         |                    |   |
         v                    v   v
+------------------+     +-------------+
|                  |     |             |
| OV5647 Camera    |     | I2C LCD     |
|                  |     |             |
+------------------+     +-------------+
                              ^
                              |
       GPIO 27 (Plastic Signal)
         |
         v
+------------------+
|                  |
| Arduino Mega     +-----------> SIM800L
|                  |
+------------------+
     |    |    |    |
     |    |    |    |
     v    v    v    v
   +----+    +----+
   |    |    |    |
+--+--+ | +--+--+ |
|Load | | |Servo| |
|Cell | | |Motors| |
+-----+ | +-----+ |
        |         |
     +--+--+   +--+--+
     |     |   |     |
     |Motor|   |Ultra|
     |     |   |sonic|
     +-----+   +-----+
```

## Safety Notes
1. Always double-check connections before powering on
2. Ensure common ground between Raspberry Pi, Arduino, and all power supplies
3. Do not exceed the current limits of GPIO pins (Raspberry Pi: 16mA per pin, Arduino: 40mA per pin)
4. Use appropriate heat sinks for the transistor if the motor draws significant current
5. For high-current devices (motors, GSM module), use separate power supplies 