# RVM Project Wiring Diagram - Updated

This document provides the complete wiring connections for all components in the RVM project, including the communication between Raspberry Pi and Arduino.

## Components
- Raspberry Pi 4 Model B
- Arduino Mega 2560
- OV5647 Camera Module
- 20x4 I2C LCD Display Module
- Pushbutton
- SIM800L v2 GSM Module
- L298N Motor Driver + DC Worm Gear Motor SGM-370 (6V)
- MG995R Servo Motor (360 degrees)
- MG996R Servo Motor (160 degrees)
- 2x HC-SR04 Ultrasonic Sensors
- 10kg Load Cell with HX711 Amplifier

## Power Supply Requirements
- Raspberry Pi: 5V/3A via USB-C
- Arduino Mega: 7-12V DC via barrel jack
- SIM800L v2: 3.7-4.2V DC (requires separate power supply)
- Servo Motors: 5-6V DC (separate power supply recommended)
- L298N Motor Driver: 12V DC for motor power
- HC-SR04 & HX711: 5V DC (can be powered from Arduino)

## Wiring Connections

### Raspberry Pi to Arduino Communication
| Raspberry Pi Pin | Arduino Mega Pin | Function |
|------------------|------------------|----------|
| USB Port         | USB Port         | Serial communication for commands |

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
| RXD         | Pin 19 (TX1)                   |
| TXD         | Pin 18 (RX1)                   |
| RST         | Pin 7                          |

#### L298N Motor Driver with DC Motor
| L298N Pin        | Arduino Mega Pin / Power Supply       |
|------------------|--------------------------------------|
| ENA              | Pin 6 (PWM)                          |
| IN1              | Pin 8                                |
| IN2              | Pin 7                                |
| OUT1             | DC Motor Terminal 1                  |
| OUT2             | DC Motor Terminal 2                  |
| +12V             | External 12V power supply positive   |
| GND              | GND (common with Arduino GND)        |
| 5V (if needed)   | Not connected (remove jumper if using external power) |

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

#### HC-SR04 Ultrasonic Sensor #1 (Container 1)
| HC-SR04 Pin | Arduino Mega Pin |
|-------------|------------------|
| VCC         | 5V               |
| Trig        | Pin 30           |
| Echo        | Pin 32           |
| GND         | GND              |

#### HC-SR04 Ultrasonic Sensor #2 (Container 2)
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
           USB Serial Communication
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
  +--+L298N|   |Ultra|
  |  |Motor|   |sonic|
  |  |Driver   |Sensor
  |  +-----+   +-----+
  |
  v
+-----+
|     |
|DC   |
|Motor|
+-----+
```

## Configuration Notes

### Load Cell Calibration
- The system uses a calibration factor of 750 for the load cell
- You may need to adjust this value based on your specific load cell and setup
- Use the WEIGHT_TEST command via serial to check and calibrate the load cell

### DC Motor Control
- The L298N driver allows for speed control via PWM and direction control
- Make sure the jumper on L298N is removed if using an external power supply
- The system uses pins 6 (PWM), 8 (IN1), and 7 (IN2) for motor control

### Ultrasonic Sensors
- Sensors are positioned to monitor container fullness
- Keep sensors away from soft or sound-absorbing surfaces
- The system is configured to detect obstructions when distance < 10cm
- Container heights are set to 90cm in the code (adjust based on your containers)

## Safety Notes
1. Always double-check connections before powering on
2. Ensure common ground between Raspberry Pi, Arduino, and all power supplies
3. Do not exceed the current limits of GPIO pins (Raspberry Pi: 16mA per pin, Arduino: 40mA per pin)
4. For high-current devices (motors, GSM module), use separate power supplies
5. The L298N motor driver can get hot during operation - ensure proper ventilation 