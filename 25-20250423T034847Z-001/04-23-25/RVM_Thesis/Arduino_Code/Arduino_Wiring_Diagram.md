# Arduino Mega 2560 Wiring Connections

This document provides the wiring connections for all modules connected to the Arduino Mega 2560 for the RVM project.

## Components
- Arduino Mega 2560
- SIM800L v2 GSM Module
- 6V DC Motor
- MG995R Servo Motor (360 degrees)
- MG996R Servo Motor (160 degrees)
- 2x HC-SR04 Ultrasonic Sensors
- 10kg Load Cell with HX711 Amplifier

## Power Supply Requirements
- Arduino Mega: USB or 7-12V DC via barrel jack
- SIM800L v2: 3.7-4.2V DC (requires separate power supply)
- Servo Motors: 5-6V DC (separate power supply recommended for heavy loads)
- DC Motor: 6V DC
- HC-SR04 & HX711: 5V DC (can be powered from Arduino)

## Wiring Connections

### SIM800L v2 GSM Module
| SIM800L Pin | Arduino Mega Pin |
|-------------|------------------|
| VCC         | External 3.7-4.2V power supply |
| GND         | GND              |
| RXD         | Pin 18 (TX1)     |
| TXD         | Pin 19 (RX1)     |
| RST         | Pin 7            |

**Note**: The SIM800L v2 requires a dedicated 3.7-4.2V power supply with good current capability (at least 2A).

### DC Motor (with L298N Driver)
| L298N Pin   | Arduino Mega Pin / Power Supply |
|-------------|--------------------------------|
| ENA         | Pin 6 (PWM)                    |
| IN1         | Pin 22                         |
| IN2         | Pin 23                         |
| OUT1        | Motor Terminal 1               |
| OUT2        | Motor Terminal 2               |
| 12V         | External 6V Power Supply       |
| GND         | GND                            |

### MG995R Servo Motor (360 degrees)
| Servo Pin   | Arduino Mega Pin / Power Supply |
|-------------|--------------------------------|
| Signal (Orange) | Pin 9 (PWM)                |
| VCC (Red)   | External 5-6V Power Supply     |
| GND (Brown) | GND                            |

### MG996R Servo Motor (160 degrees)
| Servo Pin   | Arduino Mega Pin / Power Supply |
|-------------|--------------------------------|
| Signal (Orange) | Pin 10 (PWM)               |
| VCC (Red)   | External 5-6V Power Supply     |
| GND (Brown) | GND                            |

### HC-SR04 Ultrasonic Sensor #1
| HC-SR04 Pin | Arduino Mega Pin |
|-------------|------------------|
| VCC         | 5V               |
| Trig        | Pin 24           |
| Echo        | Pin 25           |
| GND         | GND              |

### HC-SR04 Ultrasonic Sensor #2
| HC-SR04 Pin | Arduino Mega Pin |
|-------------|------------------|
| VCC         | 5V               |
| Trig        | Pin 26           |
| Echo        | Pin 27           |
| GND         | GND              |

### HX711 Load Cell Amplifier
| HX711 Pin   | Arduino Mega Pin / Load Cell |
|-------------|------------------------------|
| VCC         | 5V                           |
| GND         | GND                          |
| DT (Data)   | Pin 3                        |
| SCK (Clock) | Pin 2                        |
| E+/E- & A+/A- | Connect to load cell (see below) |

### 10kg Load Cell to HX711 Connections
| Load Cell Wire | HX711 Connection |
|----------------|------------------|
| Red            | E+               |
| Black          | E-               |
| White          | A+               |
| Green          | A-               |

## Power Supply Considerations

### For Servo Motors
It's recommended to use a separate 5-6V power supply for the servo motors, especially when they're under load. Connect the GND of this power supply to the Arduino GND for a common reference.

### For SIM800L
The SIM800L module requires a stable 3.7-4.2V power supply capable of handling current spikes up to 2A when the module is transmitting. A LiPo battery with a voltage regulator or a dedicated power supply module is recommended.

### For DC Motor
Use a separate power supply for the DC motor to prevent voltage drops affecting the Arduino. Connect all grounds together.

## Safety Notes
1. Always double-check connections before powering on
2. Ensure common ground between Arduino and external power supplies
3. Be careful not to exceed the current limits of the Arduino pins (40mA per pin, 200mA total)
4. Use capacitors across motor power inputs to reduce noise
5. For high-current devices (motors, GSM module), use external power supplies 