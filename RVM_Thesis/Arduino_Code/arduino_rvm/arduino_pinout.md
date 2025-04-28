# RVM Arduino Wiring Diagram

## Components Overview

The RVM (Reverse Vending Machine) system uses the following components connected to an Arduino Mega 2560:

1. SIM800L v2 GSM Module
2. L298N Motor Driver + DC Worm Gear Motor
3. MG995R Servo Motor (360 degrees)
4. MG996R Servo Motor (160 degrees)
5. 2x HC-SR04 Ultrasonic Sensors
6. 10kg Load Cell with HX711 Amplifier

## Components Requiring External Power Supply

The following components should be connected to an external power supply:

1. **L298N Motor Driver**: Requires 5-12V DC power supply for the DC motor
2. **SIM800L v2 GSM Module**: Requires regulated 3.7-4.2V power supply
3. **Servo Motors**: Both servo motors should be powered from a separate 5V power supply when possible to avoid voltage drop issues

## Pin Connections

### Arduino Mega 2560 Pinout

#### SIM800L GSM Module
- RX → Pin 19 (Arduino TX)
- TX → Pin 18 (Arduino RX)
- VCC → External 4.2V regulated power supply
- GND → Common ground

#### Servo Motors
- MG995R (360 degrees) Signal → Pin 9
- MG996R (160 degrees) Signal → Pin 10
- VCC → External 5V power supply
- GND → Common ground

#### DC Motor with L298N Driver
- ENA → Pin 6
- IN1 → Pin 8
- IN2 → Pin 7
- 12V → External 12V power supply
- GND → Common ground

#### Load Cell with HX711 Amplifier
- DOUT → Pin 3
- SCK → Pin 2
- VCC → Arduino 5V
- GND → Arduino GND

#### Ultrasonic Sensor 1 (HC-SR04)
- TRIG → Pin 34
- ECHO → Pin 35
- VCC → Arduino 5V
- GND → Arduino GND

#### Ultrasonic Sensor 2 (HC-SR04)
- TRIG → Pin 36
- ECHO → Pin 37
- VCC → Arduino 5V
- GND → Arduino GND

## Power Supply Recommendations

1. **Main Arduino Power**: 7-12V DC adapter or power supply
2. **Motor Power Supply**: 12V DC power supply capable of at least 2A
3. **Servo Power Supply**: Dedicated 5V 2A power supply
4. **GSM Module Power Supply**: Regulated 4.2V power supply or LiPo battery with protection circuit

## Important Notes

1. All ground connections should be common (connected together)
2. The L298N motor driver can supply 5V to the Arduino, but it's better to use a separate power supply for the Arduino for stability
3. When using external power supplies, ensure all ground connections are common
4. The SIM800L module is sensitive to power fluctuations; use a regulated power supply or LiPo battery
5. Servo motors can cause voltage drops when operating, which may reset the Arduino if powered from the same source

## Power Distribution Diagram

```
Power Sources:
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Arduino Power  │    │  Motor Power 12V │    │  Servo Power 5V │    │ GSM Power 4.2V  │
└────────┬────────┘    └────────┬─────────┘    └────────┬────────┘    └────────┬────────┘
         │                      │                       │                       │
         ▼                      ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Arduino Mega   │    │  L298N Driver    │    │  Servo Motors   │    │  SIM800L Module │
└─────────────────┘    └──────────────────┘    └─────────────────┘    └─────────────────┘
         │                      │                       │                       │
         └──────────────────────┴───────────────────────┴───────────────────────┘
                                          │
                                          ▼
                                    ┌──────────────┐
                                    │ Common Ground│
                                    └──────────────┘
``` 