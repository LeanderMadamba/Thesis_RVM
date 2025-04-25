# RVM Project - Raspberry Pi and Arduino Integration

This project implements a machine learning model for waste classification (plastic vs. waste) using a Raspberry Pi 4, OV5647 Camera Module, 20x4 I2C LCD Display, a pushbutton for user input, and an integrated Arduino-controlled mechanical system.

## System Overview

This system creates a complete Reverse Vending Machine (RVM) that:
1. Captures images of waste items using the camera
2. Classifies items as plastic or waste using a machine learning model
3. Controls mechanical components via Arduino for sorting and processing
4. Provides user feedback via LCD display
5. Monitors container fullness and detects obstructions
6. Sends SMS notifications when containers are full or obstructions are detected

## Hardware Components

### Raspberry Pi Side
- Raspberry Pi 4 Model B
- OV5647 Camera Module
- 20x4 I2C LCD Display Module
- Pushbutton
- Jumper wires

### Arduino Side
- Arduino Mega 2560
- SIM800L v2 GSM Module
- L298N Motor Driver + DC Worm Gear Motor SGM-370
- MG995R Servo Motor (360 degrees)
- MG996R Servo Motor (160 degrees)
- 2x HC-SR04 Ultrasonic Sensors
- 10kg Load Cell with HX711 Amplifier

## Hardware Connection Instructions

### Camera Module (OV5647)

1. Ensure the Raspberry Pi is powered off
2. Locate the Camera Serial Interface (CSI) connector on the Raspberry Pi
3. Gently pull up the black clip on the CSI connector
4. Insert the camera's ribbon cable with the blue side facing the Ethernet port (or USB ports on Pi Zero)
5. Push down the black clip to secure the cable

### I2C LCD Display (20x4)

The LCD module uses I2C communication, which only requires 4 pins:

| LCD Pin | Raspberry Pi Pin |
|---------|------------------|
| VCC     | 5V (Pin 2 or 4)  |
| GND     | GND (Pin 6, 9, 14, 20, 25, 30, 34, or 39) |
| SDA     | GPIO 2 (Pin 3) - I2C Data  |
| SCL     | GPIO 3 (Pin 5) - I2C Clock |

### Button

The button requires one GPIO pin and ground:

| Button Pin | Raspberry Pi Pin |
|------------|------------------|
| Pin 1      | GPIO 17 (Pin 11) |
| Pin 2      | GND (Pin 9)      |

Note: The button is configured with an internal pull-up resistor in software, so no external resistor is needed.

### Raspberry Pi to Arduino Connection

Connect the Raspberry Pi to the Arduino Mega using a USB cable. This provides both power to the Arduino and a serial communication channel.

### Arduino Components

For detailed wiring of the Arduino components, see the `Arduino_Wiring_Diagram_Updated.md` file in the `Arduino_Code` directory.

## Software Setup

### 1. Enable Camera and I2C Interface

```bash
sudo raspi-config
```

- Navigate to "Interface Options"
- Enable "Camera"
- Enable "I2C"
- Reboot when prompted: `sudo reboot`

### 2. Update System and Install Dependencies

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3-pip python3-opencv i2c-tools
```

### 3. Install Required Python Libraries

```bash
pip3 install picamera2 numpy pillow tensorflow RPLCD RPi.GPIO pyserial
```

### 4. Upload Arduino Code

1. Install the Arduino IDE on your development computer
2. Install required libraries via Arduino Library Manager:
   - HX711 library by Bogdan Necula
   - Servo
   - SoftwareSerial
3. Open the `RVM_Controller_Serial.ino` file from the `Arduino_Code/RVM_Controller_Serial` directory
4. Connect the Arduino Mega to your computer via USB
5. Select the correct board (Arduino Mega 2560) and port in the Arduino IDE
6. Upload the code to the Arduino

### 5. Test Individual Components

#### Camera Test

```bash
python3 camera_test.py
```

This will capture an image and save it as `test_capture.jpg`. Verify the image looks correct.

#### LCD Test

First, find your LCD's I2C address:

```bash
i2cdetect -y 1
```

Look for a non-zero address (commonly 0x27 or 0x3F). Update the `LCD_ADDRESS` value in `lcd_test.py` if needed.

```bash
python3 lcd_test.py
```

The LCD should display test text on all 4 lines.

#### Button Test

```bash
python3 button_test.py
```

Press the button and verify that the program detects the button presses.

#### Arduino Serial Communication Test

```bash
python3 arduino_serial_test.py
```

This will attempt to establish communication with the Arduino and run basic tests.

### 6. Run the Full System

1. Transfer your trained model file `RVM_Model_mk6.keras` to the Raspberry Pi
2. Connect all components according to the wiring diagrams
3. Run the inference script:

```bash
python3 model_inference4.py
```

4. Press the button to capture an image and perform classification
5. View results on the LCD display
6. The Arduino will automatically process the item based on the classification

## System Diagnostic and Maintenance

### Arduino Commands
You can send these commands directly to the Arduino for testing and diagnostics:
- `HELLO` - Test communication
- `STATUS` - Check if the system is ready
- `CHECK_CONTAINERS` - Check container fullness
- `CHECK_OBSTRUCTIONS` - Check for obstructions
- `MOTOR_TEST` - Test the DC motor
- `WEIGHT_TEST` - Test the load cell

### Checking Container Status
The system automatically checks container fullness every hour. You can also run:

```bash
python3 container_check.py
```

## Troubleshooting

### Camera Issues
- Ensure the ribbon cable is properly connected
- Verify camera is enabled in raspi-config
- Check for errors with `dmesg | grep -i cam`

### LCD Issues
- Verify I2C address with `i2cdetect -y 1`
- Check connections, especially SDA and SCL
- Ensure I2C is enabled in raspi-config

### Button Issues
- Verify GPIO pin connections
- Check for interference or debounce issues
- Ensure GPIO is properly initialized in code

### Arduino Communication Issues
- Check USB connection between Raspberry Pi and Arduino
- Verify the correct port is being used (usually /dev/ttyACM0 or /dev/ttyUSB0)
- Run `dmesg | tail` after connecting the Arduino to see if it's detected
- Ensure the Arduino is running the correct firmware
- Check the BAUD rate (should be 9600)

### Mechanical Component Issues
- For detailed troubleshooting of Arduino components, see the `Arduino_README.md` file in the `Arduino_Code` directory

## Files Description

### Raspberry Pi Scripts
- `camera_test.py` - Tests the OV5647 Camera module functionality
- `lcd_test.py` - Tests the 20x4 I2C LCD display
- `button_test.py` - Tests the pushbutton input
- `arduino_serial_test.py` - Tests communication with Arduino
- `model_inference4.py` - Main system script that combines all components
- `container_check.py` - Utility to check container fullness

### Arduino Files
- `RVM_Controller_Serial.ino` - Main Arduino control program
- Various test sketches for individual components 