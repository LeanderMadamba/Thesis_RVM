# RVM Project - Raspberry Pi Setup and Usage

This project implements a machine learning model for waste classification (plastic vs. waste) using a Raspberry Pi 4, OV5647 Camera Module, 20x4 I2C LCD Display, and a pushbutton for user input.

## Hardware Components

- Raspberry Pi 4 Model B
- OV5647 Camera Module
- 20x4 I2C LCD Display Module
- Pushbutton
- Jumper wires
- Breadboard (optional)

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
pip3 install picamera2 numpy pillow tensorflow RPLCD RPi.GPIO
```

### 4. Test Individual Components

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

### 5. Transfer Model and Run Inference

1. Transfer your trained model file `binary_classification_plastic_vs_waste.keras` to the Raspberry Pi
2. Run the inference script:

```bash
python3 model_inference.py
```

3. Press the button to capture an image and perform classification
4. View results on the LCD display

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

## Scripts Description

- `camera_test.py` - Tests the OV5647 Camera module functionality
- `lcd_test.py` - Tests the 20x4 I2C LCD display
- `button_test.py` - Tests the pushbutton input
- `model_inference.py` - Combines all components for image capture and classification 