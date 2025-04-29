#!/usr/bin/env python3

"""
RVM Model Inference Script - Serial Communication Version (Modified)
This script loads the trained model and performs inference on images captured by the camera
Press the button to capture and classify an image
Results are displayed on the LCD display
Images are saved with sequential naming in a specified folder
Communicates with Arduino via USB Serial connection with auto-detection
Controls Arduino-based hardware components for waste handling:
- Load cell for weight measurement (from Load_cell_test.ino)
- DC motor with L298N driver (from Mothafuckin_dc_motorz.ino)
- Ultrasonic sensors for container fullness detection (from Thesis_containers.ino)

Modified to reclassify plastic predictions with less than 80% confidence as waste
"""

import time
import numpy as np
import tensorflow as tf
from picamera2 import Picamera2
from PIL import Image
import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD
import cv2
import os
import glob
import serial
import threading
import serial.tools.list_ports

# GPIO setup
BUTTON_PIN = 17  # Button for user input


# Serial communication setup
BAUD_RATE = 9600
SERIAL_TIMEOUT = 1

# LCD setup (you may need to change the I2C address)
LCD_ADDRESS = 0x27  # Common address, use i2cdetect -y 1 to find yours

# Model path
MODEL_PATH = "RVM_Model_mk6.keras"  # Adjust based on your model's filename

# Categories (matching your model training)
CATEGORIES = ["plastic", "waste"]

# Classification configuration
PLASTIC_CONFIDENCE_THRESHOLD = 0.8  # Minimum confidence needed for plastic classification

# Image saving configuration
IMAGE_FOLDER = "images"  # Folder to save images
IMAGE_PREFIX = "image"  # Prefix for image filenames

# Serial communication globals
ser = None
arduino_ready = False
arduino_processing = False  # New flag to track if Arduino is actively processing
serial_lock = threading.Lock()
last_command_time = 0  # Track when the last command was sent


def find_arduino_port():
    """
    Auto-detect Arduino serial port
    Returns the first available Arduino port or None if not found
    """
    print("Searching for Arduino...")
    
    # Check common Arduino port names first
    common_ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
    
    # Try common ports first (faster than scanning all ports)
    for port in common_ports:
        try:
            s = serial.Serial(port, BAUD_RATE, timeout=1)
            s.close()
            print(f"Found potential Arduino at {port}")
            return port
        except (serial.SerialException, OSError):
            pass
    
    # If common ports fail, try to find all available ports
    print("Common ports not found, scanning all serial ports...")
    ports = list(serial.tools.list_ports.comports())
    
    for port in ports:
        # Arduino devices often have "Arduino" or "USB" in their description
        port_name = port.device
        description = port.description.lower()
        
        print(f"Found port: {port_name} - {description}")
        
        # Check if this looks like an Arduino
        if "arduino" in description or "acm" in port_name.lower() or "usb" in description:
            try:
                s = serial.Serial(port_name, BAUD_RATE, timeout=1)
                s.close()
                print(f"Found Arduino at {port_name}")
                return port_name
            except (serial.SerialException, OSError) as e:
                print(f"Could not open {port_name}: {e}")
    
    print("No Arduino found! Check connections and permissions.")
    return None

def get_next_image_number():
    """Get the next sequential number for image naming"""
    # Make sure the folder exists
    os.makedirs(IMAGE_FOLDER, exist_ok=True)
    
    # Find existing image files
    pattern = os.path.join(IMAGE_FOLDER, f"{IMAGE_PREFIX}*.png")
    existing_files = glob.glob(pattern)
    
    if not existing_files:
        return 1  # Start with 1 if no files exist
    
    # Extract numbers from existing filenames
    numbers = []
    for file in existing_files:
        base = os.path.basename(file)
        # Remove prefix and extension, convert to int
        try:
            num = int(base[len(IMAGE_PREFIX):-4])
            numbers.append(num)
        except ValueError:
            continue
    
    # Return the next number in sequence
    return max(numbers) + 1 if numbers else 1

def save_image(image):
    """Save image with sequential naming"""
    next_num = get_next_image_number()
    filename = f"{IMAGE_PREFIX}{next_num:03d}.png"
    filepath = os.path.join(IMAGE_FOLDER, filename)
    
    # Save image
    Image.fromarray(image).save(filepath)
    print(f"Image saved as {filepath}")
    
    return filepath

def preprocess_image(image, target_size=(224, 224)):
    """Preprocess the image for model inference"""
    # Resize
    img = cv2.resize(image, target_size)
    
    # Convert to RGB by taking only the first 3 channels if image has 4 channels
    if len(img.shape) == 3 and img.shape[2] == 4:
        img = img[:, :, :3]  # Take only the first 3 channels (RGB)
    
    # Convert to RGB if needed (Picamera2 returns BGR)
    if len(img.shape) == 3 and img.shape[2] == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # Convert to float and preprocess
    img = img.astype(np.float32)
    img = tf.keras.applications.mobilenet_v2.preprocess_input(img)
    
    # Add batch dimension
    img = np.expand_dims(img, axis=0)
    
    # Debug print
    print(f"Processed image shape: {img.shape}")
    
    return img

def serial_reader_thread():
    """Background thread to continuously read from Arduino"""
    global arduino_ready, arduino_processing, ser
    
    print("Serial reader thread started")
    
    while True:
        try:
            if ser and ser.is_open:
                with serial_lock:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8').strip()
                        if line:
                            print(f"Arduino: {line}")
                            if "READY" in line:
                                arduino_ready = True
                                arduino_processing = False
                            elif "BUSY" in line:
                                arduino_ready = False
                                arduino_processing = True
                            elif "PROCESSING" in line:
                                arduino_processing = True
                            elif "ERROR" in line:
                                # If we get an error, allow status checks again
                                if arduino_processing and not arduino_ready:
                                    print("Arduino reported an error, resetting processing state")
                                    arduino_processing = False
            time.sleep(0.1)
        except Exception as e:
            print(f"Serial reader error: {e}")
            time.sleep(1)

def initialize_hardware():
    """Initialize camera, LCD, GPIO, serial, and model"""
    global ser, arduino_ready
    
    print("Initializing hardware...")
    
    # Set up GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # Auto-detect Arduino
    arduino_port = find_arduino_port()
    
    # Set up serial communication
    if arduino_port:
        try:
            ser = serial.Serial(arduino_port, BAUD_RATE, timeout=SERIAL_TIMEOUT)
            print(f"Serial connection established on {arduino_port}")
            
            # Start serial reader thread
            reader_thread = threading.Thread(target=serial_reader_thread, daemon=True)
            reader_thread.start()
            
            # Initial handshake with Arduino
            time.sleep(2)  # Wait for Arduino to reset after serial connection
            with serial_lock:
                ser.write(b"HELLO\n")
            
            # Wait for initial ready signal
            timeout = 30
            start_time = time.time()
            while not arduino_ready:
                if time.time() - start_time > timeout:
                    print("Warning: No ready signal from Arduino. Continuing anyway.")
                    arduino_ready = True  # Force ready to continue
                    break
                time.sleep(0.1)
            
        except Exception as e:
            print(f"Serial connection error: {e}")
            print("Check if Arduino is connected and permissions are correct.")
            ser = None
    else:
        print("No Arduino detected. Will run without Arduino functionality.")
    
    # Set up camera with explicit format
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "RGB888"})  # Explicitly request RGB
    picam2.configure(config)
    
    # Set up LCD
    try:
        lcd = CharLCD('PCF8574', LCD_ADDRESS, cols=20, rows=4)
        lcd.clear()
        lcd.write_string("RVM System Ready")
        lcd.cursor_pos = (1, 0)
        lcd.write_string("Press button to")
        lcd.cursor_pos = (2, 0)
        lcd.write_string("classify an item")
        
        # Show Arduino status
        lcd.cursor_pos = (3, 0)
        if ser:
            lcd.write_string(f"Arduino: {arduino_port}")
        else:
            lcd.write_string("Arduino: Not found")
            
    except Exception as e:
        print(f"LCD Error: {e}")
        lcd = None
    
    # Load model
    try:
        model = tf.keras.models.load_model(MODEL_PATH)
        print(f"Model loaded from {MODEL_PATH}")
    except Exception as e:
        print(f"Model loading error: {e}")
        if lcd:
            lcd.clear()
            lcd.write_string("Model loading error")
        return None, None, None
    
    # Ensure image directory exists
    os.makedirs(IMAGE_FOLDER, exist_ok=True)
    print(f"Image save directory: {os.path.abspath(IMAGE_FOLDER)}")
    
    return picam2, lcd, model

def cleanup(picam2, lcd):
    """Clean up resources"""
    global ser
    
    if picam2:
        picam2.stop()
    
    if lcd:
        lcd.clear()
        lcd.write_string("System Shutdown")
        time.sleep(1)
        lcd.clear()
    
    # Close serial connection
    if ser and ser.is_open:
        with serial_lock:
            ser.write(b"SHUTDOWN\n")
            ser.close()
    
    GPIO.cleanup()
    print("Cleanup complete")

def send_command_to_arduino(command):
    """Send a command to Arduino over serial"""
    global ser, last_command_time
    
    if not ser or not ser.is_open:
        print("Serial connection not available")
        return False
    
    # Add delay between commands to prevent them running together
    current_time = time.time()
    time_since_last = current_time - last_command_time
    if time_since_last < 0.5:  # Ensure at least 500ms between commands
        time.sleep(0.5 - time_since_last)
    
    try:
        with serial_lock:
            # Make sure each command ends with a newline and flush buffer
            ser.write(f"{command}\n".encode('utf-8'))
            ser.flush()  # Ensure data is sent immediately
        
        last_command_time = time.time()
        print(f"Sent to Arduino: {command}")
        return True
    except Exception as e:
        print(f"Error sending to Arduino: {e}")
        return False

def check_arduino_status():
    """Check Arduino status but only if not actively processing"""
    global arduino_processing
    
    if not arduino_processing:
        send_command_to_arduino("STATUS")

def wait_for_arduino_ready():
    """Wait for Arduino to signal it's ready for next item"""
    global arduino_ready, arduino_processing
    
    # If no Arduino connection, don't wait
    if not ser:
        arduino_ready = True
        return
    
    print("Waiting for Arduino to be ready...")
    
    # Send a status check but only if we're not already waiting for processing
    if not arduino_processing:
        check_arduino_status()
    
    timeout = 30  # 30 second timeout
    start_time = time.time()
    
    while not arduino_ready:
        if time.time() - start_time > timeout:
            print("Timeout waiting for Arduino. Continuing anyway.")
            arduino_ready = True  # Force ready to continue
            arduino_processing = False  # Reset processing flag
            break
        
        # Only check status periodically and only if not already processing
        if not arduino_processing and (time.time() - last_command_time) > 3:
            check_arduino_status()
        
        time.sleep(0.5)
    
    print("Arduino is ready for next item")

def check_container_fullness():
    """Request Arduino to check if containers are full"""
    if not ser:
        print("Serial connection not available. Cannot check containers.")
        return False
    
    if arduino_ready:
        arduino_ready = False  # Mark as busy
        arduino_processing = True  # Mark as processing
        
        # Send container check command
        send_command_to_arduino("CHECK_CONTAINERS")
        
        # Wait for Arduino to complete the check
        wait_for_arduino_ready()
        return True
    else:
        print("Arduino busy. Cannot check containers now.")
        return False

def main():
    global arduino_ready, arduino_processing
    
    # Initialize hardware
    picam2, lcd, model = initialize_hardware()
    
    if model is None:
        print("Failed to initialize hardware. Exiting.")
        return
    
    # Start camera
    picam2.start()
    print("System ready. Press button to classify an item...")
    
    # If no Arduino, set ready flag to true
    if not ser:
        arduino_ready = True
    
    status_check_time = time.time()
    
    try:
        while True:
            # Check if Arduino is ready or if we're running without Arduino
            if arduino_ready:
                # Check if button is pressed
                if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                    if lcd:
                        lcd.clear()
                        lcd.write_string("Capturing image...")
                    
                    print("Button pressed. Capturing image...")
                    
                    # Capture image
                    image = picam2.capture_array()
                    
                    # Save image with sequential naming
                    saved_path = save_image(image)
                    
                    # Preprocess image
                    processed_img = preprocess_image(image)
                    
                    # Run inference
                    prediction = model.predict(processed_img)[0][0]
                    
                    # Determine class (for binary classification)
                    predicted_class = 1 if prediction > 0.5 else 0
                    confidence = prediction if predicted_class == 1 else 1 - prediction
                    
                    # Get base class name
                    base_class_name = CATEGORIES[predicted_class]
                    
                    # Apply confidence threshold for plastic classification
                    is_plastic = (predicted_class == 0)  # Initially determine if plastic
                    
                    # If it's predicted as plastic but has low confidence, treat as waste
                    reclassified = False
                    if is_plastic and confidence < PLASTIC_CONFIDENCE_THRESHOLD:
                        is_plastic = False  # Change to waste
                        reclassified = True
                        print(f"Low confidence plastic detection ({confidence:.2f}), reclassifying as waste")
                    
                    # Final class name after potential reclassification
                    class_name = "plastic" if is_plastic else "waste"
                    
                    # Display result with additional info about reclassification if needed
                    print(f"Prediction: {class_name} (Confidence: {confidence:.2f})")
                    if reclassified:
                        print(f"Original prediction was {base_class_name} but reclassified due to low confidence")
                    
                    if lcd:
                        lcd.clear()
                        lcd.write_string(f"Item: {class_name}")
                        lcd.cursor_pos = (1, 0)
                        lcd.write_string(f"Confidence: {confidence:.2f}")
                        lcd.cursor_pos = (2, 0)
                        lcd.write_string(f"Saved: {os.path.basename(saved_path)}")
                        if reclassified:
                            lcd.cursor_pos = (3, 0)
                            lcd.write_string("Low conf - reclassed")
                    
                    # Only send to Arduino if we have a connection
                    if ser:
                        if lcd:
                            lcd.cursor_pos = (3, 0)
                            lcd.write_string("Processing item...")
                        
                        # Send classification result to Arduino
                        arduino_ready = False  # Mark as busy
                        arduino_processing = True  # Mark as processing
                        command = "PLASTIC" if is_plastic else "WASTE"
                        send_command_to_arduino(command)
                        
                        # Wait for the Arduino to process
                        if lcd:
                            lcd.clear()
                            lcd.write_string("Arduino processing...")
                            lcd.cursor_pos = (1, 0)
                            lcd.write_string("Please wait...")
                        
                        # Wait for Arduino to signal it's ready for next item
                        wait_for_arduino_ready()
                    
                    if lcd:
                        lcd.clear()
                        lcd.write_string("Ready for next item")
                        lcd.cursor_pos = (1, 0)
                        lcd.write_string("Press button again..")
                        print("Ready for next item..")
                    
                    # Wait for button release with debounce
                    time.sleep(0.5)
                    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                        time.sleep(0.1)
            else:
                # Only check Arduino status periodically (every 5 seconds), not continuously
                current_time = time.time()
                if current_time - status_check_time > 5:
                    if lcd:
                        lcd.clear()
                        lcd.write_string("Waiting for Arduino")
                        lcd.cursor_pos = (1, 0)
                        lcd.write_string("to be ready...")
                    
                    # Only check status if not processing
                    if not arduino_processing:
                        check_arduino_status()
                    
                    status_check_time = current_time
            
            time.sleep(0.1)  # Reduce CPU usage
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    finally:
        cleanup(picam2, lcd)

if __name__ == "__main__":
    main() 