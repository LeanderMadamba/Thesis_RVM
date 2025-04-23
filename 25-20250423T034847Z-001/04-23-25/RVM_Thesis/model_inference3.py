#!/usr/bin/env python3

"""
RVM Model Inference Script
This script loads the trained model and performs inference on images captured by the camera
Press the button to capture and classify an image
Results are displayed on the LCD display
Images are saved with sequential naming in a specified folder
Sends signal to Arduino via GPIO to indicate plastic/waste detection
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

# GPIO setup
BUTTON_PIN = 17  # Button for user input
ARDUINO_OUTPUT_PIN = 27  # Pin to send signal to Arduino (1 for plastic, 0 for waste)
ARDUINO_READY_PIN = 22   # Pin to receive ready signal from Arduino

# LCD setup (you may need to change the I2C address)
LCD_ADDRESS = 0x27  # Common address, use i2cdetect -y 1 to find yours

# Model path
MODEL_PATH = "RVM_Model_mk6.keras"  # Adjust based on your model's filename

# Categories (matching your model training)
CATEGORIES = ["plastic", "waste"]

# Image saving configuration
IMAGE_FOLDER = "images"  # Folder to save images
IMAGE_PREFIX = "image"  # Prefix for image filenames

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

def initialize_hardware():
    """Initialize camera, LCD, GPIO, and model"""
    print("Initializing hardware...")
    
    # Set up GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ARDUINO_OUTPUT_PIN, GPIO.OUT)  # Output to Arduino
    GPIO.setup(ARDUINO_READY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Input from Arduino
    
    # Initialize output to Arduino as LOW (waste/default state)
    GPIO.output(ARDUINO_OUTPUT_PIN, GPIO.LOW)
    
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
    if picam2:
        picam2.stop()
    
    if lcd:
        lcd.clear()
        lcd.write_string("System Shutdown")
        time.sleep(1)
        lcd.clear()
    
    GPIO.cleanup()
    print("Cleanup complete")

def send_signal_to_arduino(is_plastic):
    """Send signal to Arduino based on classification"""
    if is_plastic:
        print("Sending PLASTIC signal to Arduino (HIGH)")
        GPIO.output(ARDUINO_OUTPUT_PIN, GPIO.HIGH)
    else:
        print("Sending WASTE signal to Arduino (LOW)")
        GPIO.output(ARDUINO_OUTPUT_PIN, GPIO.LOW)
    
    # Wait for Arduino to process (in a real system, we'd wait for a ready signal)
    time.sleep(0.5)

def wait_for_arduino_ready():
    """Wait for Arduino to signal it's ready for next item"""
    print("Waiting for Arduino to be ready...")
    
    timeout = 30  # 30 second timeout
    start_time = time.time()
    
    while GPIO.input(ARDUINO_READY_PIN) == GPIO.LOW:
        if time.time() - start_time > timeout:
            print("Timeout waiting for Arduino. Continuing anyway.")
            break
        time.sleep(0.1)
    
    print("Arduino is ready for next item")

def main():
    # Initialize hardware
    picam2, lcd, model = initialize_hardware()
    
    if model is None:
        print("Failed to initialize hardware. Exiting.")
        return
    
    # Start camera
    picam2.start()
    print("System ready. Press button to classify an item...")
    
    try:
        while True:
            # Check if Arduino is ready
            if GPIO.input(ARDUINO_READY_PIN) == GPIO.HIGH:
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
                    
                    # Get class name
                    class_name = CATEGORIES[predicted_class]
                    is_plastic = (predicted_class == 0)  # "plastic" is category 0, "waste" is category 1
                    
                    # Display result
                    print(f"Prediction: {class_name} (Confidence: {confidence:.2f})")
                    
                    if lcd:
                        lcd.clear()
                        lcd.write_string(f"Item: {class_name}")
                        lcd.cursor_pos = (1, 0)
                        lcd.write_string(f"Confidence: {confidence:.2f}")
                        lcd.cursor_pos = (2, 0)
                        lcd.write_string(f"Saved: {os.path.basename(saved_path)}")
                        lcd.cursor_pos = (3, 0)
                        lcd.write_string("Processing item...")
                    
                    # Send classification result to Arduino
                    send_signal_to_arduino(is_plastic)
                    
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
                    
                    # Wait for button release with debounce
                    time.sleep(0.5)
                    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                        time.sleep(0.1)
            else:
                if lcd:
                    # Only update occasionally to avoid flickering
                    lcd.clear()
                    lcd.write_string("Waiting for Arduino")
                    lcd.cursor_pos = (1, 0)
                    lcd.write_string("to be ready...")
                    time.sleep(2)
            
            time.sleep(0.1)  # Reduce CPU usage
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    finally:
        cleanup(picam2, lcd)

if __name__ == "__main__":
    main()