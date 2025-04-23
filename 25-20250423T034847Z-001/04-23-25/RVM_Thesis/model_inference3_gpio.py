#!/usr/bin/env python3

"""
RVM Model Inference Script - GPIO Communication Version
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

# Add debug mode (set to True for detailed GPIO debugging)
DEBUG_MODE = True

# Add delay values for more reliable communication
GPIO_SIGNAL_DURATION = 2.0  # Hold signal longer for Arduino to detect (2 seconds)
DEBOUNCE_TIME = 0.2  # Button debounce time

# Add manual override option to bypass Arduino ready check
BYPASS_ARDUINO_READY = True  # Set to True to bypass the Arduino ready signal check

# LCD setup (you may need to change the I2C address)
LCD_ADDRESS = 0x27  # Common address, use i2cdetect -y 1 to find yours

# Model path
MODEL_PATH = "RVM_Model_mk6.keras"  # Adjust based on your model's filename

# Categories (matching your model training)
CATEGORIES = ["plastic", "waste"]

# Image saving configuration
IMAGE_FOLDER = "images"  # Folder to save images
IMAGE_PREFIX = "image"  # Prefix for image filenames

def debug_print(message):
    """Print debug messages if debug mode is enabled"""
    if DEBUG_MODE:
        print(f"DEBUG: {message}")

def check_gpio_state():
    """Check and report the state of GPIO pins for debugging"""
    if not DEBUG_MODE:
        return
    
    try:
        button_state = GPIO.input(BUTTON_PIN)
        arduino_ready_state = GPIO.input(ARDUINO_READY_PIN)
        arduino_output_state = GPIO.input(ARDUINO_OUTPUT_PIN)
        
        debug_print(f"Button PIN {BUTTON_PIN}: {'Pressed (LOW)' if button_state == GPIO.LOW else 'Not Pressed (HIGH)'}")
        debug_print(f"Arduino Ready PIN {ARDUINO_READY_PIN}: {'Ready (HIGH)' if arduino_ready_state == GPIO.HIGH else 'Not Ready (LOW)'}")
        debug_print(f"Arduino Output PIN {ARDUINO_OUTPUT_PIN}: {'HIGH' if arduino_output_state == GPIO.HIGH else 'LOW'}")
        
        # Return ready state for use elsewhere
        return arduino_ready_state
    except Exception as e:
        debug_print(f"Error checking GPIO: {e}")
        return None

def toggle_arduino_ready_bypass():
    """Toggle the Arduino ready bypass setting"""
    global BYPASS_ARDUINO_READY
    BYPASS_ARDUINO_READY = not BYPASS_ARDUINO_READY
    debug_print(f"BYPASS_ARDUINO_READY set to {BYPASS_ARDUINO_READY}")
    return BYPASS_ARDUINO_READY

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
    
    # Clean up any existing GPIO configurations
    GPIO.cleanup()
    
    # Set up GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ARDUINO_OUTPUT_PIN, GPIO.OUT, initial=GPIO.LOW)  # Initialize as LOW
    GPIO.setup(ARDUINO_READY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Input from Arduino
    
    debug_print(f"GPIO pins initialized: Button={BUTTON_PIN}, Arduino Output={ARDUINO_OUTPUT_PIN}, Arduino Ready={ARDUINO_READY_PIN}")
    check_gpio_state()
    
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
        
        # Display Arduino ready bypass status
        lcd.cursor_pos = (3, 0)
        if BYPASS_ARDUINO_READY:
            lcd.write_string("Arduino bypass: ON")
        else:
            lcd.write_string("Arduino bypass: OFF")
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
    
    # Report current GPIO state
    check_gpio_state()
    
    # Hold the signal for a longer time to ensure Arduino detects it
    debug_print(f"Holding signal for {GPIO_SIGNAL_DURATION} seconds")
    time.sleep(GPIO_SIGNAL_DURATION)
    
    # Return to default state (LOW) after sending the signal
    # Comment out the next line if you want to maintain the signal instead
    # GPIO.output(ARDUINO_OUTPUT_PIN, GPIO.LOW)
    
    # Check the state after signal
    check_gpio_state()

def wait_for_arduino_ready():
    """Wait for Arduino to signal it's ready for next item"""
    if BYPASS_ARDUINO_READY:
        print("Arduino ready check bypassed.")
        return True
        
    print("Waiting for Arduino to be ready...")
    
    timeout = 30  # 30 second timeout
    start_time = time.time()
    
    # Check initial state
    ready_state = check_gpio_state()
    
    if ready_state == GPIO.HIGH:
        print("Arduino is already ready!")
        return True
    
    while GPIO.input(ARDUINO_READY_PIN) == GPIO.LOW:
        if time.time() - start_time > timeout:
            print("Timeout waiting for Arduino. Continuing anyway.")
            return False
        
        # Periodically check GPIO state
        if DEBUG_MODE and int(time.time()) % 3 == 0:  # Every 3 seconds
            check_gpio_state()
            
        time.sleep(0.1)
    
    print("Arduino is ready for next item")
    check_gpio_state()
    return True

def test_gpio_connection():
    """Test GPIO connection to Arduino"""
    print("Testing GPIO connection to Arduino...")
    
    # Test sending a HIGH signal
    GPIO.output(ARDUINO_OUTPUT_PIN, GPIO.HIGH)
    print("Set ARDUINO_OUTPUT_PIN to HIGH. Check if Arduino received it.")
    check_gpio_state()
    time.sleep(2)
    
    # Test sending a LOW signal
    GPIO.output(ARDUINO_OUTPUT_PIN, GPIO.LOW)
    print("Set ARDUINO_OUTPUT_PIN to LOW. Check if Arduino received it.")
    check_gpio_state()
    time.sleep(2)
    
    # Check if we can read the READY signal
    ready_state = GPIO.input(ARDUINO_READY_PIN)
    print(f"ARDUINO_READY_PIN is {'HIGH' if ready_state == GPIO.HIGH else 'LOW'}")
    
    # If Arduino ready pin isn't HIGH, suggest bypass
    if ready_state != GPIO.HIGH:
        print("Arduino ready signal not detected.")
        print("Enabling BYPASS_ARDUINO_READY mode for testing.")
        global BYPASS_ARDUINO_READY
        BYPASS_ARDUINO_READY = True

def main():
    # Initialize hardware
    picam2, lcd, model = initialize_hardware()
    
    if model is None:
        print("Failed to initialize hardware. Exiting.")
        return
    
    # Test GPIO connection if in debug mode
    if DEBUG_MODE:
        test_gpio_connection()
    
    # Start camera
    picam2.start()
    print("System ready. Press button to classify an item...")
    
    # Update LCD to show status
    if lcd:
        lcd.clear()
        lcd.write_string("System Ready")
        lcd.cursor_pos = (1, 0)
        lcd.write_string("Press button for")
        lcd.cursor_pos = (2, 0)
        lcd.write_string("classification")
        lcd.cursor_pos = (3, 0)
        if BYPASS_ARDUINO_READY:
            lcd.write_string("Arduino bypass: ON")
        else:
            lcd.write_string("Arduino bypass: OFF")
    
    button_pressed_time = 0
    lcd_update_time = 0
    
    try:
        while True:
            current_time = time.time()
            
            # Check if Arduino is ready or if bypass is enabled
            arduino_ready = BYPASS_ARDUINO_READY or GPIO.input(ARDUINO_READY_PIN) == GPIO.HIGH
            
            # Debug GPIO state every 5 seconds
            if DEBUG_MODE and int(current_time) % 5 == 0 and int(current_time) != int(lcd_update_time):
                arduino_ready_state = check_gpio_state()
                lcd_update_time = current_time
                
                # Extra debug for Arduino ready pin
                if arduino_ready_state == GPIO.LOW and not BYPASS_ARDUINO_READY:
                    debug_print("Arduino READY pin is LOW - not receiving ready signal")
                    debug_print("Consider checking wiring or enabling bypass mode")
            
            # Check for long button press to toggle bypass mode (5 seconds)
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                if button_pressed_time == 0:  # Start timing
                    button_pressed_time = current_time
                elif current_time - button_pressed_time > 5 and BYPASS_ARDUINO_READY == False:
                    # Toggle bypass after 5 seconds of button press
                    toggle_arduino_ready_bypass()
                    if lcd:
                        lcd.clear()
                        lcd.write_string("Arduino bypass")
                        lcd.cursor_pos = (1, 0)
                        lcd.write_string("mode enabled!")
                        lcd.cursor_pos = (2, 0)
                        lcd.write_string("Waiting 3s...")
                    time.sleep(3)
                    button_pressed_time = 0  # Reset timer
                    
                    # Update LCD
                    if lcd:
                        lcd.clear()
                        lcd.write_string("System Ready")
                        lcd.cursor_pos = (1, 0)
                        lcd.write_string("Press button for")
                        lcd.cursor_pos = (2, 0)
                        lcd.write_string("classification")
                        lcd.cursor_pos = (3, 0)
                        lcd.write_string("Arduino bypass: ON")
            else:
                button_pressed_time = 0  # Reset timer when button released
            
            if arduino_ready:
                # Check if button is pressed for normal operation
                if GPIO.input(BUTTON_PIN) == GPIO.LOW and button_pressed_time < current_time - 0.5:
                    debug_print("Button press detected for classification")
                    
                    # Debounce
                    time.sleep(DEBOUNCE_TIME)
                    if GPIO.input(BUTTON_PIN) != GPIO.LOW:
                        debug_print("Button press was too short (debounced)")
                        continue
                    
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
                        lcd.write_string(f"Conf: {confidence:.2f}")
                        lcd.cursor_pos = (2, 0)
                        lcd.write_string(f"Saved: {os.path.basename(saved_path)}")
                        lcd.cursor_pos = (3, 0)
                        lcd.write_string("Processing item...")
                    
                    # Send classification result to Arduino
                    send_signal_to_arduino(is_plastic)
                    
                    # Wait for the Arduino to process
                    if lcd:
                        lcd.clear()
                        lcd.write_string("Arduino processing")
                        lcd.cursor_pos = (1, 0)
                        lcd.write_string("Please wait...")
                        
                        if BYPASS_ARDUINO_READY:
                            lcd.cursor_pos = (3, 0)
                            lcd.write_string("Bypass mode ON")
                    
                    # Wait for Arduino to signal it's ready for next item (if not bypassed)
                    arduino_ready_response = wait_for_arduino_ready()
                    
                    if lcd:
                        lcd.clear()
                        lcd.write_string("Ready for next item")
                        lcd.cursor_pos = (1, 0)
                        lcd.write_string("Press button again")
                        lcd.cursor_pos = (3, 0)
                        if BYPASS_ARDUINO_READY:
                            lcd.write_string("Arduino bypass: ON")
                    
                    # Wait for button release with debounce
                    time.sleep(0.5)
                    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                        time.sleep(0.1)
            else:
                if DEBUG_MODE and int(current_time) % 10 == 0 and int(current_time) != int(lcd_update_time):
                    debug_print("Arduino not ready. Waiting...")
                    debug_print("Press and hold button 5s to enable bypass mode")
                    lcd_update_time = current_time
                
                if lcd and int(current_time) % 5 == 0 and int(current_time) != int(lcd_update_time):
                    # Only update occasionally to avoid flickering
                    lcd.clear()
                    lcd.write_string("Waiting for Arduino")
                    lcd.cursor_pos = (1, 0)
                    lcd.write_string("to be ready...")
                    lcd.cursor_pos = (3, 0)
                    lcd.write_string("Hold button 5s→bypass")
                    lcd_update_time = current_time
            
            time.sleep(0.1)  # Reduce CPU usage
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    finally:
        cleanup(picam2, lcd)

if __name__ == "__main__":
    main() 