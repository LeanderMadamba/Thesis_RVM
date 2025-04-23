#!/usr/bin/env python3

"""
RVM Model Inference Script
This script loads the trained model and performs inference on images captured by the camera
Press the button to capture and classify an image
Results are displayed on the LCD display
"""

import time
import numpy as np
import tensorflow as tf
from picamera2 import Picamera2
from PIL import Image
import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD
import cv2

# GPIO setup
BUTTON_PIN = 17  # Change to your actual button GPIO pin

# LCD setup (you may need to change the I2C address)
LCD_ADDRESS = 0x27  # Common address, use i2cdetect -y 1 to find yours

# Model path
MODEL_PATH = "binary_classification_plastic_vs_waste.keras"  # Adjust based on your model's filename

# Categories (matching your model training)
CATEGORIES = ["plastic", "waste"]

def preprocess_image(image, target_size=(224, 224)):
    """Preprocess the image for model inference"""
    # Resize
    img = cv2.resize(image, target_size)
    
    # Convert to RGB if needed (Picamera2 returns BGR)
    if len(img.shape) == 3 and img.shape[2] == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # Convert to float and preprocess
    img = img.astype(np.float32)
    img = tf.keras.applications.mobilenet_v2.preprocess_input(img)
    
    # Add batch dimension
    img = np.expand_dims(img, axis=0)
    
    # Ensure the shape is exactly (None, 224, 224, 3)
    # The None dimension is handled by the batch size
    if img.shape != (1, 224, 224, 3):
        print(f"Warning: Image shape is {img.shape}, expected (1, 224, 224, 3)")
        # Reshape if necessary
        img = img.reshape(1, 224, 224, 3)
    
    return img

def initialize_hardware():
    """Initialize camera, LCD, GPIO, and model"""
    print("Initializing hardware...")
    
    # Set up button
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # Set up camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration()
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
            # Check if button is pressed
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                if lcd:
                    lcd.clear()
                    lcd.write_string("Capturing image...")
                
                print("Button pressed. Capturing image...")
                
                # Capture image
                image = picam2.capture_array()
                
                # Save original image for reference
                Image.fromarray(image).save("last_capture.jpg")
                
                # Preprocess image
                processed_img = preprocess_image(image)
                
                # Run inference
                prediction = model.predict(processed_img)[0][0]
                
                # Determine class (for binary classification)
                predicted_class = 1 if prediction > 0.5 else 0
                confidence = prediction if predicted_class == 1 else 1 - prediction
                
                # Get class name
                class_name = CATEGORIES[predicted_class]
                
                # Display result
                print(f"Prediction: {class_name} (Confidence: {confidence:.2f})")
                
                if lcd:
                    lcd.clear()
                    lcd.write_string(f"Item: {class_name}")
                    lcd.cursor_pos = (1, 0)
                    lcd.write_string(f"Confidence: {confidence:.2f}")
                    lcd.cursor_pos = (3, 0)
                    lcd.write_string("Press button again...")
                
                # Wait for button release with debounce
                time.sleep(0.5)
                while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                    time.sleep(0.1)
            
            time.sleep(0.1)  # Reduce CPU usage
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    finally:
        cleanup(picam2, lcd)

if __name__ == "__main__":
    main() 