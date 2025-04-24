#!/usr/bin/env python3

"""
RVM System - Complete Implementation
Combines camera, LCD display, button, and model inference in one script
"""

import time
import numpy as np
import tensorflow as tf
import cv2
import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD
from picamera2 import Picamera2
from PIL import Image
import os

# Configuration
BUTTON_PIN = 17  # GPIO pin for button
LCD_ADDRESS = 0x27  # I2C address for LCD (use i2cdetect -y 1 to find yours)
MODEL_PATH = "binary_classification_plastic_vs_waste.keras"  # Path to your model
CATEGORIES = ["plastic", "waste"]  # Categories matching your model

class RVMSystem:
    def __init__(self):
        self.picam2 = None
        self.lcd = None
        self.model = None
        self.running = False
        
    def initialize_hardware(self):
        """Initialize all hardware components"""
        print("Initializing hardware...")
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Initialize camera
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration()
            self.picam2.configure(config)
            print("Camera initialized successfully")
        except Exception as e:
            print(f"Camera initialization error: {e}")
            return False
        
        # Initialize LCD
        try:
            self.lcd = CharLCD('PCF8574', LCD_ADDRESS, cols=20, rows=4)
            self.lcd.clear()
            self.lcd.write_string("RVM System Ready")
            print("LCD initialized successfully")
        except Exception as e:
            print(f"LCD initialization error: {e}")
            self.lcd = None
        
        # Load model
        try:
            if os.path.exists(MODEL_PATH):
                self.model = tf.keras.models.load_model(MODEL_PATH)
                print(f"Model loaded from {MODEL_PATH}")
            else:
                print(f"Model file not found at {MODEL_PATH}")
                return False
        except Exception as e:
            print(f"Model loading error: {e}")
            return False
        
        return True
    
    def preprocess_image(self, image, target_size=(224, 224)):
        """Preprocess image for model inference"""
        # Resize
        img = cv2.resize(image, target_size)
        
        # Convert to RGB if needed
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
    
    def test_camera(self):
        """Test camera functionality"""
        if not self.picam2:
            print("Camera not initialized")
            return False
        
        try:
            self.picam2.start()
            time.sleep(2)  # Warm up
            
            # Capture test image
            image = self.picam2.capture_array()
            Image.fromarray(image).save("camera_test.jpg")
            
            self.picam2.stop()
            print("Camera test successful - check camera_test.jpg")
            return True
        except Exception as e:
            print(f"Camera test failed: {e}")
            return False
    
    def test_lcd(self):
        """Test LCD functionality"""
        if not self.lcd:
            print("LCD not initialized")
            return False
        
        try:
            self.lcd.clear()
            self.lcd.write_string("LCD Test")
            self.lcd.cursor_pos = (1, 0)
            self.lcd.write_string("Line 2: Working")
            self.lcd.cursor_pos = (2, 0)
            self.lcd.write_string("Line 3: Success")
            self.lcd.cursor_pos = (3, 0)
            self.lcd.write_string("Line 4: Complete")
            print("LCD test successful")
            return True
        except Exception as e:
            print(f"LCD test failed: {e}")
            return False
    
    def test_button(self):
        """Test button functionality"""
        print("Button test - press the button (Ctrl+C to exit)...")
        press_count = 0
        
        try:
            while True:
                if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                    press_count += 1
                    print(f"Button pressed! Count: {press_count}")
                    
                    # Wait for button release
                    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                        time.sleep(0.1)
                    
                    print("Button released")
                    time.sleep(0.2)  # Debounce
                
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nButton test ended")
            return press_count > 0
    
    def run_inference(self):
        """Run the main inference loop"""
        if not all([self.picam2, self.lcd, self.model]):
            print("System not properly initialized")
            return
        
        self.picam2.start()
        self.running = True
        
        if self.lcd:
            self.lcd.clear()
            self.lcd.write_string("System Ready")
            self.lcd.cursor_pos = (1, 0)
            self.lcd.write_string("Press button to")
            self.lcd.cursor_pos = (2, 0)
            self.lcd.write_string("classify an item")
        
        print("System ready. Press button to classify an item...")
        
        try:
            while self.running:
                if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                    if self.lcd:
                        self.lcd.clear()
                        self.lcd.write_string("Capturing image...")
                    
                    print("Button pressed. Capturing image...")
                    
                    # Capture image
                    image = self.picam2.capture_array()
                    Image.fromarray(image).save("last_capture.jpg")
                    
                    # Preprocess and predict
                    processed_img = self.preprocess_image(image)
                    prediction = self.model.predict(processed_img)[0][0]
                    
                    # Determine class
                    predicted_class = 1 if prediction > 0.5 else 0
                    confidence = prediction if predicted_class == 1 else 1 - prediction
                    class_name = CATEGORIES[predicted_class]
                    
                    # Display result
                    print(f"Prediction: {class_name} (Confidence: {confidence:.2f})")
                    
                    if self.lcd:
                        self.lcd.clear()
                        self.lcd.write_string(f"Item: {class_name}")
                        self.lcd.cursor_pos = (1, 0)
                        self.lcd.write_string(f"Confidence: {confidence:.2f}")
                        self.lcd.cursor_pos = (3, 0)
                        self.lcd.write_string("Press button again...")
                    
                    # Wait for button release
                    time.sleep(0.5)
                    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                        time.sleep(0.1)
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nSystem stopped by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        
        if self.picam2:
            self.picam2.stop()
        
        if self.lcd:
            self.lcd.clear()
            self.lcd.write_string("System Shutdown")
            time.sleep(1)
            self.lcd.clear()
        
        GPIO.cleanup()
        print("Cleanup complete")

def main():
    system = RVMSystem()
    
    if not system.initialize_hardware():
        print("Failed to initialize hardware. Exiting.")
        return
    
    # Test components
    print("\nTesting camera...")
    if not system.test_camera():
        print("Camera test failed")
        return
    
    print("\nTesting LCD...")
    if not system.test_lcd():
        print("LCD test failed")
        return
    
    print("\nTesting button...")
    if not system.test_button():
        print("Button test failed")
        return
    
    # Run main inference loop
    print("\nStarting main system...")
    system.run_inference()

if __name__ == "__main__":
    main() 