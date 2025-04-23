#!/usr/bin/env python3

"""
OV5647 Camera Test Script using PiCamera2 (libcamera)
This script tests if the camera is working by capturing an image
"""

import time
from picamera2 import Picamera2
import numpy as np
from PIL import Image

def test_camera():
    print("Initializing camera...")
    
    # Initialize the camera
    picam2 = Picamera2()
    
    # Configure the camera
    config = picam2.create_preview_configuration()
    picam2.configure(config)
    
    # Start the camera
    picam2.start()
    
    # Wait for camera to warm up
    print("Warming up camera...")
    time.sleep(2)
    
    # Capture an image
    print("Capturing image...")
    image = picam2.capture_array()
    
    # Convert to PIL image and save
    print("Saving image as test_capture.jpg")
    pil_image = Image.fromarray(image)
    pil_image.save("test_capture.jpg")
    
    # Stop the camera
    picam2.stop()
    
    print("Test complete! Check test_capture.jpg to see if the camera is working properly.")
    print("Image shape:", image.shape)
    
if __name__ == "__main__":
    try:
        test_camera()
    except Exception as e:
        print(f"Error: {e}") 