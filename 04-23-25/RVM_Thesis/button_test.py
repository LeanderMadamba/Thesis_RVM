#!/usr/bin/env python3

"""
Button Test Script for Raspberry Pi
This script tests if the button is working properly
"""

import RPi.GPIO as GPIO
import time

# Define the GPIO pin connected to the button
BUTTON_PIN = 17  # Change this to your actual GPIO pin number

def test_button():
    print("Starting button test...")
    print(f"Button connected to GPIO {BUTTON_PIN}")
    
    # Set up GPIO
    GPIO.setmode(GPIO.BCM)  # Use BCM numbering
    
    # Set the button pin as input with pull-up resistor
    # With pull-up, button should connect pin to ground when pressed
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    print("Press the button (Ctrl+C to exit)...")
    press_count = 0
    
    try:
        while True:
            # Button is pressed when input is LOW (0) with pull-up
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                press_count += 1
                print(f"Button pressed! Count: {press_count}")
                
                # Wait for button release to avoid multiple counts
                while GPIO.input(BUTTON_PIN) == GPIO.LOW:
                    time.sleep(0.1)
                
                print("Button released")
                time.sleep(0.2)  # Debounce delay
            
            time.sleep(0.1)  # Small delay to reduce CPU usage
            
    except KeyboardInterrupt:
        print("\nTest ended by user")
    finally:
        # Clean up GPIO
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    test_button() 