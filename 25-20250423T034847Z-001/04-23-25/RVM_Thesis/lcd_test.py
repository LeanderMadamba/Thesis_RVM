#!/usr/bin/env python3

"""
20x4 I2C LCD Display Test Script
This script tests if the LCD display is working by displaying text
"""

import time
import smbus
from RPLCD.i2c import CharLCD

def test_lcd():
    print("Initializing I2C LCD display...")
    
    # Default I2C address is 0x27, but might be different (0x3F is another common address)
    # You can find the address using 'i2cdetect -y 1' command
    try:
        # Try with default address
        lcd = CharLCD('PCF8574', 0x27, cols=20, rows=4)
        print("Connected to LCD at address 0x27")
    except:
        try:
            # Try with alternative address
            lcd = CharLCD('PCF8574', 0x3F, cols=20, rows=4)
            print("Connected to LCD at address 0x3F")
        except Exception as e:
            print(f"Failed to connect to LCD: {e}")
            print("Try running 'i2cdetect -y 1' to find the correct I2C address")
            return
    
    # Clear the display
    lcd.clear()
    
    # Test display with different text on each line
    print("Writing to LCD...")
    lcd.write_string("20x4 LCD Test")
    lcd.cursor_pos = (1, 0)  # Move to second line
    lcd.write_string("Line 2: Testing...")
    lcd.cursor_pos = (2, 0)  # Move to third line
    lcd.write_string("Line 3: RVM Project")
    lcd.cursor_pos = (3, 0)  # Move to fourth line
    lcd.write_string("Line 4: Success!")
    
    print("Test complete! The LCD should display text on all 4 lines.")
    print("Press Ctrl+C to exit...")
    
    try:
        # Keep the display on until interrupted
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # Clear display when done
        lcd.clear()
        print("\nTest ended by user.")

if __name__ == "__main__":
    test_lcd() 