/*
 * Servo Motors Test
 * 
 * This sketch tests two servo motors:
 * - MG995R (360-degree continuous rotation servo)
 * - MG996R (160-degree standard servo)
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - MG995R Servo Motor (360 degrees)
 * - MG996R Servo Motor (160 degrees)
 * 
 * Connections:
 * - MG995R Signal to Arduino Pin 9
 * - MG996R Signal to Arduino Pin 10
 * - Servo VCC to external 5-6V power supply
 * - Servo GND to Arduino GND
 */

#include <Servo.h>

// Define servo pins
#define MG995R_PIN 9  // 360-degree servo
#define MG996R_PIN 10 // 160-degree servo

// Create servo objects
Servo continuousServo;  // MG995R 360-degree servo
Servo standardServo;    // MG996R 160-degree servo

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Servo Motors Test");
  
  // Attach servos to their pins
  continuousServo.attach(MG995R_PIN);
  standardServo.attach(MG996R_PIN);
  
  // Center both servos initially
  continuousServo.write(90);  // 90 is stop position for continuous servo
  standardServo.write(90);    // 90 is center position for standard servo
  
  Serial.println("Servos initialized and set to center/stop position");
  delay(2000);
}

void loop() {
  // Test the continuous rotation servo (MG995R)
  testContinuousServo();
  
  // Test the standard servo (MG996R)
  testStandardServo();
}

void testContinuousServo() {
  Serial.println("Testing 360-degree continuous rotation servo (MG995R)");
  
  // Rotate clockwise at full speed
  Serial.println("Rotating clockwise at full speed...");
  continuousServo.write(0);
  delay(3000);
  
  // Stop
  Serial.println("Stopping...");
  continuousServo.write(90);
  delay(2000);
  
  // Rotate counter-clockwise at full speed
  Serial.println("Rotating counter-clockwise at full speed...");
  continuousServo.write(180);
  delay(3000);
  
  // Stop
  Serial.println("Stopping...");
  continuousServo.write(90);
  delay(2000);
  
  // Rotate clockwise at half speed
  Serial.println("Rotating clockwise at half speed...");
  continuousServo.write(45);
  delay(3000);
  
  // Rotate counter-clockwise at half speed
  Serial.println("Rotating counter-clockwise at half speed...");
  continuousServo.write(135);
  delay(3000);
  
  // Stop
  Serial.println("Stopping...");
  continuousServo.write(90);
  delay(2000);
}

void testStandardServo() {
  Serial.println("Testing 160-degree standard servo (MG996R)");
  
  // Move to minimum position
  Serial.println("Moving to minimum position (10 degrees)...");
  for (int pos = 90; pos >= 10; pos -= 5) {
    standardServo.write(pos);
    delay(100);
  }
  delay(1000);
  
  // Move to center position
  Serial.println("Moving to center position (90 degrees)...");
  for (int pos = 10; pos <= 90; pos += 5) {
    standardServo.write(pos);
    delay(100);
  }
  delay(1000);
  
  // Move to maximum position
  Serial.println("Moving to maximum position (170 degrees)...");
  for (int pos = 90; pos <= 170; pos += 5) {
    standardServo.write(pos);
    delay(100);
  }
  delay(1000);
  
  // Return to center position
  Serial.println("Returning to center position (90 degrees)...");
  for (int pos = 170; pos >= 90; pos -= 5) {
    standardServo.write(pos);
    delay(100);
  }
  delay(1000);
  
  // Perform a sweep
  Serial.println("Performing a full sweep...");
  for (int pos = 10; pos <= 170; pos += 5) {
    standardServo.write(pos);
    delay(100);
  }
  for (int pos = 170; pos >= 10; pos -= 5) {
    standardServo.write(pos);
    delay(100);
  }
  
  // Return to center position
  standardServo.write(90);
  delay(2000);
  
  Serial.println("Servo test completed");
  Serial.println("--------------------");
  delay(5000);
} 