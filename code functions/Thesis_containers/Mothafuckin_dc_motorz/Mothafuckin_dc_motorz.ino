/*
 * Arduino Mega 2560 + L298N Motor Driver + DC Worm Gear Motor SGM-370 Test
 * 
 * This script allows you to test a DC motor connection using the L298N motor driver.
 * It will cycle the motor through different speeds and directions.
 * 
 * Hardware Connections:
 * - L298N ENA -> Arduino pin 9 (PWM)
 * - L298N IN1 -> Arduino pin 8
 * - L298N IN2 -> Arduino pin 7
 * - L298N +12V and GND connected to external 12V power supply
 * - L298N motor outputs connected to your SGM-370 motor
 * - Don't forget to connect Arduino GND to L298N GND
 */

// Define control pins
const int ENA = 9;  // PWM pin for controlling motor speed
const int IN1 = 8;  // Motor direction control 1
const int IN2 = 7;  // Motor direction control 2

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  Serial.println("DC Motor Test with L298N");
  
  // Set pin modes
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Initially stop the motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  Serial.println("Setup complete. Starting motor test in 3 seconds...");
  delay(3000);
}

void loop() {
  // Test motor forward at increasing speeds
  Serial.println("Running motor forward at increasing speed...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  for (int speed = 50; speed <= 255; speed += 51) {
    Serial.print("Speed: ");
    Serial.println(speed);
    analogWrite(ENA, speed);
    delay(2000);
  }
  
  // Stop the motor
  Serial.println("Stopping motor...");
  analogWrite(ENA, 0);
  delay(2000);
  
  // Test motor backward at increasing speeds
  Serial.println("Running motor backward at increasing speed...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  for (int speed = 50; speed <= 255; speed += 51) {
    Serial.print("Speed: ");
    Serial.println(speed);
    analogWrite(ENA, speed);
    delay(2000);
  }
  
  // Stop the motor
  Serial.println("Stopping motor...");
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(2000);
  
  // Test quick direction changes at medium speed
  Serial.println("Testing direction changes...");
  analogWrite(ENA, 150);  // Set to medium speed
  
  Serial.println("Forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(3000);
  
  Serial.println("Backward");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(3000);
  
  // Stop the motor and wait before repeating test
  Serial.println("Test complete. Restarting in 5 seconds...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  delay(5000);
}