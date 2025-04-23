/*
 * DC Motor Test
 * 
 * This sketch tests a 6V DC motor using direct control
 * with a single transistor.
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - 6V DC Motor
 * - NPN Transistor (e.g., TIP120, 2N2222, etc.)
 * - 1K resistor
 * - Diode (1N4001 or similar) for back-EMF protection
 * 
 * Connections:
 * - Arduino Pin 6 (PWM) through 1K resistor to transistor base
 * - Transistor collector to motor negative lead
 * - Transistor emitter to GND
 * - Motor positive lead to external 6V power supply
 * - Diode across motor terminals (cathode to positive)
 * - External power supply GND to Arduino GND
 */

// Define motor control pin
#define MOTOR_PIN 6  // PWM pin for speed control

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("DC Motor Test - Direct Control");
  
  // Set motor control pin as output
  pinMode(MOTOR_PIN, OUTPUT);
  
  // Initially ensure motor is stopped
  analogWrite(MOTOR_PIN, 0);
  
  Serial.println("Motor initialized. Starting test sequence...");
  delay(2000);
}

void loop() {
  // Test different motor speeds
  testMotorSpeed();
  
  // End of test cycle
  Serial.println("Test cycle complete. Restarting in 5 seconds...");
  delay(5000);
}

void testMotorSpeed() {
  Serial.println("Testing motor at different speeds:");
  
  // Test off state
  Serial.println("- Motor OFF");
  setMotorSpeed(0);
  delay(2000);
  
  // Test 25% speed
  Serial.println("- 25% speed");
  setMotorSpeed(64);  // ~25% duty cycle
  delay(3000);
  
  // Test 50% speed
  Serial.println("- 50% speed");
  setMotorSpeed(128); // ~50% duty cycle
  delay(3000);
  
  // Test 75% speed
  Serial.println("- 75% speed");
  setMotorSpeed(192); // ~75% duty cycle
  delay(3000);
  
  // Test 100% speed
  Serial.println("- 100% speed");
  setMotorSpeed(255); // 100% duty cycle
  delay(3000);
  
  // Test acceleration
  testAcceleration();
  
  // Stop motor
  stopMotor();
}

void testAcceleration() {
  Serial.println("Testing acceleration and deceleration:");
  
  // Accelerate from 0 to 100%
  Serial.println("- Accelerating...");
  for (int speed = 0; speed <= 255; speed += 5) {
    setMotorSpeed(speed);
    delay(50);
  }
  
  // Hold at max speed
  delay(2000);
  
  // Decelerate from 100% to 0
  Serial.println("- Decelerating...");
  for (int speed = 255; speed >= 0; speed -= 5) {
    setMotorSpeed(speed);
    delay(50);
  }
  
  stopMotor();
  delay(2000);
}

void setMotorSpeed(int speed) {
  // Set motor speed using PWM
  analogWrite(MOTOR_PIN, speed);
}

void stopMotor() {
  Serial.println("Stopping motor");
  // Disable motor
  analogWrite(MOTOR_PIN, 0);
} 