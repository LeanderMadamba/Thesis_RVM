/*
 * HC-SR04 Ultrasonic Sensors Test
 * 
 * This sketch tests two HC-SR04 ultrasonic sensors by measuring
 * distance and displaying the values on the Serial Monitor.
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - 2x HC-SR04 Ultrasonic Sensors
 * 
 * Connections:
 * - Sensor 1:
 *   - VCC to Arduino 5V
 *   - Trig to Arduino Pin 24
 *   - Echo to Arduino Pin 25
 *   - GND to Arduino GND
 * - Sensor 2:
 *   - VCC to Arduino 5V
 *   - Trig to Arduino Pin 26
 *   - Echo to Arduino Pin 27
 *   - GND to Arduino GND
 */

// Define pins for the first ultrasonic sensor
#define TRIG_PIN_1 24
#define ECHO_PIN_1 25

// Define pins for the second ultrasonic sensor
#define TRIG_PIN_2 26
#define ECHO_PIN_2 27

// Constants
#define SOUND_SPEED 0.034 // Speed of sound in cm/microsecond

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Configure ultrasonic sensor pins
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  
  // Ensure trigger pins start low
  digitalWrite(TRIG_PIN_1, LOW);
  digitalWrite(TRIG_PIN_2, LOW);
  
  Serial.println("HC-SR04 Ultrasonic Sensors Test");
  Serial.println("--------------------------------");
  delay(1000);
}

void loop() {
  // Measure distance with the first sensor
  float distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
  
  // Measure distance with the second sensor
  float distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
  
  // Display readings
  Serial.print("Sensor 1 Distance: ");
  if (distance1 >= 400 || distance1 <= 2) {
    Serial.println("Out of range");
  } else {
    Serial.print(distance1);
    Serial.println(" cm");
  }
  
  Serial.print("Sensor 2 Distance: ");
  if (distance2 >= 400 || distance2 <= 2) {
    Serial.println("Out of range");
  } else {
    Serial.print(distance2);
    Serial.println(" cm");
  }
  
  Serial.println("--------------------------------");
  
  // Wait before next reading
  delay(500);
}

float measureDistance(int trigPin, int echoPin) {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin, return the sound wave travel time in microseconds
  unsigned long duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  float distance = duration * SOUND_SPEED / 2;
  
  return distance;
} 