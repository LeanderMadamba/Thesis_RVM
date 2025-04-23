/*
 * RVM Controller
 * 
 * Main controller script for the Reverse Vending Machine Arduino component
 * This script:
 * 1. Receives a signal from Raspberry Pi (0 for waste, 1 for plastic)
 * 2. If plastic, measures weight using load cell
 * 3. Controls servo mechanisms based on weight (>50g or <=50g)
 * 4. Controls DC motor
 * 5. Monitors ultrasonic sensors for obstructions
 * 6. Sends SMS notification if obstructions detected
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - SIM800L v2 GSM Module
 * - 6V DC Motor with transistor driver
 * - MG995R Servo Motor (360 degrees)
 * - MG996R Servo Motor (160 degrees)
 * - 2x HC-SR04 Ultrasonic Sensors
 * - 10kg Load Cell with HX711 Amplifier
 */

#include <SoftwareSerial.h>
#include <Servo.h>
#include "HX711.h"

// Pin Definitions
// Communication pins
#define RPI_INPUT_PIN 4      // Pin to receive signal from Raspberry Pi
#define RPI_READY_PIN 5      // Pin to signal Arduino is ready for next item

// SIM800L GSM Module
#define GSM_RX_PIN 19       // Connect to SIM800L TXD
#define GSM_TX_PIN 18       // Connect to SIM800L RXD

// Servo Motors
#define SERVO_360_PIN 9     // MG995R 360-degree servo
#define SERVO_160_PIN 10    // MG996R 160-degree servo

// DC Motor
#define MOTOR_PIN 6         // DC motor control

// Load Cell
#define LOADCELL_DOUT_PIN 3
#define LOADCELL_SCK_PIN 2
#define CALIBRATION_FACTOR -455.0 // Adjust based on calibration

// Ultrasonic Sensors
#define TRIG_PIN_1 24
#define ECHO_PIN_1 25
#define TRIG_PIN_2 26
#define ECHO_PIN_2 27

// Constants
#define SOUND_SPEED 0.034   // Speed of sound in cm/microsecond
#define WEIGHT_THRESHOLD 50 // Weight threshold in grams
#define DISTANCE_THRESHOLD 10 // Distance threshold in cm for obstruction detection
#define PHONE_NUMBER "+639278557480" // Updated with country code format

// Global objects
SoftwareSerial gsmSerial(GSM_RX_PIN, GSM_TX_PIN);
Servo servo360;  // MG995R 360-degree servo
Servo servo160;  // MG996R 160-degree servo
HX711 scale;

// GSM variables
int _timeout;
String _buffer;

// State variables
bool rpiSignalReceived = false;
int plasticDetected = 0;  // 0 = waste, 1 = plastic
bool obstructionDetected = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  gsmSerial.begin(9600);
  
  Serial.println("RVM Controller Initializing...");
  
  // Set up pins
  pinMode(RPI_INPUT_PIN, INPUT);
  pinMode(RPI_READY_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  
  // Set initial states
  digitalWrite(RPI_READY_PIN, HIGH); // Signal ready to receive
  digitalWrite(MOTOR_PIN, LOW);      // Motor off
  digitalWrite(TRIG_PIN_1, LOW);
  digitalWrite(TRIG_PIN_2, LOW);
  
  // Initialize servo motors
  servo360.attach(SERVO_360_PIN);
  servo160.attach(SERVO_160_PIN);
  
  // Center servos initially
  servo360.write(90); // Stop position for continuous servo
  servo160.write(90); // Center position for standard servo
  
  // Initialize GSM module - simplified initialization
  _buffer.reserve(50);
  delay(3000); // Wait for GSM module to initialize
  Serial.println("GSM module initialized");
  
  // Initialize load cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  if (scale.is_ready()) {
    scale.set_scale(CALIBRATION_FACTOR);
    scale.tare(); // Reset scale to zero
    Serial.println("Load cell initialized");
  } else {
    Serial.println("Load cell not found. Check wiring!");
  }
  
  Serial.println("RVM Controller Ready");
}

void loop() {
  // Check for signal from Raspberry Pi
  if (digitalRead(RPI_INPUT_PIN) == HIGH && !rpiSignalReceived) {
    delay(50); // Debounce
    if (digitalRead(RPI_INPUT_PIN) == HIGH) {
      rpiSignalReceived = true;
      plasticDetected = 1; // Raspberry Pi sends HIGH for plastic
      Serial.println("Plastic detected signal received from Raspberry Pi");
      processItem();
    }
  } else if (digitalRead(RPI_INPUT_PIN) == LOW && !rpiSignalReceived) {
    delay(50); // Debounce
    if (digitalRead(RPI_INPUT_PIN) == LOW) {
      rpiSignalReceived = true;
      plasticDetected = 0; // Raspberry Pi sends LOW for waste
      Serial.println("Waste detected signal received from Raspberry Pi");
      processItem();
    }
  }
  
  // Check for obstructions
  checkForObstructions();
  
  // Reset state when Raspberry Pi signal goes back to neutral
  // For testing, we'll use a delay (in real system, would wait for RPI signal to change)
  if (rpiSignalReceived) {
    digitalWrite(RPI_READY_PIN, LOW); // Signal not ready for new item
    delay(5000); // Simulate processing time
    rpiSignalReceived = false;
    digitalWrite(RPI_READY_PIN, HIGH); // Signal ready for new item
    Serial.println("System ready for next item");
  }
}

void processItem() {
  if (plasticDetected) {
    // Process plastic item
    float weight = measureWeight();
    Serial.print("Measured weight: ");
    Serial.print(weight);
    Serial.println(" g");
    
    if (weight > WEIGHT_THRESHOLD) {
      // Weight exceeds threshold, activate MG995R servo (360-degree)
      activateServo360();
    } else {
      // Weight below threshold, activate MG996R servo (160-degree)
      activateServo160();
    }
  } else {
    // If not plastic, we don't need to measure weight or activate servos
    Serial.println("Waste item detected, bypassing weight measurement and servo activation");
  }
  
  // Activate motor in either case
  activateMotor();
}

float measureWeight() {
  Serial.println("Measuring weight...");
  if (scale.is_ready()) {
    // Take average of multiple readings for stability
    float weight = scale.get_units(10); // Average of 10 readings
    
    // Convert to grams (if your calibration factor is for kg)
    weight = weight * 1000.0;
    
    return weight;
  } else {
    Serial.println("Scale not ready!");
    return 0.0;
  }
}

void activateServo360() {
  Serial.println("Activating 360-degree servo (weight > 50g)");
  
  // Rotate clockwise at full speed
  servo360.write(0);
  delay(3000);
  
  // Stop
  servo360.write(90);
  delay(1000);
  
  Serial.println("360-degree servo operation complete");
}

void activateServo160() {
  Serial.println("Activating 160-degree servo (weight <= 50g)");
  
  // Move to minimum position
  for (int pos = 90; pos >= 10; pos -= 5) {
    servo160.write(pos);
    delay(50);
  }
  delay(1000);
  
  // Return to center position
  for (int pos = 10; pos <= 90; pos += 5) {
    servo160.write(pos);
    delay(50);
  }
  
  Serial.println("160-degree servo operation complete");
}

void activateMotor() {
  Serial.println("Activating DC motor");
  
  // Ramp up motor speed
  for (int speed = 0; speed <= 255; speed += 5) {
    analogWrite(MOTOR_PIN, speed);
    delay(20);
  }
  
  // Run at full speed for 3 seconds
  delay(3000);
  
  // Ramp down motor speed
  for (int speed = 255; speed >= 0; speed -= 5) {
    analogWrite(MOTOR_PIN, speed);
    delay(20);
  }
  
  // Ensure motor is stopped
  analogWrite(MOTOR_PIN, 0);
  
  Serial.println("DC motor operation complete");
}

float measureDistance(int trigPin, int echoPin) {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin, return the sound wave travel time in microseconds
  unsigned long duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  float distance = duration * SOUND_SPEED / 2;
  
  return distance;
}

void checkForObstructions() {
  // Measure distances from both sensors
  float distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
  float distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
  
  // Check if either sensor detects an obstruction
  if ((distance1 < DISTANCE_THRESHOLD && distance1 > 0) || 
      (distance2 < DISTANCE_THRESHOLD && distance2 > 0)) {
    
    if (!obstructionDetected) {
      obstructionDetected = true;
      Serial.println("Obstruction detected!");
      Serial.print("Sensor 1: ");
      Serial.print(distance1);
      Serial.print(" cm, Sensor 2: ");
      Serial.print(distance2);
      Serial.println(" cm");
      
      // Send SMS notification with updated method signature (no phone number parameter)
      sendSMS("RVM Alert: Obstruction detected. Please check the machine.");
    }
  } else {
    obstructionDetected = false;
  }
}

void sendSMS(String message) {
  Serial.println("Sending SMS...");
  
  // Set SMS text mode
  gsmSerial.println("AT+CMGF=1");
  delay(200);
  
  // Set recipient phone number
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(PHONE_NUMBER);
  gsmSerial.println("\"\r");
  delay(200);
  
  // Send the message content
  gsmSerial.print(message);
  delay(100);
  
  // Send Ctrl+Z to indicate end of message
  gsmSerial.println((char)26);
  delay(200);
  
  _buffer = readGSMResponse();
  Serial.println("SMS Sent!");
}

String readGSMResponse() {
  _timeout = 0;
  while (!gsmSerial.available() && _timeout < 12000) {
    delay(13);
    _timeout++;
  }
  if (gsmSerial.available()) {
    return gsmSerial.readString();
  }
  return ""; // Return empty string if no data available
} 