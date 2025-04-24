/*
 * RVM Controller - Serial Communication Version
 * 
 * Main controller script for the Reverse Vending Machine Arduino component
 * This script:
 * 1. Receives commands from Raspberry Pi via USB serial
 * 2. Processes "PLASTIC" or "WASTE" commands
 * 3. If plastic, measures weight using load cell
 * 4. Controls servo mechanisms based on weight (>50g or <=50g)
 * 5. Controls DC motor
 * 6. Monitors ultrasonic sensors for obstructions
 * 7. Sends SMS notification if obstructions detected
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
#define TRIG_PIN_1 30
#define ECHO_PIN_1 32
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
bool systemBusy = false;
bool obstructionDetected = false;
String inputBuffer = "";
bool inputComplete = false;

void setup() {
  // Initialize serial communication with Raspberry Pi
  Serial.begin(9600);
  
  // Initialize GSM serial
  gsmSerial.begin(9600);
  
  // Initialize pins
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  
  // Set initial states
  digitalWrite(MOTOR_PIN, LOW); // Motor off
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
  Serial.println("INFO: GSM module initialized");
  
  // Initialize load cell with proper reset sequence
  Serial.println("Initializing load cell...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  // Reset the load cell
  scale.power_down();
  delay(500);
  scale.power_up();
  
  if (scale.is_ready()) {
    scale.set_scale(CALIBRATION_FACTOR);
    
    // Multiple tare to ensure stability
    Serial.println("Taring load cell (remove any weight now)...");
    scale.tare(); // First tare
    delay(1000);
    scale.tare(); // Second tare for stability
    delay(1000);
    Serial.println("Load cell initialized and tared");
    
    // Check if reading is near zero after tare
    float test_reading = scale.get_units(5);
    Serial.print("Initial reading after tare: ");
    Serial.println(test_reading);
    
    if (abs(test_reading) > 1.0) {
      Serial.println("WARNING: Load cell may not be properly calibrated");
    }
  } else {
    Serial.println("Load cell not found. Check wiring!");
  }
  
  // Signal that Arduino is ready
  systemBusy = false;
  Serial.println("READY: RVM Controller ready for commands");
}

void loop() {
  // Check for incoming serial commands
  checkSerialCommands();
  
  // Check for obstructions
  // checkForObstructions();
  
  // Small delay to prevent CPU overuse
  delay(10);
}

void checkSerialCommands() {
  // Read serial input
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Collect characters until newline
    if (inChar == '\n') {
      inputComplete = true;
    } else {
      inputBuffer += inChar;
    }
  }
  
  // Process command if complete
  if (inputComplete) {
    processCommand(inputBuffer);
    
    // Clear the buffer for next command
    inputBuffer = "";
    inputComplete = false;
  }
}

void processCommand(String command) {
  command.trim(); // Remove any whitespace
  Serial.print("Processing command: ");
  Serial.println(command);
  
  if (command == "HELLO") {
    // Handshake
    Serial.println("READY: Hello from Arduino RVM Controller");
  }
  else if (command == "STATUS") {
    // Report current status
    if (systemBusy) {
      Serial.println("BUSY: System is currently processing");
    } else {
      Serial.println("READY: System is ready for next command");
    }
  }
  else if (command == "PLASTIC" && !systemBusy) {
    // Process plastic item
    systemBusy = true;
    Serial.println("BUSY: Processing plastic item");
    processPlasticItem();
    systemBusy = false;
    Serial.println("READY: Plastic processing complete");
  }
  else if (command == "WASTE" && !systemBusy) {
    // Process waste item
    systemBusy = true;
    Serial.println("BUSY: Processing waste item");
    processWasteItem();
    systemBusy = false;
    Serial.println("READY: Waste processing complete");
  }
  else if (command == "SHUTDOWN") {
    // Prepare for shutdown
    Serial.println("INFO: Shutting down...");
    // Stop any active operations
    stopAllMotors();
  }
  else if (command == "CHECK_OBSTRUCTIONS") {
    // Explicitly check for obstructions when requested
    Serial.println("INFO: Checking for obstructions...");
    
    // Force the obstruction flag to reset so we can detect new obstructions
    obstructionDetected = false;
    
    // Run the obstruction check directly
    checkForObstructions();
    
    // Send back status
    if (obstructionDetected) {
      Serial.println("WARNING: Obstruction detected during explicit check!");
    } else {
      Serial.println("INFO: No obstructions detected during explicit check");
    }
    
    Serial.println("READY: Obstruction check complete");
  }
  else {
    // Unknown command
    Serial.print("ERROR: Unknown command: ");
    Serial.println(command);
  }
}

void processPlasticItem() {
  // Measure weight
  float weight = measureWeight();
  Serial.print("INFO: Measured weight: ");
  Serial.print(weight);
  Serial.println(" g");
  
  // No obstruction check here - we'll check after servos
  
  if (weight > WEIGHT_THRESHOLD) {
    // Weight exceeds threshold, activate MG995R servo (360-degree)
    activateServo360();
  } else {
    // Weight below threshold, activate MG996R servo (160-degree)
    activateServo160();
  }
  
  // Now check for obstructions after servo activation
  Serial.println("Checking for obstructions after servo activation...");
  checkForObstructions();
  
  // Activate motor
  activateMotor();
  
  // Final check for obstructions after everything is complete
  //Serial.println("Final obstruction check...");
  //checkForObstructions();
}

void processWasteItem() {
  // For waste items, we skip weight measurement
  Serial.println("INFO: Waste item detected, bypassing weight measurement");
  
  // No obstruction check here - we'll check after servo
  
  // Activate 360 degree servo for waste items
  activateServo360();
  
  // Now check for obstructions after servo activation
  Serial.println("Checking for obstructions after servo activation...");
  checkForObstructions();
  
  // Activate motor
  activateMotor();
  
  // Final check for obstructions after everything is complete
  Serial.println("Final obstruction check...");
  checkForObstructions();
}

float measureWeight() {
  Serial.println("INFO: Measuring weight...");
  if (scale.is_ready()) {
    // Take average of multiple readings for stability
    float weight = 0;
    int validReadings = 0;
    
    // Take 10 readings and average the valid ones
    for (int i = 0; i < 10; i++) {
      float reading = scale.get_units();
      
      // Filter out obviously invalid readings (negative or extremely large)
      if (reading >= 0 && reading < 1000) {
        weight += reading;
        validReadings++;
      }
      delay(100);
    }
    
    // Calculate average if we have valid readings
    if (validReadings > 0) {
      weight = weight / validReadings;
    } else {
      weight = 0; // Default to zero if all readings were invalid
    }
    
    // Convert to grams (if your calibration factor is for kg)
    weight = weight * 1000.0;
    
    // Print raw value for debugging
    Serial.print("Raw weight value (g): ");
    Serial.println(weight);
    
    return weight;
  } else {
    Serial.println("ERROR: Scale not ready!");
    return 0.0;
  }
}

void activateServo360() {
  Serial.println("INFO: Activating 360-degree servo (weight > 50g)");
  
  // Rotate clockwise at full speed
  servo360.write(0);
  delay(3000);
  
  // Stop
  servo360.write(90);
  delay(1000);
  
  Serial.println("INFO: 360-degree servo operation complete");
}

void activateServo160() {
  Serial.println("INFO: Activating 160-degree servo (weight <= 50g)");
  
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
  
  Serial.println("INFO: 160-degree servo operation complete");
}

void activateMotor() {
  Serial.println("INFO: Activating DC motor");
  
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
  
  Serial.println("INFO: DC motor operation complete");
}

float measureDistance(int trigPin, int echoPin) {
  // Take average of multiple measurements for stability
  float totalDistance = 0;
  int validReadings = 0;
  int attempts = 5; // Make 5 attempts to get a valid reading
  
  for (int i = 0; i < attempts; i++) {
    // Clear the trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Set the trigger pin HIGH for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Read the echo pin, return the sound wave travel time in microseconds
    // Adjust timeout to avoid waiting too long (15000µs = ~2.5m range)
    unsigned long duration = pulseIn(echoPin, HIGH, 15000);
    
    // If duration is 0, the pulseIn timed out (no echo detected)
    if (duration > 0) {
      // Calculate the distance
      float distance = duration * SOUND_SPEED / 2;
      
      // Only accept reasonable distances (0 to 200cm)
      if (distance > 0 && distance < 200) {
        totalDistance += distance;
        validReadings++;
      }
    }
    
    // Small delay between measurements
    delay(10);
  }
  
  // Calculate average distance if we have valid readings
  if (validReadings > 0) {
    return totalDistance / validReadings;
  } else {
    return -1; // Return -1 to indicate no valid reading
  }
}

void checkForObstructions() {
  // Measure distances from both sensors
  float distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
  float distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);
  
  // Log the distances for troubleshooting (but not as debug spam)
  //Serial.print("Sensor readings - S1: ");
  //Serial.print(distance1);
  //Serial.print(" cm, S2: ");
  //Serial.print(distance2);
  //Serial.println(" cm");
  
  // Check if either sensor detects a valid obstruction
  // -1 means no valid reading was obtained
  bool sensor1_obstruction = (distance1 > 0 && distance1 < DISTANCE_THRESHOLD);
  bool sensor2_obstruction = (distance2 > 0 && distance2 < DISTANCE_THRESHOLD);
  
  if (sensor1_obstruction || sensor2_obstruction) {
    // Only send alert if this is a new obstruction
    if (!obstructionDetected) {
      obstructionDetected = true;
      Serial.println("WARNING: Obstruction detected!");
      
      // Ensure the GSM module is ready
      gsmSerial.println("AT");
      delay(500);
      
      // Build message with sensor information
      String message = "RVM Alert: Obstruction detected.";
      
      // Only include valid sensor readings
      if (sensor1_obstruction) {
        message += " S1:" + String(distance1, 1) + "cm";
      }
      if (sensor2_obstruction) {
        message += " S2:" + String(distance2, 1) + "cm";
      }
      
      // Send the SMS
      sendSMS(message);
    }
  } else {
    // If obstruction is cleared, update status
    if (obstructionDetected) {
      obstructionDetected = false;
      Serial.println("INFO: Obstruction cleared");
    }
  }
}

// Updated GSM functions based on the working implementation
void sendSMS(String message) {
  Serial.println("INFO: Sending SMS...");
  
  // Reset the GSM module with AT command first
  gsmSerial.println("AT");
  delay(500);
  _buffer = readGSMResponse();
  Serial.println("GSM Response to AT: " + _buffer);
  
  // Set SMS text mode
  gsmSerial.println("AT+CMGF=1");
  delay(500);
  _buffer = readGSMResponse();
  Serial.println("GSM Response to CMGF: " + _buffer);
  
  // Set recipient phone number
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(PHONE_NUMBER);
  gsmSerial.println("\"\r");
  delay(1000);  // Increased delay for network response
  _buffer = readGSMResponse();
  
  // Send the message content
  gsmSerial.print(message);
  delay(500);
  
  // Send Ctrl+Z to indicate end of message
  gsmSerial.write(26);  // ASCII code for Ctrl+Z
  delay(5000);  // Increased delay to allow SMS to be sent
  
  _buffer = readGSMResponse();
  
  if (_buffer.indexOf("OK") != -1 || _buffer.indexOf("+CMGS") != -1) {
    Serial.println("INFO: SMS Sent successfully!");
  } else {
    Serial.println("WARNING: SMS may not have been sent. GSM Response: " + _buffer);
  }
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

void stopAllMotors() {
  // Stop all motors and servos
  servo360.write(90);  // Stop position
  servo160.write(90);  // Center position
  analogWrite(MOTOR_PIN, 0);  // Stop DC motor
} 