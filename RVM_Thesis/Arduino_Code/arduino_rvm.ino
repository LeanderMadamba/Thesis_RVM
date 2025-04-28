/*
 * RVM Controller - Main Script
 * 
 * Receives commands from Raspberry Pi via USB serial
 * Processes "PLASTIC" or "WASTE" commands
 * Controls servo mechanisms, DC motor, and monitors containers
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - SIM800L v2 GSM Module
 * - L298N Motor Driver + DC Worm Gear Motor
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
#define GSM_RX_PIN 19
#define GSM_TX_PIN 18

// Servo Motors
#define SERVO_360_PIN 9
#define SERVO_160_PIN 10

// DC Motor with L298N Driver
#define MOTOR_ENA 6
#define MOTOR_IN1 8
#define MOTOR_IN2 7

// Load Cell
#define LOADCELL_DOUT_PIN 3
#define LOADCELL_SCK_PIN 2
#define CALIBRATION_FACTOR 750

// Ultrasonic Sensors
#define TRIG_PIN_1 24
#define ECHO_PIN_1 25
#define TRIG_PIN_2 22
#define ECHO_PIN_2 23

// Constants
#define SOUND_SPEED 0.034
#define WEIGHT_THRESHOLD 50
#define DISTANCE_THRESHOLD 10
#define PHONE_NUMBER "+639278557480"

// Global objects
SoftwareSerial gsmSerial(GSM_RX_PIN, GSM_TX_PIN);
Servo servo360;
Servo servo160;
HX711 scale;

// GSM variables
int _timeout;
String _buffer;

// State variables
bool systemBusy = false;
bool obstructionDetected = false;
String inputBuffer = "";
bool inputComplete = false;

// Container parameters for ultrasonic sensors
const float container1Height = 90.0;
const float container2Height = 90.0;
const unsigned long totalMeasurementTime = 30000; // 30 seconds of total measurement
const unsigned long verificationTime = 10000;     // 10 seconds verification for fullness

void setup() {
  Serial.begin(9600);
  gsmSerial.begin(9600);
  
  // Initialize DC Motor pins
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
  
  // Ultrasonic sensor pins
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  digitalWrite(TRIG_PIN_1, LOW);
  digitalWrite(TRIG_PIN_2, LOW);
  
  // Initialize servo motors
  servo360.attach(SERVO_360_PIN);
  servo160.attach(SERVO_160_PIN);
  servo360.write(90); // Stop position
  servo160.write(90); // Center position
  
  // Initialize GSM module
  _buffer.reserve(50);
  delay(3000);
  Serial.println("INFO: GSM module initialized");
  
  // Initialize load cell
  Serial.println("Initializing load cell...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.power_down();
  delay(500);
  scale.power_up();
  
  if (scale.is_ready()) {
    scale.set_scale(CALIBRATION_FACTOR);
    Serial.println("Taring load cell (remove any weight now)...");
    scale.tare();
    delay(1000);
    scale.tare(); // Second tare for stability
    delay(1000);
    Serial.println("Load cell initialized and tared");
    
    float test_reading = scale.get_units(5);
    Serial.print("Initial reading after tare: ");
    Serial.println(test_reading);
    
    if (abs(test_reading) > 1.0) {
      Serial.println("WARNING: Load cell may not be properly calibrated");
    }
  } else {
    Serial.println("Load cell not found. Check wiring!");
  }
  
  systemBusy = false;
  Serial.println("READY: RVM Controller ready for commands");
}

void loop() {
  checkSerialCommands();
  
  static unsigned long lastObstructionCheck = 0;
  if (millis() - lastObstructionCheck > 5000) { // Check every 5 seconds
    checkForObstructions();
    lastObstructionCheck = millis();
  }
  
  delay(10);
}

void checkSerialCommands() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      inputComplete = true;
    } else {
      inputBuffer += inChar;
    }
  }
  
  if (inputComplete) {
    processCommand(inputBuffer);
    inputBuffer = "";
    inputComplete = false;
  }
}

void processCommand(String command) {
  command.trim();
  Serial.print("Processing command: ");
  Serial.println(command);
  
  if (command == "HELLO") {
    Serial.println("READY: Hello from Arduino RVM Controller");
  }
  else if (command == "STATUS") {
    if (systemBusy) {
      Serial.println("BUSY: System is currently processing");
    } else {
      Serial.println("READY: System is ready for next command");
    }
  }
  else if (command == "PLASTIC" && !systemBusy) {
    systemBusy = true;
    Serial.println("BUSY: Processing plastic item");
    processPlasticItem();
    systemBusy = false;
    Serial.println("READY: Plastic processing complete");
  }
  else if (command == "WASTE" && !systemBusy) {
    systemBusy = true;
    Serial.println("BUSY: Processing waste item");
    processWasteItem();
    systemBusy = false;
    Serial.println("READY: Waste processing complete");
  }
  else if (command == "SHUTDOWN") {
    Serial.println("INFO: Shutting down...");
    stopAllMotors();
  }
  else if (command == "CHECK_CONTAINERS") {
    Serial.println("INFO: Checking container fullness...");
    startContainerMeasurement();
    Serial.println("READY: Container check complete");
  }
  else if (command == "MOTOR_TEST") {
    Serial.println("INFO: Running DC motor test...");
    testDcMotor();
    Serial.println("READY: DC motor test complete");
  }
  else if (command == "WEIGHT_TEST") {
    Serial.println("INFO: Running weight measurement test...");
    float weight = measureWeight();
    Serial.print("Weight measurement: ");
    Serial.print(weight);
    Serial.println(" g");
    Serial.println("READY: Weight test complete");
  }
  else {
    Serial.print("ERROR: Unknown command: ");
    Serial.println(command);
  }
}

void processPlasticItem() {
  float weight = measureWeight();
  Serial.print("INFO: Measured weight: ");
  Serial.print(weight);
  Serial.println(" g");
  
  if (weight > WEIGHT_THRESHOLD) {
    activateServo360();
  } else {
    activateServo160();
  }
  
  Serial.println("Checking for fullness of containers...");
  startContainerMeasurement();
  
  activateDcMotor();
  
}

void processWasteItem() {
  Serial.println("INFO: Waste item detected, bypassing weight measurement");
  
  activateServo360();
  
  Serial.println("Checking for fullness of containers...");
  startContainerMeasurement();
  
  //activateDcMotor();
  
}

float measureWeight() {
  Serial.println("INFO: Measuring weight...");
  if (scale.is_ready()) {
    float weight = 0;
    int validReadings = 0;
    
    for (int i = 0; i < 5; i++) {
      float reading = scale.get_units();
      if (reading >= 0 && reading < 1000) {
        weight += reading;
        validReadings++;
      }
      delay(100);
    }
    
    if (validReadings > 0) {
      weight = weight / validReadings;
    } else {
      weight = 0;
    }
    
    weight = weight * 1000.0;
    
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
  servo360.write(0);
  delay(3000);
  servo360.write(90);
  delay(1000);
  Serial.println("INFO: 360-degree servo operation complete");
}

void activateServo160() {
  Serial.println("INFO: Activating 160-degree servo (weight <= 50g)");
  for (int pos = 90; pos >= 10; pos -= 5) {
    servo160.write(pos);
    delay(50);
  }
  delay(1000);
  for (int pos = 10; pos <= 90; pos += 5) {
    servo160.write(pos);
    delay(50);
  }
  Serial.println("INFO: 160-degree servo operation complete");
}

void activateDcMotor() {
  Serial.println("INFO: Activating DC motor with L298N driver");
  
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  
  for (int speed = 50; speed <= 255; speed += 15) {
    analogWrite(MOTOR_ENA, speed);
    delay(100);
  }
  
  analogWrite(MOTOR_ENA, 255);
  delay(3000);
  
  for (int speed = 255; speed >= 0; speed -= 15) {
    analogWrite(MOTOR_ENA, speed);
    delay(100);
  }
  
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  
  Serial.println("INFO: DC motor operation complete");
}

void testDcMotor() {
  Serial.println("Running motor forward at increasing speed...");
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  
  for (int speed = 50; speed <= 255; speed += 51) {
    Serial.print("Speed: ");
    Serial.println(speed);
    analogWrite(MOTOR_ENA, speed);
    delay(1000);
  }
  
  Serial.println("Stopping motor...");
  analogWrite(MOTOR_ENA, 0);
  delay(1000);
  
  Serial.println("Running motor backward at increasing speed...");
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  
  for (int speed = 50; speed <= 255; speed += 51) {
    Serial.print("Speed: ");
    Serial.println(speed);
    analogWrite(MOTOR_ENA, speed);
    delay(1000);
  }
  
  Serial.println("Stopping motor...");
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  delay(1000);
}


// Integrated from Thesis_containers.ino
void startContainerMeasurement() {
  // Variables for tracking fullness status
  unsigned long container1FullStartTime = 0;
  unsigned long container2FullStartTime = 0;
  bool container1PreviouslyFull = false;
  bool container2PreviouslyFull = false;
  bool container1VerifiedFull = false;
  bool container2VerifiedFull = false;
  
  // Variables for measurements
  float distance1, distance2;
  float fillPercentage1, fillPercentage2;
  bool isContainer1Full = false;
  bool isContainer2Full = false;
  
  // Variables for timing
  unsigned long startTime = millis();
  unsigned long currentTime;
  
  Serial.println("Measuring containers for 30 seconds...");
  
  // Run measurement loop for specified time
  while ((millis() - startTime) < totalMeasurementTime) {
    // Get readings from both sensors
    distance1 = getUltrasonicDistance(TRIG_PIN_1, ECHO_PIN_1);
    delay(50);
    distance2 = getUltrasonicDistance(TRIG_PIN_2, ECHO_PIN_2);
    
    currentTime = millis();
    
    // Process container 1 status
    if (isValidReading(distance1)) {
      fillPercentage1 = max(0, min(100, (container1Height - distance1) / container1Height * 100));
      isContainer1Full = (distance1 <= DISTANCE_THRESHOLD);
      
      // Start or reset timer for container 1
      if (isContainer1Full && !container1PreviouslyFull) {
        container1FullStartTime = currentTime;
        container1PreviouslyFull = true;
      } else if (!isContainer1Full) {
        container1PreviouslyFull = false;
        container1VerifiedFull = false;
      }
      
      // Check if container 1 has been full for the verification time
      if (isContainer1Full && container1PreviouslyFull && 
          (currentTime - container1FullStartTime >= verificationTime) && 
          !container1VerifiedFull) {
        container1VerifiedFull = true;
        Serial.print("Container #1 is ");
        Serial.print(fillPercentage1, 1);
        Serial.println("% full.");
      }
    }
    
    // Process container 2 status
    if (isValidReading(distance2)) {
      fillPercentage2 = max(0, min(100, (container2Height - distance2) / container2Height * 100));
      isContainer2Full = (distance2 <= DISTANCE_THRESHOLD);
      
      // Start or reset timer for container 2
      if (isContainer2Full && !container2PreviouslyFull) {
        container2FullStartTime = currentTime;
        container2PreviouslyFull = true;
      } else if (!isContainer2Full) {
        container2PreviouslyFull = false;
        container2VerifiedFull = false;
      }
      
      // Check if container 2 has been full for the verification time
      if (isContainer2Full && container2PreviouslyFull && 
          (currentTime - container2FullStartTime >= verificationTime) && 
          !container2VerifiedFull) {
        container2VerifiedFull = true;
        Serial.print("Container #2 is ");
        Serial.print(fillPercentage2, 1);
        Serial.println("% full.");
      }
    }
    
    delay(100); // Small delay for stability
  }
  
  // Measurement complete
  Serial.println("Container measurement completed!");
  
  // Report final status if containers weren't verified as full
  if (!container1VerifiedFull) {
    Serial.print("Container #1 was not consistently full. Last reading: ");
    Serial.print(fillPercentage1, 1);
    Serial.println("% full.");
  }
  
  if (!container2VerifiedFull) {
    Serial.print("Container #2 was not consistently full. Last reading: ");
    Serial.print(fillPercentage2, 1);
    Serial.println("% full.");
  }
}

float getUltrasonicDistance(int trigPin, int echoPin) {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin (returns the travel time in microseconds)
  long duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance (speed of sound = 0.034 cm/microsecond)
  // Distance = (Time x Speed) / 2 (divided by 2 because sound travels to object and back)
  float distance = (duration * 0.034) / 2.0;
  
  return distance;
}

bool isValidReading(float distance) {
  // Check if reading is within reasonable bounds
  // HC-SR04 has a typical range of 2cm to 400cm
  return (distance >= 2.0 && distance <= 400.0);
}

void sendSMS(String message) {
  Serial.println("INFO: Sending SMS...");
  
  gsmSerial.println("AT");
  delay(500);
  _buffer = readGSMResponse();
  Serial.println("GSM Response to AT: " + _buffer);
  
  gsmSerial.println("AT+CMGF=1");
  delay(500);
  _buffer = readGSMResponse();
  Serial.println("GSM Response to CMGF: " + _buffer);
  
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(PHONE_NUMBER);
  gsmSerial.println("\"\r");
  delay(1000);
  _buffer = readGSMResponse();
  
  gsmSerial.print(message);
  delay(500);
  
  gsmSerial.write(26);  // ASCII code for Ctrl+Z
  delay(5000);
  
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
  return "";
}

void stopAllMotors() {
  servo360.write(90);
  servo160.write(90);
  
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
} 