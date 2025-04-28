/*
 * Serial-Triggered Container Fullness Detection
 * 
 * This script uses 2 HC-SR04 ultrasonic sensors to detect whether two containers are full.
 * The detection process starts when the user inputs "1" in the serial monitor.
 * The system measures for 30 seconds total while checking if containers remain full for 10 consecutive seconds.
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - 2x HC-SR04 Ultrasonic Sensors
 */

// Pin definitions for Container 1 Sensor
const int trigPin1 = 34;
const int echoPin1 = 35;

// Pin definitions for Container 2 Sensor
const int trigPin2 = 36;
const int echoPin2 = 37;

// Container parameters (in centimeters)
const float container1Height = 90.0;  // Container 1 height
const float container2Height = 90.0;  // Container 2 height
const float fullThreshold = 10.0;     // Distance threshold for considering container full

// Timing parameters
const unsigned long totalMeasurementTime = 30000; // 30 seconds of total measurement
const unsigned long verificationTime = 10000;     // 10 seconds verification for fullness

// Global variables for serial handling
bool measurementInProgress = false;
char incomingByte;

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  
  // Configure Sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  Serial.println("Container Fullness Detection System");
  Serial.println("==================================");
  Serial.println("Enter '1' in the serial monitor to start measuring container fullness");
}

void loop() {
  // Check if data is available to read from Serial
  if (Serial.available() > 0 && !measurementInProgress) {
    // Read the incoming byte
    incomingByte = Serial.read();
    
    // Check if '1' was entered
    if (incomingByte == '1') {
      Serial.println("Starting measurement...");
      startContainerMeasurement();
    }
    
    // Clear any remaining characters in the buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
  
  delay(50); // Small delay for stability
}

void startContainerMeasurement() {
  measurementInProgress = true;
  
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
  
  // Run measurement loop for 30 seconds
  while ((millis() - startTime) < totalMeasurementTime) {
    // Get readings from both sensors
    distance1 = getUltrasonicDistance(trigPin1, echoPin1);
    Serial.println("distance1 :" + String(distance1));
    delay(50); // Small delay between sensor readings
    distance2 = getUltrasonicDistance(trigPin2, echoPin2);
    Serial.println("distance2 :" + String(distance2));
    
    currentTime = millis();
    
    // Process container 1 status
    if (isValidReading(distance1)) {
      fillPercentage1 = max(0, min(100, (container1Height - distance1) / container1Height * 100));
      isContainer1Full = (distance1 <= fullThreshold);
      
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
        Serial.println(" % full.");
      }
    }
    
    // Process container 2 status
    if (isValidReading(distance2)) {
      fillPercentage2 = max(0, min(100, (container2Height - distance2) / container2Height * 100));
      isContainer2Full = (distance2 <= fullThreshold);
      
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
        Serial.println(" % full.");
      }
    }
    
    // Display time remaining every 5 seconds
    //if ((millis() - startTime) % 5000 < 100) {
    //  int timeRemaining = (totalMeasurementTime - (millis() - startTime)) / 1000;
    //  Serial.print("Time remaining: ");
    //  Serial.print(timeRemaining);
    //  Serial.println(" seconds");
    //}
    
    delay(100); // Small delay for stability
  }
  
  // Measurement complete
  Serial.println("Measurement completed!");
  
  // Report final status if containers weren't verified as full
  if (!container1VerifiedFull) {
    Serial.print("Container #1 was not consistently full. Last reading: ");
    Serial.print(fillPercentage1, 1);
    Serial.println(" % full.");
  }
  
  if (!container2VerifiedFull) {
    Serial.print("Container #2 was not consistently full. Last reading: ");
    Serial.print(fillPercentage2, 1);
    Serial.println(" % full.");
  }
  
  Serial.println("Enter '1' to start a new measurement.");
  measurementInProgress = false;
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