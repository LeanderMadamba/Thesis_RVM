/*
 * L298N Motor Driver Test Sketch
 * 
 * This sketch tests the L298N motor driver with a DC motor
 * It demonstrates:
 * 1. Forward rotation
 * 2. Reverse rotation
 * 3. Speed control
 * 4. Stop function
 * 
 * Connections:
 * L298N -> Arduino
 * ENA -> Pin 6 (PWM)
 * IN1 -> Pin 7
 * IN2 -> Pin 8
 * 12V -> Power Supply (6-12V)
 * GND -> Arduino GND
 * 5V -> Arduino 5V (optional)
 * 
 * Motor -> L298N
 * Motor + -> OUT1
 * Motor - -> OUT2
 */

// Motor control pins
#define ENA_PIN 6    // PWM pin for speed control
#define IN1_PIN 7    // Direction control pin 1
#define IN2_PIN 8    // Direction control pin 2

// Speed settings (0-255)
#define MIN_SPEED 50
#define MAX_SPEED 255
#define SPEED_STEP 5

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("L298N Motor Test");
  Serial.println("----------------");
  
  // Initialize motor control pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  
  // Set initial motor state (stopped)
  stopMotor();
  
  Serial.println("Motor initialized. Ready for commands.");
  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 'f':  // Forward
        Serial.println("Moving forward...");
        moveForward();
        break;
        
      case 'r':  // Reverse
        Serial.println("Moving reverse...");
        moveReverse();
        break;
        
      case 's':  // Stop
        Serial.println("Stopping motor...");
        stopMotor();
        break;
        
      case 't':  // Test sequence
        Serial.println("Running test sequence...");
        runTestSequence();
        break;
        
      case 'h':  // Help
        printHelp();
        break;
        
      default:
        Serial.println("Unknown command. Type 'h' for help.");
        break;
    }
  }
}

void moveForward() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  rampSpeed(MAX_SPEED);
}

void moveReverse() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  rampSpeed(MAX_SPEED);
}

void stopMotor() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, 0);
}

void rampSpeed(int targetSpeed) {
  // Ramp up speed gradually
  for (int speed = 0; speed <= targetSpeed; speed += SPEED_STEP) {
    analogWrite(ENA_PIN, speed);
    delay(50);
  }
}

void runTestSequence() {
  // Forward test
  Serial.println("Testing forward movement...");
  moveForward();
  delay(2000);
  
  // Stop
  Serial.println("Stopping...");
  stopMotor();
  delay(1000);
  
  // Reverse test
  Serial.println("Testing reverse movement...");
  moveReverse();
  delay(2000);
  
  // Stop
  Serial.println("Stopping...");
  stopMotor();
  delay(1000);
  
  // Speed test
  Serial.println("Testing speed control...");
  for (int speed = MIN_SPEED; speed <= MAX_SPEED; speed += SPEED_STEP) {
    Serial.print("Speed: ");
    Serial.println(speed);
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, speed);
    delay(500);
  }
  
  // Final stop
  stopMotor();
  Serial.println("Test sequence complete!");
}

void printHelp() {
  Serial.println("\nAvailable commands:");
  Serial.println("f - Move forward");
  Serial.println("r - Move reverse");
  Serial.println("s - Stop motor");
  Serial.println("t - Run test sequence");
  Serial.println("h - Show this help message");
  Serial.println();
} 