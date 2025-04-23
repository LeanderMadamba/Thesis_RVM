/*
 * SIM800L GSM Module Test - Updated
 * 
 * This sketch tests the SIM800L GSM module with functionality to:
 * - Send SMS messages
 * - Receive SMS messages
 * - Make phone calls
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - SIM800L v2 GSM Module
 * 
 * Connections:
 * - SIM800L VCC to external 3.7-4.2V power supply
 * - SIM800L GND to Arduino GND
 * - SIM800L RXD to Arduino Pin 18 (TX1)
 * - SIM800L TXD to Arduino Pin 19 (RX1)
 * - No RST connection needed in this implementation
 */

#include <SoftwareSerial.h>

// Initialize software serial for SIM800L
SoftwareSerial sim(19, 18); // RX, TX pins for GSM module

// Define the recipient phone number with country code
String number = "+639278557480"; // Change with your number

int _timeout;
String _buffer;

void setup() {
  // Initialize serial communications
  Serial.begin(9600);
  _buffer.reserve(50);
  
  Serial.println("SIM800L GSM Module Test - Updated Version");
  Serial.println("Initializing...");
  
  // Initialize GSM module
  sim.begin(9600);
  delay(3000); // Give time for the module to initialize
  
  Serial.println("System Started...");
  Serial.println("Type commands:");
  Serial.println("- 's' to send an SMS");
  Serial.println("- 'r' to receive SMS");
  Serial.println("- 'c' to make a call");
}

void loop() {
  // Check for commands from Serial Monitor
  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case 's':
        SendMessage();
        break;
      case 'r':
        RecieveMessage();
        break;
      case 'c':
        callNumber();
        break;
    }
  }
  
  // Forward any response from the GSM module to the Serial Monitor
  if (sim.available() > 0) {
    Serial.write(sim.read());
  }
}

void SendMessage() {
  Serial.println("Sending Message...");
  sim.println("AT+CMGF=1");    // Sets the GSM Module in Text Mode
  delay(200);
  
  Serial.println("Setting recipient number...");
  sim.println("AT+CMGS=\"" + number + "\"\r"); // Mobile phone number to send message
  delay(200);
  
  String SMS = "RVM Project: GSM Module Test Successful!";
  sim.println(SMS);
  delay(100);
  
  sim.println((char)26); // ASCII code of CTRL+Z to indicate end of message
  delay(200);
  
  _buffer = _readSerial();
  Serial.println("Message sent!");
}

void RecieveMessage() {
  Serial.println("Setting up to receive SMS...");
  sim.println("AT+CMGF=1");
  delay(200);
  sim.println("AT+CNMI=1,2,0,0,0"); // AT Command to receive a live SMS
  delay(200);
  Serial.println("SMS receive mode active. New messages will display automatically.");
}

String _readSerial() {
  _timeout = 0;
  while (!sim.available() && _timeout < 12000) {
    delay(13);
    _timeout++;
  }
  if (sim.available()) {
    return sim.readString();
  }
  return ""; // Add return for case when no data is available
}

void callNumber() {
  Serial.println("Calling " + number);
  sim.print("ATD");
  sim.print(number);
  sim.print(";\r\n");
  _buffer = _readSerial();
  Serial.println(_buffer);
} 