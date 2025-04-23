/*
 * Load Cell Measurement System
 * 
 * This sketch uses the HX711_ADC library to read weight data from a load cell
 * and display the values on the Serial Monitor.
 * 
 * Hardware:
 * - Arduino Mega 2560
 * - Load Cell
 * - HX711 Load Cell Amplifier
 * 
 * Connections:
 * HX711 to Arduino:
 * - VCC to 5V
 * - GND to GND
 * - DT (Data) to Pin 4
 * - SCK (Clock) to Pin 5
 * 
 * Load Cell to HX711:
 * - Red wire to E+
 * - Black wire to E-
 * - White wire to A+
 * - Green wire to A-
 * 
 * This sketch requires the HX711_ADC library by Olav Kallhovd:
 * https://github.com/olkal/HX711_ADC
 * Install via Library Manager in Arduino IDE
 */

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// HX711 circuit wiring
const int HX711_dout = 4; // mcu > HX711 dout pin
const int HX711_sck = 5;  // mcu > HX711 sck pin

// HX711 constructor
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

void setup() {
  Serial.begin(57600);
  delay(10);
  
  Serial.println();
  Serial.println("Load Cell Measurement System");
  Serial.println("----------------------------");
  Serial.println("Starting...");

  LoadCell.begin();
  //LoadCell.setReverseOutput(); // uncomment to turn a negative output value to positive
  
  // Calibration value - adjust this based on your specific load cell calibration
  float calibrationValue = 696.0;
  
#if defined(ESP8266)|| defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  // Stabilizing time to improve precision right after power-up
  unsigned long stabilizingtime = 2000;
  boolean _tare = true; // set this to false if you don't want tare to be performed at startup
  
  LoadCell.start(stabilizingtime, _tare);
  
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1); // Stop if not found
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
    Serial.println("Send 't' from serial terminal to tare the scale");
    Serial.println("----------------------------");
  }
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; // increase value to slow down serial print activity

  // Check for new data/start next conversion
  if (LoadCell.update()) newDataReady = true;

  // Get smoothed value from the dataset
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float weight = LoadCell.getData();
      Serial.print("Weight: ");
      Serial.print(weight, 2); // Print with 2 decimal places
      Serial.println(" g");
      
      newDataReady = 0;
      t = millis();
    }
  }

  // Receive command from serial terminal, send 't' to initiate tare operation
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      Serial.println("Tare operation started");
      LoadCell.tareNoDelay();
    }
  }

  // Check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}

/*
 * Calibration Instructions:
 * 
 * 1. Set calibrationValue to an initial value (default is 696.0)
 * 2. Upload the sketch and open Serial Monitor at 57600 baud
 * 3. Send 't' to tare the scale when empty
 * 4. Place a known weight on the scale (e.g., 100g)
 * 5. Note the displayed weight
 * 6. Adjust calibrationValue using this formula:
 *    new_factor = old_factor * (known_weight / displayed_weight)
 * 7. Update the calibrationValue in the code
 * 8. Repeat steps 2-7 until the displayed weight matches the known weight
 */ 