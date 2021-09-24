/* AARC Sensors & Prospecting ---> Author: John Stanfield _-_-_-_-_-_-
 WOB Sensor Code - Testing File (one module)  -_-_-_-_-_-_-_-_-
-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
 Notation: _______________________________________________________________________________________________________
 DR_Cell = Load cell for drill
 EX_Cell = Load cell for extraction system
 LoadCell = load cell module
 HX711_ADC = the adc used within our sensor system
 data = data out pin
 clck = serial clock pin
 Serial.println = prints to serial port with new line command
 calval_ = calibration value set from either hand calc or tare operation (calibration file)
_______________________________________________________________________________________________________________
*/

#include <HX711_ADC.h> // Include ADC Libraries

// declare pins: ------------- Must do before operation
const int HX711_data_1 = 12;
const int HX711_clck_1 = 13;
const int LoadCellLeftData = 37;
const int LoadCellLeftClock = 39;
const int LoadCellRightData = 41;
const int LoadCellRightClock = 43;

//Match pins to adc/sensor modules
HX711_ADC LoadCell(LoadCellRightData, LoadCellRightClock); // Module 1 for drilling system

unsigned long t = 0;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting...");

  float calval_LoadCell;

  // Define calibration values given by the calibration script
  calval_LoadCell = -7500;

  LoadCell.begin();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte LoadCell_ready = 0;

  //run startup, stabilization and tare, both modules simultaniously
  while ((LoadCell_ready) < 1) {
    if (!LoadCell_ready) LoadCell_ready = LoadCell.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check wiring and pin designations for load cell module");
  }
  // Apply calibration value
  LoadCell.setCalFactor(calval_LoadCell);
  Serial.println("Startup is complete");
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 50; //increase value to slow down serial print

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t + serialPrintInterval) {
      float WOB = LoadCell.getData();
      Serial.print("    Drilling Load Cell output val: ");
      Serial.println(WOB);
      
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      LoadCell.tareNoDelay();
    }
  }

  //check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare Load Cell Complete");
  }
  
}
// (Jay)Note: Credit for the ADC libray goes to Olav Kallhovd (GitHub)
