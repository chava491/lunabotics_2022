#include <Wire.h>
#include "Adafruit_VL53L0X.h"
/*
 *   To be able to better implement this into a smaller merge with the robot, I have to try and take the bare necessities from 
 *    a) Adafruit_VL53L0X.h
 *      - WEBSITE: https://github.com/adafruit/Adafruit_VL53L0X
 *      - MAKE SURE: Does the download of a library cause extra os for the arduino when uploading program? (AKA increase upload kernel file size?)
 *          -- ANSWER: _____________INSERT HERE___________________
 *      - DATA SHEET: https://cdn-learn.adafruit.com/downloads/pdf/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout.pdf
 *          -- The data sheet has important info on what continuous, single, etc readings are. This explains sensor.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); 
 *              more. This is in the 'Files & Datasheets section' of the data sheet. In that file, look for 'Firmware state machine description'
 *
 *
 *    b) <Wire.h> (THIS IS PROBABLY NOT NECESSARY, BUT WILL HAVE TO SEE)
 */
Adafruit_VL53L0X sensor;

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  if (!sensor.begin()) {
    Serial.println("Failed to initialize sensor");
    while (1);
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  sensor.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  sensor.setMeasurementTimingBudgetMicroSeconds(100000);
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  sensor.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.print("*");
  }

  delay(100);
}
