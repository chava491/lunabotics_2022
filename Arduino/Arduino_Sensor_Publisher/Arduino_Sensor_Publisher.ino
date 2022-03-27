/* Authors:   Abraham Yakisan & Chris Piszczek
 * Created:   2.19.2021
  
  This arduino script is used to take readings from all of the sensors 
  onboard the robot. This script creates a ROS node and publishes all the
  sensor readings to the pyduino_reader_node (Subscriber), which is in a python script, 
  over the /sensor_data topic.
 
  A tutorial explaining how to setup the arduino as a rosserial ROS node can be found at: 
  https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros

  In order to compile in Arduino IDE,
  you must install Rosserial Arduino Library and make
  the following modifications to the ros/msg.h file

  Summary: (Changes) File - ros/msg.h
    Change -> #include <cstring> to #include <string.h>
    Change -> std::memcpy() to memcpy()

  these steps can be found here at https://answers.ros.org/question/361930/rosserial-arduino-compilation-error-no-cstring/

  Load Cells:
  ===========
   * The load cells complete a tare in the script setup. Rerun script to tare sensors.
   * 
   * 2 calibration values for the load cells are hardcoded in this script: newCalibrationValue and calFactor; 
   * Here are the steps to determine these two values:
   * 
   * 1. run the HX711_Calibration_lbs arduino script and follow the commands given through the serial monitor. After placing 
   * a known mass, the program will calculate and output a "calibration value". Record that value and set the variable newCalibrationValue 
   * in this script to that value.
   * 
   * 2. calFactor is used to correct the raw readings from load cell, the raw reading is divided 
   * by calFactor to correct the mass reading, set to 1.0 to read the raw value
   * 
   *        Ex: if raw value is 1000g but actual mass on sensors is 200g, set calFactor to 5.
   * 
   * We won't need to recalibrate the weight sensors for the robot unless we notice the readings are very off, 
   * so this is allows for a one and done calibration.
*/

#include <ros.h>
#include <std_msgs/Float32.h>
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif


//Setup ros node handler object.
ros::NodeHandle node_handle;

//message to publish data for dump bucket weight.
std_msgs::Float32 weight_reading;

//Setup a publisher object to publish on "sensor_data" topic
ros::Publisher sensor_data_publisher("sensor_data", &weight_reading);


//pins:
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAddress = 0;
unsigned long t = 0;


void setup() {
  //Initilization of ROS node
  node_handle.initNode();
  node_handle.advertise(sensor_data_publisher);
  
  Serial.begin(57600);//57600
  delay(10);

  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000;// preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = false; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if(LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()){
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else{
    // user set calFactor value (float), raw value is divided by calFactor to 
    // correct the mass reading, set to 1.0 to read raw value
    float calFactor = -105.28;
    LoadCell.setCalFactor(calFactor); 
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate();
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity.

  //check for new data/start new conversion:
  if(LoadCell.update()) newDataReady = true;
  
  if(newDataReady){
    if(millis() > t + serialPrintInterval){
      float mass = LoadCell.getData(); //in grams
      Serial.println(mass);
      weight_reading.data = mass;      //setting data field of message to be published
      newDataReady = 0;
      t = millis();
    }
  }
  
  sensor_data_publisher.publish(&weight_reading);
  node_handle.spinOnce();

  delay(100);

}

void calibrate(){
  //Call functions to prepare for calibration
  LoadCell.update();
  LoadCell.tareNoDelay();    //Tare the readings
  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct

  //user set newCalibrationValue (float), follow instructions in comments at top of file to determine value
  float newCalibrationValue = -105.20;
  
//Save Calibration value in HX711's EEPROM
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAddress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAddress, newCalibrationValue);
 
}
