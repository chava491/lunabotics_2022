/*This arduino script is used to take readings from all of the sensors 
  onboard the robot. This script creates a ROS node and publishes all the
  sensor readings to the pyduino_reader_node (Subscriber) which is in a python script.
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
  // put your setup code here, to run once:

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
    LoadCell.setCalFactor(26.13); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate();
  float newCalibrationValue = 4.47;
  /*  Hardcoding calibration value; determined by running the HX711_Calibration_lbs
   *   arduino script and using the digging replacement motor with a mass of 8.1 lbs,
   *   at which point the script determined the calibration value. We won't need to continuously
   *   recalibrate the weight sensors for the robot, so this is allows for a one and done
   *   calibration.
   */
}

void loop() {
  // put your main code here, to run repeatedly:
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
  Serial.println("PENIS");
  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    LoadCell.tareNoDelay();
    _resume = true;
    
  }
  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = 4.47;
  
  _resume = false;
  while(_resume == false){
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAddress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAddress, newCalibrationValue);

        _resume = true;
  }
 
}
