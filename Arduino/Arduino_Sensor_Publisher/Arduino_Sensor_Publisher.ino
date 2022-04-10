/* Authors:   Abraham Yakisan & Chris Piszczek
 * Created:   2.19.2021
  
  This arduino script is used to take readings from all of the sensors 
  onboard the robot. This script creates a ROS node and publishes all the
  sensor readings to the arduino_reader_node (Subscriber), which is in a python script, 
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


#include </home/mars/Arduino/libraries/ros_lib/mars_robot_msgs/sensor_msg.h>
//#include <sensor_msg.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <HX711_ADC.h>
//#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

//Setup ros node handler object.
ros::NodeHandle node_handle;

//message to publish data for dump bucket weight.
mars_robot_msgs::sensor_msg collected_data;

//Setup a publisher object to publish on "sensor_data" topic
ros::Publisher sensor_data_publisher("sensor_data", &collected_data);
//=================================
//  Limit Switch Variables
//=================================
const int depth_bottom_switch = 52; //use pin 52

//=================================
//  LoadCell Variables
//=================================
//pins:
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

//=================================
//  Gyro Variables
//=================================
const int calVal_eepromAddress = 0;
unsigned long t = 0;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================
// INTERRUPT DETECTION ROUTINE
// ================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//=================================
//  Laser Receiver Variables
//=================================
const int laser_top_pin = 22;
const int laser_left_pin = 24;
const int laser_right_pin = 26;


// ================================================================
// ===                    MAIN PROGRAM SETUP                    ===
// ================================================================


void setup() {
  //Initilization of ROS node
  node_handle.initNode();
  node_handle.advertise(sensor_data_publisher);
  
  Serial.begin(57600);//baud 57600
  delay(10);

  //=================================
  //  Gyro Setup
  //=================================
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
  }
   else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  
  //=================================
  //  Load Cell Setup
  //=================================
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
  //=================================
  //  Laser Receiver Setup
  //=================================
  pinMode(laser_top_pin, INPUT);
  pinMode(laser_left_pin, INPUT);
  pinMode(laser_right_pin, INPUT);

  //=================================
  //  Limit Switch Setup
  //=================================
  pinMode(depth_bottom_switch, INPUT);
  
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  //=================================
  //  Gyro loop
  //=================================
  
  //if programming failed, don't try to do anything
  if (!dmpReady) return;
  //read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
    #endif
    
    collected_data.yaw = ypr[0]; //set field in ros message
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }    

  //=================================
  //  Load Cell loop
  //=================================
  
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity.

  //check for new data/start new conversion:
  if(LoadCell.update()) newDataReady = true;
  
  if(newDataReady){
    if(millis() > t + serialPrintInterval){
      float mass = LoadCell.getData(); //in grams
      Serial.print("Mass: ");
      Serial.println(mass);
      collected_data.mass = mass;      //set field in ros message
      newDataReady = 0;
      t = millis();
    }
  }

  //=================================
  //  Laser Receiver loop
  //=================================
  bool top_hit = digitalRead(laser_top_pin);  //bool value
  bool left_hit = digitalRead(laser_left_pin);  //bool value
  bool right_hit = digitalRead(laser_right_pin);  //bool value

  //set fields in ros message
  collected_data.laser_top_hit = top_hit;
  collected_data.laser_left_hit = left_hit;
  collected_data.laser_right_hit = right_hit;

  
  if(top_hit){
    Serial.println("TOP LASER RECEIVER HIT"); 
  }
  else if(left_hit){
    Serial.println("LEFT LASER RECEIVER HIT");
  }
  else if(right_hit){
    Serial.println("RIGHT LASER RECEIVER HIT");
  }

  //=================================
  //  Limit Switch loop
  //=================================
  bool depth_bottom_hit = digitalRead(depth_bottom_switch);  //bool value
  if(depth_bottom_hit){
    Serial.println("MAX DEPTH REACHED"); 
  } 
  collected_data.depth_bottom_switch = depth_bottom_hit;
  
  //=================================
  //  ROS Publish
  //=================================
  sensor_data_publisher.publish(&collected_data);
  node_handle.spinOnce();

  //delay(100);

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
