/* This script was written to test manual control of a relay
 * using the arduino MEGA over ROS
 *  
 * Written by: Abraham Yakisan
 * Date: 5/10/2022
 */

#include <ros.h>
#include <std_msgs/String.h>

#define RELAY_PIN 27

//Define Keybindings
String VIBRATOR_ON = "[";
String VIBRATOR_OFF = "]";

//Setup ros node handler object
ros::NodeHandle node_handle;

//message to publish/subscribe
std_msgs::String keyboard_msg;

//defines subscriber callback function
void subscriberCallback(const std_msgs::String& keyboard_msg){
  String key_press = keyboard_msg.data;
  if (key_press == VIBRATOR_ON){
    digitalWrite(RELAY_PIN, HIGH);
  }
  else if(key_press == VIBRATOR_OFF){
    digitalWrite(RELAY_PIN, LOW);
  }
}

//Setup a subscriber object to subscribe to "main_control" topic
ros::Subscriber<std_msgs::String> main_control_subscriber("main_control", &subscriberCallback);

void setup() {
  // Set RELAY_PIN as digital output
  pinMode(RELAY_PIN, OUTPUT);

  //initialize node
  node_handle.initNode();
  node_handle.subscribe(main_control_subscriber);
}

void loop() {
  // put your main code here, to run repeatedly:
  node_handle.spinOnce();

  delay(50);
}
