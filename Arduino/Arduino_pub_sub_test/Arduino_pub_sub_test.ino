// This arduino code was made using the tutorial
// found at https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros
//
// In order to compile in Arduino IDE,
// you must install Rosserial Arduino Library and make
// the following modifications to the ros/msg.h file
//
// Summary: (Changes) File - ros/msg.h
//    Change -> #include <cstring> to #include <string.h>
//    Change -> std::memcpy() to memcpy()
//
// these steps can be found here at https://answers.ros.org/question/361930/rosserial-arduino-compilation-error-no-cstring/

//hello world from the land of atom

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#define BUTTON 4
#define LED 3

//Setup ros node handler object
ros::NodeHandle node_handle;

//message to publish/subscribe
std_msgs::String button_msg;
std_msgs::UInt16 led_msg;

//defines subscriber callback function
void subscriberCallback(const std_msgs::UInt16& led_msg){
  if (led_msg.data == 1){
    digitalWrite(LED, HIGH);
  }
  else{
    digitalWrite(LED, LOW);
  }
}

//Setup a publisher object to publish on "button_press" topic
//Setup a subscriber object to subscribe to "toggle_led" topic
ros::Publisher button_publisher("button_press", &button_msg);
ros::Subscriber<std_msgs::UInt16> led_subscriber("toggle_led", &subscriberCallback);

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);

  //initialize node
  node_handle.initNode();
  node_handle.advertise(button_publisher);
  node_handle.subscribe(led_subscriber);

}

void loop() {
  if(digitalRead(BUTTON) == HIGH){
    button_msg.data = "Pressed";
  }
  else{
    button_msg.data = "NOT pressed";
  }

  button_publisher.publish(&button_msg);
  node_handle.spinOnce();

  delay(100);
}
