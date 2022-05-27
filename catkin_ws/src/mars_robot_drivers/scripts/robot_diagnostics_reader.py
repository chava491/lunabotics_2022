#!/usr/bin/env python3
"""
This script is used to recieve the data/diagnostics readings from the various sensors connected to the arduino, and from the various motor drivers.

In order to run properly, you must also be running a script from the arduino which is using the <ros.h> 
arduino library to publish data on the /sensor_data topic. Additionally, you must start a ros_serrial_python 
node which serves as the bridge between the arduino and the ROS environment. To do this use the command:

rosrun rosserial_python serial_node.py /dev/tty<USB# or ACM#>

where <USB# or ACM#> corresponds the device port the arduino is connected to.
"""


import rospy
from mars_robot_msgs.msg import sensor_msg
from digging_locomotion_driver import Digging_Locomotion

class Robot_Diagnostics_Reader:
    def __init__(self):
        self.motor_sub = rospy.Subscriber('motor_data', sensor_msg, self.callback_function)
        self.sensor_sub = rospy.Subscriber('sensor_data', sensor_msg, self.callback_function)
        rospy.spin()

    def callback_function(self, sensor_msg):
        rospy.loginfo(sensor_msg)
        rospy.loginfo("=====================")


if __name__ == "__main__":
    rospy.init_node("robot_diagnostics_reader")
    rospy.loginfo("Starting Robot Diagnostics Reader")

    Diagnostics_Reader = Robot_Diagnostics_Reader()


