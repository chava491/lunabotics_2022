#!/usr/bin/env python
"""
This script is used to recieve the data readings from the various sensors connected to the arduino.

In order to run properly, you must also be running a script from the arduino which is using the <ros.h> 
arduino library to publish data on the /sensor_data topic. Additionally, you must start a ros_serrial_python 
node which serves as the bridge between the arduino and the ROS environment. To do this use the command:

rosrun rosserial_python serial_node.py /dev/tty<USB# or ACM#>

where <USB# or ACM#> corresponds the device port the arduino is connected to.
"""


import rospy
from std_msgs.msg import Float32

def subscriber():
    sub = rospy.Subscriber('sensor_data', Float32, callback_function)
    rospy.spin()

def callback_function(weight_reading):
    rospy.loginfo("I received: %f" % weight_reading.data)
    bin_status = is_bin_full(weight_reading.data)
    rospy.loginfo("Bin Full?: " + str(bin_status))
    rospy.loginfo("=====================")

#This function checks the current bin weight to see if it is over collection threshold (1kg)
#Returns true if full, false if not
def is_bin_full(weight):
    return (weight > 1000)


if __name__ == "__main__":
    rospy.init_node("arduino_reader_node")
    rospy.loginfo("Starting Subscriber")
    subscriber()
