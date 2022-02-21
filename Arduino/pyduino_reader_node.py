#!/usr/bin/env python

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
    rospy.init_node("pyduino_subscriber")
    rospy.loginfo("Starting Subscriber")
    subscriber()
