#!/usr/bin/env python

"""
This script is used to perform start a dumping node to perform dumping operations manually
"""

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

from dumping_driver import Dumping

class Dumping_WrapperROS:

    def __init__(self):
        rc_port = rospy.get_param('/mars_robot/ports/roboclaw_port')    #device port of roboclaw

        self.dumping = Dumping(rc_port)
        
        self.speed = rospy.get_param('mars_robot/motor_speeds/dumpa_speed')

        rospy.Subscriber("main_control", String, self.callback_main)
    
    def callback_main(self, msg):
        opcode = msg.data

        if opcode == rospy.get_param('/mars_robot/manual_control_keys/dumpa_extend_key'):
            self.dumping.actuator_extend(self.speed)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/dumpa_stop_key'):
            self.dumping.actuator_stop()
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/dumpa_retract_key'):
            self.dumping.actuator_retract(self.speed)

    def stop(self):
        self.dumping.disable_roboclaw()
    
    def engage(self):
        self.dumping.enable_roboclaw()

if __name__ == "__main__":
    rospy.init_node("dumping_node")

    dumping_wrapper = Dumping_WrapperROS()

    rospy.on_shutdown(dumping_wrapper.stop)

    rospy.loginfo("Dumping node initialized successfully")

    rospy.spin()
