#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

from dumping_driver import Dumping

class dumpingWrapperROS:

    def __init__(self):
        self.dumping = Dumping()

        self.opcode = -1

        rospy.Subscriber("main_manual", String, self.callback_main)
        rospy.Subscriber("emergency_stop", Int32, self.callback_stop)
    
    def callback_main(self, msg):
        self.opcode = msg.data

        if self.opcode == rospy.get_param('/mars_robot/manual_control_keys/dumpa_extend_key'):
            self.dumping.actuator_extend()
        if self.opcode == rospy.get_param('/mars_robot/manual_control_keys/dumpa_stop_key'):
            self.dumping.actuator_stop()
        if self.opcode == rospy.get_param('/mars_robot/manual_control_keys/dumpa_retract_key'):
            self.dumping.actuator_retract()

    def callback_stop(self, msg):
        self.opcode = msg.data
        if self.opcode == 1:
            try:
                self.stop()
                print(self.opcode)
                print("Successfully shutdown the Dumping subsystem")
            except:
                print("Something went wrong with Dumping shutdown")
        elif self.opcode == 2:
            self.engage()

    def stop(self):
        self.dumping.disable_roboclaw()
    
    def engage(self):
        self.dumping.enable_roboclaw()

if __name__ == "__main__":
    rospy.init_node("dumping_node")

    dumping_wrapper = dumpingWrapperROS()

    rospy.on_shutdown(dumping_wrapper.stop)

    rospy.loginfo("Dumping node initialized successfully")

    rospy.spin()
