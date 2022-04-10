#!/usr/bin/env python

"""
This script is used to perform the entire sequence of events for autonomous dumping.

This script creates a ROS subscriber node which subscribes to the main_control topic.
The autonomous operations begin when the auto_dumping_start control key is pressed.
Autonomous dumping continues until auto_stop key is pressed.

Created: 4/9/2022
"""


import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
import time
from dumping_driver import Dumping
    
class Dumping_Autonomy:
    def __init__(self):
        rc_port = rospy.get_param('/mars_robot/ports/roboclaw_port')    #device port of roboclaw
        self.dumping = Dumping(rc_port)
        
        self.speed = rospy.get_param('mars_robot/motor_speeds/dumpa_speed')

        rospy.Subscriber("main_control", String, self.callback_main)

        #Virtual E-stop for autonomous operations
        #   True: E-stop pushed; halt operations    
        #   False: E-stop not pushed; continue operations
        self.auto_stop = False

        #Seperate subscriber required for virtual E-stop
        rospy.Subscriber("emergency_stop", String, self.callback_stop)

    def callback_main(self, msg):
        opcode = msg.data

        if opcode == rospy.get_param('/mars_robot/manual_control_keys/dumping_auto_start'):

            rospy.loginfo("Starting Dumping Autonomy...")
            
            rospy.loginfo("Start Driving Forward")

            right_laser_hit = False
            Left_laser_hit = False

            while(True): #<--- Should be while right & left laser not hit
                rospy.loginfo("Check Both lasers")

                #Stop if virtual E-Stop is hit
                if self.auto_stop == True: 
                    break
            
            #===============================================
            # Depending on which sensor was hit, 
            # stop/slow the appropriate wheel to allign with laser
            #===============================================

            while(True): #<--- Should be while top laser not hit
                rospy.loginfo("Veer locomotion left")

                while(True): #<--- Should be while right laser or top laser not hit
                    rospy.loginfo("Check top laser")
                    rospy.loginfo("Check Right laser")

                    #Stop if virtual E-Stop is hit
                    if self.auto_stop == True: 
                        break

                rospy.loginfo("Veer locomotion right")

                while(True): #<--- Should be while Left laser or top laser not hit
                    rospy.loginfo("Check top laser")
                    rospy.loginfo("Check Right laser")

                    #Stop if virtual E-Stop is hit
                    if self.auto_stop == True: 
                        break

                #Stop if virtual E-Stop is hit
                if self.auto_stop == True: 
                    break
            
            if self.auto_stop == False: 
                rospy.loginfo("Extend Dumpa")
                rospy.loginfo("Pause")
                rospy.loginfo("Retract Dumpa")
            
            rospy.loginfo("Finishing Dumping Autonomy...")


    def callback_stop(self, msg):
        opcode = msg.data
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/auto_stop'):
            self.auto_stop = True
            rospy.loginfo("Virtual E-Stop Pushed!")

    def stop(self):
        self.dumping.disable_roboclaw()
    
    def engage(self):
        self.dumping.enable_roboclaw()

if __name__ == "__main__":
    rospy.init_node("dumping_node")

    dumping_autonomy = Dumping_Autonomy()

    rospy.on_shutdown(dumping_autonomy.stop)

    rospy.loginfo("Dumping node initialized successfully")

    rospy.spin()
