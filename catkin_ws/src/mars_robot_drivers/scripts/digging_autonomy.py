#!/usr/bin/env python3

"""
This script is used to perform the entire sequence of events for autonomous digging.

This script creates a ROS subscriber node which subscribes to the main_control topic.
The autonomous operations begin when the auto_digging_start control key is pressed.
Autonomous digging continues until auto_stop key is pressed.

Created: 4/9/2022
"""

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import time
from digging_locomotion_driver import Digging_Locomotion

class Digging_Autonomy:
    def __init__(self):
        #Get driver serial numbers:
        depth_SN = rospy.get_param('/mars_robot/serial_nums/depth_stepper') #Depth tic36v4 stepper driver serial number
        pitch_SN = rospy.get_param('/mars_robot/serial_nums/pitch_stepper') #Pitch tic36v4 stepper driver serial number
        odrv0_SN = rospy.get_param('/mars_robot/serial_nums/auger_odrive') #Auger Odrive serial number
        odrv1_SN = rospy.get_param('/mars_robot/serial_nums/loco_odrive') #Locomotion Odrive serial number

        #self.digging_locomotion = Digging_Locomotion(depth_SN, pitch_SN, odrv0_SN, odrv1_SN)

        #Get motor speeds:
        self.loco_left_speed = rospy.get_param('/mars_robot/motor_speeds/loco_left_speed')
        self.loco_right_speed = rospy.get_param('/mars_robot/motor_speeds/loco_right_speed')
        self.auger_speed = rospy.get_param('/mars_robot/motor_speeds/auger_speed')
        self.pitch_speed = rospy.get_param('/mars_robot/motor_speeds/pitch_speed')
        self.depth_speed = rospy.get_param('/mars_robot/motor_speeds/depth_speed')

        #Subscriber that recieves all keyboard cmds
        rospy.Subscriber("main_control", String, self.callback_main)
        
        #Virtual E-stop for autonomous operations
        #   True: E-stop pushed; halt operations    
        #   False: E-stop not pushed; continue operations
        self.auto_stop = False

        #Seperate subscriber required for virtual E-stop
        rospy.Subscriber("emergency_stop", String, self.callback_stop)

    def callback_main(self, msg):
        opcode = msg.data
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/digging_auto_start'):

            rospy.loginfo("Starting Digging Autonomy...")
            rospy.loginfo("Increase Auger Pitch...")
            rospy.loginfo("Turn Auger On...")

            while(True): #<---- Should be until depth bottom limit switch is not hit or max mass collected not reached
                rospy.loginfo("Depth Decrease...")

                time.sleep(1)
                
                rospy.loginfo("Depth Retract to Deposit...")
                
                rospy.loginfo("Check Limit Switch...")
                rospy.loginfo("Check Mass Collected...")
                
                #Stop if virtual E-Stop is hit
                if self.auto_stop == True: 
                    break
            
            #Homing Sequence
            if self.auto_stop == False: 
                rospy.loginfo("Fully pull out auger...")
                rospy.loginfo("Turn Auger Off...")
                rospy.loginfo("Decrease Auger Pitch...")
                rospy.loginfo("Finished Digging Autonomy...")

            rospy.loginfo("Finishing Digging Autonomy...")
            
            #Reset virtual E-stop
            self.auto_stop = False

            

    def callback_stop(self, msg):
        opcode = msg.data
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/auto_stop'):
            self.auto_stop = True
            rospy.loginfo("Virtual E-Stop Pushed!")

    def stop(self):
        # self.digging_locomotion.digging_motors_disengage()
        print("Successfully shutdown the Digging subsystems")

    
if __name__ == "__main__":
    rospy.init_node("digging_auto_node")

    digging_autonomy = Digging_Autonomy()

    rospy.on_shutdown(digging_autonomy.stop)

    rospy.loginfo("Digging_Autonomy node initialized successfully")

    rospy.spin()