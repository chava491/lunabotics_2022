#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from digging_locomotion_driver import Digging_Locomotion

class digging_locomotion_WrapperROS:

    def __init__(self):
        self.digging_locomotion = Digging_Locomotion()


        rospy.Subscriber("main_manual", String, self.callback_main)
        rospy.Subscriber("emergency_stop", Int32, self.callback_stop)

    def callback_main(self, msg):
        opcode = msg.data
        print("opcode: " + str(msg.data))

        #locomotion
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/loco_forward_key'):
            self.digging_locomotion.loco_forward(67)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/loco_left_key'):
            self.digging_locomotion.loco_left(67)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/loco_backward_key'):
            self.digging_locomotion.loco_back(67)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/loco_right_key'):
            self.digging_locomotion.loco_right(67)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/loco_stop_key'):
            self.digging_locomotion.loco_stop()
        
        #auger
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/auger_dig_key'):
            self.digging_locomotion.auger_motor_turn(15)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/auger_stop_key'):
            self.digging_locomotion.auger_motor_stop()
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/auger_reverse_key'):
            self.digging_locomotion.auger_motor_turn(-15)

        #pitch
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/pitch_increase_key'):
            self.digging_locomotion.pitch_motor_turn(500000)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/pitch_stop_key'):
            self.digging_locomotion.pitch_motor_stop()
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/pitch_decrease_key'):
            self.digging_locomotion.pitch_motor_turn(-500000)
        
        #depth
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/depth_decrease_key'):
            self.digging_locomotion.depth_motor_turn(500000)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/depth_stop_key'):
            self.digging_locomotion.depth_motor_stop()
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/depth_increase_key'):
            self.digging_locomotion.depth_motor_turn(-500000)

    
    def callback_stop(self, msg):
        opcode = msg.data
        if opcode == 1:
            try:
                self.stop()
                print(opcode)
                print("Successfully shutdown the Digging subsystem")
            except:
                print("Something went wrong with Digging shutdown")
        elif opcode == 2:
            self.engage()

    def stop(self):
        self.digging_locomotion.digging_motors_disengage()
        self.digging_locomotion.loco_disengage_motors()
        

if __name__ == "__main__":
    rospy.init_node("digging_locomotion_node")

    digging_locomotion_wrapper = digging_locomotion_WrapperROS()

    rospy.on_shutdown(digging_locomotion_wrapper.stop)

    rospy.loginfo("Digging_Locomotion node initialized successfully")

    rospy.spin()
