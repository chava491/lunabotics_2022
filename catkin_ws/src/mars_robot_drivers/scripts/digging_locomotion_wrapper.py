#!/usr/bin/env python3

"""
This script is used to perform start a digging_locomotion node to perform these operations manually
"""

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from mars_robot_msgs.msg import motor_data_msg
from digging_locomotion_driver import Digging_Locomotion


class Digging_Locomotion_WrapperROS:

    def __init__(self):
        #Get driver serial numbers:
        depth_SN = rospy.get_param('/mars_robot/serial_nums/depth_stepper') #Depth tic36v4 stepper driver serial number
        pitch_SN = rospy.get_param('/mars_robot/serial_nums/pitch_stepper') #Pitch tic36v4 stepper driver serial number
        odrv0_SN = rospy.get_param('/mars_robot/serial_nums/auger_odrive') #Auger Odrive serial number
        odrv1_SN = rospy.get_param('/mars_robot/serial_nums/loco_odrive') #Locomotion Odrive serial number

        self.digging_locomotion = Digging_Locomotion(depth_SN, pitch_SN, odrv0_SN, odrv1_SN)

        #Get motor speeds:
        self.loco_left_speed = rospy.get_param('/mars_robot/motor_speeds/loco_left_speed')
        self.loco_right_speed = rospy.get_param('/mars_robot/motor_speeds/loco_right_speed')
        self.auger_speed = rospy.get_param('/mars_robot/motor_speeds/auger_speed')
        self.pitch_speed = rospy.get_param('/mars_robot/motor_speeds/pitch_speed')
        self.depth_up_speed = rospy.get_param('/mars_robot/motor_speeds/depth_up_speed')
        self.depth_down_speed = rospy.get_param('/mars_robot/motor_speeds/depth_down_speed')

        self.subscriber = rospy.Subscriber("main_control", String, self.callback_main)

        self.publisher = rospy.Publisher('motor_data', motor_data_msg, queue_size=10)
        

    def callback_main(self, msg):
        opcode = msg.data
        #print("opcode: " + str(msg.data))

        #locomotion
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/loco_forward_key'):
            self.digging_locomotion.loco_forward(self.loco_left_speed, self.loco_right_speed)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/loco_left_key'):
            self.digging_locomotion.loco_left(self.loco_left_speed, self.loco_right_speed)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/loco_backward_key'):
            self.digging_locomotion.loco_back(self.loco_left_speed, self.loco_right_speed)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/loco_right_key'):
            self.digging_locomotion.loco_right(self.loco_left_speed, self.loco_right_speed)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/loco_stop_key'):
            self.digging_locomotion.loco_stop()
        
        #auger
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/auger_dig_key'):
            self.digging_locomotion.auger_motor_turn(self.auger_speed)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/auger_stop_key'):
            self.digging_locomotion.auger_motor_stop()
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/auger_reverse_key'):
            self.digging_locomotion.auger_motor_turn(-1*self.auger_speed)

        #pitch
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/pitch_increase_key'):
            self.digging_locomotion.pitch_motor_turn(self.pitch_speed)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/pitch_stop_key'):
            self.digging_locomotion.pitch_motor_stop()
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/pitch_decrease_key'):
            self.digging_locomotion.pitch_motor_turn(-1*self.pitch_speed)
        
        #depth (FIX LABELLING OF DOWN/UP)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/depth_decrease_key'):
            self.digging_locomotion.depth_motor_turn(self.depth_down_speed)
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/depth_stop_key'):
            self.digging_locomotion.depth_motor_stop()
        if opcode == rospy.get_param('/mars_robot/manual_control_keys/depth_increase_key'):
            self.digging_locomotion.depth_motor_turn(self.depth_up_speed)

        #Print Data of Motors
        if opcode == rospy.get_param('/mars_robot/diagnostic_values/print_motor_data'):
            motor_data = motor_data_msg()
            motor_data.auger_current = self.digging_locomotion.get_auger_motor_current()
            motor_data.auger_speed = self.digging_locomotion.get_auger_motor_vel()
            motor_data.right_loco_current = self.digging_locomotion.get_right_loco_motor_current()
            motor_data.left_loco_current = self.digging_locomotion.get_left_loco_motor_current()

            self.publisher.publish(motor_data)


    def stop(self):
        self.digging_locomotion.digging_motors_disengage()
        self.digging_locomotion.loco_disengage_motors()
        #print("Successfully shutdown the Digging_Locomotion subsystems")
        

if __name__ == "__main__":
    rospy.init_node("digging_locomotion_node")

    digging_locomotion_wrapper = Digging_Locomotion_WrapperROS()

    rospy.on_shutdown(digging_locomotion_wrapper.stop)

    #rospy.loginfo("Digging_Locomotion node initialized successfully")

    rospy.spin()


