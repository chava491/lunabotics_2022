#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from digging_locomotion_driver import Digging_Locomotion

class digging_locomotion_WrapperROS:

    def __init__(self):
        self.digging_locomotion = Digging_Locomotion()
        self.speed = 20
        self.opcode = -1

        rospy.Subscriber("main_manual", Int32, self.callback_main)
        rospy.Subscriber("emergency_stop", Int32, self.callback_stop)

    def callback_main(self, msg):
        self.opcode = msg.data
        print("opcode: " + str(msg.data))

        if self.opcode >= 0 and self.opcode <= 13:
            #locomotion
            if self.opcode == 0:
                self.digging_locomotion.loco_forward(67)
            if self.opcode == 1:
                self.digging_locomotion.loco_left(67)
            if self.opcode == 2:
                self.digging_locomotion.loco_back(67)
            if self.opcode == 3:
                self.digging_locomotion.loco_right(67)
            if self.opcode == 4:
                self.digging_locomotion.loco_stop()
            
            #auger
            if self.opcode == 5:
                self.digging_locomotion.auger_motor_turn(15)
            if self.opcode == 6:
                self.digging_locomotion.auger_motor_stop()
            if self.opcode == 7:
                self.digging_locomotion.auger_motor_turn(-15)

            #pitch
            if self.opcode == 8:
                self.digging_locomotion.pitch_motor_turn(500000)
            if self.opcode == 9:
                self.digging_locomotion.pitch_motor_stop()
            if self.opcode == 10:
                self.digging_locomotion.pitch_motor_turn(-500000)
            
            #depth
            if self.opcode == 11:
                self.digging_locomotion.depth_motor_turn(500000)
            if self.opcode == 12:
                self.digging_locomotion.depth_motor_stop()
            if self.opcode == 13:
                self.digging_locomotion.depth_motor_turn(-500000)

    
    def callback_stop(self, msg):
        self.opcode = msg.data
        if self.opcode == 1:
            try:
                self.stop()
                print(self.opcode)
                print("Successfully shutdown the Digging subsystem")
            except:
                print("Something went wrong with Digging shutdown")
        elif self.opcode == 2:
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
