#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from digging_driver import Digging

class diggingWrapperROS:

    def __init__(self):
        self.digging = Digging()
        self.speed = 20
        self.opcode = -1

        rospy.Subscriber("main_manual", Int32, self.callback_main)
        rospy.Subscriber("emergency_stop", Int32, self.callback_stop)

    def callback_main(self, msg):
        self.opcode = msg.data
        print("opcode: " + str(msg.data))

        if self.opcode >= 5 and self.opcode <= 13:
            #auger
            if self.opcode == 5:
                self.digging.auger_motor_turn(15)
            if self.opcode == 6:
                self.digging.auger_motor_stop()
            if self.opcode == 7:
                self.digging.auger_motor_turn(-15)

            #pitch
            if self.opcode == 8:
                self.digging.pitch_motor_turn(500000)
            if self.opcode == 9:
                self.digging.pitch_motor_stop()
            if self.opcode == 10:
                self.digging.pitch_motor_turn(-500000)
            
            #depth
            if self.opcode == 11:
                self.digging.depth_motor_turn(500000)
            if self.opcode == 12:
                self.digging.depth_motor_stop()
            if self.opcode == 13:
                self.digging.depth_motor_turn(-500000)


    
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
        self.digging.digging_motors_disengage()
        

if __name__ == "__main__":
    rospy.init_node("digging_node")

    digging_wrapper = diggingWrapperROS()

    rospy.on_shutdown(digging_wrapper.stop)

    rospy.loginfo("Digging node initialized successfully")

    rospy.spin()
