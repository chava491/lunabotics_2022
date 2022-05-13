#!/usr/bin/env python

import subprocess
import yaml
import time
import serial
from roboclaw import Roboclaw  # roboclaw package is used strictly because of RoboClaw motor controller that is used;
# made by BasicMicro


class Dumping:
    # --------------------------------------------------------------------
    # Dumping initialization function
    #
    # Establish the roboclaw connection for the linear actuator
    # --------------------------------------------------------------------
    def __init__(self):
        self.serial_num = "00320100"  # may change for this year's robot
        try:
            print("Searching for dumping roboclaw, this may take a few seconds...")
            self.roboclaw = Roboclaw("/dev/ttyACM0", 38400)#ttyAMC0 to ttyAMC1
            self.roboclaw.Open()
            print("Dumping roboclaw connected successfully")
        except:
            print("Unable to find dumping roboclaw")

    # --------------------------------------------------------------------
    # Extend the linear actuator forward to dump contents of bucket.
    # --------------------------------------------------------------------
    def actuator_extend(self):
        if self.roboclaw != None:
            self.roboclaw.BackwardM1(128, 127)
            # extending uses "BackwardM1" (you would think its ForwardM1);
            # determine if this is due to position of actuator,
            # or if it is because of the way the code for package is written.

    # --------------------------------------------------------------------
    # Retract the linear actuator back to idle position so BP-1
    # can be dumped into it.
    # --------------------------------------------------------------------
    def actuator_retract(self):
        if self.roboclaw != None:
            self.roboclaw.ForwardM1(128, 127)  # retracting uses "ForwardM1" (you would think its BackwardM1)

    # --------------------------------------------------------------------
    # Stops the linear actuator
    # --------------------------------------------------------------------
    def actuator_stop(self):
        if self.roboclaw != None:
            self.roboclaw.ForwardM1(128, 0)

    # --------------------------------------------------------------------
    # Enables the roboclaw to communicate on the ACM0 port
    # --------------------------------------------------------------------
    def enable_roboclaw(self):
        self.roboclaw = Roboclaw("/dev/ttyACM1", 38400)  # /dev/ttyACM0 refers to connection to Udoo; shouldn't change
        # but keep in mind.

        self.roboclaw.Open()

    # --------------------------------------------------------------------
    # Disables the roboclaw from communicating on the ACM0 port
    # --------------------------------------------------------------------
    def disable_roboclaw(self):
        self.actuator_stop()

        time.sleep(0.1)

        self.roboclaw.Close()

        
if __name__ == '__main__':
    Dumpa = Dumping()
    Dumpa.enable_roboclaw()
    Dumpa.actuator_retract()
    time.sleep(2.0)
    Dumpa.actuator_extend()
    time.sleep(2.0)
    Dumpa.actuator_stop()
    print("END OF TEST")
