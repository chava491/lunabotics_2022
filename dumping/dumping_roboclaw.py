#!/usr/bin/env python
"""
This file houses all of the dumping functionality

@created: 3-22-2022
"""

import time
from roboclaw import Roboclaw

class Dumping:

    #---------------------------------------------------------------------
    # Dumping initialization function
    #
    # Establish the roboclaw connection for the linear actuator
    #---------------------------------------------------------------------
    def __init__(self):
        try:
            print("Searching for dumping roboclaw, this may take a few seconds...")
            #self.enable_roboclaw()
            self.roboclaw = Roboclaw('/dev/ttyACM2', 38400)
            self.roboclaw.Open()
            print("Dumping roboclaw connected successfully")
        except:
            print("Unable to find dumping roboclaw")

    #--------------------------------------------------------------------
    # Extend the linear actuator forward for its full length
    #--------------------------------------------------------------------
    def actuator_extend(self):
        if self.roboclaw != None:
            self.roboclaw.ForwardM1(128, 127)

    #--------------------------------------------------------------------
    # Fully retract the linear actuator
    #--------------------------------------------------------------------
    def actuator_retract(self):
        if self.roboclaw != None:
            self.roboclaw.BackwardM1(128, 127)

    #--------------------------------------------------------------------
    # Stop the linear actuator
    #--------------------------------------------------------------------
    def actuator_stop(self):
        if self.roboclaw != None:
            self.roboclaw.ForwardM1(128, 0)

    #--------------------------------------------------------------------
    # A full dump algorithm
    #--------------------------------------------------------------------
    def full_dump(self):
        self.actuator_extend()
        time.sleep(10)
        self.actuator_retract()
        time.sleep(10)

    #--------------------------------------------------------------------
    # Enables the roboclaw to communicate on the ACM# port
    #--------------------------------------------------------------------
    def enable_roboclaw(self):
        self.roboclaw = Roboclaw("/dev/ttyACM2", 38400)
        self.roboclaw.Open()

    #--------------------------------------------------------------------
    # Disables the roboclaw to communicate on the ACM# port
    #--------------------------------------------------------------------
    def disable_roboclaw(self):
        self.actuator_stop()
        
        time.sleep(0.1)
