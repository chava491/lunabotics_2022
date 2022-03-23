"""
This file houses all of the dumping functionality

For this script, simple enum commands are sent over the serial port /dev/ttyACM#.
Then these commands are handled in the arduino code and the corresponding relays are opened/closed.
The commands are as follows:
    1 - extend
    2 - retract
    3 - stop

@created: 3-22-2022
"""

import serial
import time
from roboclaw import Roboclaw

class Dumping:

    #---------------------------------------------------------------------
    # Dumping initialization function
    #
    # Establish the roboclaw connection for the linear actuator
    #---------------------------------------------------------------------
    def __init__(self):
        self.arduino_port = '/dev/ttyACM2'

        try:
            print("Searching for arduino roboclaw, this may take a few seconds...")
            #self.enable_roboclaw()
            self.arduino = serial.Serial(port=self.arduino_port, baudrate=115200, timeout=.1)
            self.roboclaw.Open()
            print("Dumping roboclaw connected successfully")
        except:
            print("Unable to find dumping roboclaw")

    #--------------------------------------------------------------------
    # Extend the linear actuator forward for its full length
    #--------------------------------------------------------------------
    def actuator_extend(self):
        self.arduino.write(bytes('1', 'utf-8'))

    #--------------------------------------------------------------------
    # Fully retract the linear actuator
    #--------------------------------------------------------------------
    def actuator_retract(self):
        self.arduino.write(bytes('2', 'utf-8'))

    #--------------------------------------------------------------------
    # Stop the linear actuator
    #--------------------------------------------------------------------
    def actuator_stop(self):
        self.arduino.write(bytes('3', 'utf-8'))

    #--------------------------------------------------------------------
    # A full dump algorithm
    #--------------------------------------------------------------------
    def full_dump(self):
        self.actuator_extend()
        time.sleep(12)
        self.stepper_forward()
        time.sleep(4)
        self.stepper_backward()
        time.sleep(4)
        self.actuator_retract()
        time.sleep(12)
