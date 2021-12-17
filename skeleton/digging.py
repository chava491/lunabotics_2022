import odrive
from odrive.utils import dump_errors
from odrive.enums import *
import subprocess
import yaml
import time

class Digging:
    def __int__(self):
        self.serial_num = "00320097"
        try:
            print("Searching for digging o-drive, this may take a few seconds...")
            self.odrv0 = odrive.find_any(serial_number="207939834D4D") #seaching for o-drive with specific serial number; might change for this years robot
        except:
            print("Unable to find digging o-drive")

        self.dig_engage_depth()
        self.dig_engage_zipper()


    #--------------------------------------------------------------------
    # Spin the auger, digging the material below it
    #
    # param: speed -- set the speed of belt movement (max at 67)
    #--------------------------------------------------------------------
    def auger_dig(self, speed):

    # --------------------------------------------------------------------
    # Spin the auger opposite, to get it unstuck in the case of digging
    # incorrect material
    #
    # param: speed -- set the speed of belt movement (max at 67)
    # --------------------------------------------------------------------
    def auger_undig(self, speed):

    # --------------------------------------------------------------------
    # Stop the auger at its current location
    # --------------------------------------------------------------------
    def auger_stop(self):

    # --------------------------------------------------------------------
    # Extends the auger shaft and casing deeper into the ground
    #
    # param: speed -- set the speed of depth adjustment (max at 50)
    # --------------------------------------------------------------------
    def depth_extend(self, speed):

    # --------------------------------------------------------------------
    # Retracts the auger shaft and casing from the hole it has dug
    #
    # param: speed -- set the speed of the depth adjustment (max at 50)
    # --------------------------------------------------------------------
    def depth_retract(self, speed):

    def depth_stop(self):

    # --------------------------------------------------------------------
    # Helper function to operate the stepper motor
    #
    # param: *args -- a variable set of arguments used to send commands
    # --------------------------------------------------------------------
    def ticcmd(self, *args):

    # --------------------------------------------------------------------
    # Rotate the auger forward with the stepper motor
    # --------------------------------------------------------------------
    def stepper_forward(self, pos):

    # --------------------------------------------------------------------
    # Rotate the auger backward with the stepper motor
    # --------------------------------------------------------------------
    def stepper_backward(self, pos):

    # --------------------------------------------------------------------
    # Stop the auger by deactivating the stepper motor
    # --------------------------------------------------------------------
    def stepper_stop(self):

    # --------------------------------------------------------------------
    # Engages the depth motor by setting their state
    # --------------------------------------------------------------------
    def dig_engage_depth(self):

    # --------------------------------------------------------------------
    # Disengages the depth motor by setting their state
    # --------------------------------------------------------------------
    def dig_disengage_depth(self):

    # --------------------------------------------------------------------
    # Engages the auger motor by setting their state
    # --------------------------------------------------------------------
    def dig_engage_auger(self):

    # --------------------------------------------------------------------
    # Disengages the auger motor by setting their state
    # --------------------------------------------------------------------
    def dig_disengage_auger(self):

    # --------------------------------------------------------------------
    # Disengages the stepper motor by resetting the state
    # --------------------------------------------------------------------
    def dig_disengage_pitch(self):

    # --------------------------------------------------------------------
    # Dumps all errors from the locomotion odrive
    # --------------------------------------------------------------------
    def dig_dump_errors(self):


