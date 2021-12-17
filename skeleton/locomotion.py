import odrive
import time
from odrive.utils import dump_errors
from odrive.enums import *

class Locomotion:

    # ------------------------------------------------------
    # Locomotion initialization function
    #
    # Establish connection to the o-drive for locomotion
    # ------------------------------------------------------
    def __init__(self):
        try:
            print("Searching for locomotion o-drive, this may take a few seconds...")
            self.odrv1 = odrive.find_any(serial_number = "20863880304E") #safe to assume serial_number will change based on odrive used for the locomotion this year. Will just leave it as last years for now.
            print("Locomotion o-drive found and connected successfully")
        except:
            print("Unable to find locomotion o-drive")

        self.loco_engage_motors()

    # -----------------------------------------------------------------
    # Drives the robot forward
    #
    # parameter: speed == set the speed of the movement (max at 67)
    # -----------------------------------------------------------------
    def loco_forward(self):
       self.odrv1.axis0.controller.input_vel = 0
       self.odrv1.axis1.controller.input_vel = 0

       time.sleep(0.1)

       self.odrv1.axis0.controller.input_vel = -67
       self.odrv1.axis1.controller.input_vel = 67

    def loco_backwards(self):


    def loco_left(self):

    def loco_right(self):


    def loco_stop(self):


    def loco_engage_motors(self):


    def loco_disengage_motors(self):


    def loco_dump_errors(self):



