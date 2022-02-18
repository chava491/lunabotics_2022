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

    # --------------------------------------------------------------------
    # Drives robot in reverse
    #
    # Input_vel values are opposite due to the original orientation of the motors.
    # Think about changing orientation of motors so that they rotate the same direction,
    # also allowing for the programming to make more sense; +67 for both axis0 and axis1
    # result in moving robot forward, while -67 for both axes results in backwards movement.
    #
    # time.sleep(0.1) most likely resulting in on motor starting before the other, causing
    # alignment problems as the initial instruction for movement occurs.
    #
    # parameter: speed == set the speed of the movement (max at 67)
    # --------------------------------------------------------------------
    def loco_backwards(self):
        self.odrv1.axis0.controller.input_vel = 0
        self.odrv1.axis1.controller.input_vel = 0

        time.sleep(0.1)

        self.odrv1.axis0.controller.input_vel = 67
        self.odrv1.axis1.controller.input_vel = -67

    # --------------------------------------------------------------------
    # Zero point turn left
    #
    # parameter: speed == set the speed of the movement (max at 67)
    # --------------------------------------------------------------------
    def loco_left(self):
        self.odrv1.axis0.controller.input_vel = 0
        self.odrv1.axis1.controller.input_vel = 0

        time.sleep(0.1)

        self.odrv1.axis0.controller.input_vel = 67
        self.odrv1.axis1.controller.input_vel = 67

    # --------------------------------------------------------------------
    # Zero point turn right
    #
    # parameter: speed == set the speed of the movement (max at 67)
    # --------------------------------------------------------------------
    def loco_right(self):
        self.ordv1.axis0.controller.input_vel = 0
        self.ordv1.axis1.controller.input_vel = 0

        time.sleep(0.1)

        self.ordv1.axis0.controller.input_vel = -67
        self.ordv1.axis1.controller.input_vel = -67

    # --------------------------------------------------------------------
    # Stops all movement
    #
    # Stopping right to 0 causes ringing in the motor, so stopping is set
    # to the following: (speed) -> 0 -> 5 -> 0
    # --------------------------------------------------------------------
    def loco_stop(self):
        self.ordv1.axis0.controller.input_vel = 0
        self.ordv1.axis1.controller.input_vel = 0

        time.sleep(0.1)

        #one of the motors rotation might need to be -5 due to the orientation of the motors being flip-flopped.
        self.ordv1.axis0.controller.input_vel = 5
        self.ordv1.axis1.controller.input_vel = 5

        time.sleep(0.1)

        self.ordv1.axis0.controller.input_vel = 0
        self.ordv1.axis1.controller.input_vel = 0

    # --------------------------------------------------------------------
    # Engages the locomotion motors by setting their state.
    # --------------------------------------------------------------------
    def loco_engage_motors(self):
        self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    # --------------------------------------------------------------------
    # Disengages the locomotion motors by setting their state.
    # --------------------------------------------------------------------
    def loco_disengage_motors(self):
        self.loco_stop()

        time.sleep(0.1)

        self.odrv1.axis0.requested_state = AXIS_STATE_IDLE
        self.ordv1.axis1.requested_state = AXIS_STATE_IDLE

    # --------------------------------------------------------------------
    # Dumps all errors from the locomotion odrive.
    #
    # parameter ordv1 might need to be referenced as self.ordv1.
    # --------------------------------------------------------------------
    def loco_dump_errors(self):
        dump_errors(ordv1, True)






