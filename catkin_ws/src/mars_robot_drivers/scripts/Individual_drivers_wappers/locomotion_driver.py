#!/usr/bin/env python3
"""
This file houses all of the locomotion functionality.

@created: 3-22-2022
"""

import odrive
import time
from odrive.utils import dump_errors
from odrive.enums import *

class Locomotion:
    
    #--------------------------------------------------------------------
    # Locomotion intialiation function
    #
    # Establish the odrive connection for locomotion
    #--------------------------------------------------------------------
    def __init__(self):
        try:
            print("Searching locomotion odrive, this may take a few seconds...")
            self.odrv1 = odrive.find_any(serial_number="20863880304E")
            print("Locomotion odrive connected successfully")
        except:
            print("Unable to find locomotion odrive")

        self.loco_engage_motors()     

    #--------------------------------------------------------------------
    # Drives robot forward
    #
    # param: speed -- set the speed of movement (max at 67)
    #--------------------------------------------------------------------
    def loco_forward(self, speed):
        self.odrv1.axis0.controller.input_vel = 0
        self.odrv1.axis1.controller.input_vel = 0

        time.sleep(0.1)

        self.odrv1.axis0.controller.input_vel = -speed
        self.odrv1.axis1.controller.input_vel = speed

    #--------------------------------------------------------------------
    # Zero point turn left
    #
    # param: speed -- set the speed of movement (max at 67)
    #--------------------------------------------------------------------
    def loco_left(self, speed):
        self.odrv1.axis0.controller.input_vel = 0
        self.odrv1.axis1.controller.input_vel = 0

        time.sleep(0.1)

        self.odrv1.axis0.controller.input_vel = speed
        self.odrv1.axis1.controller.input_vel = speed
    
    #--------------------------------------------------------------------
    # Drives robot in reverse
    #
    # param: speed -- sets the speed of movement (max at speed)
    #--------------------------------------------------------------------
    def loco_back(self, speed):
        self.odrv1.axis0.controller.input_vel = 0
        self.odrv1.axis1.controller.input_vel = 0

        time.sleep(0.1)

        self.odrv1.axis0.controller.input_vel = speed
        self.odrv1.axis1.controller.input_vel = -speed

    #--------------------------------------------------------------------
    # Zero point turn right 
    #
    # param: speed -- sets the speed of movement (max at 50)
    #--------------------------------------------------------------------
    def loco_right(self, speed):
        self.odrv1.axis0.controller.input_vel = 0
        self.odrv1.axis1.controller.input_vel = 0

        time.sleep(0.1)

        self.odrv1.axis0.controller.input_vel = -speed
        self.odrv1.axis1.controller.input_vel = -speed
    
    #--------------------------------------------------------------------
    # Stops all movement
    # 
    # Stopping right to 0 causes ringing in the motor, so stopping is set
    # to the following: (speed) -> 0 -> 5 -> 0
    #--------------------------------------------------------------------
    def loco_stop(self):
        self.odrv1.axis0.controller.input_vel = 0
        self.odrv1.axis1.controller.input_vel = 0

        time.sleep(0.1)

        self.odrv1.axis0.controller.input_vel = -5
        self.odrv1.axis1.controller.input_vel = 5

        time.sleep(0.1)

        self.odrv1.axis0.controller.input_vel = 0
        self.odrv1.axis1.controller.input_vel = 0

    #--------------------------------------------------------------------
    # Engages the locomotion motors by setting their state and control mode (velocity)
    #--------------------------------------------------------------------
    def loco_engage_motors(self):
        self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis0.controller.config.control_mode = 2 #Velocity control
        self.odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis1.controller.config.control_mode = 2 #Velocity control

    #--------------------------------------------------------------------
    # Disengages the locomotion motors by setting their state
    #--------------------------------------------------------------------
    def loco_disengage_motors(self):
        self.loco_stop()

        time.sleep(0.1)

        self.odrv1.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv1.axis1.requested_state = AXIS_STATE_IDLE

    #---------------------------------------------------------------------
    # Returns: the current of the loco motor
    #
    # motor.current_control.Iq_setpoint is commanded current 
    # motor.current_control.Iq_measured is measured motor current
    #   Odrive reference states setpoint value is less noisy and still accurate 
    #   unless motor is near max rpm
    #---------------------------------------------------------------------
    def get_left_loco_motor_current(self):
        return self.odrv1.axis0.motor.current_control.Iq_setpoint

    def get_right_loco_motor_current(self):
        return self.odrv1.axis1.motor.current_control.Iq_setpoint
    
    #---------------------------------------------------------------------
    # Returns: the position estimation of the loco motor [turns]
    #---------------------------------------------------------------------
    def get_left_loco_motor_pos(self):
        return self.odrv1.axis0.encoder.pos_estimate

    def get_right_loco_motor_pos(self):
        return self.odrv1.axis1.encoder.pos_estimate
    
    #---------------------------------------------------------------------
    # Returns: the velocity estimation of the auger motor [turns/s]
    #---------------------------------------------------------------------
    def get_left_loco_motor_vel(self):
        return self.odrv1.axis0.encoder.vel_estimate
    
    def get_right_loco_motor_vel(self):
        return self.odrv1.axis1.encoder.vel_estimate    

    #--------------------------------------------------------------------
    # Dumps all errors from the locomotion odrive
    #--------------------------------------------------------------------
    def loco_dump_errors(self):
        dump_errors(self.odrv1, True)