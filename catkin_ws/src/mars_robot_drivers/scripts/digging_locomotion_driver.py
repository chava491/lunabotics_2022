#!/usr/bin/env python3
"""
This file houses all of the digging and locomotion functionality
These two systems were combined into 1 script because we were continuosly
recvieving a "[USB] Could not claim interface � on USB device: -6"

@created: 3-22-2022
"""

import odrive
from odrive.utils import dump_errors
from odrive.enums import *
import subprocess
import yaml
import time

class Digging_Locomotion:
    #---------------------------------------------------------------------
    # Digging initialize function
    # 
    # Establish the odrive connection for auger
    #---------------------------------------------------------------------
    def __init__(self):
        self.depth_serial_num = "00320097" #Depth tic36v4 stepper driver serial number
        self.pitch_serial_num = "00320100" #Pitch tic36v4 stepper driver serial number
        self.odrv0_serial_num = "207939834D4D" #Digging Odrive serial number
        self.odrv1_serial_num = "20863880304E" #Locomotion Odrive serial number

        try:
            print("Searching for digging odrive, this may take a few seconds...")
            self.odrv0 = odrive.find_any(serial_number=self.odrv0_serial_num)
            print("Digging odrive connected successfully") 
        except:
            print("Unable to find digging odrive")

        try:
            print("Searching for locomotion odrive, this may take a few seconds...")
            self.odrv1 = odrive.find_any(serial_number=self.odrv1_serial_num)
            print("Locomotion odrive connected successfully") 
        except:
            print("Unable to find locomotion odrive")

        self.auger_motor_engage()
        self.pitch_motor_engage()
        self.depth_motor_engage()
        self.loco_engage_motors()
        
    #---------------------------------------------------------------------
    # Helper function to operate the stepper motor from tic36v4
    # 
    # param: *args -- a variable set of arguments used to send commands
    #---------------------------------------------------------------------
    def ticcmd(self, *args):
        return subprocess.check_output(['ticcmd'] + list(args))

    def digging_motors_disengage(self):
        self.auger_motor_disengage()
        self.depth_motor_disengage()
        self.pitch_motor_disengage()

    ##====================================================================
    ##      AUGER MOTOR FUNCTIONS:
    ##====================================================================

    #---------------------------------------------------------------------
    # Start turning the auger
    # param: speed [turn/s] (max speed: 16.67)
    #        + speed = [ENTER CW OR CCW HERE]
    #        - speed = [ENTER CCW OR CW HERE]
    #---------------------------------------------------------------------
    def auger_motor_turn(self, speed):
        self.odrv0.axis0.controller.input_vel = speed

    #---------------------------------------------------------------------
    # Stops the auger at its current location
    #---------------------------------------------------------------------
    def auger_motor_stop(self):
        self.odrv0.axis0.controller.input_vel = 0

        time.sleep(0.1)

        self.odrv0.axis0.controller.input_vel = 5

        time.sleep(0.1)

        self.odrv0.axis0.controller.input_vel = 0

    #---------------------------------------------------------------------
    # Engages the auger motor by setting its state and control mode (velocity)
    #---------------------------------------------------------------------
    def auger_motor_engage(self):
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.control_mode = 2 #Velocity control

    #---------------------------------------------------------------------
    # Disengages the auger motor by setting state to idle
    #---------------------------------------------------------------------
    def auger_motor_disengage(self):
        self.auger_motor_stop()

        time.sleep(0.1)

        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
    
    #---------------------------------------------------------------------
    # Returns: the current of the auger motor
    #
    # motor.current_control.Iq_setpoint is commanded current 
    # motor.current_control.Iq_measured is measured motor current
    #   Odrive reference states setpoint value is less noisy and still accurate 
    #   unless motor is near max rpm
    #---------------------------------------------------------------------
    def get_auger_motor_current(self):
        return self.odrv0.axis0.motor.current_control.Iq_setpoint

    #---------------------------------------------------------------------
    # Returns: the position estimation of the auger motor [turns]
    #---------------------------------------------------------------------
    def get_auger_motor_pos(self):
        return self.odrv0.axis0.encoder.pos_estimate
    
    #---------------------------------------------------------------------
    # Returns: the velocity estimation of the auger motor [turns/s]
    #---------------------------------------------------------------------
    def get_auger_motor_vel(self):
        return self.odrv0.axis0.encoder.vel_estimate

    #---------------------------------------------------------------------
    # Dumps all errors from the digging odrive
    #---------------------------------------------------------------------
    def dig_dump_errors(self):
        self.odrv0.dump_errors(self.odrv0, True)

    ##====================================================================
    ##      DEPTH MOTOR FUNCTIONS:
    ##====================================================================

    #---------------------------------------------------------------------
    # Starts the depth motor by setting a velocity
    # param: speed [microsteps/100000 seconds] (max speed: [ENTER MAX HERE])
    #        + speed = [ENTER CW OR CCW HERE]
    #        - speed = [ENTER CCW OR CW HERE]
    #---------------------------------------------------------------------
    def depth_motor_turn(self, speed):
        self.ticcmd('-d', self.depth_serial_num, '--exit-safe-start', '-y', str(speed))

    #---------------------------------------------------------------------
    # Stops the depth motor by setting speed to 0
    #---------------------------------------------------------------------
    def depth_motor_stop(self):
        self.ticcmd('-d', self.depth_serial_num, '--exit-safe-start', '-y', str(0))

    #---------------------------------------------------------------------
    # Engages the depth motor by reseting zero position & resuming tic36v4
    #---------------------------------------------------------------------
    def depth_motor_engage(self):
        self.ticcmd('-d', self.depth_serial_num, '--reset')
        self.ticcmd('-d', self.depth_serial_num, '--resume')

    #---------------------------------------------------------------------
    # Disengages the depth motor by deenergizing the tic36v4
    #---------------------------------------------------------------------
    def depth_motor_disengage(self):
        self.depth_motor_stop()
        self.ticcmd('-d', self.depth_serial_num, '--deenergize')

    #---------------------------------------------------------------------
    # Returns: the position estimation of the digging motor [microsteps]
    #---------------------------------------------------------------------
    def get_depth_motor_pos(self):
        status = yaml.safe_load(self.ticcmd('-d', self.depth_serial_num, '-s', '--full'))
        return status['Current position']

    #---------------------------------------------------------------------
    # Returns: the velocity estimation of the digging motor [microsteps/ 10000 seconds]
    #---------------------------------------------------------------------
    def get_depth_motor_vel(self):
        status = yaml.safe_load(self.ticcmd('-d', self.depth_serial_num, '-s', '--full'))
        return status['Current velocity']


    ##====================================================================
    ##      PITCH MOTOR FUNCTIONS:
    ##====================================================================

    #---------------------------------------------------------------------
    # Starts the pitch motor by setting a velocity
    # param: speed [microsteps/100000 seconds] (max speed: [ENTER MAX HERE])
    #        + speed = [ENTER CW OR CCW HERE]
    #        - speed = [ENTER CCW OR CW HERE]
    #---------------------------------------------------------------------
    def pitch_motor_turn(self, speed):
        self.ticcmd('-d', self.pitch_serial_num, '--exit-safe-start', '-y', str(speed))

    #---------------------------------------------------------------------
    # Stops the pitch motor by setting speed to 0
    #---------------------------------------------------------------------
    def pitch_motor_stop(self):
        self.ticcmd('-d', self.pitch_serial_num, '--exit-safe-start', '-y', str(0))

    #---------------------------------------------------------------------
    # Engages the pitch motor by reseting zero position & resuming tic36v4
    #---------------------------------------------------------------------
    def pitch_motor_engage(self):
        self.ticcmd('-d', self.pitch_serial_num, '--reset')
        self.ticcmd('-d', self.pitch_serial_num, '--resume')

    #---------------------------------------------------------------------
    # Disengages the pitch motor by deenergizing the tic36v4
    #---------------------------------------------------------------------
    def pitch_motor_disengage(self):
        self.pitch_motor_stop()
        self.ticcmd('-d', self.pitch_serial_num, '--deenergize')

    #---------------------------------------------------------------------
    # Returns: the position estimation of the digging motor [microsteps]
    #---------------------------------------------------------------------
    def get_pitch_motor_pos(self):
        status = yaml.safe_load(self.ticcmd('-d', self.pitch_serial_num, '-s', '--full'))
        return status['Current position']

    #---------------------------------------------------------------------
    # Returns: the velocity estimation of the digging motor [microsteps/ 10000 seconds]
    #---------------------------------------------------------------------
    def get_pitch_motor_vel(self):
        status = yaml.safe_load(self.ticcmd('-d', self.pitch_serial_num, '-s', '--full'))
        return status['Current velocity']

    ##====================================================================
    ##      LOCOMOTION MOTOR FUNCTIONS:
    ##====================================================================

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