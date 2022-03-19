#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script that is used to perform setup and calibration of locomotion motors

@created: Mar. 18, 2022
"""

import odrive
from odrive.enums import *
import fibre.libfibre
import time
import math
import py_compile
from pynput import keyboard

if __name__ == '__main__':

    print("=========================================================================")
    print("This script is used to configure the parameters for the locomotion motors")
    print("and run calibration sequences")
    print("=========================================================================")
    print("Searching for odrive, this may take a few seconds...")
    odrv0 = odrive.find_any()
    print("odrive found...")

    #Setup parameters for motors
    current_lim = 4
    odrv0.axis0.motor.config.current_lim = current_lim
    odrv0.axis1.motor.config.current_lim = current_lim
    print("Set current lim: " + str(current_lim))

    #Velocity limit in turns/s
    vel_limit = 67
    odrv0.axis0.controller.config.vel_limit = vel_limit
    odrv0.axis1.controller.config.vel_limit = vel_limit
    print("Set vel limit: " + str(vel_limit))

    calibration_current = 1
    odrv0.axis0.motor.config.calibration_current = calibration_current
    odrv0.axis1.motor.config.calibration_current = calibration_current
    print("Set calibration current: " + str(calibration_current))

    enable_brake_resistor = 1
    odrv0.config.enable_brake_resistor = enable_brake_resistor
    print("Set enable brake resistor: " + str(enable_brake_resistor))

    brake_resistance = 2.0
    odrv0.config.brake_resistance = brake_resistance
    print("Set brake resistance: " + str(brake_resistance))

    dc_max_negative_current = -10e-6
    odrv0.config.dc_max_negative_current = dc_max_negative_current
    print("Set dc_max_negative_current: " + str(dc_max_negative_current))

    #Num of poles divided by 2
    pole_pairs = 4
    odrv0.axis0.motor.config.pole_pairs = pole_pairs
    odrv0.axis1.motor.config.pole_pairs = pole_pairs
    print("Set pole_pairs: " + str(pole_pairs))

    #set to 8.27 / (motorKV+ "")
    torque_constant = 8.27 / 4.0
    odrv0.axis0.motor.config.torque_constant = torque_constant
    odrv0.axis1.motor.config.torque_constant = torque_constant
    print("Set torque_constant: " + str(torque_constant))

    motor_type = MOTOR_TYPE_HIGH_CURRENT
    odrv0.axis0.motor.config.motor_type = motor_type
    odrv0.axis0.motor.config.motor_type = motor_type
    print("Set motor_type: MOTOR_TYPE_HIGH_CURRENT")

    #Counts per rev. Set to 6*(pole_pairs) for Hall Sensor feedback
    cpr = 6 * pole_pairs
    odrv0.axis0.encoder.config.cpr = cpr
    odrv0.axis1.encoder.config.cpr = cpr
    print("Set cpr: " + str(cpr))

    odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
    odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
    print("Set encoder mode: ENCODER_MODE_HALL")

    odrv0.config.gpio9_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio10_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio11_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio12_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio13_mode = GPIO_MODE_DIGITAL
    odrv0.config.gpio14_mode = GPIO_MODE_DIGITAL
    print("Set gpio9-14 modes to GPIO_MODE_DIGITAL")

    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    print("Set state: AXIS_STATE_CLOSED_LOOP_CONTROL")

    odrv0.axis0.controller.config.control_mode = 2 #Velocity control
    odrv0.axis1.controller.config.control_mode = 2 #Velocity control
    print("Set control mode: VELOCITY ")

    print("Saving configuration to odrive...")
    print("Rebooting...")

    try:
        odrv0.save_configuration()
    except:
        fibre.libfibre.ObjectLostError
        pass # Saving configuration makes the device reboot


    print("Searching for odrive, this may take a few seconds...")
    odrv0 = odrive.find_any()
    print("odrive found...")

    #Start Performing Calibrations and Checks
    calib_time = 15

    print("axis0: Performing AXIS_STATE_MOTOR_CALIBRATION")
    odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(calib_time)
    print("done...")

    print("axis1: Performing AXIS_STATE_MOTOR_CALIBRATION")
    odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(calib_time)
    print("done...")

    #Errors should be 0
    print("===================================================")
    print("===================================================")
    print("Calibration results (All Errors Should be 0):")
    print("===================================================")
    print("axis0.motor.error: " + str(odrv0.axis0.motor.error))
    print("axis0.motor.config.phase_inductance: " + str(odrv0.axis0.motor.config.phase_inductance))
    print("axis0.motor.config.phase_resistance: " + str(odrv0.axis0.motor.config.phase_resistance))
    print("===================================================")
    print("axis1.motor.error: " + str(odrv0.axis1.motor.error))
    print("axis1.motor.config.phase_inductance: " + str(odrv0.axis1.motor.config.phase_inductance))
    print("axis1.motor.config.phase_resistance: " + str(odrv0.axis1.motor.config.phase_resistance))
    print("===================================================")
    print("===================================================")

    #These lines assume no errors occured
    odrv0.axis0.motor.config.pre_calibrated = True
    odrv0.axis1.motor.config.pre_calibrated = True

    print("axis0: Performing AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION")
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
    time.sleep(calib_time)
    print("done...")

    print("axis1: Performing AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION")
    odrv0.axis1.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
    time.sleep(calib_time)
    print("done...")

    print("===================================================")
    print("===================================================")
    print("Calibration results (All Errors Should be 0):")
    print("===================================================")
    print("axis0.encoder.error: " + str(odrv0.axis0.encoder.error))
    print("axis1.encoder.error: " + str(odrv0.axis1.encoder.error))
    print("===================================================")
    print("===================================================")

    print("axis0: Performing AXIS_STATE_ENCODER_OFFSET_CALIBRATION")
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(calib_time)
    print("done...")

    print("axis1: Performing AXIS_STATE_ENCODER_OFFSET_CALIBRATION")
    odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(calib_time)
    print("done...")

    print("===================================================")
    print("===================================================")
    print("Calibration results (All Errors Should be 0:")
    print("phase_offset_float should be -1.5,-0.5,0.5,1.5)")
    print("===================================================")
    print("axis0.encoder.error: " + str(odrv0.axis0.encoder.error))
    print("axis0.encoder.config.phase_offset_float: " + str(odrv0.axis0.encoder.config.phase_offset_float))
    print("===================================================")
    print("axis1.encoder.error: " + str(odrv0.axis1.encoder.error))
    print("axis1.encoder.config.phase_offset_float: " + str(odrv0.axis1.encoder.config.phase_offset_float))
    print("===================================================")
    print("===================================================")

    #These lines assume no errors occured
    odrv0.axis0.encoder.config.pre_calibrated = True
    odrv0.axis1.encoder.config.pre_calibrated = True

    print("Saving configuration to odrive...")
    print("Rebooting...")

    try:
        odrv0.save_configuration()
    except:
        fibre.libfibre.ObjectLostError
        pass # Saving configuration makes the device reboot

    print("Searching for odrive, this may take a few seconds...")
    odrv0 = odrive.find_any()
    print("odrive found...")

    print("Ending program...")
