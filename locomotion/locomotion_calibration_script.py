#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script that is used to perform setup and calibration of locomotion motors

There are a few situations that need to be taken into account when running this script, and any errors that might occur
while calibrating.

This is documentation for the main errors that have been run into while setting up the o-drive for the locomotion motors.
If other errors occur that are not discussed in this documentation, look up the error number by typing "odrv0.axis0.motor"
when calibrating the motor and the error number when calibrating the encoders by typing "odrv0.axis0.encoder", as well
as "dump_errors(odrv0)" to check error titles after every calibration test. Then you will have to do independent research
of those errors in relation to your situation. The official o-drive error and parameter documentation, which can be found
at https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html , can be helpful in certain
situations.

1. The locomotion o-drive NEEDS to be powered by a battery and not from a power supply. The power supplies do NOT supply
   enough current for the motors and o-drive, while the battery does supply enough current.
        No matter what, if you try to calibrate the locomotion o-drive and it is powered by a power supply that does NOT
        supply enough current, you will get past the motor calibration step, but will not get past the hall encoder
        calibration step. Again, you will pass "odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION", but you
        will NOT pass "odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION". You will run into
        a system error that says "DC_BUS_UNDER_VOLTAGE". Even though if you look under "odrv0.axis0.encoder" it will say
        "error: 0" it says that because the calibration process never even completed. The power supply's current
        spiked and the voltage dropped, causing the power supply to protect itself and turn off and on again during the
        hall encoder calibration step.

2. When running this calibration script for the locomotion motors, the motors can NOT be under any sort of load at all.
   So, if the motors are on the robot, the chains need to be removed from the single gear attached to the sprocket on both
   motors.
        That also goes for the Auger motor when running it on the Auger o-drive, there can NOT be any load on the motor.

3. If you run the first calibration step "odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION", check
   "odrv0.axis0.motor" and see if "error: 0". Whether it is 0 or not, ALWAYS look at the errors by typing
   "dump_errors(odrv0).
        If the error is "PHASE_RESISTANCE_OUT_OF_RANGE" then you need to change the ".motor.config.calibration_current"
        lower. Default is 10, good practice would be to set it to 2.

        If the error is "CURRENT_LIMIT_VIOLATION" then the value of ".motor.config.current_lim" +
        ".motor.config.current_lim_margin" is less than the current that is being supplied to the o-drive, therefore it will
        stop itself from having too much current supplied to it. Setting both ".motor.config.current_lim" and
        ".motor.config.current_lim_margin" equal to 8, is a good starting point. We know that the motors WILL operate
        under those conditions.

        If the error is "BRAKE_RESISTOR_DISARMED" make sure that "odrv0.config.enable_brake_resistor" is set to "True".
        Then check the value of "odrv0.config.brake_resistance" and make sure that it is set to 2. The locomotion o-drive
        must use one of the 2 ohm brake resistors. If either of those parameters are changed, you NEED to run
        "odrv0.save_configuration()" right after changing those parameters.
            If the error "BRAKE_RESISTOR_DISARMED" occurs any other time, it is usually associated with another error.

4. When running each of the different calibrations, you NEED to pass each one before moving onto the next. You CANNOT do
   the "odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION" without passing
   "odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION". Same with the 2nd and 3rd calibration tests,
   you CANNOT run "odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION" before running
   "odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION". Each calibration relies on the previous
   calibration. So if there is an error after running "odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION"
   you need to solve it before moving on. If there is an error after running
   "odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION" you need to solve it before moving on.
   If there is an error after running "odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION", you need to
   solve it in order for the calibration process to be completed and the motors to properly function.


@created: Mar. 18, 2022
"""

import odrive
from odrive.enums import *
import fibre
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
    odrv0 = odrive.find_any(serial_number="20863880304E")
    print("odrive found...")

    print("Erasing previous configuration...")
    try:
        odrv0.erase_configuration()
    except:
        fibre.libfibre.ObjectLostError
        pass # Erasing configuration makes the device reboot

    print("Searching for odrive, this may take a few seconds...")
    odrv0 = odrive.find_any(serial_number="20863880304E")
    print("odrive found...")

    #Setup current parameters for motors
    current_lim = 8
    odrv0.axis0.motor.config.current_lim = current_lim
    odrv0.axis1.motor.config.current_lim = current_lim
    print("Set current lim: " + str(current_lim))

    current_lim_margin = 8
    odrv0.axis0.motor.config.current_lim_margin = current_lim_margin
    odrv0.axis1.motor.config.current_lim_margin = current_lim_margin
    print("Set current lim: " + str(current_lim_margin))

    #Velocity limit in turns/s
    vel_limit = 67
    odrv0.axis0.controller.config.vel_limit = vel_limit
    odrv0.axis1.controller.config.vel_limit = vel_limit
    print("Set vel limit: " + str(vel_limit))

    calibration_current = 2
    odrv0.axis0.motor.config.calibration_current = calibration_current
    odrv0.axis1.motor.config.calibration_current = calibration_current
    print("Set calibration current: " + str(calibration_current))

    enable_brake_resistor = 1
    odrv0.config.enable_brake_resistor = enable_brake_resistor
    print("Set enable brake resistor: " + str(enable_brake_resistor))

    brake_resistance = 2.0
    odrv0.config.brake_resistance = brake_resistance
    print("Set brake resistance: " + str(brake_resistance))

    dc_max_negative_current = -0.01
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
    odrv0 = odrive.find_any(serial_number="20863880304E")
    print("odrive found...")

    #Start Performing Calibrations and Checks
    calib_time = 15

    print("axis0: Performing AXIS_STATE_MOTOR_CALIBRATION")
    odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(calib_time)
    print("done...")

    print("axis1: Performing AXIS_STATE_MOTOR_CALIBRATION")
    odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
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

    print("Setting proper gain values for motors....")

    #Gain settings
    pos_gain = 1.3
    odrv0.axis0.controller.config.pos_gain = pos_gain
    odrv0.axis1.controller.config.pos_gain = pos_gain
    print("Set pos_gain for axis0: " + str(pos_gain))
    print("Set pos_gain for axis1: " + str(pos_gain))

    vel_gain = 0.16
    odrv0.axis0.controller.config.vel_gain = vel_gain
    odrv0.axis1.controller.config.vel_gain = vel_gain
    print("Set vel_gain for axis0: " + str(vel_gain))
    print("Set vel_gain for axis1: " + str(vel_gain))

    vel_integrator_gain = 2.5
    odrv0.axis0.controller.config.vel_integrator_gain = vel_integrator_gain
    odrv0.axis1.controller.config.vel_integrator_gain = vel_integrator_gain
    print("Set vel_integrator_gain for axis0: " + str(vel_integrator_gain))
    print("Set vel_integrator_gain for axis1: " + str(vel_integrator_gain))
    print("===================================================")
    print("===================================================")

    print("Saving configuration to odrive...")
    print("Rebooting...")

    try:
        odrv0.save_configuration()
    except:
        fibre.libfibre.ObjectLostError
        pass # Saving configuration makes the device reboot

    print("Searching for odrive, this may take a few seconds...")
    odrv0 = odrive.find_any(serial_number="20863880304E")
    print("odrive found...")

    print("Ending program...")
