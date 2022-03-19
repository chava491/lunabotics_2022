#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The second locomotion test file for testing our robot's locomotion.
Attempting to include keyboard operations to the motor's output, allowinf direct control over motion.

@created: Oct 4, 2020
"""

import odrive
from odrive.enums import *
import time
import math
import py_compile
from pynput import keyboard

"""
This function reads the keyboard input from the user and moves the robot in the according
"""
def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
        key = key.char
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

    if key in ['w']:

        odrv0.axis0.controller.input_vel = -50
        odrv0.axis1.controller.input_vel = 50

    elif key in ['a']:
        odrv0.axis0.controller.input_vel = 50
        odrv0.axis1.controller.input_vel = 50

    elif key in ['s']:
        odrv0.axis0.controller.input_vel = 50
        odrv0.axis1.controller.input_vel = -50

    elif key in ['d']:
        odrv0.axis0.controller.input_vel = -50
        odrv0.axis1.controller.input_vel = -50

    elif key in ['space']:
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis1.controller.input_vel = 0

    #-------------------------------------------
    # Individual motor test
    #-------------------------------------------
    elif key in ['r']:
        odrv0.axis0.controller.input_vel = 20
        
    elif key in ['l']:
        odrv0.axis1.controller.input_vel = 20

    elif key in ['e']:
        odrv0.axis.encoder

    elif key in ['m']:
        odrv0.axis.motor

def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

    elif key in ['w']:
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis1.controller.input_vel = 0

    elif key in ['a']:
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis1.controller.input_vel = 0

    elif key in ['s']:
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis1.controller.input_vel = 0

    elif key in ['d']:
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis1.controller.input_vel = 0

    elif key in ['space']:
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis1.controller.input_vel = 0

    # -------------------------------------------
    # Individual motor test
    # -------------------------------------------
    elif key in ['r']:
        odrv0.axis0.controller.input_vel = 0

    elif key in ['l']:
        odrv0.axis1.controller.input_vel = 0


if __name__ == '__main__':

    # default speed value
#     speed = 50

    print("Searching for odrive, this may take a few seconds...\n")
    odrv0 = odrive.find_any()

    odrv0.axis0.controller.config.control_mode = 2 #Velocity control
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.control_mode = 2 #Velocity control
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    print("It is time to control the robot!\nThe controls are simple: wasd or the arrow keys move the robot directionally.")
    print("Space will stop the robot in its tracks, and escape will end the control period altogether.")
    # Collect events until released
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()

    # listener = keyboard.Listener(on_press=key_press)
    # listener.start()
    # listener.join()
    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv0.axis1.requested_state = AXIS_STATE_IDLE
    print("Ending program.")
