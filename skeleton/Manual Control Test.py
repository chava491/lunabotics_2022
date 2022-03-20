#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The second locomotion key file for keying our robot's locomotion.
Attempting to include key.charboard operations to the motor's output, allowinf direct control over motion.

@created: Mar 14, 2022
"""

import odrive
from odrive.enums import *
import time
import math
import py_compile
from pynput import keyboard
import subprocess
from yaml import load, dump
import dumping

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


"""
This function reads the key.charboard input from the user and moves the robot in the according
"""
def on_press(key):
    try:
        print('alphanumeric key.char {0} pressed'.format(key.char))

        #Loco Forward
        if key.char in ['w']:
            odrv1.axis0.controller.input_vel = -50
            odrv1.axis1.controller.input_vel = 50
        #Loco Left
        elif key.char in ['a']:
            odrv1.axis0.controller.input_vel = 50
            odrv1.axis1.controller.input_vel = 50
        #Loco Backward
        elif key.char in ['s']:
            odrv1.axis0.controller.input_vel = 50
            odrv1.axis1.controller.input_vel = -50
        #Loco Right
        elif key.char in ['d']:
            odrv1.axis0.controller.input_vel = -50
            odrv1.axis1.controller.input_vel = -50
        #Normal Mining
        elif key.char in ['m']:
            odrv0.axis0.controller.input_vel = 16
        #Reverse Mining
        elif key.char in ['n']:
            odrv0.axis0.controller.input_vel = -16
        #Increase Pitch
        elif key.char in ['h']:
            ticcmd('--resume')
            new_target_vel = 500000
            ticcmd('--exit-safe-start', '-y', str(new_target_vel))
        #Decrease Pitch
        elif key.char in ['k']:
            ticcmd('--resume')
            new_target_vel = -500000
            ticcmd('--exit-safe-start', '-y', str(new_target_vel))
        #Stop Pitch
        elif key.char in ['j']:
            ticcmd('--resume')
            new_target_vel = -500000
            ticcmd('--exit-safe-start', '-y', str(new_target_vel))

        #-------------------------------------------
        # Individual motor key
        #-------------------------------------------
        #Right motor
        elif key.char in ['r']:
            odrv1.axis0.controller.input_vel = 20
        #Left motor
        elif key.char in ['l']:
            odrv1.axis1.controller.input_vel = 20

    except AttributeError:
        print('special key.char {0} pressed'.format(key))

def on_release(key):
    print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        ticcmd('--deenergize')
        return False

    elif key == keyboard.Key.space:
        odrv0.axis0.controller.input_vel = 0

    elif key == keyboard.Key.up:

        dump.actuator_extend()

    elif key.char in ['w']:
        odrv1.axis0.controller.input_vel = 0
        odrv1.axis1.controller.input_vel = 0

    elif key.char in ['a']:
        odrv1.axis0.controller.input_vel = 0
        odrv1.axis1.controller.input_vel = 0

    elif key.char in ['s']:
        odrv1.axis0.controller.input_vel = 0
        odrv1.axis1.controller.input_vel = 0

    elif key.char in ['d']:
        odrv1.axis0.controller.input_vel = 0
        odrv1.axis1.controller.input_vel = 0

    # -------------------------------------------
    # Individual motor key
    # -------------------------------------------
    elif key.char in ['r']:
        odrv1.axis0.controller.input_vel = 0

    elif key.char in ['l']:
        odrv1.axis1.controller.input_vel = 0

def ticcmd(*args):
    return subprocess.check_output(['ticcmd'] + list(args))

if __name__ == '__main__':
    dump = dumping.Dumping()
    print("Searching for odrive, this may take a few seconds...\n")

    odrv1 = odrive.find_any(serial_number="20863880304E")#Locomotion motors/odrive
    odrv0 = odrive.find_any(serial_number="207939834D4D")#Mining motors/odrive
    #Start Roboclaw
    dump.enable_roboclaw()
    #Set Odrive state and control modes
    odrv0.axis0.controller.config.control_mode = 2 #Velocity control
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis0.controller.config.control_mode = 2 #Velocity control
    odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis1.controller.config.control_mode = 2 #Velocity control
    odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #Reset roboclaw zero position
    ticcmd('--reset')

    print("It is time to control the robot!\nThe controls are simple: wasd or the arrow key.chars move the robot directionally.")
    print("Space will stop the robot in its tracks, and escape will end the control period altogether.")
    # Collect events until released
    with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:
        listener.join()

    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv1.axis0.requested_state = AXIS_STATE_IDLE
    odrv1.axis1.requested_state = AXIS_STATE_IDLE

    print("Ending program.")
