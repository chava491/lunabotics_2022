#!/usr/bin/env python3
from json import encoder
import odrive
from odrive.enums import *
import time
from pynput import keyboard


if __name__ == "__main__":
    print("Finding an odrive, this may take a few seconds...")
    odrv0 = odrive.find_any()
    print("Odrive found...")
    odrv0.axis0.controller.config.control_mode = 2 #Velocity control
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.control_mode = 2 #Velocity control
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    odrv0.axis0.controller.input_vel = 67
    odrv0.axis1.controller.input_vel = -67

    time.sleep(2)

    odrv0.axis0.controller.input_vel = -67
    odrv0.axis1.controller.input_vel = 67

    time.sleep(2)

    odrv0.axis0.controller.input_vel = 0
    odrv0.axis1.controller.input_vel = 0


    print("Ending program.")