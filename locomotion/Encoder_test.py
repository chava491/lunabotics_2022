#!/usr/bin/env python3
from json import encoder
import odrive
from odrive.enums import *
import time
from pynput import keyboard

def key_press(key):
    # stop looking for key inputs 
    if key == keyboard.Key.esc:
        return False
    
    try:
        # meant for single character keys - w, a, s, d etc.
        k = key.char
    except:
        # meant for other keys - arrow keys, etc.
        k = key.name
    
    if k in ['w']:
        odrv0.axis0.controller.input_vel = -67
        odrv0.axis1.controller.input_vel = 67
    elif k in ['s']:
        odrv0.axis0.controller.input_vel = 67
        odrv0.axis1.controller.input_vel = -67
    elif k in ['q']:
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis1.controller.input_vel = 0
        print("Left wheel:\t" + str(odrv0.axis0.encoder.pos_estimate) + " counts")
        print("Right wheel:\t" + str(odrv0.axis1.encoder.pos_estimate) + " counts")

if __name__ == "__main__":
    print("Finding an odrive, this may take a few seconds...")
    odrv0 = odrive.find_any()
    odrv0.axis0.controller.config.control_mode = 2 #Velocity control
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.control_mode = 2 #Velocity control
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    print("Odrive found...")
    
    print("Ready to control")
    listener = keyboard.Listener(on_press=key_press)
    listener.start()
    listener.join()
    print("Ending program.")