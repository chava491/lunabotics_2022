#!/usr/bin/env python3
from json import encoder
import odrive
from odrive.enums import *
import time
from pynput import keyboard

#Global Variables
left_count = 0
left_pos_last = 0
left_m = 0
left_conv_fact = 0.0051664  #m/count
right_count = 0
right_pos_last = 0
right_m = 0
right_conv_fact = 0.0050994 #m/count

def key_press(key):
    global left_count
    global left_pos_last
    global left_m
    global left_conv_fact
    global right_count
    global right_pos_last
    global right_m
    global right_conv_fact

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
        #Forward
        odrv0.axis0.controller.input_vel = -67  #Left wheel
        odrv0.axis1.controller.input_vel = 67   #right wheel
    elif k in ['s']:
        #Backward
        odrv0.axis0.controller.input_vel = 67   #Left wheel
        odrv0.axis1.controller.input_vel = -67  #Right wheel
    elif k in ['q']:
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis1.controller.input_vel = 0

        left_count = odrv0.axis0.encoder.pos_estimate - left_pos_last
        right_count = odrv0.axis1.encoder.pos_estimate - right_pos_last

        left_pos_last = odrv0.axis0.encoder.pos_estimate
        right_pos_last = odrv0.axis1.encoder.pos_estimate

        left_m = left_count * left_conv_fact
        right_m = right_count * right_conv_fact

        print("Left wheel:\t" + str(left_count) + " counts")
        print("Right wheel:\t" + str(right_count) + " counts")
        print("-----------------------------------------------------------------")
        print("Left wheel:\t" + str(left_m) + " m")
        print("Right wheel:\t" + str(right_m) + " m")
        print("=================================================================")


if __name__ == "__main__":


    print("Finding an odrive, this may take a few seconds...")
    odrv0 = odrive.find_any(serial_number="20863880304E")#serial_number="20863880304E"
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