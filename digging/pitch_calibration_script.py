"""
Created on 3-24-2022
By: Abraham Yakisan

This script sets all of the parameters of the pitch stepper motor.

UPDATE: I believe implementation of such a file would involve senting bytes over serial to the tic
but I have not been able to succesfully do this. The effort involved to perform this does not seem to 
balance the benefits. Just use ticgui terminal command and use the GUI to update parameters manually

"""
import subprocess
import time
import yaml
import serial


def ticcmd(*args):
    return subprocess.check_output(['ticcmd'] + list(args))

if __name__ == '__main__':
    pitch_tic_port = '/dev/ttyACM0' #Port of pitch tic36v4 stepper driver
    pitch_tic_serial_num = '00320097' #pitch tic36v4 stepper driver serial number

    setter = serial.Serial(port=pitch_tic_port, baudrate=9600, timeout=.1)

    #msg that should set step mode to 1/8th
    msg = b'\0x40\0x13\0x03\0x41\0x00'

    setter.write(msg)
    
    # # Reset the 'current position' of the motor
    # ticcmd('-d', pitch_tic_serial_num, 
