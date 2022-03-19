"""
Created on 3-19-2022
By: Antonio Carrasco 

This script allows you to control the direction of the motor's rotation by pressing and holding the appropiate keys.


"""
import subprocess
from pynput import keyboard
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


"""
This function reads the keyboard input from the user and moves the robot in the according
"""
def on_press(key):
    if key == keyboard.Key.esc:
        # Stop listener
        print("Ending program.")
        ticcmd('--deenergize')
        return False

    try:
        #print('alphanumeric key {0} pressed'.format(key.char))
        ticcmd('--resume')
        if key.char in ['a']:
            new_target_vel = 500000
            ticcmd('--exit-safe-start', '-y', str(new_target_vel))
        elif key.char in ['d']:
            new_target_vel = -500000
            ticcmd('--exit-safe-start', '-y', str(new_target_vel))
    except AttributeError:
        print('special key {0} pressed'.format(key))

def on_release(key):
    #print('{0} released'.format(key))
    new_target_vel = 0
    ticcmd('--exit-safe-start', '-y', str(new_target_vel))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

if __name__ == '__main__':
 
    def ticcmd(*args):
        return subprocess.check_output(['ticcmd'] + list(args))

    #status = load(ticcmd('-s', '--full'), Loader=Loader)
    #position = status['Current position']

    # Reset the 'current position' of the motor
    ticcmd('--reset')

    print("Control the direction of the motors rotation by pressing")
    print("'a' or 'd' on the keyboard. (Press and hold)")
    print("Press the 'esc' key end the program")
    
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()