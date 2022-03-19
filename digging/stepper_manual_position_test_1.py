"""
Comment here

"""
import subprocess
#import time
#import math
#import py_compile
from pynput import keyboard
#import subprocess
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


"""
This function reads the keyboard input from the user and moves the robot in the according
"""
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


    if k in ['a']:
        status = load(ticcmd('-s', '--full'), Loader=Loader)
        position = status['Current position']
        new_target = 200
        ticcmd('--exit-safe-start', '-p', str(new_target))
        while position != new_target:
            try:
                ticcmd('--resume')
                status = load(ticcmd('-s', '--full'), Loader=Loader)
                position = status['Current position']
            except:
                print("Unable to resume") 

    elif k in ['d']:
        status = load(ticcmd('-s', '--full'), Loader=Loader)
        position = status['Current position']
        new_target = -200
        ticcmd('--exit-safe-start', '-p', str(new_target))
        while position != new_target:
            try:
                ticcmd('--resume')
                status = load(ticcmd('-s', '--full'), Loader=Loader)
                position = status['Current position']
            except:
                print("Unable to resume")

    elif k in ['s']:
        status = load(ticcmd('-s', '--full'), Loader=Loader)
        position = status['Current position']
        new_target = 0
        ticcmd('--exit-safe-start', '-p', str(new_target))
        while position != new_target:
            try:
                ticcmd('--resume')
                status = load(ticcmd('-s', '--full'), Loader=Loader)
                position = status['Current position']
            except:
                print("Unable to resume")

if __name__ == '__main__':
 
    def ticcmd(*args):
        return subprocess.check_output(['ticcmd'] + list(args))

    #status = load(ticcmd('-s', '--full'), Loader=Loader)
    #position = status['Current position']

    # Reset the 'current position' of the motor
    ticcmd('--reset')

    print("It is time to control the robot!\nThe controls are simple: wasd or the arrow keys move the robot directionally.")
    print("Space will stop the robot in its tracks, and escape will end the control period altogether.")
    
    listener = keyboard.Listener(on_press=key_press)
    listener.start()
    listener.join()
    
    print("Ending program.")
    ticcmd('--deenergize')
