"""
Created on 3-10-2022
By: Abraham Yakisan

This script tests the functionality of the stepper motor velocity command.


"""
import subprocess
import time
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


def ticcmd(*args):
    return subprocess.check_output(['ticcmd'] + list(args))

if __name__ == '__main__':

    # Reset the 'current position' of the motor
    ticcmd('--reset')

    print("Tests the velocity control function of the tic36v4")

    #Energizes the stepper motor
    ticcmd('--resume')

    #Sets a velocity target in units of steps per 10000 seconds
    new_target_vel = 500000
    ticcmd('--exit-safe-start', '-y', str(new_target_vel))
    time.sleep(10)

    #Sets a velocity target in units of steps per 10000 seconds
    new_target_vel = -500000
    ticcmd('--exit-safe-start', '-y', str(new_target_vel))
    time.sleep(10)

    #Denergizes stepper motor
    ticcmd('--deenergize')
    print("Ending program.")
