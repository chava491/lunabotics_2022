"""
Created on 3-10-2022
By: Abraham Yakisan

This script tests the functionality of the stepper motor velocity command.


"""
import subprocess
import time
import yaml


def ticcmd(*args):
    return subprocess.check_output(['ticcmd'] + list(args))

if __name__ == '__main__':

    # Reset the 'current position' of the motor
    ticcmd('--reset')

    print("Tests the velocity control function of the tic36v4")

    #Energizes the stepper motor
    ticcmd('--resume')

    #Sets a position target in units of steps
    new_target_pos = 200
    ticcmd('--exit-safe-start', '-p', str(new_target_pos))
    time.sleep(3)

    #print current position
    status = yaml.safe_load(ticcmd('-s', '--full'))
    curr_pos = status['Current position']
    print(curr_pos)

    #Sets a position target in units of steps
    new_target_pos = -200
    ticcmd('--exit-safe-start', '-p', str(new_target_pos))
    time.sleep(3)

    #Denergizes stepper motor
    ticcmd('--deenergize')
    print("Ending program.")
