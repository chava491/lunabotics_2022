import odrive
from odrive.utils import dump_errors
from odrive.enums import *
import subprocess
import yaml
import time


class Digging:
    # --------------------------------------------------------------------
    # Digging initialize function
    #
    # Establish the odrive connection for digging
    # --------------------------------------------------------------------
    def __int__(self):
        self.serial_num = "00320097"
        try:
            print("Searching for digging o-drive, this may take a few seconds...")
            self.odrv0 = odrive.find_any(serial_number="207939834D4D")  # seaching for o-drive with specific serial number; might change for this years robot
        except:
            print("Unable to find digging o-drive")

        self.dig_engage_depth()
        self.dig_engage_auger()


    # --------------------------------------------------------------------
    # Spin the auger, digging the material below it
    #
    # param: speed -- set the speed of auger movement (max at 67)
    # --------------------------------------------------------------------
    def auger_dig(self, speed):
        print("\n Current drawn from auger digging: \n")
        print(self.odrv0.axis1.motor.current_control.Iq_measured)
        self.odrv0.axis1.controller.input_vel = speed  # Can be either speed or -speed
        # Depends on orientation of motor to spin auger.

    # --------------------------------------------------------------------
    # Spin the auger opposite, to get it unstuck in the case of digging
    # incorrect material
    #
    # param: speed -- set the speed of auger movement (max at 67)
    # --------------------------------------------------------------------
    def auger_undig(self, speed):
        print("\n Current drawn from auger un-digging: \n")
        print(self.odrv0.axis1.motor.current_control.Iq_measured)
        self.odrv0.axis1.controller.input_vel = -speed  # Can be either -speed or speed
        # Depends on orientation of motor to spin auger.

    # --------------------------------------------------------------------
    # Stop the auger at its current location
    #
    # Implementation of stopping by changing input_vel to 0, then delaying,
    # changing to 5, delaying, and changing back to 0 could be to slow down
    # motor before coming to complete stop, so it isn't as aggressive as a stop.
    #
    # This is a hypothesis that needs to be further discussed and tested. If
    # hypothesis is correct, the altering of input_vel can be removed only needing
    # one set to 0.
    # --------------------------------------------------------------------
    def auger_stop(self):
        self.odrv0.axis1.controller.input_vel = 0

        time.sleep(0.1)

        self.odrv0.axis1.controller.input_vel = 5

        time.sleep(0.1)

        self.odrv0.axis1.controller.input_vel = 0

    # --------------------------------------------------------------------
    # Extends the auger shaft and casing deeper into the ground
    #
    # param: speed -- set the speed of depth adjustment (max at 50)
    # --------------------------------------------------------------------
    def depth_extend(self, speed):
        print("\n Current drawn from extending depth: \n")
        print(self.odrv0.axis0.motor.current_control.Iq_measured)
        self.odrv0.axis0.controller.input_vel = speed

    # --------------------------------------------------------------------
    # Retracts the auger shaft and casing from the hole it has dug
    #
    # param: speed -- set the speed of the depth adjustment (max at 50)
    # --------------------------------------------------------------------
    def depth_retract(self, speed):
        print("\n Current drawn from retracting depth: \n")
        print(self.odrv0.axis0.motor.current_control.Iq_measured)
        self.odrv0.axis0.controller.input_vel = -speed

    # --------------------------------------------------------------------
    # Stop the auger at current depth position
    #
    # The implementation to stop the motor that controls the depth, the same
    # way that the stopping of the motor that controls the auger doesn't make sense.
    # The speed at which the depth changes is much slower than the speed at which
    # the auger motor spins. Therefore, it doesn't make sense to slow down the depth
    # motor before stopping it completely.
    #
    # Further discussion and testing needs to be conducted to determine the
    # reasoning behind this implementation and whether it can be simplified.
    # --------------------------------------------------------------------
    def depth_stop(self):
        self.odrv0.axis0.controller.input_vel = 0

        time.sleep(0.1)

        self.odrv0.axis0.controller.input_vel = 5

        time.sleep(0.1)

        self.odrv0.axis0.controller.input_vel = 0

    # --------------------------------------------------------------------
    # Helper function to operate the stepper motor
    #
    # param: *args -- a variable set of arguments used to send commands
    # --------------------------------------------------------------------
    def ticcmd(self, *args):
        return subprocess.check_output(['ticcmd'] + list(args))

    # --------------------------------------------------------------------
    # Rotate the auger forward with the stepper motor
    # --------------------------------------------------------------------
    def stepper_forward(self, pos):
        new_target = (-1 * pos)
        self.ticcmd('--exit-safe-start', 'd', self.serial_num, '--position-relative', str(new_target))

    # --------------------------------------------------------------------
    # Rotate the auger backward with the stepper motor
    # --------------------------------------------------------------------
    def stepper_backward(self, pos):
        new_target = pos
        self.ticcmd('--exit-safe-start', '-d', self.serial_num, '--position-relative', str(new_target))

    # --------------------------------------------------------------------
    # Stop the auger by deactivating the stepper motor
    # --------------------------------------------------------------------
    def stepper_stop(self):
        self.ticcmd('-d', self.serial_num, '--reset')

    # --------------------------------------------------------------------
    # Engages the depth motor by setting their state
    # --------------------------------------------------------------------
    def dig_engage_depth(self):
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    # --------------------------------------------------------------------
    # Disengages the depth motor by setting their state
    # --------------------------------------------------------------------
    def dig_disengage_depth(self):
        self.depth_stop()

        time.sleep(0.1)

        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE

    # --------------------------------------------------------------------
    # Engages the auger motor by setting their state
    # --------------------------------------------------------------------
    def dig_engage_auger(self):
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    # --------------------------------------------------------------------
    # Disengages the auger motor by setting their state
    # --------------------------------------------------------------------
    def dig_disengage_auger(self):
        self.auger_stop()

        time.sleep(0.1)

        self.odrv0.axis1.requested_state = AXIS_STATE_IDLE

    # --------------------------------------------------------------------
    # Disengages the stepper motor by resetting the state
    # --------------------------------------------------------------------
    def dig_disengage_pitch(self):
        self.ticcmd('-d', self.serial_num, '--reset')

    # --------------------------------------------------------------------
    # Dumps all errors from the locomotion odrive
    # --------------------------------------------------------------------
    def dig_dump_errors(self):
        dump_errors(self.odrv0, True)


