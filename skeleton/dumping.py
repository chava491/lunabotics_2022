import subprocess
import yaml
import time
from roboclaw import Roboclaw

class Dumping:

    # --------------------------------------------------------------------
    # Dumping initialization function
    #
    # Establish the roboclaw connection for the linear actuator
    # --------------------------------------------------------------------
    def __init__(self):
        self.serial_num = "00320100" #may change for this years robot
        try:
            print("Searching for dumping roboclaw, this may take a few seconds...")
            self.roboclaw = Roboclaw("/dev/ttyACM0", 38400)
            self.roboclaw.Open()
            print("Dumping roboclaw connected successfully")
        except:
            print("Unable to find dumping roboclaw")

    def ticcmd(self, *args):


    def stepper_forward