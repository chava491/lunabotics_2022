#!/usr/bin/env python

"""
This script is used to take readings from keyboard button pushes and publish them for manual control
"""

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty

instructions = '---------------------------------------------------------\n' \
    'Reading from the keyboard and Publishing to /main_control!\n' \
    'Use the following keys to control the robot:\n' \
    '---------------------------------------------------------\n' \
    'locomotion: \n' \
        '\tforward/backward & right/left:\n' \
        '\t\t\t' + str(rospy.get_param('/mars_robot/manual_control_keys/loco_forward_key')) + '\n' +\
        '\t\t' + str(rospy.get_param('/mars_robot/manual_control_keys/loco_left_key')) + \
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/loco_backward_key')) + \
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/loco_right_key')) + '\n' +\
        '\tspace: stop loco motors\n' \
    'Auger motor:\n'    \
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/auger_dig_key')) + ':      auger on\n' +\
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/auger_stop_key')) + ':      auger stop\n' +\
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/auger_dig_key')) + ':      auger reverse\n' +\
    'Pitch motor:\n'    \
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/pitch_increase_key')) + ':      pitch increase\n' +\
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/pitch_stop_key')) + ':      pitch stop\n' +\
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/pitch_decrease_key')) + ':      pitch decrease\n' +\
    'Depth motor:\n'    \
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/depth_decrease_key')) + ':      depth decrease\n' +\
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/depth_stop_key')) + ':      depth stop\n' +\
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/depth_increase_key')) + ':      depth increase\n' +\
    'Dumping actuator:\n'    \
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/dumpa_extend_key')) + ':      extend actuator\n' +\
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/dumpa_stop_key')) + ':      stop actuator\n' +\
        '\t' + str(rospy.get_param('/mars_robot/manual_control_keys/dumpa_retract_key')) + ':      retract actuator\n' +\
    'CTRL-C to quit'


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('main_control', String, queue_size=1)

        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def stop(self):
        self.done = True
        self.join()


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    #Create a mars_robot_teleop node
    rospy.init_node('mars_robot_teleop')

    #Publisher for String codes on /main_control topic
    pub = rospy.Publisher('main_control', String, queue_size=1)
    control_msg = String()

    #Get keybindings from ROS parameter server (defined in mars_robot_params.yaml file)
    keyBindings = rospy.get_param('/mars_robot/manual_control_keys')

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    try:
        #pub_thread.wait_for_subscribers()
        print(instructions)


        
        while(1):
            key = getKey(key_timeout)
            #print("key pressed: " + str(key))
            
            #Publish String codes on /main_control topic
            if key in keyBindings.values():
                control_msg.data = key #set the msg data field
                pub.publish(control_msg)
            
            else:
                # Skip updating main_control if key timeout and robot already
                # stopped.
                if key == '':
                    continue
                if (key == '\x03'):
                    break

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
