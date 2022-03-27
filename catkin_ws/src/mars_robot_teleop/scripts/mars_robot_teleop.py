#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

import sys, select, termios, tty

instructions = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
locomotion:
    space: stop loco motors
        w    
   a    s    d    

Auger motor:
    m:      auger on
    ,:      auger stop
    .:      auger reverse
Pitch motor:
    j:      pitch increase
    k:      pitch stop
    l:      pitch decrease
Depth motor:
    u:      depth decrease
    i:      depth stop
    o:      depth increase
Dumping actuator:
    7:      extend
    8:      stop
    9:      retract


CTRL-C to quit
"""

keyBindings = {
        'w':0,  #loco forward
        'a':1,  #loco left
        's':2,  #loco backward
        'd':3,  #loco right
        ' ':4,  #loco stop
        'm':5,  #auger dig on 
        ',':6,  #auger stop 
        '.':7,  #auger dig reverse
        'j':8,  #increase pitch
        'k':9,  #stop pitch
        'l':10,  #decrease pitch
        'u':11, #decrease depth
        'i':12, #stop depth
        'o':13, #increase depth
        '7':14, #extend dumpa
        '8':15, #stop dumpa
        '9':16, #retract dumpa
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('main_manual', Int32, queue_size=1)

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

    #Publisher for Int32 codes on /main_manual topic
    pub = rospy.Publisher('main_manual', Int32, queue_size=1)
    control_msg = Int32()

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    try:
        #pub_thread.wait_for_subscribers()
        print(instructions)
        
        while(1):
            key = getKey(key_timeout)
            print("key pressed: " + str(key))
            
            #Publish Int32 codes on /main_manual topic
            if key in keyBindings:
                control_msg.data = keyBindings[key] #get code from dictionary
                pub.publish(control_msg)
            
            else:
                # Skip updating main_manual if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
