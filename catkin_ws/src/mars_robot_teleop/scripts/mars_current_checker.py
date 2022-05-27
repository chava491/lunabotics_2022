#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time


if __name__ == "__main__":
    rospy.init_node("motor_current_checker_node")

    motor_current_checker = rospy.Publisher('main_control', String, queue_size=10)
    check_motor_current_msg = String()
    check_motor_current_msg.data = rospy.get_param('/mars_robot/diagnostic_values/print_motor_data')
    print_interval = rospy.get_param('/mars_robot/diagnostic_values/print_interval')

    while(not rospy.is_shutdown()):
        motor_current_checker.publish(check_motor_current_msg)
        time.sleep(print_interval)



