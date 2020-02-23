#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " ", data.data)

def listen_cmd():
    print('Motor ctrl listening...')

    rospy.init_node('motor_node', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()


if __name__ == '__main__':
    listen_cmd()
