!# /usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " ", data.data)

def listen_cmd():
    print('Listening...')
    
    rospy.init_node('motor_ctrl', anonymous=True)
    rospy.Subscriber("scan", String, callback)
    rospy.spin()
