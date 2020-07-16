#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

def callback(twist):
    log_txt = "%s received -> Linear: %s Angular: %s" % (rospy.get_caller_id(), twist.linear, twist.angular)
    log_txt = log_txt.replace('\n', ' ')
    rospy.loginfo(log_txt, logger_name="motor_node_logger")

    # motor pins
    left_forw = 12
    left_back = 32
    right_forw = 33
    right_back = 35

    #TODO set real direction here from Twist
    l_pin = left_forw
    r_pin = right_back
    rospy.loginfo("%s %s " % (l_pin, r_pin))

    # gpio setup
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(l_pin, GPIO.OUT)
    GPIO.setup(r_pin, GPIO.OUT)

    l_pwm = GPIO.PWM(l_pin, 1)
    r_pwm = GPIO.PWM(r_pin, 1)

    # control
    l_pwm.start(50)
    r_pwm.start(50)
    time.sleep(1)

    # stop
    l_pwm.stop()
    r_pwm.stop()
    GPIO.cleanup()


def listen_cmd():
    rospy.loginfo('Motor ctrl listening...')

    rospy.init_node('motor_node', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listen_cmd()
