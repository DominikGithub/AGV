#! /usr/bin/env python

import os
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

def twist2velocity(twist):
    """
    Convert twist to motor velocities.
    @param twist: ROS topic twist (linear, angular)
    """
    if twist.angular.z == 0:
        speed_left = twist.linear.x
        speed_right = speed_left
    elif twist.angular.z != 0 and twist.linear.x == 0:
        speed_right = twist.angular.z * 0.19
        speed_left = - speed_right
    else:
        speed_angular = twist.angular.z * 0.19
        speed_left = twist.linear.x - speed_angular
        speed_right = twist.linear.x + speed_angular
  
    setSpeed(speed_left, speed_right)

def constrain(value, low, high):
    """
    Limit max velocity.
    @param value: actual speed
    @param low: max negative speed limit
    @param high: max positive speed limit
    @returns: constrained speed value
    """
    if value < low: return low
    if value > high: return high
    return value

def setSpeed(s_left, s_right):
    """
    Actuate differential drive motors.
    @param s_left: speed left motor
    @param s_right: speed right motor
    """

    rospy.loginfo("TWIST SPEED: %s, %s\r" % (s_left, s_right))

    s_left  = s_left * 100
    s_right = s_right * 100

    # motor
    try:
        # motor pins
        left_forw = 12
        left_back = 32
        right_forw = 33
        right_back = 35

        if s_left == 0:  pass
        elif s_left > 0: l_pin = left_forw
        else:            l_pin = left_back

        if  s_right == 0: pass
        elif s_right > 0: r_pin = right_forw
        else:             r_pin = right_back
        
        # GPIO setup
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        if s_left != 0:GPIO.setup(l_pin, GPIO.OUT)
        if s_right != 0:GPIO.setup(r_pin, GPIO.OUT)

        if s_left != 0: l_pwm = GPIO.PWM(l_pin, 10)
        if s_right != 0:r_pwm = GPIO.PWM(r_pin, 10)

        # actuate
        if s_left != 0: l_pwm.start(abs(s_left) if abs(s_right) < 100 else 100)
        if s_right != 0:r_pwm.start(abs(s_right) if abs(s_right) < 100 else 100)
        time.sleep(1)

        # stop
        if s_left != 0: l_pwm.stop()
        if s_right != 0:r_pwm.stop()
    except Exception as ex:
        rospy.logerr("%s\r" % ex)
    finally:
        # reset pin in/out state
        GPIO.cleanup()

def callback(twist):
    """
    Ros topic cmd_vel subscription callback.
    @param twist: ROS topic twist
    """
    log_txt = "%s received -> Linear: %s Angular: %s\r" % (rospy.get_caller_id(), twist.linear, twist.angular)
    log_txt = log_txt.replace('\n', ' ')
    rospy.loginfo(log_txt, logger_name="motor_node_logger")

    # convert twist to motor velocity
    twist2velocity(twist)


def listen_cmd():
    """
    Motor node main method.
    """
    # load startup node logo
    with open('./name_ascii.txt', 'r') as file:
        ascii_art_str = file.read()
        print("\033[1;34m" + ascii_art_str + "\033[0m")

    # init node
    rospy.init_node('motor_node', anonymous=False)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listen_cmd()
