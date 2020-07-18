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
  
    vel = setSpeedms(speed_left, speed_right)

def setSpeedms(speed_left_ms, speed_right_ms):
    """
    Convert speed to time.
    @param speed_left_ms:
    @param speed_right_ms:
    """
    speed_left = msToSteps(speed_left_ms)
    speed_right = msToSteps(speed_right_ms)
    setSpeed(speed_left, speed_right)

def msToSteps(speed):
    """
    Convert time to steps.
    @param speed: speed in time [ms]
    @returns speed: speed in steps
    """
    MM_PER_ROTATION = 2 * 3.142 * 10
    STEPS_PER_REVOLUTION = 1.0
    return (int)((speed * 1000 * STEPS_PER_REVOLUTION) / MM_PER_ROTATION)

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

def setSpeed(speed_left, speed_right):
    """
    Actuate differential drive motors.
    @param speed_left: speed left motor
    @param speed_right: speed right motor
    """

    # limit max speed
    DEFAULT_TOP_SPEED = 0.1
    s_left = constrain(speed_left, -DEFAULT_TOP_SPEED, DEFAULT_TOP_SPEED)
    s_right = constrain(speed_right, -DEFAULT_TOP_SPEED, DEFAULT_TOP_SPEED)

    rospy.loginfo("MOTOR SPEED: %s, %s" % (s_left, s_right))

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
        #rospy.loginfo("%s %s " % (l_pin, r_pin))

        # GPIO setup
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        if s_left != 0:GPIO.setup(l_pin, GPIO.OUT)
        if s_right != 0:GPIO.setup(r_pin, GPIO.OUT)

        if s_left != 0: l_pwm = GPIO.PWM(l_pin, 1)
        if s_right != 0:r_pwm = GPIO.PWM(r_pin, 1)

        # actuate
        if s_left != 0: l_pwm.start(100)
        if s_right != 0:r_pwm.start(100)
        time.sleep(0.5)

        # stop
        if s_left != 0: l_pwm.stop()
        if s_right != 0:r_pwm.stop()
    except Exception as ex:
        rospy.logerr("%s" % ex)
    finally:
        # reset pin in/out state
        GPIO.cleanup()

def callback(twist):
    """
    Ros topic cmd_vel subscription callback.
    @param twist: ROS topic twist
    """
    log_txt = "%s received -> Linear: %s Angular: %s" % (rospy.get_caller_id(), twist.linear, twist.angular)
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
