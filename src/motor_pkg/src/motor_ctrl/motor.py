#! /usr/bin/env python

import os
from multiprocessing import Process, Pipe
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

# RPI 3 PWM motor pins
LEFT_FORW = 12
LEFT_BACK = 32
RIGHT_FORW = 33
RIGHT_BACK = 35

def twist2velocity_cb(twist, args):
    """
    Ros topic cmd_vel subscription callback.
    @param twist: ROS topic twist
    """
    # get process pipe connection
    ros_write_conn = args[0]

    # logging
    log_txt = "%s received -> Linear: %s Angular: %s\r" % (rospy.get_caller_id(), twist.linear, twist.angular)
    log_txt = log_txt.replace('\n', ' ')
    rospy.loginfo(log_txt, logger_name="motor_node_logger")

    # convert twist msg to motor velocity
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
  
    # send motor velocities to motor process
    ros_write_conn.send([speed_left, speed_right])

def motor_driver_proc(motor_read_conn):
    """
    Actuate motors process.
    @param s_left: speed left motor
    @param s_right: speed right motor
    """
    # init conn once
    s_left, s_right = motor_read_conn.recv()
    s_left  = s_left * 100
    s_right = s_right * 100

    # spin worker
    while True:
        try:
            if s_left == 0:  pass
            elif s_left > 0: l_pin = LEFT_FORW
            else:            l_pin = LEFT_BACK

            if  s_right == 0: pass
            elif s_right > 0: r_pin = RIGHT_FORW
            else:             r_pin = RIGHT_BACK
            
            # GPIO setup
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            if s_left != 0:GPIO.setup(l_pin, GPIO.OUT)
            if s_right != 0:GPIO.setup(r_pin, GPIO.OUT)

            if s_left != 0: l_pwm = GPIO.PWM(l_pin, 100)
            if s_right != 0:r_pwm = GPIO.PWM(r_pin, 100)

            # actuate
            if s_left != 0: l_pwm.start(abs(s_left) if abs(s_right) < 100 else 100)
            if s_right != 0:r_pwm.start(abs(s_right) if abs(s_right) < 100 else 100)

            # exec till new cmd arrives
            #time.sleep(0)
            s_left_new, s_right_new = motor_read_conn.recv()
            rospy.loginfo("TWIST SPEED: %s, %s\r" % (s_left_new, s_right_new))
            s_left_new  = s_left_new * 100
            s_right_new = s_right_new * 100

            # stop old cmd execution
            if s_left != 0: l_pwm.stop()
            if s_right != 0:r_pwm.stop()

            # update
            s_left = s_left_new
            s_right = s_right_new

        except Exception as ex:
            rospy.logerr("%s\r" % ex)
        finally:
            # reset pin in/out state
            GPIO.cleanup()
    
        # yield thread
        time.sleep(0)

def init_ros_node():
    """
    Motor node main method.
    """
    # load startup node logo
    with open('./name_ascii.txt', 'r') as file:
        ascii_art_str = file.read()
        print("\033[1;34m" + ascii_art_str + "\033[0m")

    # init motor driver process
    motor_read_conn, ros_write_conn = Pipe()
    pm = Process(target=motor_driver_proc, args=(motor_read_conn,))
    pm.start()

    # init ros subscriber node
    rospy.init_node('motor_node', anonymous=False)
    rospy.Subscriber("cmd_vel", Twist, twist2velocity_cb, (ros_write_conn,))
    rospy.spin()

    # join worker
    ros_write_conn.close()
    motor_read_conn.close()
    pm.join()

if __name__ == '__main__':
    init_ros_node()
