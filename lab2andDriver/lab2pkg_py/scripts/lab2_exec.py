#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
import time
from math import pi
from lab2_header import *
from lab2_func import *
from rospy.client import spin

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])
zero_position = np.radians([180, 0, 0, 0, 0, 0])

twist = Twist()

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False
current_twist_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

# Robot current velocities
current_twist = Twist()

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True
    
"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 

"""
Whenever /cmd_vel publishes info this callback function is called.
"""
def twist_callback(msg):

    global twist
    global current_twist
    global current_twist_set

    rospy.loginfo(msg)
    twist.linear.x = msg.linear.x
    twist.linear.y = msg.linear.y
    twist.linear.z = msg.linear.z  
    twist.angular.x = msg.angular.x
    twist.angular.y = msg.angular.y
    twist.angular.z = msg.angular.z
    
    current_twist.linear.x = twist.linear.x
    current_twist.linear.y = twist.linear.y
    current_twist.linear.z = twist.linear.z  
    current_twist.angular.x = twist.angular.x
    current_twist.angular.y = twist.angular.y
    current_twist.angular.z = twist.angular.z

    current_twist_set = True

def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")
        print(abs(thetas[0]-driver_msg.destination[0]),
            abs(thetas[1]-driver_msg.destination[1]),
            abs(thetas[2]-driver_msg.destination[2]),
            abs(thetas[3]-driver_msg.destination[3]),
            abs(thetas[4]-driver_msg.destination[4]),
            abs(thetas[5]-driver_msg.destination[5]))
        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def move_cart(pub_cmd, loop_rate, dest_twist):

    global twist
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = Twist()
    driver_msg.linear.x = dest_twist.linear.x
    driver_msg.linear.y = dest_twist.linear.y
    driver_msg.linear.z = dest_twist.linear.z
    driver_msg.angular.x = dest_twist.angular.x
    driver_msg.angular.y = dest_twist.angular.y
    driver_msg.angular.z = dest_twist.angular.z
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while at_goal == 0:

        if( abs(twist.linear.x-driver_msg.linear.x) < 0.05 and \
            abs(twist.linear.y-driver_msg.linear.y) < 0.05 and \
            abs(twist.linear.z-driver_msg.linear.z) < 0.05 and \
            abs(twist.angular.x-driver_msg.angular.x) < 0.05 and \
            abs(twist.angular.y-driver_msg.angular.y) < 0.05 and \
            abs(twist.angular.z-driver_msg.angular.z) < 0.05 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    global digital_in_0

    # calculate joint angles 
    Q_start_default = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], 0.1, 0)
    Q_start = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], 0.03, 0)
    Q_end_default = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], 0.1, 0)
    Q_end = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], 0.03, 0)

    # move arm to start location
    print("move to default")
    move_arm(pub_cmd, loop_rate, Q_start_default, vel, accel)
    print("move to default")
    move_arm(pub_cmd, loop_rate, Q_start, vel, accel)
    
    # turn on gripper
    rospy.loginfo("Gripper on")
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(0.5)

    # Check if gripping
    if digital_in_0 == 0:
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        move_arm(pub_cmd, loop_rate, Q_start_default, vel, accel)
        rospy.loginfo("GRIPPER AINT GRIPPIN")
        return error

    # move arm to end location
    move_arm(pub_cmd, loop_rate, Q_start_default, vel, accel)
    move_arm(pub_cmd, loop_rate, Q_end_default, vel, accel)
    move_arm(pub_cmd, loop_rate, Q_end, vel, accel)
    
    # turn off gripper
    rospy.loginfo("Gripper off")
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(0.5)

    # move arm to end location
    move_arm(pub_cmd, loop_rate, Q_end_default, vel, accel)
    error = 0

    # ========================= Student's code ends here ===========================

    return error


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    
    # Intialize subscriber to ur3/gripper and callback function
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)
    
    # Initialize publisher for /cmd_vel
    pub_twist = rospy.Publisher('cart_controller/cmd_vel', Twist, queue_size=1)
    
    # Initialize subscribe to /cmd_vel
    sub_twist = rospy.Subscriber('cart_controller/cmd_vel', Twist, twist_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    dest_twist = Twist()
    dest_twist.linear.x = 0.5
    # dest_twist.angular.z = 0.1
    
    
    # while True:
    #     pub_twist.publish(dest_twist)
    # start_time = time.time()
    # while abs(start_time - time.time()) < 10:
    #     # print(start_time - time.time())
    #     move_cart(pub_twist, loop_rate, dest_twist)
    
    # for _i in range(SPIN_RATE*5):
    #     loop_rate.sleep() 
    
    move_block(pub_command, loop_rate, [0.30582680585318511, -0.16385970163279578, -0.1], [0.19886217857288158, -0.10150232111239771, -0.1], 4.0, 4.0)

    # val = 0
    # while True:
    #     while val > -np.pi:
    #         thetas[1] = val
    #         move_arm(pub_command, loop_rate, thetas, 4.0, 4.0)
    #         print(thetas)
    #         val -= 0.1
    #     while val < 0.:
    #         thetas[1] = val
    #         move_arm(pub_command, loop_rate, thetas, 4.0, 4.0)
    #         print(thetas)
    #         val += 0.1


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
