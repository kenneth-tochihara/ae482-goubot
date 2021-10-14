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
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)


def gripper_callback(msg):
    """Whenever ur3/gripper_input publishes info this callback function is called for the gripper.

    Args:
        msg (msg): Message published by ur3/gripper_input.
    """

    global digital_in_0
    global analog_in_0
    global analog_in_1
    global current_gripper_set

    digital_in_0 = msg.DIGIN
    analog_in_0 = msg.AIN0
    analog_in_1 = msg.AIN1

    current_gripper_set = True


def position_callback(msg):
    """Whenever ur3/position publishes info, this callback function is called for the position.

    Args:
        msg (msg): Message published by ur3/position.
    """

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


def gripper(pub_cmd, loop_rate, io_0):
    """[summary]

    Args:
        pub_cmd ([type]): [description]
        loop_rate ([type]): [description]
        io_0 (bool): Sets state of gripper

    Returns:
        int: Verifies success of gripper
    """

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

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):
    """[summary]

    Args:
        pub_cmd (Publisher): Object to callback instantaneous location of roobot
        loop_rate (Rate): Rate in which data is ingested
        dest (list): A list of angles of theta (theta1 - theta6) for all the joints
        vel (float): Velocity of each joint angle in deg/s
        accel (float): acceleration of each joint angle in deg/s^2

    Returns:
        int: Verifies success of move
    """

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

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    # Initialize subscriber to ur3/position and callback fuction each time data is published
    sub_gripper = rospy.Subscriber('ur3/gripper', gripper_input, gripper_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")
    rospy.loginfo("Sending Goals ...")
    loop_rate = rospy.Rate(SPIN_RATE)

    # move arm to home
    move_arm(pub_command, loop_rate, home, 4.0, 4.0)
    joint_angles = home
    
    # loop through joints
    for idx in range(6):
        
        # log index of joint
        rospy.loginfo("Sending offsets for joint " + str(idx + 1) +  "...")
        
        # offset joint by 30 degrees
        joint_angles[idx] += np.radians(30)
        move_arm(pub_command, loop_rate, joint_angles, 4.0, 4.0)
        
        # return joint back to original location
        joint_angles[idx] -= np.radians(30)
        move_arm(pub_command, loop_rate, joint_angles, 4.0, 4.0)

   
    # send suction_on command to gripper
    rospy.loginfo("Sending suction for gripper...")
    gripper(pub_command, loop_rate, suction_on)
    
    # verify that the gripper did not grip but suction is on
    if (digital_in_0 == 0) and current_io_0: error = 0
    else: error = 1
    
    # send suction_off command to gripper
    gripper(pub_command, loop_rate, suction_off)
    
    # verify that the gripper suction is off
    if not current_io_0: error = 0
    else: error = 1
    
    # print test results
    if error == 0: rospy.loginfo("Success!")
    else: rospy.loginfo("Failed!")


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
