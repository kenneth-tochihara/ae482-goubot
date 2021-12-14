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
import tf.transformations
import time
from math import pi
from lab2_header import *
from lab2_func import *
from blob_search import *
from rospy.client import spin
from gazebo_msgs.srv import DeleteModel, SpawnModel
# from kinematics import *
from cv_bridge import CvBridge

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of green and light blue blocks
xw_yw_G = []
xw_yw_B = []
xw_yw_W = []

# goals for the blocks
goal_B = [[0.20582680585318511, -0.16385970163279578, 0.], 
         [0.19886217857288158, -0.10150232111239771, 0.]]
goal_G = [[0.2684808585461177, -0.16575428914965623, 0.],
         [0.26338096736614386, -0.10081618208925211, 0.]]

# 20Hz
SPIN_RATE = 40

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])
zero_position = np.radians([180, 0, 0, 0, 0, 0])

cart_twist = Twist()
cart_pose = Pose()

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False
current_odom_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

# Robot current odometry
current_odom = Odometry()


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
Whenever cart_controller/odom publishes info this callback function is called.
"""
def odom_callback(msg):

    global cart_twist
    global cart_pose
    global current_odom
    global current_odom_set
    
    # twist copy
    cart_twist.linear.x = msg.twist.twist.linear.x
    cart_twist.linear.y = msg.twist.twist.linear.y
    cart_twist.linear.z = msg.twist.twist.linear.z  
    cart_twist.angular.x = msg.twist.twist.angular.x
    cart_twist.angular.y = msg.twist.twist.angular.y
    cart_twist.angular.z = msg.twist.twist.angular.z
        
    # pose copy
    cart_pose.position.x = msg.pose.pose.position.x
    cart_pose.position.y = msg.pose.pose.position.y
    cart_pose.position.z = msg.pose.pose.position.z  
    cart_pose.orientation.x = msg.pose.pose.orientation.x
    cart_pose.orientation.y = msg.pose.pose.orientation.y
    cart_pose.orientation.z = msg.pose.pose.orientation.z
    cart_pose.orientation.w = msg.pose.pose.orientation.w
    
    # twist save
    current_odom.twist.twist.linear.x = cart_twist.linear.x
    current_odom.twist.twist.linear.y = cart_twist.linear.y
    current_odom.twist.twist.linear.z = cart_twist.linear.z  
    current_odom.twist.twist.angular.x = cart_twist.angular.x
    current_odom.twist.twist.angular.y = cart_twist.angular.y
    current_odom.twist.twist.angular.z = cart_twist.angular.z
    
    # pose save
    current_odom.pose.pose.position.x = cart_pose.position.x
    current_odom.pose.pose.position.y = cart_pose.position.y
    current_odom.pose.pose.position.z = cart_pose.position.z  
    current_odom.pose.pose.orientation.x = cart_pose.orientation.x
    current_odom.pose.pose.orientation.y = cart_pose.orientation.y
    current_odom.pose.pose.orientation.z = cart_pose.orientation.z
    current_odom.pose.pose.orientation.w = cart_pose.orientation.w
    
    current_odom_set = True

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

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.001 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.001 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.001 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.001 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.001 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.001 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def move_cart(pub_cmd, loop_rate, dest_twist):

    global cart_twist
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

        if( abs(cart_twist.linear.x-driver_msg.linear.x) < 0.05 and \
            abs(cart_twist.linear.y-driver_msg.linear.y) < 0.05 and \
            abs(cart_twist.linear.z-driver_msg.linear.z) < 0.05 and \
            abs(cart_twist.angular.x-driver_msg.angular.x) < 0.05 and \
            abs(cart_twist.angular.y-driver_msg.angular.y) < 0.05 and \
            abs(cart_twist.angular.z-driver_msg.angular.z) < 0.05 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def move_cart_location(pub_cmd, loop_rate, target_xy, vel):
    
    global cart_pose
    global SPIN_RATE

    # calculate displacement of the xy coordinates
    displacement_xy = np.array([target_xy[0] - cart_pose.position.x, target_xy[1] - cart_pose.position.y])
    displacement_xy_uvec = displacement_xy / np.linalg.norm(displacement_xy)
    
    # publish desired velocity
    velocity = displacement_xy_uvec * vel
    dest_twist = Twist()
    dest_twist.linear.x = velocity[0]
    dest_twist.linear.y = velocity[1]
    move_cart(pub_cmd, loop_rate, dest_twist)
    
    # check if cart is at desired location
    spin_count = 0
    at_goal = 0
    while at_goal == 0:

        if( abs(target_xy[0]-cart_pose.position.x) < 0.05 and \
            abs(target_xy[1]-cart_pose.position.y) < 0.05):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):
            spin_count = 0

        spin_count = spin_count + 1
    
    # set velocity to zero
    dest_twist = Twist()
    move_cart(pub_cmd, loop_rate, dest_twist)

def pick_block(pub_cmd, loop_rate, target_xyz_world, vel, accel):

    """
    target_xyz_world: where to pick the block in world coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    global digital_in_0
    ur3_base_transform = ur3_base_T(cart_pose)

    # calculate joint angles 
    Q_target_default = lab_invk(target_xyz_world[0], target_xyz_world[1], BLOCK_HEIGHT + 0.2, ur3_base_transform, 0)
    Q_target = lab_invk(target_xyz_world[0], target_xyz_world[1], BLOCK_HEIGHT, ur3_base_transform, 0)

    # move arm to start location
    rospy.loginfo("Target location default")
    move_arm(pub_cmd, loop_rate, Q_target_default, vel, accel)
    time.sleep(3.0)
    rospy.loginfo("Target location location block")
    move_arm(pub_cmd, loop_rate, Q_target, vel, accel)
    
    # turn on gripper
    rospy.loginfo("Gripper on")
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(0.5)

    # Check if gripping
    if digital_in_0 == 0:
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        move_arm(pub_cmd, loop_rate, Q_target_default, vel, accel)
        rospy.loginfo("GRIPPER AINT GRIPPIN")
        return error

    # move arm to end location
    move_arm(pub_cmd, loop_rate, Q_target_default, vel, accel)
    error = 0

    # ========================= Student's code ends here ===========================

    return error

def place_block(pub_cmd, loop_rate, target_xyz_world, vel, accel):

    """
    target_xyz_world: where to place the block in world coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    global digital_in_0
    ur3_base_transform = ur3_base_T(cart_pose)

    # calculate joint angles 
    Q_target_default = lab_invk(target_xyz_world[0], target_xyz_world[1], BLOCK_HEIGHT + 0.2, ur3_base_transform, 0)
    Q_target = lab_invk(target_xyz_world[0], target_xyz_world[1], BLOCK_HEIGHT + 0.01, ur3_base_transform, 0)

    # move arm to start location
    rospy.loginfo("Target location default")
    move_arm(pub_cmd, loop_rate, Q_target_default, vel, accel)
    time.sleep(3.0)
    rospy.loginfo("Target location location block")
    move_arm(pub_cmd, loop_rate, Q_target, vel, accel)
    
    # turn off gripper
    rospy.loginfo("Gripper off")
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(0.5)

    # move arm to end location
    move_arm(pub_cmd, loop_rate, Q_target_default, vel, accel)
    error = 0

    # ========================= Student's code ends here ===========================

    return error

class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):

        global xw_yw_G # store found green blocks in this list
        global xw_yw_B # store found light blue blocks in this list
        global xw_yw_W # store found white blocks in this list

        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_G & xw_yw_Y respectively.

        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_G or xw_yw_Y.

        # Remember, xw_yw_G & xw_yw_Y are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

        xw_yw_W = blob_search(cv_image, "white")
        # blob_search(cv_image, "orange")


def main():

    global home
    global Q
    global SPIN_RATE
    global xw_yw_W

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    
    # Intialize subscriber to ur3/gripper and callback function
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)
    
    # Initialize publisher for cart_controller/cmd_vel
    pub_twist = rospy.Publisher('cart_controller/cmd_vel', Twist, queue_size=10)
    
    # Initialize subscribe to ur3/odom
    sub_twist = rospy.Subscriber('cart_controller/odom', Odometry, odom_callback)
    
    # Initialize spawn service for models
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    # rospy.wait_for_service("gazebo/delete_urdf_model")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    
    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")
    rospy.loginfo("Sending Goals ...")
    loop_rate = rospy.Rate(SPIN_RATE)
    move_arm(pub_command, loop_rate, go_away, 3.0, 3.0)

    # Initialize CV interface
    ic = ImageConverter(SPIN_RATE)  

    # Fetch block URDF file
    rospack = rospkg.RosPack()
    urdf_filepath = rospack.get_path('ur_description')
    block_urdf = open(os.path.join(urdf_filepath, 'urdf', 'block.urdf'), 'r').read()
    
    # Spawn block
    rospy.loginfo("Spawning block...")
    block_orient = Quaternion(x=0, y=0, z=0, w=1)
    block_location = block_spawn_location()
    block_pose = Pose(Point(x=block_location[0], y=block_location[1], z=0.0159), block_orient)
    spawn_model("block", block_urdf, "", block_pose, "world")
    time.sleep(5)
    
    # Check for interferences in motion path
    if ((abs(xw_yw_W[0][1]) < 0.3) and (xw_yw_W[0][0] < 0)):
    
        # Move around the location of the block but still arrive in offset by 0.5 in the x-direction
        move_cart_location(pub_twist, loop_rate, [0.0, xw_yw_W[0][1] + 0.8], 0.5)
        move_cart_location(pub_twist, loop_rate, [xw_yw_W[0][0] - 0.5, xw_yw_W[0][1] + 0.8], 0.5)
        move_cart_location(pub_twist, loop_rate, [xw_yw_W[0][0] - 0.5, xw_yw_W[0][1]], 0.5)
    
    else:
        
        # Move to location of block offset by 0.5 in the x-direction
        move_cart_location(pub_twist, loop_rate, [xw_yw_W[0][0] - 0.5, 0.0], 0.5)
        move_cart_location(pub_twist, loop_rate, [xw_yw_W[0][0] - 0.5, xw_yw_W[0][1]], 0.5)
    
    # Pick up block
    pick_block(pub_command, loop_rate, xw_yw_W[0], 3.0, 3.0)
    move_arm(pub_command, loop_rate, go_away, 3.0, 3.0)
    
    # Move back to origin
    move_cart_location(pub_twist, loop_rate, [0.0, 0.0], 1.0)
    
    # Place block back in front
    place_block(pub_command, loop_rate, [0.5, 0.0], 3.0, 3.0)
    move_arm(pub_command, loop_rate, go_away, 3.0, 3.0)
    
    # restart the program
    time.sleep(3)
    delete_model("block")
    time.sleep(1)
    
    return


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
