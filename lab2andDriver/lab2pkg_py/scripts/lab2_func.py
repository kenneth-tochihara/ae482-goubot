#!/usr/bin/env python
from math import atan2
import numpy as np
from scipy.linalg import expm
from lab2_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def screw_axis_matrix(w,v):
	w = np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
	S = np.concatenate((w,np.vstack(v)), axis=1)
	return np.vstack((S,[0,0,0,0]))

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	
	R06 = np.array([[0,-1,0],
					[0,0,-1],
					[1,0,0]],dtype=float) 
	p = np.vstack([0, 0, 0.05])/1000.0; q1 = np.hstack(p)
	p = p + (np.vstack([0,120,152])/1000.0); q2 = np.hstack(p)
	p = p + (np.vstack([244,0,0])/1000.0); q3 = np.hstack(p)
	p = p + (np.vstack([213,-93,0])/1000.0); q4 = np.hstack(p) 
	p = p + (np.vstack([0,83,0])/1000.0); q5 = np.hstack(p)
	p = p + (np.vstack([83,0,0])/1000.0);q6 = np.hstack(p)
	p = p + (np.vstack([0,59+82,53.5])/1000.0)

	M = np.concatenate((R06,p),axis=1)
	M = np.vstack((M,np.array([0,0,0,1])))

	w1 = np.array([0,0,1]); v1 = np.cross(-1*w1,q1); #print("v1,: ",v1)
	w2 = np.array([0,1,0]); v2 = np.cross(-1*w2,q2)
	w3 = np.array([0,1,0]); v3 = np.cross(-1*w3,q3)
	w4 = np.array([0,1,0]); v4 = np.cross(-1*w4,q4)
	w5 = np.array([1,0,0]); v5 = np.cross(-1*w5,q5)
	w6 = np.array([0,1,0]); v6 = np.cross(-1*w6,q6)

	S = [screw_axis_matrix(w1,v1),
		 screw_axis_matrix(w2,v2),
		 screw_axis_matrix(w3,v3),
		 screw_axis_matrix(w4,v4),
		 screw_axis_matrix(w5,v5),
		 screw_axis_matrix(w6,v6)]
	# ==============================================================#
	return M, S

"""
Function that converts the pose of the cart to the transformation matrix for
the base of the UR3
"""
def ur3_base_T(cart_pose):
    
    # initialize matrix
    T = [[0.0, 0.0, 0.0, 0.0], \
		 [0.0, 0.0, 0.0, 0.0], \
		 [0.0, 0.0, 0.0, 0.0], \
		 [0.0, 0.0, 0.0, 1.0]]
    
    # define quaternion values
    qxx = cart_pose.orientation.x**2
    qxy = cart_pose.orientation.x*cart_pose.orientation.y
    qxz = cart_pose.orientation.x*cart_pose.orientation.z
    qxw = cart_pose.orientation.x*cart_pose.orientation.w
    
    qyy = cart_pose.orientation.y**2
    qyz = cart_pose.orientation.y*cart_pose.orientation.z
    qyw = cart_pose.orientation.y*cart_pose.orientation.w
    
    qzz = cart_pose.orientation.z**2
    qzw = cart_pose.orientation.z*cart_pose.orientation.w
	
    # convert to rotation matrix
    T[0][0] = 1.0 - 2.0 * (qyy + qzz)
    T[0][1] = 2.0 * (qxy - qzw)
    T[0][2] = 2.0 * (qxz + qyw)
    
    T[1][0] = 2.0 * (qxy + qzw)
    T[1][1] = 1.0 - 2.0 * (qxx + qzz)
    T[1][2] = 2.0 * (qyz - qxw)
    
    T[2][0] = 2.0 * (qxz - qyw)
    T[2][1] = 2.0 * (qyz + qxw)
    T[2][2] = 1.0 - 2.0 * (qxx + qyy)
    
    # convert to offset UR3 base and the world 
    T[0][3] = cart_pose.position.x
    T[1][3] = cart_pose.position.y
    T[2][3] = cart_pose.position.z + 0.1
    
    return T

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated")

	# =================== Your code starts here ====================#

	thetas = np.array([theta1, theta2, theta3, theta4, theta5, theta6])

	[M,S] = Get_MS()
	T = np.eye(4)
	for i in range(6):
		T = np.matmul(T,expm(S[i]*thetas[i])) 
	T = np.matmul(T,M)
	print("T: ")
	print(T) 

	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

def lawcos(a,b,c):
	return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, T_ur3, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	r = 0.0535
	L1 = (152.0)/1000.0
	L3 = (244.0)/1000.0
	L5 = (213.0)/1000.0
	L6plus = (83.0+27.0)/1000.0
	L7 = 83.0/1000.0
	L8 = 82.0/1000.0
	L10 = 59.0/1000.0

	#Step 1
	p0grip = np.array([xWgrip - T_ur3[0][3], yWgrip - T_ur3[1][3], zWgrip - T_ur3[2][3]])
 	
	#Step 2
	pCen = np.array([0.0,0.0,0.0])
	pCen[0] = p0grip[0] - r * np.cos(np.radians(yaw_WgripDegree))
	pCen[1] = p0grip[1] - r * np.sin(np.radians(yaw_WgripDegree))
	pCen[2] = p0grip[2]

	#Step 3
	rCenxy = np.sqrt(pCen[0]**2 + pCen[1]**2)
	theta1big = atan2(pCen[1], pCen[0])
	theta1small = np.arcsin(L6plus/rCenxy)
	theta1 = theta1big - theta1small

	#Step 4
	theta6 = np.radians(90.0 - yaw_WgripDegree) + theta1

	#Step 5
	pInt = np.array([pCen[0] - L7 * np.cos(theta1), pCen[1] - L7 * np.sin(theta1), 0])
	p3end = np.array([pInt[0] + L6plus * np.sin(theta1), pInt[1] - L6plus * np.cos(theta1), pCen[2] + (L8 + L10)])

	#Step 6
	r3endxy = np.sqrt(p3end[0]**2 + p3end[1]**2)
	r3end = np.sqrt(r3endxy**2 + (p3end[2] - L1)**2)

	theta3 = PI - lawcos(L3, L5, r3end)

	theta2int1 = lawcos(L3, r3end, L5)
	theta2int2 = atan2(p3end[2]-L1, r3endxy)
	theta2 = -1 * (theta2int1 + theta2int2)

	theta4 = -1 * (PI - (-theta2 + (PI - theta3)))
	theta5 = -PI / 2

	# invoke forward kinematics
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)