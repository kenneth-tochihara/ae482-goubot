#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab2_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

PI=np.pi

def build_S(omega, q):
	v = -np.cross(omega, q)
	return np.array([[0., -omega[2], omega[1], v[0]], 
				     [omega[2], 0., -omega[0], v[1]], 
				     [-omega[1], omega[0], 0., v[2]],
				     [0., 0., 0., 0.]])
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.array([[0., -1., 0., 0.390], 
				  [0., 0., -1., 0.401], 
				  [1., 0., 0., 0.2155], 
				  [0., 0., 0., 1.]])

	omega1 = np.array([0., 0., 1.])
	q1 = np.array([-0.15, 0.15, 0.162])
	S1 = build_S(omega1, q1)

	omega2 = np.array([0., 1., 0.])
	q2 = np.array([-0.15, 0.27, 0.162])
	S2 = build_S(omega2, q2)

	omega3 = np.array([0., 1., 0.])
	q3 = np.array([0.094, 0.27, 0.162])
	S3 = build_S(omega3, q3)

	omega4 = np.array([0., 1., 0.])
	q4 = np.array([0.307, 0.177, 0.162])
	S4 = build_S(omega4, q4)

	omega5 = np.array([1., 0., 0.])
	q5 = np.array([0.307, 0.26, 0.162])
	S5 = build_S(omega5, q5)

	omega6 = np.array([0., 1., 0.])
	q6 = np.array([0.39, 0.26, 0.162])
	S6 = build_S(omega6, q6)

	S = [S1, S2, S3, S4, S5, S6]

	return M, S
"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Forward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	T02 = np.matmul(expm(S[0]*theta1), expm(S[1]*theta2))
	T03 = np.matmul(T02, expm(S[2]*theta3))
	T04 = np.matmul(T03, expm(S[3]*theta4))
	T05 = np.matmul(T04, expm(S[4]*theta5))
	T06 = np.matmul(T05, expm(S[5]*theta6))
	T = np.matmul(T06, M)

	# ==============================================================#

	print(str(T) + "\n")



	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	# define joint lengths
	L1 = 0.152
	L2 = 0.120
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.083
	L7 = 0.083
	L8 = 0.082
	L9 = 0.0535
	L10 = 0.059

	# convert from world frame to a frame centered at robot base\
	xgrip = xWgrip + 0.15
	ygrip = yWgrip - 0.15 
	zgrip = zWgrip - 0.01

	# convert yaw to radians
	yaw_rad = yaw_WgripDegree * (np.pi/180)

	# Question 1
	xcen = xgrip - L9*np.cos(yaw_rad)
	ycen = ygrip - L9*np.sin(yaw_rad)
	zcen = zgrip

	# Question 2
	hy = np.sqrt(xcen**2 + ycen**2)
	theta1 = np.arctan2(ycen, xcen) - np.arcsin((L6+0.027)/ hy)

	# Question 3
	theta6 = theta1 + np.pi/2 - yaw_rad

	# Question 4
	x3_end = -L7*np.cos(theta1) + (L6+0.027)*np.sin(theta1) + xcen
	y3_end = -L7*np.sin(theta1) - (L6+0.027)*np.cos(theta1) + ycen
	z3_end = zcen + L8 + L10

	T_fc = np.array([[np.cos(theta1), -np.sin(theta1), 0., xcen], [np.sin(theta1), np.cos(theta1), 0., ycen], [0., 0., 1., 0.], [0., 0., 0., 1.]])
	p_c = np.array([[-L7], [-(L6 + 0.027)], [0], [1.]])
	p_f = np.matmul(T_fc, p_c)
	
	x3_end = p_f[0][0]
	y3_end = p_f[1][0]
	z3_end = zcen + L10 + L8
	
	p_3end = np.array([[x3_end], [y3_end], [z3_end]])

	# Question 5
	d = np.sqrt(x3_end**2 + y3_end**2)
	c = np.sqrt((z3_end - L1)**2 + d**2)

	thetax = np.arctan2(z3_end - L1, d)
	thetay = np.arccos((L3**2 + c**2 - L5**2)/(2*L3*c))
	theta2 = -(thetax + thetay)

	thetac = np.arccos((L3**2 + L5**2 - c**2)/(2*L3*L5))
	theta3 = np.pi - thetac

	theta4 = -theta2 - theta3

	theta5 = -np.pi/2
	
	vals = lab_fk(float(theta1), float(theta2), float(theta3), float(theta4), float(theta5), float(theta6))

	return vals

	# ==============================================================#
	pass 