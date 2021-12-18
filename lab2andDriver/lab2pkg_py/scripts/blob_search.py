#!/usr/bin/env python

import cv2
import numpy as np
from math import atan2

# ========================= Student's code starts here =========================

# Params for camera calibration
# theta = 0.01637035800040541
# beta = 754.12171154586292
# tx = -0.052737794912171128
# ty = -0.34366522285472678
theta = 0.
beta = 170.
tx = 0.0
ty = 0.0

# arrays for calibration
beta_list = []
theta_list = []
tx_list = []
ty_list = []

# Function that converts image coord to world coord
# Note: input x corresponds to columns in the image, input y is rows in the image
def IMG2W(row,col):
    R = np.array([[np.cos(theta),-np.sin(theta),0],
                        [np.sin(theta),np.cos(theta),0],
                        [0,0,1]])

    # convert column pixels into world coordinates
    colw = (col - 320)/beta; yw = colw
    roww = (row - 240)/beta; xw = roww
    pw = np.matmul(R, np.vstack((xw,yw,0))) - np.vstack((tx,ty,0))
    return (pw[0][0],pw[1][0])
    

def calibration_frame(blob_image_center):

    global beta, theta

    # convert row value to X, column value to Y
    xc = np.round(blob_image_center[0][0])
    yc = np.round(blob_image_center[0][1])
    print("X: ", xc)
    print("Y: ", yc)
    print()

    # adjust origin to the middle of the screen
    xc -= 240
    yc -= 320
    print("X (adj): ", xc)
    print("Y (adj): ", yc)

    # define x1,y1 (leftmost) and x2,y2 (rightmost)
    if blob_image_center[0][0] < blob_image_center[1][0]:
        x1 = blob_image_center[0][0]
        y1 = blob_image_center[0][1]
        x2 = blob_image_center[1][0]
        y2 = blob_image_center[1][1]
    else:
        x1 = blob_image_center[1][0]
        y1 = blob_image_center[1][1]
        x2 = blob_image_center[0][0]
        y2 = blob_image_center[0][1]

    # check length of array
    if len(beta_list) < 100:
        # get the distance between two points to calibrate BETA
        dist = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        beta_list.append(dist/0.1)

        # calculate the angle of the plate wrt the grid
        theta_list.append(atan2(y2-y1,x2-x1))
    else:
        beta = np.average(beta_list)
        print("BETA: ", beta)

        theta = np.average(theta_list)
        print("THETA: ", theta)

def calibration_position(blob_image_center):
    
    global tx, ty, beta, theta

    # calculate rotation matrix
    R = np.array([[np.cos(theta),-np.sin(theta),0],
                [np.sin(theta),np.cos(theta),0],
                [0,0,1]])

    # calculate x and y in distance
    x1 = (blob_image_center[0][0] - 240)/beta
    y1 = (blob_image_center[0][1] - 320)/beta 

    # check length of array
    if len(tx_list) < 100:

        # real world conversion
        T = np.matmul(R, np.vstack((x1,y1,0)))
        tx_list.append(T[0][0]); ty_list.append(T[1][0])
        
    else:
        tx = np.average(tx_list)
        print("TX: ", tx)

        ty = np.average(ty_list)
        print("TY: ", ty)

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color='orange'):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True

    # block
    params.minArea = 10
    params.maxArea = 150

    # params.minArea = 625 - 150
    # params.maxArea = 625 + 150

    # orange blob
    # params.minArea = np.pi*(7.5**2)
    # params.maxArea = np.pi*(12.5**2)

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    # orange = (191,69,0) RGB
    # orange = (22/2,100,74.9) HSV

    # green = (23,140,37)
    # green = (127/2,83.6,54.9) HSV

    # pink = (230, 70, 50)
    # pink = (194/2, 78.3, 90.2) HSV

    # orange = (11,255,191)
    # green = (63.5,213.18,140)
    # pink = (97, 97.41, 170.1) HSV

    if color == 'lblue':
        lower = (87,50,50) 
        upper = (107,255,255) 
    elif color == 'green':
        lower = (53.5,50,50)
        upper = (73.5,255,255)
    elif color == 'orange':
        lower = (1,50,50)
        upper = (21,255,255)
    elif color == 'white':
        lower = (0,0,240)
        upper = (255,15,255)

    # lower = (110,50,50)     # blue lower
    # upper = (130,255,255)   # blue upper

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================
    
    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]),(0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    xw_yw = []

    if(num_blobs == 0):
        # print("No block found!")
        pass
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][1], blob_image_center[i][0]))

        # # calibration functions
        # calibration_frame(blob_image_center)
        # calibration_position(blob_image_center)

    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    if color == "white":
        cv2.namedWindow("Mask View")
        cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
