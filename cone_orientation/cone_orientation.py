from __future__ import print_function
from __future__ import division
from imutils.video import VideoStream
import cv2 as cv
import numpy as np
import argparse
import socket
import struct
import time
import sys
from math import atan2, cos, sin, sqrt, pi, degrees

def drawAxis(img, p_, q_, colour, scale):
    p = list(p_)
    q = list(q_)
    ## [visualization1]
    angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))

    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)

    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)

    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
    ## [visualization1]

def getOrientation(pts, img, area):
    ## [pca]
    # Construct a buffer used by the pca analysis
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i,0] = pts[i,0,0]
        data_pts[i,1] = pts[i,0,1]

    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)

    # Store the center of the object
    cntr = (int(mean[0,0]), int(mean[0,1]))
    ## [pca]

    ## [visualization]
    # Draw the principal components
    cv.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
    p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
    drawAxis(img, cntr, p1, (0, 255, 0), 1)
    drawAxis(img, cntr, p2, (255, 255, 0), 5)

    angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
    
    UDP_IP = "10.39.32.2"
    UDP_PORT = 5005
    
    values = (round(time.time() * 1000), cntr[0], cntr[1], degrees(angle), area)
    print(*values)
    
    packer = struct.Struct('d d d d d')
    packed_data = packer.pack(*values)
    
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UDP
    sock.sendto(packed_data, (UDP_IP, UDP_PORT))
    
    ## [visualization]

    return angle

## [pre-process]
vs = VideoStream(src=0).start()
print("Start up of cone_orientation")

while True:
    src = vs.read()
    # Check if image is loaded successfully
    if src is None:
        q('Could not open or find the image: ', args.input)
        exit(0)
    hsv = cv.cvtColor(src, cv.COLOR_BGR2HSV)
    
    lower = np.array([19,89,116])  #-- Lower range --
    upper = np.array([30,234,249])  #-- Upper range --
    
    # find the colors within the boundaries
    mask = cv.inRange(hsv, lower, upper)
    
    #define kernel size  
    kernel = np.ones((7,7),np.uint8)

    # Remove unnecessary noise from mask

    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    
    segmented_img = cv.bitwise_and(src, src, mask=mask)
    
    contours, hierarchy = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    output = cv.drawContours(segmented_img, contours, -1, (0, 0, 255), 3)
    
    gray = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
    
    output = cv.bitwise_not(gray)
    _, bw = cv.threshold(output, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    
    for i, c in enumerate(contours):
        # Calculate the area of each contour
        area = cv.contourArea(c)	
        # Ignore contours that are too small or too large
        if 1200 > area or area > 57000:
            continue
        # print('area - '+str(area))
        # Draw each contour only for visualisation purposes
        cv.drawContours(output, contours, i, (0, 0, 255), 2)
        # Find the orientation of each shape
        angle = getOrientation(c, output, area)
        # print(degrees(angle))
    if(len(sys.argv)) <= 1:
        cv.imshow('src', src)
        cv.imshow('output', output)
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
           break
    
