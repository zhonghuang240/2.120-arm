#!/usr/bin/env python
 
# 4/22/21
# INSTRUCTIONS

# Install this package: https://github.com/IntelRealSense/realsense-ros
# Instructions for using ROS bag:http://wiki.ros.org/rqt_bag
#
# Quick instructions:
# You must do this in your VM (which already has ROS melodic installed)
# In a terminal run following lines: 
#   source /opt/ros/melodic/setup.bash
#   rqt_bag 
# When rqt_bag interface pops up, open colored_bottles_image.bag
# Right click the camera/color/image_raw topic and click publish
# Hit green play button at top of window to play (this will start publishing this topic)

# Open new terminal and navigate to folder this script is located in and run

from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from tkinter import *
# from tk import *
import numpy as np
import time
import math


global canny, center, dist, d_p, l_h, u_h, l_s, u_s, l_v, u_v, bridge, pxl_to_dist
# Start with red
center=10 	#Threshold for center detection. 1
dist=1000	# Minimum distance between new circle centers. 1000 
d_p= 4		# How accepting to degradation of circles are you willing to be 4
canny=250 	#Upper threshold for Canny edge detection 250

bottle_lid_radius_pxl = 5*4
bottle_lid_radius_cm = 1.5
pxl_to_dist = bottle_lid_radius_cm/bottle_lid_radius_pxl

while True:
    color = raw_input("What color do you want to find?\n")
    if color == "red" or color == "Red" or color == "r":
    	# Red
    	l_h = 0 #0 
    	u_h = 30 #255
    	l_s = 120 # 195
	u_s = 255 #236
	l_v = 0 #146
	u_v = 200 #233
	d_p= 2 # red needs more acceptance of circle degredation
	break
    elif color == "blue" or color == "Blue" or color == "b":
	# Blue
	l_h = 90
	u_h = 150 #150 
	l_s = 80 #80
	u_s = 255 #255 
	l_v = 0 #115
	u_v = 255 #180
	d_p= 2
	break
    elif color == "yellow" or color == "Yellow" or color == "y":
	# Yellow
	l_h = 20
	u_h = 100
	l_s = 80
	u_s = 255 
	l_v = 115
	u_v = 180
	break
    elif color == "green" or color == "Green" or color == "g":
	# Green
	l_h = 50
	u_h = 150
	l_s = 60
	u_s = 180
	l_v = 100 
	d_p= 5
	u_v = 170
	d_p= 2
	break
    else:
	print("Not a color, silly, try again.")


bridge = CvBridge()

def callback(data):
    global location_pub

    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (rows/2,cols/2), 10, 255)

    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

    # visualize it in a cv window
    # alignmentX=100 #The alignment dot's location, scaled for the smaller image
    # alignmentY=62 #The alignment dot's location, scaled for the smaller image
    # cv2.circle(cv_image,(alignmentX*4,alignmentY*4),2,(255,0,0),3)
    cv2.imshow("Original_Image", cv_image)
    cv2.waitKey(3)

    ################ HSV THRESHOLDING ####################
    # convert to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # get threshold values
    # lower_bound_HSV = np.array([l_h.get(), l_s.get(), l_v.get()])
    # upper_bound_HSV = np.array([u_h.get(), u_s.get(), u_v.get()])
    lower_bound_HSV = np.array([l_h, l_s, l_v])
    upper_bound_HSV = np.array([u_h, u_s, u_v])

    # threshold
    mask_HSV = cv2.inRange(hsv_image, lower_bound_HSV, upper_bound_HSV)

    # get display image
    disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV)
    cv2.imshow("HSV_Thresholding", disp_image_HSV)
    cv2.waitKey(3)
    time.sleep(0.05)

    ## Track circle
    # gray and blur
    resizeIm = cv2.resize(disp_image_HSV, (0,0), fx=0.25, fy=0.25)
    blurIm = cv2.medianBlur(resizeIm,5)
    grayIm = cv2.cvtColor(blurIm,cv2.COLOR_BGR2GRAY) # Convert to grascale image

    # hough
    #TODO: Play with these values
    arg1 = canny#.get()     #Upper threshold for Canny edge detection
    arg2 = center#.get()       #Threshold for center detection. 
    min_distance = dist#.get()    # Minimum distance between new circle centers. 
    dp = d_p#.get()             # How accepting to degradation of circles are you willing to be
    circles = cv2.HoughCircles(grayIm,cv2.HOUGH_GRADIENT,dp,min_distance,param1=arg1,param2=arg2, minRadius=0,maxRadius=60)
    offset=10
    if circles is not None:
        circles = np.uint16(np.around(circles,0))
        for i in circles[0,:]:
            # draw circle
            cv2.circle(disp_image_HSV,(4*i[0],4*i[1]),4*i[2],(0,255,0),2)
    	    cv2.circle(disp_image_HSV,(4*i[0],4*i[1]), 2,(0,255,0),3)
            # cv2.circle(disp_image_HSV,(4*i[0]-4*offset,4*i[1]),2,(0,0,255),3) 
            # cv2.circle(disp_image_HSV,(4*alignmentX,4*alignmentY),2,(255,0,0),3)               
            # print("Object Radius (pxls): ",i[2],"Object Offset (pxls): ",math.ceil(1.1*i[2]))
    
    	    # Locating the bottles (x, y) position
            (rows,cols,channels) = disp_image_HSV.shape
            # print("rows ", rows, "cols ", cols)
            x_fromcenter = pxl_to_dist*((4*i[0])-(cols/2))
            y_fromcenter = pxl_to_dist*((rows/2)-(4*i[1]))
            #print("x = ", x_fromcenter, "y = ", y_fromcenter)
            cv2.circle(disp_image_HSV, (cols/2,rows/2), 2,(0,255,0),3)
     
    	    # Calculate x and y position from the UR5 coordinate system
            y_ur5 = (1.5*y_fromcenter) - 29.5 + 40.5 + 1.5
    	    x_ur5 = (1.5*x_fromcenter) + 47 + 31 - 8
    	    print("x = ", x_ur5, "y = ", y_ur5)

    	    # Publish x and y positions to bottle_location topic
    	    location_pub.publish(x_ur5)
    	    location_pub.publish(y_ur5)
        
    cv2.imshow("Hough Circle Detection", disp_image_HSV)
    cv2.waitKey(3)


def listener():
    global location_pub

    location_pub = rospy.Publisher("bottle_location", Float32, queue_size=10)
    image_sub = rospy.Subscriber("/cam_ur/color/image_raw",Image, callback)

    # depth_sub = rospy.Subscriber("/camera/infra1/image_rect_raw",Image, callback)
    # image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

def main(args):
   
    rospy.init_node('image_converter', anonymous=True)
    
    listener()

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


