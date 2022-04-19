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
import geometry_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import math
import threading
#from tkinter import *
#from tk import *

global canny, center, dist, d_p, l_h, u_h, l_s, u_s, l_v, u_v, bridge, rgb_l, rgb_u
rgb_l = np.array([0,0,0])
rgb_u = np.array([255,255,255])

#colors = ["red", "green", "blue", "orange"]
desired_color = "blue"

if desired_color == "red":
    l_h = 0
    u_h = 105 #255
    l_s = 170 #195
    u_s = 255 #236
    l_v = 126 #146
    u_v = 255 #233
    center=9 #12
    dist=1000
    d_p=1 #2
    canny=250
    rgb_u[0] = 100
    rgb_u[1] = 100
elif desired_color == "green":
    l_h = 41
    u_h = 83
    l_s = 34
    u_s = 165 
    l_v = 0
    u_v = 200
    center=10+2
    dist=1000
    d_p=4-2
    canny=150+100
elif desired_color == "blue":
    l_h = 79
    u_h = 120
    l_s = 74 #94
    u_s = 255
    l_v = 99
    u_v = 220 #200
    center=8 #10
    dist=1000
    d_p=1 #2
    canny=150+100
elif desired_color == "orange":
    l_h = 5
    u_h = 35
    l_s = 60
    u_s = 245
    l_v = 135
    u_v = 188
    center=20-10
    dist=1000
    d_p=2
    canny=150+100
    rgb_l[1] = 50
else:
    raise ValueError("Must enter valid color")

bridge = CvBridge()

def callback(data):

    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

    # visualize it in a cv window
    alignmentX=100 #The alignment dot's location, scaled for the smaller image
    alignmentY=62 #The alignment dot's location, scaled for the smaller image
    cv2.circle(cv_image,(alignmentX*4,alignmentY*4),2,(255,0,0),3)
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
    mask_RGB = cv2.inRange(cv_image, rgb_l, rgb_u)
    # threshold
    mask_HSV = cv2.inRange(hsv_image, lower_bound_HSV, upper_bound_HSV)

    # get display image
    disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_RGB)
    disp_image_HSV = cv2.bitwise_and(disp_image_HSV,disp_image_HSV, mask= mask_HSV)
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
    circles = cv2.HoughCircles(grayIm,cv2.HOUGH_GRADIENT,dp,min_distance,param1=arg1,param2=arg2, minRadius=0,maxRadius=12)
    offset=10
    if circles is not None:
        circles = np.uint16(np.around(circles,0))
        for i in circles[0,:]:
            # draw circle
            cv2.circle(disp_image_HSV,(4*i[0],4*i[1]),4*i[2],(0,255,0),2)
            # cv2.circle(disp_image_HSV,(4*i[0]-4*offset,4*i[1]),2,(0,0,255),3) 
            # cv2.circle(disp_image_HSV,(4*alignmentX,4*alignmentY),2,(255,0,0),3)               
            print("Object Radius (pxls): ",i[2],"Object Offset (pxls): ",math.ceil(1.1*i[2]))
            print("(x, y) (in pixels)", i[0], i[1])
            print("(x,y) (in meters)", i[0]*0.015/i[2], i[1]*0.015/i[2])
            positionPublisher([i[0]*0.015/4, i[1]*0.015/4]) # i[2] is radius, hardcoded to 4 pixels
             
    cv2.imshow("Hough Circle Detection", disp_image_HSV)
    cv2.waitKey(3)

def listener():

    image_sub = rospy.Subscriber("/cam_ur/color/image_raw",Image, callback)

    # depth_sub = rospy.Subscriber("/camera/infra1/image_rect_raw",Image, callback)
    image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)


def positionPublisher(position):

    pub = rospy.Publisher('bottlePos', geometry_msgs.msg.Pose)

    # TODO Convert pixel position to real world position


    pos = geometry_msgs.msg.Pose()
    pos.position.x = position[0]
    pos.position.y = position[1]
    pos.orientation.x = 0
    pos.orientation.y = 0
    pos.orientation.z = 0
    pos.orientation.w = 0

    pub.publish(pos)
    print("Location Published")
    #rospy.sleep(0.1)

def main(args):
   
    rospy.init_node('image_converter', anonymous=False)
    #rospy.init_node('bottleInfo', anonymous = True)
    listener()

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
