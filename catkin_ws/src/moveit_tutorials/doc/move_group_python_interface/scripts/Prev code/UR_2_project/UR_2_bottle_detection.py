        
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
# Right click the cam_ur/color/image_raw topic and click publish
# Hit green play button at top of window to play (this will start publishing this topic)

# Open new terminal and navigate to folder this script is located in and run

#!/usr/bin/env python
from __future__ import print_function

import roslib
#  roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
# from tkinter import *
# from tk import *
import numpy as np
import time
import math


global canny, center, dist, d_p, l_h, u_h, l_s, u_s, l_v, u_v, bridge, circles

# Start with red
center=10
dist=10
d_p=2
canny=200

# current values 5/20

# Red
l_h = 0
u_h = 255
l_s = 195
u_s = 236 
l_v = 100
u_v = 233

# Blue
# l_h = 65
# u_h = 170
# l_s = 130
# u_s = 255 
# l_v = 20
# u_v = 235

# Green
# l_h = 50
# u_h = 70
# l_s = 60
# u_s = 125 
# l_v = 100
# u_v = 150


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
	print(circles)
        for i in circles[0,:]:
            # draw circle
            cv2.circle(disp_image_HSV,(4*i[0],4*i[1]),4*i[2],(0,255,0),2)
            # cv2.circle(disp_image_HSV,(4*i[0]-4*offset,4*i[1]),2,(0,0,255),3) 
            # cv2.circle(disp_image_HSV,(4*alignmentX,4*alignmentY),2,(255,0,0),3)               
            # print("Object Radius (pxls): ",i[2],"Object Offset (pxls): ",math.ceil(1.1*i[2]))
            
            #correct resizing (from original image)
            screenCoordx = 4 * i[0]
            screenCoordy = 4 * i[1] 
            
            disp_image_HSV[screenCoordx, screenCoordy, :] = (0, 255, 0)

            #conver pixels to meters
            conversion = 0.015/12 # radius 6 original pixels = 24 pixels = 3 cm/2 = 1.5 cm
            xm = screenCoordx*conversion
            ym = screenCoordy*conversion
            

            #convert screen coordinates (m) to UR coordinates (m)
            x_offset = 0.31-0.01
            y_offset = 0.435+0.02


            #convert camera cartesian to measurements for (in METERS)
            ###YOOOO THESE COORDS ARE IN METERS 
            URCoordx = xm+x_offset
            URCoordy = -ym + y_offset
            URCoordz = 0.170
            URCoordArr = [URCoordx, URCoordy, URCoordz]
            print(URCoordArr)

            coordinates_pub = rospy.Publisher('bottle_coordinates', Float32MultiArray, queue_size = 10)
            coord_list = Float32MultiArray()
            coord_list.data = URCoordArr
            coordinates_pub.publish(coord_list)





    cv2.imshow("Hough Circle Detection", disp_image_HSV)
    cv2.waitKey(3)

def talker():
    coordinates_pub = rospy.Publisher('bottle_coordinates', String, queue_size = 10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
		hello_str = "hello world" 
		rospy.loginfo(hello_str) 
		coordinates_pub.pub("hello")
		rate.sleep()


def listener():

    image_sub = rospy.Subscriber("/cam_ur/color/image_raw",Image, callback) # camera topic name adjusted

    # depth_sub = rospy.Subscriber("/camera/infra1/image_rect_raw",Image, callback)
    image_pub = rospy.Publisher("image_topic_2",Image)	
  #   coordinates_pub = rospy.Publisher('bottle_coordinates', String, queue_size = 10)
  #   rospy.init_node('talker', anonymous=True)
  #   rate = rospy.Rate(10)

  #   while not rospy.is_shutdown():
		# hello_str = "hello world" 
		# rospy.loginfo(hello_str) 
		# coordinates_pub.pub("hello")
		# rate.sleep()


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
    # try:
    # 	talker()
    # except rospy.ROSInterruptException:
    # 	pass


