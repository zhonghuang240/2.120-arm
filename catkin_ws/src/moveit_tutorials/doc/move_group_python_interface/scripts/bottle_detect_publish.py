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

#!/usr/bin/env python3
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from tkinter import *
#from tk import *
import numpy as np
import time
import math

import speech_recognition as sr
import pyttsx3 

global canny, center, dist, d_p, l_h, u_h, l_s, u_s, l_v, u_v, bridge, UR_centers_f

UR_centers_f = []
# Initialize the recognizer 
r = sr.Recognizer() 
  
colors=['red', 'orange', 'green', 'blue']

# **** Uncomment Here for Voice Recognition ***

# Loop infinitely for user to speak
# while(1):    
#     try:  
#         # use the microphone as source for input.
#         with sr.Microphone() as source2:
#             # wait for a second to let the recognizer
#             # adjust the energy threshold based on
#             # the surrounding noise level 
#             r.adjust_for_ambient_noise(source2, duration=0.2)
              
#             print("listening...")
#             #listens for the user's input 
#             audio2 = r.listen(source2)
              
#             # Using ggogle to recognize audio
#             MyText = r.recognize_google(audio2)
#             MyText = MyText.lower()
#             print(MyText)

#             if MyText in colors:
#                 break
              
#     except sr.RequestError as e:
#         print("Could not request results; {0}".format(e))
          
#     except sr.UnknownValueError:
#         print("unknown error occured")

# print("out: "+ MyText)

# red_target = False
# orange_target = False
# green_target = False
# blue_target = False

# if MyText=="red":
#     red_target = True
# elif MyText=="orange":
#     orange_target = True
# elif MyText=="green":
#     green_target = True
# elif MyText=="blue":
#     blue_target = True

# print(red_target, orange_target, green_target, blue_target)
# *** voice recognition ends here ***

red_target = False
orange_target = False
green_target = True
blue_target = False

# Start with red
center=10 #1
dist=50 #1000
d_p=1.5 #4
canny=80 #250


# Red 1
l_h_red = 0		#0
u_h_red = 15 	#255
l_s_red = 70	#195
u_s_red = 255	#236
l_v_red = 50	#146
u_v_red = 255	#233

# Red 2
l_h_red2 = 160		#0
u_h_red2 = 180 	#255
l_s_red2 = 70	#195
u_s_red2 = 255	#236
l_v_red2 = 50	#146
u_v_red2 = 255	#233

# orange
l_h_orange = 20
u_h_orange = 40
l_s_orange = 150
u_s_orange = 236
l_v_orange = 150
u_v_orange = 230

# green
l_h_green = 40
u_h_green = 70
l_s_green = 50
u_s_green = 236
l_v_green = 50
u_v_green = 230

# blue
l_h_blue = 80
u_h_blue = 120
l_s_blue = 50
u_s_blue = 255
l_v_blue = 50
u_v_blue = 250


def pic_coord_to_UR_coord(row_pixel, col_pixel, centers):
    # Function takes in the centers for the detected bottle caps and converts the picture coordinate
    # to the UR's coordinate frame

    pixel_to_meter = 0.275/(row_pixel/2)
    pic_center_x_adjustment = col_pixel/2
    pic_center_y_adjustment = row_pixel/2
    UR_x_adjustment = 0.31 + 0.47 -0.0854 # UR robot located 33 cm to the left of the desk and camera is 47 cm from the edge of the desk
    UR_y_adjustment = 0.11 + 0.013 # UR robot is 4 cm closer to the back of the desk

    UR_centers = []
    position = 0
    for coord in centers:
        if position%2 == 0:
            camera_x_pixel = coord - pic_center_x_adjustment
            camera_x_meter = camera_x_pixel*pixel_to_meter
            UR_x_meter = camera_x_meter + UR_x_adjustment
            UR_centers.append(UR_x_meter)
        else:
            camera_y_pixel = (coord - pic_center_y_adjustment)*-1
            camera_y_meter = camera_y_pixel*pixel_to_meter
            UR_y_meter = camera_y_meter + UR_y_adjustment
            UR_centers.append(UR_y_meter)
            UR_centers.append(0.17)
            UR_centers.append(0)
            UR_centers.append(0.7071)
            UR_centers.append(0)
            UR_centers.append(0.7071)
        position += 1

    # rospy.loginfo(UR_centers)
    return UR_centers


bridge = CvBridge()

def callback(data):

    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    # rospy.loginfo([rows,cols])
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

    # visualize it in a cv window
    alignmentX=cols #The alignment dot's location, scaled for the smaller image
    alignmentY=rows #The alignment dot's location, scaled for the smaller image
    cv2.circle(cv_image,(alignmentX,alignmentY),2,(255,0,0),3)
    cv2.imshow("Original_Image", cv_image)
    cv2.waitKey(3)

    ################ HSV THRESHOLDING ####################
    # convert to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # get threshold values
    # lower_bound_HSV = np.array([l_h.get(), l_s.get(), l_v.get()])
    # upper_bound_HSV = np.array([u_h.get(), u_s.get(), u_v.get()])
    lower_bound_HSV_red = np.array([l_h_red, l_s_red, l_v_red])
    upper_bound_HSV_red = np.array([u_h_red, u_s_red, u_v_red])
    
    lower_bound_HSV_red2 = np.array([l_h_red2, l_s_red2, l_v_red2])
    upper_bound_HSV_red2 = np.array([u_h_red2, u_s_red2, u_v_red2])

    lower_bound_HSV_orange = np.array([l_h_orange, l_s_orange, l_v_orange])
    upper_bound_HSV_orange = np.array([u_h_orange, u_s_orange, u_v_orange])

    lower_bound_HSV_green = np.array([l_h_green, l_s_green, l_v_green])
    upper_bound_HSV_green = np.array([u_h_green, u_s_green, u_v_green])

    lower_bound_HSV_blue = np.array([l_h_blue, l_s_blue, l_v_blue])
    upper_bound_HSV_blue = np.array([u_h_blue, u_s_blue, u_v_blue])

    # threshold
    mask_HSV_red1 = cv2.inRange(hsv_image, lower_bound_HSV_red, upper_bound_HSV_red)
    mask_HSV_red2 = cv2.inRange(hsv_image, lower_bound_HSV_red2, upper_bound_HSV_red2)
    mask_HSV_red = mask_HSV_red1 | mask_HSV_red2
    mask_HSV_orange = cv2.inRange(hsv_image, lower_bound_HSV_orange, upper_bound_HSV_orange)
    mask_HSV_green = cv2.inRange(hsv_image, lower_bound_HSV_green, upper_bound_HSV_green)
    mask_HSV_blue = cv2.inRange(hsv_image, lower_bound_HSV_blue, upper_bound_HSV_blue)

    # get display image
    if red_target:
        targ_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV_red)
        obs_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV_orange+mask_HSV_green+mask_HSV_blue)
    elif orange_target:
    	targ_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV_orange)
    	obs_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV_red+mask_HSV_green+mask_HSV_blue)
    elif green_target:
    	targ_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV_green)
    	obs_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV_red+mask_HSV_orange+mask_HSV_blue)
    elif blue_target:
    	targ_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV_blue)
    	obs_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV_red+mask_HSV_orange+mask_HSV_green)
    else:
    	obs_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV_red+mask_HSV_orange+mask_HSV_green+mask_HSV_blue)

    #disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV_red+mask_HSV_orange+mask_HSV_green+mask_HSV_blue)
    cv2.imshow("HSV_Thresholding_Target", targ_disp_image_HSV)
    cv2.imshow("HSV_Thresholding_Obstacle", obs_disp_image_HSV)
    cv2.waitKey(3)
    time.sleep(0.05)

    ## Track circle
    # gray and blur
    obs_resizeIm = cv2.resize(obs_disp_image_HSV, (0,0), fx=0.25, fy=0.25)
    obs_blurIm = cv2.medianBlur(obs_resizeIm,5)
    obs_grayIm = cv2.cvtColor(obs_blurIm,cv2.COLOR_BGR2GRAY) # Convert to grascale image

    targ_resizeIm = cv2.resize(targ_disp_image_HSV, (0,0), fx=0.25, fy=0.25)
    targ_blurIm = cv2.medianBlur(targ_resizeIm,5)
    targ_grayIm = cv2.cvtColor(targ_blurIm,cv2.COLOR_BGR2GRAY)

    # hough
    #TODO: Play with these values
    arg1 = canny#.get()     #Upper threshold for Canny edge detection
    arg2 = center#.get()       #Threshold for center detection.
    min_distance = dist#.get()    # Minimum distance between new circle centers.
    dp = d_p#.get()             # How accepting to degradation of circles are you willing to be

    obs_circles = cv2.HoughCircles(obs_grayIm,cv2.HOUGH_GRADIENT,dp,min_distance,param1=arg1,param2=arg2, minRadius=0,maxRadius=60)
    targ_circles = cv2.HoughCircles(targ_grayIm,cv2.HOUGH_GRADIENT,dp,min_distance,param1=arg1,param2=arg2, minRadius=0,maxRadius=60)

    offset=0
    centers = []
    if targ_circles is not None:
        targ_circles = np.uint16(np.around(targ_circles,0))
        for i in targ_circles[0,:]:
            # draw circle
            cv2.circle(targ_disp_image_HSV,(4*i[0],4*i[1]),4*i[2],(0,255,0),2)
            cv2.circle(targ_disp_image_HSV,(4*i[0]-4*offset,4*i[1]),2,(255,255,255),3)
            centers = [4*i[0],4*i[1]]
            # cv2.circle(disp_image_HSV,(4*alignmentX,4*alignmentY),2,(255,0,0),3)
            # print("Object Radius (pxls): ",i[2],"Object Offset (pxls): ",math.ceil(1.1*i[2]))

    if obs_circles is not None:
        obs_circles = np.uint16(np.around(obs_circles,0))
        for i in obs_circles[0,:]:
            # draw circle
            cv2.circle(obs_disp_image_HSV,(4*i[0],4*i[1]),4*i[2],(0,255,0),2)
            cv2.circle(obs_disp_image_HSV,(4*i[0]-4*offset,4*i[1]),2,(255,255,255),3)
            centers.append(4*i[0])
            centers.append(4*i[1])
            # cv2.circle(disp_image_HSV,(4*alignmentX,4*alignmentY),2,(255,0,0),3)
            # print("Object Radius (pxls): ",i[2],"Object Offset (pxls): ",math.ceil(1.1*i[2]))

    UR_pose = Pose()
    if len(centers) > 0:
    	# rospy.loginfo(centers)
    	global UR_centers_f
    	UR_centers_f = pic_coord_to_UR_coord(rows, cols, centers)
        UR_pose.position.x = UR_centers_f[0]
        UR_pose.position.y = UR_centers_f[1]
    	# stop_spin =  True

    publisher(UR_pose)

    cv2.imshow("Hough Circle Detection Obstacles", obs_disp_image_HSV)
    cv2.imshow("Hough Circle Detection Target", targ_disp_image_HSV)
    cv2.waitKey(3)

def listener():

    image_sub = rospy.Subscriber("/cam_ur/color/image_raw",Image, callback)

    # depth_sub = rospy.Subscriber("/camera/infra1/image_rect_raw",Image, callback)
    image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
    # rospy.loginfo(Image)

def publisher(ur_coords):
    coord_pub = rospy.Publisher("ur_coords", Pose, queue_size=10)
    coord_pub.publish(ur_coords)

    rospy.sleep(0.1)

def main(args):

    rospy.init_node('image_converter', anonymous=True)

    listener()


    try:
      # if stop_spin = False:
      	# rospy.spin()
      rospy.spin()
      rospy.loginfo("this is main")
      rospy.loginfo(UR_centers_f)
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
