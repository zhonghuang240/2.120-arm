#!/usr/bin/python

# 2.12 Lab 9
# Luke Roberto Oct 2017
# Jacob Guggenheim March 2019  
# Jerry Ng March 2019, 2020
# Phillip Daniel April 2021

import numpy as np
import cv2  # OpenCV module
import time
import math
from tkinter import *

global tk
tk = Tk()
global canny, center, dist, d_p
canny = Scale(tk, from_ = 1, to = 1500, label = 'Upper threshold', orient = HORIZONTAL)
canny.pack()
canny.set(250)
center = Scale(tk, from_ = 1, to = 1000, label = 'Center Threshold', orient = HORIZONTAL)
center.pack()
center.set(1)
dist = Scale(tk, from_ = 1, to = 1000, label = 'Min Centers dist', orient = HORIZONTAL)
dist.pack()
dist.set(1000)
d_p = Scale(tk, from_ = 0.1, to = 5, label = 'Accumulator Ratio', orient = HORIZONTAL, resolution = 0.01)
d_p.pack()
d_p.set(4)
def main():
    # convert ROS image to opencv format
    cap = cv2.VideoCapture(0)

    while True:

        tk.update()
        # Read from the webcam, frame by frame
        ret, cv_image = cap.read()

        # visualize it in a cv window
        cv2.imshow("Original_Image", cv_image)
        cv2.waitKey(3)

        # gray and blur
        resizeIm = cv2.resize(cv_image, (0,0), fx=0.25, fy=0.25)
        blurIm = cv2.medianBlur(resizeIm,5)
        grayIm = cv2.cvtColor(blurIm,cv2.COLOR_BGR2GRAY) # Convert to grascale image

        # hough
        #TODO: Play with these values
        arg1 = canny.get()     #Upper threshold for Canny edge detection
        arg2 = center.get()       #Threshold for center detection. 
        min_distance = dist.get()    # Minimum distance between new circle centers. 
        dp = d_p.get()             # How accepting to degradation of circles are you willing to be
        circles = cv2.HoughCircles(grayIm,cv2.HOUGH_GRADIENT,dp,min_distance,param1=arg1,param2=arg2, minRadius=11,maxRadius=15)
        offset=15
        if circles is not None:
            circles = np.uint16(np.around(circles,0))
            for i in circles[0,:]:
                # draw circle
                cv2.circle(resizeIm,(i[0],i[1]),i[2],(0,255,0),2)
                cv2.circle(resizeIm,(i[0]-offset,i[1]),2,(0,0,255),3)                
                cv2.circle(resizeIm,(100,62),2,(255,0,0),3)
                print("Object Radius (pxls): ",i[2],"Object Offset (pxls): ",math.ceil(1.1*i[2]))

        cv2.imshow("Hough Circle Detection", resizeIm)
        cv2.waitKey(3)

        

if __name__=='__main__':
    main()
