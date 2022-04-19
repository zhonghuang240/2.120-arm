#!/usr/bin/python

# 2.12 Lab 9
# Luke Roberto Oct 2017
# Jacob Guggenheim March 2019  
# Jerry Ng March 2019, 2020

import numpy as np
import cv2  # OpenCV module
import time
import math
from tkinter import *
import serial

serialComm = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)
print(serialComm.name)

def main():
    serialComm.close()

        

if __name__=='__main__':
    main()
