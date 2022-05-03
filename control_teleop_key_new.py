#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from me212bot.msg import WheelCmdVel, autoNavi

import sys, select, tty, termios

MAX_LIN_VEL = 1 # m/s
MAX_ANG_VEL = 1 #rad/s

r=0.2294
b=0.035

isAuto = 0

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

msg = """
Teleoperation Control
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (~1 m/s)
a/d : increase/decrease angular velocity (~1 rad/s)

space key, s : force stop
---------------------------
Motor Modes:
0:hold
1:scoop up             2:scoop down  
3:open dump door(100)  4:close dump door(100)
5:scoop up slightly    6:scoop down slightly
7:close dump door(50)
---------------------------
Auto-navi Stages:
n:path 1
m:path 2
,:path 3
.:path 4
/:back to manual control

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel, mode):
    return "currently:\tlinear vel(m/s) %s\t angular vel(rad/s) %s\t mode: %s " % (target_linear_vel,target_angular_vel,mode)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    return constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)

def checkAngularLimitVelocity(vel):
    return constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_control')
    pub = rospy.Publisher('/cmdvel', WheelCmdVel, queue_size=10)
    pub_autoNavi = rospy.Publisher('/naviMode', autoNavi, queue_size=10)

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    target_mode = 0

    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    control_mode = 0

    try:
        print(msg)
        while(1):
            target_mode = 0
            navigation_mode = autoNavi()

            key = getKey()

            if key == '/' :
                isAuto = 0
                
                navigation_mode.isAuto = 0
                navigation_mode.path = 0
                navigation_mode.isWrong = 1

                target_linear_vel   = 0.0
                target_angular_vel  = 0.0
                target_mode = 0
                print('force to manual control')
                print(vels(target_linear_vel,target_angular_vel,target_mode))

            elif key == 'n' :
                if isAuto:
                    print('Auto-navigation mode, keyboard is blocked. Enter \'/\' for Manual-control mode')
                else:
                    isAuto = 1

                    navigation_mode.isAuto = 1
                    navigation_mode.path = 1
                    navigation_mode.isWrong = 0

                    print('Now go to Auto-navigation mode 1, keyboard is blocked. Enter \'/\' for Manual-control mode')

            elif key == 'm' :
                if isAuto:
                    print('Auto-navigation mode, keyboard is blocked. Enter \'/\' for Manual-control mode')
                else:
                    isAuto = 1

                    navigation_mode.isAuto = 1
                    navigation_mode.path = 2
                    navigation_mode.isWrong = 0

                    print('Now go to Auto-navigation mode 2, keyboard is blocked. Enter \'/\' for Manual-control mode')

            elif key == ',' :
                if isAuto:
                    print('Auto-navigation mode, keyboard is blocked. Enter \'/\' for Manual-control mode')
                else:
                    isAuto = 1

                    navigation_mode.isAuto = 1
                    navigation_mode.path = 3
                    navigation_mode.isWrong = 0

                    print('Now go to Auto-navigation mode 3, keyboard is blocked. Enter \'/\' for Manual-control mode')

            elif key == '.' :
                if isAuto:
                    print('Auto-navigation mode, keyboard is blocked. Enter \'/\' for Manual-control mode')
                else:
                    isAuto = 1

                    navigation_mode.isAuto = 1
                    navigation_mode.path = 4
                    navigation_mode.isWrong = 0

                    print('Now go to Auto-navigation mode 4, keyboard is blocked. Enter \'/\' for Manual-control mode')

            if isAuto == 0:
                if key == 'w' :
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel,target_mode))
                elif key == 'x' :
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel,target_mode))
                elif key == 'a' :
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel,target_mode))
                elif key == 'd' :
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel,target_mode))
                elif key == ' ' or key == 's' :
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    status = status +1
                    print(vels(target_linear_vel, target_angular_vel,target_mode))
                elif key == '1' :
                    target_mode = 1
                    status = status +1
                    print(vels(target_linear_vel, target_angular_vel,target_mode))
                elif key == '2' :
                    target_mode = 2
                    status = status +1
                    print(vels(target_linear_vel, target_angular_vel,target_mode))
                elif key == '3' :
                    target_mode = 3
                    status = status +1
                    print(vels(target_linear_vel, target_angular_vel,target_mode))
                elif key == '4' :
                    target_mode = 4
                    status = status +1
                    print(vels(target_linear_vel, target_angular_vel,target_mode))
            
                elif key == '5' :
                    target_mode = 5
                    status = status +1
                    print(vels(target_linear_vel, target_angular_vel,target_mode))
                elif key == '6' :
                    target_mode = 6
                    status = status +1
                    print(vels(target_linear_vel, target_angular_vel,target_mode))
                elif key == '7' :
                    target_mode = 7
                    status = status +1
                    print(vels(target_linear_vel, target_angular_vel,target_mode))
                else:
                    if (key == '\x03'):
                        break

                if status == 20 :
                    print(msg)
                    status = 0

                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                control_mode = target_mode

                wcv = WheelCmdVel()
                wcv.desiredWV_R = control_linear_vel + control_angular_vel
                wcv.desiredWV_L = control_linear_vel - control_angular_vel
                wcv.mode = control_mode

                #print(wcv.mode)

                pub.publish(wcv)

            pub_autoNavi.publish(navigation_mode)
            

    except:
        print(e)

    finally:
        wcv = WheelCmdVel()
        wcv.desiredWV_R = 0.0
        wcv.desiredWV_L = 0.0
        wcv.mode = 0
        pub.publish(wcv)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
