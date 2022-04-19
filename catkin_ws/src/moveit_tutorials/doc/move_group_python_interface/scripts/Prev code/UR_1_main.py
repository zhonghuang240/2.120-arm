#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# 2.12 Final Project UR5 Script Version ONE [05/15/2021]
# Authors: Darius Chan, Mikey Fernandez, Carl Andrew Seelhoff, Ankita Singh, Max Thomsen

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

""" Part One: Imports from UR5 Gripper script """
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from time import sleep

""" Part Two: Imports from Bottle Detection script """
# roslib.load_manifest('my_package')
import cv2
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from tkinter import *
# from tk import *
import numpy as np
import time
import math

# Extra imports from Mobile Base Script
# import terminos
import tty
""" Part Three: Initialization """

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=False)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.command = outputMsg.Robotiq2FGripper_robot_output();
    self.baseArrived = False
    self.taskComplete = False
    self.bottlePositionReceived = False
    #self.bottlePosition = [0, 0]

  def go_to_joint_state(self, goal):
    ## Inputs: desired joint angles in a list (goal)
    ## Outputs: arm moves to desired location

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(goal, current_joints, 0.01)


  def go_to_pose_goal(self, position, orientation):
    ## Inputs: desired coordinates of endpoint [x, y, z] and desired oreintation as a quaternion [x, y, z, w]
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = position[0]
    pose_goal.position.y = position[1]
    pose_goal.position.z = position[2]
    pose_goal.orientation.x = orientation[0]
    pose_goal.orientation.y = orientation[1]
    pose_goal.orientation.z = orientation[2]
    pose_goal.orientation.w = orientation[3]
    self.move_group.clear_pose_targets()
    self.move_group.set_pose_target(pose_goal)
    print("pose target set")
    print(pose_goal)
    ## Now, we call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1, waypoints=[]):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group
    print(waypoints)

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def genCommand(self, char):
      """Update the command according to the character entered by the user."""    
      # command = self.command

      if char == 'a':
          self.command = outputMsg.Robotiq2FGripper_robot_output();
          self.command.rACT = 1
          self.command.rGTO = 1
          self.command.rSP  = 255
          self.command.rFR  = 150

      if char == 'r':
          self.command = outputMsg.Robotiq2FGripper_robot_output();
          self.command.rACT = 0

      if char == 'c':
          self.command.rPR = 255

      if char == 'o':
          self.command.rPR = 0   

      #If the command entered is a int, assign this value to rPRA
      try: 
          self.command.rPR = int(char)
          if self.command.rPR > 255:
              self.command.rPR = 255
          if self.command.rPR < 0:
              self.command.rPR = 0
      except ValueError:
          pass                    
          
      if char == 'f':
          self.command.rSP += 25
          if self.command.rSP > 255:
              self.command.rSP = 255
              
      if char == 'l':
          self.command.rSP -= 25
          if self.command.rSP < 0:
              self.command.rSP = 0

              
      if char == 'i':
          self.command.rFR += 25
          if self.command.rFR > 255:
              self.command.rFR = 255
              
      if char == 'd':
          self.command.rFR -= 25
          if self.command.rFR < 0:
              self.command.rFR = 0

      return self.command
  
  def CreateWaypoints(self, direction = str, isNegative = False):
    """ 
    Function to make list of lists for waypoints so the ManualArmCommand method isn't super scary looking.
    #Inputs: Direction (x, y, or z, as a string); isNegative (boolean)#
    """

    Waypoints = []
    ArmPosition = self.move_group.get_current_pose().pose.position
    #determine which coordinate to modify
    if direction == 'x':
      PositionMod = 0
    elif direction == 'y':
      PositionMod = 1
    elif direction == 'z':
      PositionMod = 2
    if isNegative == True:
      ArmPosition[PositionMod] -= 0.01
    elif isNegative == False:
      ArmPosition[PositionMod] += 0.01
    Test = Point(ArmPosition[0], ArmPosition[1], ArmPosition[2])
    NewPosition = Pose()
    NewPosition.position = Test
    NewPosition.orientation = Quaternion(0.0, 0.7071, 0.0, 0.7071)
    Waypoints.append(NewPosition)
    return Waypoints

  def ManualArmCommand(self, EmergencyStop = False):
      """ A function to parse a keyboard input into motion of the UR5    
      Input: Character from manual input (to be determined)
      Output: some specified movement is performed, be it for the UR5 or the gripper itself."""
      # This is a modified version of the Manual function from the mobile robot team.
      # Positive X: D
      # Negative X: A
      # Positive Y: W
      # Negative Y: S
      # Positive Z: E
      # Negative Z: Q
      # Open Gripper: Z
      # Close Gripper: C
      if not EmergencyStop:
        print("before raw_input")
        char = raw_input() #I think this is wrong? Should it not be char instead of inputchar - Darius
        print("a" in char)
        #ArmPosition = self.move_group.get_current_pose().pose.position
        if "d" in char:
          ManualWaypoints = self.CreateWaypoints('x', False)
          (ManualPlan, ManualFraction) = self.plan_cartesian_path(1, ManualWaypoints)
          self.execute_plan(ManualPlan)
          print("Move arm in POSITIVE X")
        elif "a" in char:
          ManualWaypoints = self.CreateWaypoints('x', True)
          (ManualPlan, ManualFraction) = self.plan_cartesian_path(1, ManualWaypoints)
          self.execute_plan(ManualPlan)
          print("Move arm in NEGATIVE X")
        elif "w" in char:
          ManualWaypoints = self.CreateWaypoints('y', False)
          (ManualPlan, ManualFraction) = self.plan_cartesian_path(1, ManualWaypoints)
          self.execute_plan(ManualPlan)
          print("Move arm in POSITIVE Y")
        elif "s" in char:
          ManualWaypoints = self.CreateWaypoints('y', True)
          (ManualPlan, ManualFraction) = self.plan_cartesian_path(1, ManualWaypoints)
          self.execute_plan(ManualPlan)
          print("Move arm in NEGATIVE Y")
        elif "e" in char:
          ManualWaypoints = self.CreateWaypoints('z', False)
          (ManualPlan, ManualFraction) = self.plan_cartesian_path(1, ManualWaypoints)
          self.execute_plan(ManualPlan) 
          print("Move arm in POSITIVE Z")
        elif "q" in char: 
          ManualWaypoints = self.CreateWaypoints('z', True)
          (ManualPlan, ManualFraction) = self.plan_cartesian_path(1, ManualWaypoints)
          self.execute_plan(ManualPlan)
          print("Move arm in NEGATIVE Z")
        elif "z" in char:
          self.GripperPublisher('o')
          print(" OPEN GRIPPER")
        elif "c" in char:
          self.GripperPublisher('c')
          print("CLOSE GRIPPER")

        else:
          print("Stop")
          
      #return self.ManualArmCommand(EmergencyStop = False)
    
  def GripperPublisher(self, char):
      """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
      pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)

      self.command = self.genCommand(char)            
      print(self.command)
      pub.publish(self.command)

      rospy.sleep(0.1)

  def ArmCompletionPublisher(self, msg): 
      # instantiate a publisher that will notify the mobile robot that the bottle has been successfully placed in the basket.
      CompletePub = rospy.Publisher('UR5TaskCompletion', Bool, queue_size=1)
      print(msg)
      result = Bool()
      result.data = True
      CompletePub.publish(result)

  def arrivalCallback(self, msg):
    self.baseArrived = msg.data
    
  def MobileArrivalListener(self):
      # instantiate a subscriber that will receive coordinates of the basket's center once the robot has arrived.
      # I'm not sure if this is correct or not.
    rospy.Subscriber('mobile_base_ready', Bool, self.arrivalCallback, queue_size = 10)
    print('Mobile Listener Started')

  def getBottlePosition(self, data):
    if not self.bottlePositionReceived:
      self.bottlePosition = [data.position.x + 0.400, -data.position.y + 0.37]##This may need to be tuned##
      self.bottlePositionReceived = True
      print("Bottle found")
      print(data.position.x + 0.395, -data.position.y + 0.37)

  def bottleListener(self):
    rospy.Subscriber('bottlePos', Pose, self.getBottlePosition, queue_size = 10)
    print('Bottle Listener Started')
    
  def updateBaseX(self, data):
    self.baseX = data.data #[TODO] CHECK THE OFFSET
    
  def updateBaseY(self, data):
    self.baseY = data.data #[TODO] CHECK THE OFFSET
    
  def baseTracker(self):
    rospy.Subscriber('mobile_base_x_pos', Float32, self.updateBaseX, queue_size=10)
    rospy.Subscriber('mobile_base_y_pos', Float32, self.updateBaseY, queue_size=10)

class StopError(Exception):
  pass
      
def check_manual(keyboard_input):
  if 'manual' in keyboard_input.lower():
        raise KeyboardInterrupt 
  elif 'stop' in keyboard_input.lower():
        raise StopError
  else:
        pass
                
      
def main():
  while not rospy.is_shutdown():
    try:
      print("")
      print("----------------------------------------------------------")
      print("Beginning Simulation of 2.12 Project Tasks")
      print("----------------------------------------------------------")
      print("Press Ctrl-D to exit at any time")
      print("Type 'Stop' or 'Manual' when prompted to engage manual control")
      print("")
      print("============ Beginning the tutorial by setting up the moveit_commander ...")
      # keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      tutorial = MoveGroupPythonIntefaceTutorial()
      print('Built Tutorial')
      # start listeners
      print('Starting Listeners')
      tutorial.MobileArrivalListener()
      tutorial.baseTracker()
      
      print("============ Awaiting Response from Mobile Robot ...")
      # while Subscriber Message from Mobile Robot is Not Received:
      #  Do nothing
      while not tutorial.baseArrived: ##Commented out for testing##
      	rospy.sleep(1)
      
      print("============ Mobile Robot Position Received. Beginning Bottle Detection ...")

      tutorial.bottleListener()
      rospy.sleep(0.5)
      
      print("============ Identifying desired bottle ...")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      # Implement and execute method for computer vision [TODO]
      # tutorial.getBottlePosition()
      rospy.sleep(0.5)
      
      print("============ Initializing Gripper Trajectory to Bottle...")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)

      BottleGoalOrientation = Quaternion(0, 0.7071, 0, 0.7071)
      BottlePoseOver = Pose(Point(tutorial.bottlePosition[0], tutorial.bottlePosition[1], 0.33), BottleGoalOrientation)
      BottlePoseGrasp = Pose(Point(tutorial.bottlePosition[0], tutorial.bottlePosition[1], 0.18), BottleGoalOrientation)
      homePose = Pose(Point(0.4, 0.1, 0.3), BottleGoalOrientation)

      # uncomment if not using bottle detection
      #BottleGoalPosition = Point(0.75, 0.15, 0.2) #tutorial.bottlePosition
      #BottlePoseOver = Pose(Point(0.75, 0.15, 0.3), BottleGoalOrientation)
      #BottlePoseGrasp = Pose(Point(0.75, 0.15, 0.2), BottleGoalOrientation)

      # They should be outputted and/or updated by the method for computer vision and detecting the bottle. 
      # Orientation should be fixed, as the gripper should be facing straight down.
      tutorial.GripperPublisher('r')
      tutorial.GripperPublisher('a')
      tutorial.GripperPublisher('o')
      rospy.sleep(1)
      
      print("============ Moving to Home ...")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      (BottlePlan, fraction) = tutorial.plan_cartesian_path(waypoints=[homePose])
      tutorial.execute_plan(BottlePlan)
      rospy.sleep(1)
      
      print("============ Moving over Bottle ...")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      (BottlePlan2, fraction) = tutorial.plan_cartesian_path(waypoints = [BottlePoseOver])
      tutorial.execute_plan(BottlePlan2)
      rospy.sleep(1)
      
      print("============ Moving to Grasp Bottle ...")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      (BottlePlan3, fraction) = tutorial.plan_cartesian_path(waypoints = [BottlePoseGrasp])
      tutorial.execute_plan(BottlePlan3)
      rospy.sleep(1)
      
      print("============ Grabbing the Bottle ...")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      tutorial.GripperPublisher('c')

      # print("============ Press 'Enter' to move the arm to the basket position ...")
      # keyboard_input = raw_input("Awaiting input: ")
      # check_manual(keyboard_input)
      rospy.sleep(1)
      
      print("============ Moving Upwards ...")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      (BottlePlan2, fraction) = tutorial.plan_cartesian_path(waypoints = [BottlePoseOver])
      tutorial.execute_plan(BottlePlan2)
      rospy.sleep(1)
      
      print("============ Moving above Basket ...")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      #BasketAbovePosition = Point(.74, .55, 0.3) #TESTING
      BasketAbovePosition = Point(tutorial.baseX, tutorial.baseY, 0.33)
      BasketPoseAbove = Pose(BasketAbovePosition, BottleGoalOrientation)
      (BasketPlan1, fraction) = tutorial.plan_cartesian_path(waypoints=[BasketPoseAbove])
      tutorial.execute_plan(BasketPlan1)
 
      rospy.sleep(1)
      
      print("============ Moving to Drop Position ...")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      #BasketGoalPosition = Point(.74, .55, 0.23) ##Added for testing## in meters
      BasketGoalPosition = Point(tutorial.baseX, tutorial.baseY, 0.25)
      BasketPoseDrop = Pose(BasketGoalPosition, BottleGoalOrientation)

      #BasketGoalPosition will be received from MobileArrivalSubscriber, which was initialized earlier as the position of the basket according to the mobile robot.
      (BasketPlan2, fraction) = tutorial.plan_cartesian_path(waypoints=[BasketPoseDrop])
      tutorial.execute_plan(BasketPlan2)
      rospy.sleep(1)
      
      print("============ Releasing the Bottle ...")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      tutorial.GripperPublisher('o')
      rospy.sleep(1)
      
      print("============ Returning Gripper to Home Position")
      #keyboard_input = raw_input("Awaiting input: ")
      #check_manual(keyboard_input)
      
      (BottlePlan, fraction) = tutorial.plan_cartesian_path(waypoints=[homePose])
      tutorial.execute_plan(BottlePlan)
      rospy.sleep(1)
      
      print("============ Bottle Delivered Successfully! Publishing message to Mobile Robot ...")
      tutorial.taskComplete = True
      tutorial.ArmCompletionPublisher("UR5 Task Complete")

      ### End of Mandatory UR5 Task ###
      ### Extra Credit stuff ??? ###
      

    except rospy.ROSInterruptException:
      print('rospy Exception')
      return
    except KeyboardInterrupt:
      print("\n!!~~~ Manual Control Enabled ~~~!!")
      try:
        while True:
          tutorial.ManualArmCommand(EmergencyStop = False)
      except:
        print('Emergency Stop')
        break
    except:
      break
  print('Shutting Down')

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/melodic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
