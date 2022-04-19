#!/usr/bin/env python2

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
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep


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
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

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
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.bottle_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.command = outputMsg.Robotiq2FGripper_robot_output();
    self.pose_goal = geometry_msgs.msg.Pose()

    self.pose_goal.position.z = 0.4
    self.pose_goal.orientation.x = 0
    self.pose_goal.orientation.y = 0.7071
    self.pose_goal.orientation.z = 0
    self.pose_goal.orientation.w = 0.7071
    self.docked = False


  def go_to_joint_state(self, joint_goal):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    # joint_goal = move_group.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = -pi/2
    # joint_goal[2] = pi/2
    # joint_goal[3] = -pi/2-0.1
    # joint_goal[4] = -pi/2
    # joint_goal[5] = 0
    # joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, pose_goal):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

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

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    # waypoints = []

    # wpose = move_group.get_current_pose().pose
    # wpose.position.z -= scale * 0.1  # First move up (z)
    # wpose.position.y += scale * 0.2  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * 0.1  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    print waypoints
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
    ## END_SUB_TUTORIA

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "ee_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.07 # slightly above the end effector
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.044, 0.030, 0.111))
    #scene.add_cylinder(box_name, box_pose, 0.1, 0.1)

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def add_bottle(self, name, pose, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    bottle_name = self.bottle_name
    scene = self.scene
    pose.pose.position.z = 0.111/2 # slightly above the end effector

    scene.add_box(name, pose, size=(0.044, 0.030, 0.111))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.bottle_name=bottle_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def add_plane(self, name, pose, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    bottle_name = self.bottle_name
    scene = self.scene


    scene.add_plane(name, pose)

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.bottle_name=bottle_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'endeffector'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

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

  def publisher(self, char):
      """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
      pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

      self.command = self.genCommand(char)
      print(self.command)
      pub.publish(self.command)

      rospy.sleep(0.1)

  def callback_vision(self, data):
      if data.position.x > 0:
          self.pose_goal.position.x = data.position.x
          self.pose_goal.position.y = data.position.y
          print data.position.x
          print data.position.y
          rospy.spin()
      else:
          print("waiting on coords from cv...")
          # rospy.spin()

  def callback_mobile(self, data):
      """Receives String msg from mobile robot to signal when docked"""
      if data.data == 'docked':
          self.docked = True
          pass
      else:
          print("waiting on mobile robot to dock...")

  def mobile_pub(self):
      pub = rospy.Publisher('docker_status', String)
      pub.publish("dropped off")

      rospy.sleep(0.1)


def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()
    pose_goal_sub = rospy.Subscriber("ur_coords", geometry_msgs.msg.Pose, tutorial.callback_vision)
    # subscribe in the begininng and quit the subscription so robot doesn't get confused

    init_pose_goal = tutorial.pose_goal

    # print "Pose Goal is : "
    # print init_pose_goal


    print "Tutorial Pose Goal is : "
    print tutorial.pose_goal
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = "world"
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.x = 0.5
    # box_pose.pose.position.y = 0
    # box_pose.pose.position.z = 0.05 # slightly above the end effector


    joint_goal1 = tutorial.move_group.get_current_joint_values()
    joint_goal1[0] = 0
    joint_goal1[1] = -pi/2
    joint_goal1[2] = pi/2
    joint_goal1[3] = -pi/2
    joint_goal1[4] = -pi/2
    joint_goal1[5] = 0


    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()

    print "Pose Goal 2 is : "
    print init_pose_goal


    print "Tutorial Pose Goal 2 is : "
    print tutorial.pose_goal

    tutorial.go_to_joint_state(joint_goal1)
    
    # plane_pose = geometry_msgs.msg.PoseStamped()
    # plane_pose.header.frame_id = "world"
    # plane_pose.pose.orientation.w = 1.0
    # plane_pose.pose.position.x = 0
    # plane_pose.pose.position.y = 0
    # plane_pose.pose.position.z = 0. # slightly above the end effector

    # print "============ Press `Enter` to add a plane to the planning scene ..."
    # raw_input()
    # tutorial.add_plane("ground",plane_pose)

    # box_pose.pose.position.x = 0.5
    # box_pose.pose.position.y = 0
    # box_pose.pose.position.z = 0.05 # slightly above the end effector

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_bottle("bottle1",box_pose)

    # box_pose.pose.position.x = 0.6
    # box_pose.pose.position.y = 0
    # box_pose.pose.position.z = 0.05 # slightly above the end effector

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_bottle("bottle2",box_pose)

    # box_pose.pose.position.x = 0.8
    # box_pose.pose.position.y = 0
    # box_pose.pose.position.z = 0.05 # slightly above the end effector

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_bottle("bottle3",box_pose)

    # box_pose.pose.position.x = 0.7
    # box_pose.pose.position.y = 0
    # box_pose.pose.position.z = 0.05 # slightly above the end effector

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_bottle("bottle4",box_pose)
    # print "============ Press `Enter` to execute a movement using a joint state goal ..."
    # raw_input()
    # tutorial.go_to_joint_state()

    # print "============ Press `Enter` to execute a movement to inital pose goal ..."
    # raw_input()
    # tutorial.go_to_pose_goal(init_pose_goal)

    # You can control the position of the gripper usingn numbers between 0-255. 0: fully opened, 255: fully closed.
    print "============ Press `Enter` to reset the gripper ..."
    # raw_input()

    tutorial.publisher('r')
    rospy.sleep(5)
    print "============ Press `Enter` to activate the gripper ..."
    # raw_input()
    tutorial.publisher('a')
    rospy.sleep(5)
    print "============ Press `Enter` to open the gripper ..."
    # raw_input()
    tutorial.publisher('o')
    rospy.sleep(5)
    # init_pose_goal.position.x = 0.6
    # init_pose_goal.position.y = 0

    waypoints_1 = []

    wpose = tutorial.move_group.get_current_pose().pose
    # wpose.position.x += (init_pose_goal.position.x - wpose.position.x)/2
    # waypoints_1.append(copy.deepcopy(wpose))

    # print wpose

    wpose.position.z = 0.3
    waypoints_1.append(copy.deepcopy(wpose))

    wpose.position.y = init_pose_goal.position.y
    waypoints_1.append(copy.deepcopy(wpose))

    if init_pose_goal.position.x > 0.8:
        wpose.position.x = 0.8
    else:
         wpose.position.x = init_pose_goal.position.x
    
    waypoints_1.append(copy.deepcopy(wpose))

    wpose.position.z = 0.17
    waypoints_1.append(copy.deepcopy(wpose))

    # print waypoints_1
    # you can create your own waypoint and run the following code.
    print "============ Press `Enter` to plan and display a Cartesian path to above bottle ..."
    # raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path(1, waypoints_1)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    tutorial.execute_plan(cartesian_plan)
    rospy.sleep(5)
    # print "============ Press `Enter` to move above the target bottle ..."
    # raw_input()
    # tutorial.go_to_pose_goal(init_pose_goal)

    # init_pose_goal.position.z = 0.17

    # # print(init_pose_goal)

    # print "============ Press `Enter` to move down to grab the bottle ..."
    # raw_input()
    # tutorial.go_to_pose_goal(init_pose_goal)

    # print(init_pose_goal)

    # print "============ Press `Enter` to close the gripper ..."
    # raw_input()
    tutorial.publisher('c')

    rospy.sleep(5)

    init_pose_goal.position.z = 0.4

    # print "============ Press `Enter` to move back up ..."
    # raw_input()
    tutorial.go_to_pose_goal(init_pose_goal)

    waypoints_2 = []

    wpose = tutorial.move_group.get_current_pose().pose

    wpose.position.z = 0.3
    waypoints_2.append(copy.deepcopy(wpose))

    wpose.position.x = 0.1
    wpose.position.y = 1.0
    waypoints_2.append(copy.deepcopy(wpose))

    # print "============ Press `Enter` to plan and display a Cartesian path to mobile robot ..."
    # raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path(waypoints=waypoints_2)

    #subscribe
    while not tutorial.docked:
        subscriber = rospy.Subscriber("docker_status", String, tutorial.callback_mobile)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    tutorial.execute_plan(cartesian_plan)

    # publish
    

    # print "============ Press `Enter` to execute a movement using a joint state goal ..."
    # raw_input()
    # tutorial.go_to_joint_state(joint_goal1)

    # init_pose_goal.position.x = 0
    # init_pose_goal.position.y = 0.6

    # print "============ Press `Enter` to execute move to mobile robot basket ..."
    # raw_input()
    # # subscribe to mobile robot topic to determine when docked
    # #basket_goal_sub = rospy.Subscriber("dock_status", String, tutorial.callback_mobile)
    # tutorial.go_to_pose_goal(init_pose_goal)

    # print "============ Press `Enter` to open the gripper ..."
    # raw_input()
    tutorial.publisher('o')

    rospy.sleep(5)

    init_pose_goal = geometry_msgs.msg.Pose()
    init_pose_goal.position.x = 0.4
    init_pose_goal.position.y = 0
    init_pose_goal.position.z = 0.4
    init_pose_goal.orientation.x = 0
    init_pose_goal.orientation.y = 0.7071
    init_pose_goal.orientation.z = 0
    init_pose_goal.orientation.w = 0.7071

    tutorial.mobile_pub()

    # print "============ Press `Enter` to execute a movement using a joint state goal ..."
    # raw_input()
    tutorial.go_to_joint_state(joint_goal1)

    joint_goal2 = joint_goal1
    joint_goal2[5] = pi/4

    joint_goal3 = joint_goal1
    joint_goal3[5] = pi/2

    joint_goal4 = joint_goal1
    joint_goal4[5] = 3*pi/2

    joint_goal5 = joint_goal1
    joint_goal5[5] = 2*pi

    # print "============ Press `Enter` to execute victory dance ..."
    # raw_input()

    tutorial.go_to_joint_state(joint_goal2)
    tutorial.go_to_joint_state(joint_goal3)
    tutorial.go_to_joint_state(joint_goal2)


    dance_pose_goal = geometry_msgs.msg.Pose()
    dance_pose_goal.position.x = 0.6
    dance_pose_goal.position.y = 0
    dance_pose_goal.position.z = 0.17
    dance_pose_goal.orientation.x = 0
    dance_pose_goal.orientation.y = 0.7071
    dance_pose_goal.orientation.z = 0
    dance_pose_goal.orientation.w = 0.7071

    tutorial.go_to_joint_state(joint_goal1)
    tutorial.go_to_pose_goal(dance_pose_goal)
    tutorial.go_to_joint_state(joint_goal1)
    tutorial.go_to_pose_goal(dance_pose_goal)
    tutorial.go_to_joint_state(joint_goal1)
    tutorial.go_to_pose_goal(dance_pose_goal)
    tutorial.go_to_joint_state(joint_goal1)

    # you can create your own waypoint and run the following code.
    # print "============ Press `Enter` to plan and display a Cartesian path ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path(waypoints=waypoints_1)

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # tutorial.display_trajectory(cartesian_plan)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    # tutorial.execute_plan(cartesian_plan)


    # you can create your own waypoint and run the following code.
    # print "============ Press `Enter` to plan and display a Cartesian path ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path()

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    # tutorial.execute_plan(cartesian_plan)

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # tutorial.display_trajectory(cartesian_plan)


    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_box()

    # print "============ Press `Enter` to attach a Box to the Panda robot ..."
    # raw_input()
    # tutorial.attach_box()

    # print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    # tutorial.execute_plan(cartesian_plan)

    # print "============ Press `Enter` to detach the box from the Panda robot ..."
    # raw_input()
    # tutorial.detach_box()

    # print "============ Press `Enter` to remove the box from the planning scene ..."
    # raw_input()
    # tutorial.remove_box()


    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

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
