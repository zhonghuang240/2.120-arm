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
from std_msgs.msg import Float32MultiArray
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
from std_msgs.msg import Bool

armDone = False
robotDocked = False

handoffMsg = Bool()
handoffMsg.data = False

# For UR Team
def handoffCallback(msg):
    '''
    Set robotDocked when robot publishes "docked" message
    '''
    global robotDocked
    robotDocked = msg.data

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

global coords
coords = Float32MultiArray()
global waypoints_1
waypoints_1 = []
global waypoints_2 
waypoints_2 = []
global waypoints_3
waypoints_3 = []

class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()

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
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.command = outputMsg.Robotiq2FGripper_robot_output();


  def go_to_joint_state(self):
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
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/3
    joint_goal[2] = pi/2
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
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
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
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

  def callback(self, data):  ############################# THIS METHOD ADDED
      # Here we change the global variable docking based on the new message on this topic. We check this variable in the main method, once it is true, we know to execute our code for this task
      global coords
      global found_bottle
      global waypoints_1
      global waypoints_2
      global waypoints_3
      global robotDocked
      global handoffMsg
      global handoff_pub


      
      coords = data.data
      coords = list(coords)

      wpose = geometry_msgs.msg.Pose() # above bottle
      wpose.position.x = coords[0]
      wpose.position.y = coords[1]
      wpose.position.z = coords[2]+0.12
      # prepare for vertical bottle orientation
      wpose.orientation.x = 0.0
      wpose.orientation.y = 0.7071
      wpose.orientation.z = 0.0
      wpose.orientation.w = 0.7071

      wpose1 = copy.deepcopy(wpose)
      wpose1.position.z = coords[2] # on bottle top
      
      wpose2 = copy.deepcopy(wpose) # above basket
      wpose2.position.x = 0.43+0.31-0.11/2 # distance from table edge to basket center + distance from base to table edge - half bottle length              # bottle centered over basket
      wpose2.position.y = 0.435-0.04/2 # distance from base to table edge + distance from table edge to basket center - half bottle width            # bottle centered over basket
      wpose2.position.z = coords[2]-0.87+0.91+0.1+0.01 # (coords[2] = height of gripper when bottle is just sitting on tabletop) - (tabletop to floor) + (floor to basket bottom) + (basket bottom to basket top) + 1 cm for clearance
      # turn bottle to rest horizontally in basket
      # wpose2.orientation.x = 0.683
      # wpose2.orientation.y = 0.183
      # wpose2.orientation.z = 0.683
      # wpose2.orientation.w = -0.183

      wpose3 = copy.deepcopy(wpose2) # in basket
      wpose3.position.z = wpose2.position.z-0.1 # (wpose2.position.z = height of gripper after clearing basket top) - (basket top to basket bottom)

      print("got coordinates")

      found_bottle = True
      print(coords)
      waypoints_1 = [wpose,wpose1] # hover above bottle
      waypoints_2 = [wpose,wpose2,wpose3] # pick bottle up & clear other bottles
      waypoints_3 = [wpose2,wpose] # clear basket top
      # print(waypoints_1)
      print('--------------- FOUND BOTTLE is TRUE --------------------')
      # waypoints_1 = [coords]
      # waypoints_2 = []

  # try:
  #   print ""
  #   print "----------------------------------------------------------"
  #   print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
  #   print "----------------------------------------------------------"
  #   print "Press Ctrl-D to exit at any time"
  #   print ""
  #   print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
  #   raw_input()
    
  #   tutorial = MoveGroupPythonIntefaceTutorial()
  #   tutorial.listener()
      print("============ go to joint state")
      self.go_to_joint_state()
      print "============ reset the gripper ..."
      #raw_input()
      self.publisher('r')

      print "============ activate the gripper ..."
      #raw_input()
      self.publisher('a')

      print "============ close the gripper ..."
      #raw_input()
      self.publisher('c')
    
      print "============ open the gripper ..."
      #raw_input()
      self.publisher('o')

  #   print "============ Press `Enter` to execute a movement using a joint state goal ..."
  #   raw_input()
  #   # print "============ Press `Enter` to execute a movement using a pose goal ..."
  #   # raw_input()
      #self.go_to_pose_goal(init_pose_goal)
    
  #   # you can create your own waypoint and run the following code.
      # print "============ Press `Enter` to plan and display a Cartesian path ..."
      # print(waypoints_1)
      #raw_input()
      print("=========== go to bottle")
      self.publisher('o') # open the gripper
      cartesian_plan, fraction = self.plan_cartesian_path(waypoints=waypoints_1) # plan to get to top of bottle

  #   print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
  #   raw_input()
  #   tutorial.display_trajectory(cartesian_plan)

  #   print "============ Press `Enter` to execute a saved path ..."
  #   raw_input()
      self.execute_plan(cartesian_plan) # move to top of bottle

  #   # You can control the position of the gripper usingn numbers between 0-255. 0: fully opened, 255: fully closed.
  #   print "============ Press `Enter` to close the gripper ..."
      #raw_input()
      print("grab bottle")
      r=rospy.sleep(3.)
      self.publisher(150) #grab
      r=rospy.sleep(3)
  #   # you can create your own waypoint and run the following code.
  #   print "============ Press `Enter` to plan and display a Cartesian path ..."
      #raw_input()
      print("go to basket")
      cartesian_plan, fraction = self.plan_cartesian_path(waypoints=waypoints_2) # plan to move to basket

  #   print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
  #   raw_input()
  #   tutorial.display_trajectory(cartesian_plan)

  #   print "============ Press `Enter` to execute a saved path ..."
  #   raw_input()
      self.execute_plan(cartesian_plan) # move to basket

  #   print "============ Press `Enter` to open the gripper ..."
  #   raw_input()
      print("release bottle")
      self.publisher('o') # let go
      print("return to start")
      cartesian_plan, fraction = self.plan_cartesian_path(waypoints=waypoints_3) # plan to move away
      
      self.execute_plan(cartesian_plan)
      print("setting armDone to True")
      armDone = True
      robotDocked = False
      handoffMsg.data = armDone
      print("publishing handoff msg")
      handoff_pub.publish(handoffMsg) # move away

  #   print "============ Press `Enter` to add a box to the planning scene ..."
  #   raw_input()
  #   tutorial.add_box()

  #   print "============ Press `Enter` to attach a Box to the Panda robot ..."
  #   raw_input()
  #   tutorial.attach_box()

  #   print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
  #   raw_input()
  #   cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
  #   tutorial.execute_plan(cartesian_plan)

  #   print "============ Press `Enter` to detach the box from the Panda robot ..."
  #   raw_input()
  #   tutorial.detach_box()

  #   print "============ Press `Enter` to remove the box from the planning scene ..."
  #   raw_input()
  #   tutorial.remove_box()

  #   print "============ Python tutorial demo complete!"
      #tutorial = MoveGroupPythonInterfaceTutorial()
 #      try:
 #        #self.listener()
 #        waypoints_1 = [wpose,wpose1] # hover above bottle
 #        waypoints_2 = [wpose,wpose2,wpose3] # pick bottle up & clear other bottles
 #        waypoints_3 = [wpose2,wpose] # clear basket top
	# #print("waypoints set")
 #        #print "============ Press `Enter` to plan and display a Cartesian path ..."
 #        #print(waypoints_1)
 #        #raw_input()
 #        #cartesian_plan, fraction = self.plan_cartesian_path(waypoints=waypoints_1)
 #      except rospy.ROSInterruptException:
 #        return
 #      except KeyboardInterrupt:
 #        return


  def listener(self): ############################# THIS METHOD ADDED
      global robotDocked
      global handoffMsg
      global handoff_pub

      print("============ go to joint state")
      self.go_to_joint_state()
      print("Waiting for MR")  
      if robotDocked:
        # arm routine
        print("waiting for bottle detection")
        image_sub = rospy.Subscriber("/bottle_coordinates",Float32MultiArray, self.callback)
      print("go!")
      handoff_pub.publish(handoffMsg)
      print("after listener")
      rospy.spin()
      
      # every time a new message is recieved on this topic, the callback function is invoked
#global found_bottle
found_bottle = False

handoff_sub = rospy.Subscriber("/robot_arrived", Bool, handoffCallback)
handoff_pub = rospy.Publisher("/arm_done", Bool, queue_size=1) 
def main():
  try:

    tutorial = MoveGroupPythonInterfaceTutorial()
    # while found_bottle == False:
    tutorial.listener()

  
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





## Handoff code

#from std_msgs.msg import Bool

#armDone = False
#robotDocked = False

#handoffMsg = Bool()
#handoffMsg.data = False

## For Mobile Robot Team
#def handoffCallback(msg):
#    '''
#    Set ArmDone when arm publishes "finished" message
#    '''
#    global armDone
#    armDone = msg.data
    
#handoff_sub = rospy.Subscriber("/arm_done", Bool, handoffCallback) 
#handoff_pub = rospy.Publisher("/robot_arrived", Bool, queue_size=1)
#handoff_pub.publish(handoffMsg)

## When robot is docked, send message and wait until arm is done
#if # docked conditions#:
#    robotDocked = True
#    handoffMsg.data = robotDocked
#    handoff_pub.publish(handoffMsg)
#    if armDone:
#        # Do next part
    
## For UR Team
#def handoffCallback(msg):
#    '''
#    Set robotDocked when robot publishes "docked" message
#    '''
#    global robotDocked
#    robotDocked = msg.data
    
#handoff_sub = rospy.Subscriber("/robot_arrived", Bool, handoffCallback) 
#handoff_pub = rospy.Publisher("/arm_done", Bool, queue_size=1)
#handoff_pub.publish(handoffMsg)

## When robot is docked, do arm things and send message when done
#if robotDocked:
#    # arm routine
#    armDone = True
#    handoffMsg.data = armDone
#    handoff_pub.publish(handoffMsg)
    
