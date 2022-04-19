#!/usr/bin/env python

import sys
import copy
import rospy
import Queue
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from moveit_commander.conversions import pose_to_list

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
import time


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


class UR5_Simulation(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(UR5_Simulation, self).__init__()

    ## First initialize `moveit_commander`
    moveit_commander.roscpp_initialize(sys.argv)

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

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    #print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    #print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    print robot.get_current_state()
    #print ""
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
    self.command = outputMsg.Robotiq2FGripper_robot_output()


  def go_to_joint_state(self, target):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    # pi, -pi/2, 2pi/3, -2pi/3, -pi/2, 0
    # 0, -pi/4, pi/4, -pi/2, 0, pi/3
    joint_goal[0] = target[0]
    joint_goal[1] = target[1]
    joint_goal[2] = target[2]
    joint_goal[3] = target[3]
    joint_goal[4] = target[4]
    joint_goal[5] = target[5]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):
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
    pose_goal = geometry_msgs.msg.Pose()
    current_pose = move_group.get_current_pose().pose
    #print(current_pose)
    #print(type(current_pose))
    #print(current_pose.position.x)
    #print(type(current_pose.position.x))

    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.7071
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.7071
    pose_goal.position.x = current_pose.position.x
    pose_goal.position.y = current_pose.position.y
    pose_goal.position.z = current_pose.position.z

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


  def plan_cartesian_path(self, x_cm, y_cm, z_cm, order, scale=1):
    # This function receives x and y in centimeters, convert to meters.
    move_group = self.move_group

    x_m = x_cm/100.0
    y_m = y_cm/100.0
    z_m = z_cm/100.0
    
    waypoints = []
    
    wpose = move_group.get_current_pose().pose
    for k in range(len(order)):
	if order[k] == 'x':
    	    wpose.position.x = scale * x_m 
    	    waypoints.append(copy.deepcopy(wpose))
	elif order[k] == 'y':
            wpose.position.y = scale * y_m
    	    waypoints.append(copy.deepcopy(wpose))
	elif order[k] == 'z':
    	    wpose.position.z = scale * z_m
    	    waypoints.append(copy.deepcopy(wpose))
	else:
	    print("Order of operations not valid, double check function argument.")    

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
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## Displaying a Trajectory
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


  def execute_plan(self, plan):
    move_group = self.move_group

    ## Executing a Plan
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


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


def callback_location(data):
  global data_queue
  data_queue.put(float(data.data))


def callback_docking(data):
  global docking_queue
  docking_queue.put(Bool(data.data))


def main():
  global data_queue, docking_queue
  rospy.init_node('ur5_trajectory_node', anonymous=True) 
  data_queue = Queue.Queue()
  docking_queue = Queue.Queue()

  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the UR5 Trajectory Planner of the Best 2.120 Group"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to set up the moveit_commander ..."
    raw_input()
    robot = UR5_Simulation()

    print "============ Press `Enter` to move UR5 Robot to starting position ..."
    raw_input()
    robot.go_to_joint_state([0, -pi/4, pi/4, -pi/2, 0, pi/3])
    
    #print "============ Press `Enter` to move UR5 Robot to start position ..."
    #raw_input()
    robot.go_to_joint_state([0, -2*pi/3, 2*pi/3, -pi/2, 0, pi/3])
    robot.go_to_joint_state([pi/7, -2*pi/3, 2*pi/3, -pi/2, 0, pi/3])
    robot.go_to_joint_state([pi/7, -2*pi/3, 2*pi/3, -pi/2, -pi/2, pi/3])
    robot.go_to_joint_state([pi/7, -2*pi/3, 2*pi/3, -pi/2, -pi/2, 0])

    #print "============ Press `Enter` to reset gripper ..."
    #raw_input()
    robot.publisher('r')

    print "============ Press `Enter` to activate gripper ..."
    raw_input()
    robot.publisher('a') 
    robot.publisher('o')
    
    print "============ Start bottle detection, then press `Enter` ..."
    raw_input()
    ## Start loop here eventually

    print "============ Waiting for bottle positions ...\n"
    location_sub = rospy.Subscriber("bottle_location", Float32, callback_location) 
    counter = 0
    pos = []
    while counter < 2:
	pos.append(data_queue.get(True))
	print("Data received!")
	counter += 1
    print("Received position: ", pos)
    x_final = pos[0]
    y_final = pos[1]
    z_final = 18.0
    location_sub.unregister()

    # Given position (normally gotten from ROS topic)
    #x_final = 70 # 70
    #y_final = 8.587 # 95.49425

    print "============ Press `Enter` to plan and execute the path to the bottle ..."
    raw_input()
    cartesian_plan, fraction = robot.plan_cartesian_path(x_final, y_final, z_final, "xyz")
    robot.display_trajectory(cartesian_plan)
    robot.execute_plan(cartesian_plan)

    #print "============ Press `Enter` to orient wrist properly ..."
    #raw_input()
    robot.go_to_pose_goal()

    #print "============ Press `Enter` to close gripper ..."
    #raw_input()
    robot.publisher('c')
    print("Gripper closed, bottle grabbed")

    print "============ Press `Enter` to plan and execute motion to basket ..."
    raw_input()
    x_basket = 65 #74
    y_basket = 41 #33.5
    z_basket = 45.0
    cartesian_plan, fraction = robot.plan_cartesian_path(x_basket, y_basket, z_basket, "zxy")
    robot.display_trajectory(cartesian_plan)
    robot.execute_plan(cartesian_plan)

    print "============ Waiting for mobile robot to dock ..."
    docking_sub = rospy.Subscriber("/docked", Bool, callback_docking) 
    docked = False
    while not docked:
	docked = docking_queue.get(True)
	print("Mobile robot is docked!")
    docking_sub.unregister()

    # raw_input()
    robot.publisher('o')
    print("Gripper opened!")

    print "============ Press `Enter` to return to start position ..."
    raw_input()
    robot.go_to_joint_state([0, -pi/4, pi/4, -pi/2, 0, pi/3])

    print "============ UR5 demo complete!"
    armdone_pub = rospy.Publisher("/armdone", Bool, queue_size=1)
    armdone_pub.publish(Bool(True))
    print "Published arm done to mobile robot"
    

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

