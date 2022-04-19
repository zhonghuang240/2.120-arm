#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from trac_ik_python.trac_ik import IK
from math import degrees
import math

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Q1 = [2.2,0,-1.57,0,0,0]
# Q2 = [1.5,0,-1.57,0,0,0]
# Q3 = [1.5,-0.2,-1.57,0,0,0]

# Q1 = [2.2,0,-1.57,0,0,0]
# Q2 = [1.5,0,-1.57,0,0,0]
# Q3 = [1.5,-0.2,-1.57,1.5,1.5,1.5]


def move1():
    find_ik()
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=ik_solution, velocities=[0]*6, time_from_start=rospy.Duration(7.0))]
        # JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        # JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    g.trajectory.points = [
        JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    client.wait_for_result()
    
def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    
    d = 4.0
    g.trajectory.points = []
    for i in range(10):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    
    client.send_goal(g)
    time.sleep(2.0)
    print "Interrupting"
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def find_ik():
    # for j in range(Module.JOINTS):
        # self.qinit[j] = self.control['position'][self.control['i']]['right_j'+str(j)]
    global ik_solution
    ik_solution = ik_solver.get_ik(
        qinit,
        iktest_pgoal[0],
        iktest_pgoal[1],
        iktest_pgoal[2],
        iktest_pgoal[3],
        iktest_pgoal[4],
        iktest_pgoal[5],
        iktest_pgoal[6])
    for x in list(ik_solution):
        print(degrees(x))
def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
def main():
    global client
    global ik_solution, qinit, iktest_pgoal
    global ik_solver
    ik_solution = [0.0]*6
    qinit = [0.0]*6
            # self.iktest_pgoal = [0.3125, -0.615, -0.1562, -0.3024, 0.9401, -0.1391, 0.074]
    # iktest_pgoal = [0.15, -0.470, 0.162, 0, 0, 0, 1]
    # iktest_pgoal = [0.15, -0.470, 0.133, 0, 0, 0, 1]
    # iktest_pgoal = [0.1, 0.1, 0.133, 0, 0, 0, 1]
    # iktest_pgoal = [0.24, -0.22, 0.383, 0, 0, 0, 1]

    # iktest_pgoal = [0.2, -0.15, 0.5, -0.1178, 0.4547, 0.6361, -0.6122]
    # ax = 0
    # ay = 0
    # az = 1

    # angle = 2*math.pi
    # # angle = 0
    # qx = ax*math.sin(angle/2)
    # qy = ay*math.sin(angle/2)
    # qz = az*math.sin(angle/2)
    # qw = math.cos(angle/2)

    qx = 1
    qy = 0
    qz = 0
    qw = 0


    # iktest_pgoal = [0.0, -0.2, 0.5, 0.9239, 0.3827, 0, 0]
    # iktest_pgoal = [0.0, -0.2, 0.5, qx, qy, qz, qw]
    # iktest_pgoal = [0.0, -0.2, 0.65, qx, qy, qz, qw]

    # goal_bottlehover = [-0.75, 0.0, 0.15, qx, qy, qz, qw]

    # iktest_pgoal = [-0.75, 0.0, 0.15, qx, qy, qz, qw]
    # iktest_pgoal = [-0.75, 0.0, 0.10, qx, qy, qz, qw]

    # iktest_pgoal = [-0.5, -0.45, 0.40, qx, qy, qz, qw]


    client = None

    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"

        urdf_str = rospy.get_param('/robot_description')
        ik_solver = IK("base", "tool0", urdf_string=urdf_str)
        # print('************************************')
        # print(urdf_str)
        iktest_pgoal = [-0.75, 0.0, 0.15, qx, qy, qz, qw]
        move1()
        iktest_pgoal = [-0.75, 0.0, 0.10, qx, qy, qz, qw]
        move1()
        iktest_pgoal = [-0.5, -0.45, 0.40, qx, qy, qz, qw]
        move1()
        print(ik_solution)
        # move_repeated()
        #move_disordered()
        #move_interrupt()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()