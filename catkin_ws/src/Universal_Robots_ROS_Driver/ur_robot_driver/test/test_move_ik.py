#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from trac_ik_python.trac_ik import IK
from math import radians, degrees


# Q1 = [2.2,0,-1.57,0,0,0]
# Q2 = [1.5,0,-1.57,0,0,0]
# Q3 = [1.5,-0.2,-1.57,0,0,0]

# Q1 = [2.2,0,-1.57,0,0,0]
# Q2 = [1.5,0,-1.57,0,0,0]
# Q3 = [1.5,-0.2,-1.57,1.5,1.5,1.5]


class Module():
    JOINTS = 6
    

    def __init__(self):

        # self.client = None
        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        try:
            rospy.init_node("test_move", anonymous=True, disable_signals=True)
            self.client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            print "Waiting for server..."
            self.client.wait_for_server()
            print "Connected to server"

            urdf_str = rospy.get_param('/robot_description')
            self.ik_solution = [0.0]*6 
            self.ik_solver = IK("base", "tool0", urdf_string=urdf_str)
            self.qinit = [0.0]*6
            # self.iktest_pgoal = [0.3125, -0.615, -0.1562, -0.3024, 0.9401, -0.1391, 0.074]
            self.iktest_pgoal = [0.2, -0.470, 0.162, 0, 0, 0, 1]



        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

        
        # self.find_ik()
        # # print(ik_solution)
        # self.move1()
        # 1.039, -0.958, 1.258
        #     # move1()
        #     # move_repeated()
        #     #move_disordered()
        #     #move_interrupt()
        # self.control = {
        #     'i': 0, # index we are on out of the order values (0 to order-1)
        #     'order': 60, # how many we are keeping for filtering
        #     # Structured self.control['effort'][tap #, e.g. i][joint, e.g. 'right_j0']
        #     'effort': [], # effort is read-only. UR does not support direct effort/torque control for safety reasons.
        #     'position': [],
        #     'velocity': []
        # }

    def rad_to_deg(list_q):
        return [degrees(j) for j in list_q]

    def move1(self):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        g.trajectory.points = [
            JointTrajectoryPoint(positions=list(self.ik_solution), velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
            # JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            # JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        # print(self.ik_solution)
        self.client.send_goal(g)
        try:
            print('1111')
            self.client.wait_for_result()
            print('2222')
        except KeyboardInterrupt:
            self.client.cancel_goal()
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
        self.client.send_goal(g)
        self.client.wait_for_result()
        
    def move_repeated():
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        
        d = 2.0
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

    def find_ik(self):
        # for j in range(Module.JOINTS):
            # self.qinit[j] = self.control['position'][self.control['i']]['right_j'+str(j)]
        self.ik_solution = self.ik_solver.get_ik(
            self.qinit,
            self.iktest_pgoal[0],
            self.iktest_pgoal[1],
            self.iktest_pgoal[2],
            self.iktest_pgoal[3],
            self.iktest_pgoal[4],
            self.iktest_pgoal[5],
            self.iktest_pgoal[6])
        # print(self.ik_solution)
        # if ik_solution != None:
            # print([round(j, 1) for j in self.rad_to_deg(list(ik_solution))])
        # else:
            # print("None")
        # self.thread_test = threading.Timer(0.2, self.find_ik)
        # self.thread_test.start()

def main():
    # global client
    


    m = Module()
    m.find_ik()
    print(m.ik_solution)
    m.move1()

    try:
        rospy.spin()
    except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
    # except rospy.ROSInterruptException:
    #     pass
    # finally:
    #     m.thread_print.cancel()
    #     m.thread_test.cancel()

    # except KeyboardInterrupt:
    #     rospy.signal_shutdown("KeyboardInterrupt")
    #     raise

if __name__ == '__main__': main()