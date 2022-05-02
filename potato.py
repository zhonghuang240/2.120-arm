import time
import serial
import geometry_msgs.msg
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

#import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
#from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

from move_group_python_interface_tutorial_w_gripper_wo_custom_topic import all_close, MoveGroupPythonIntefaceTutorial


import numpy as np

class Arm(object):
    """Robot arm"""

    def __init__(self, COM_port, stack_number, UR5_object):
        ### instance variables ###

        # constants
        self.FORCE_THRESHOLD = 100

        # dimensions
        self.BRICK_HEIGHT = 82.6  # total brick height [mm]
        self.BRICK_STACKED_HEIGHT = 64.8  # brick height not including protrusions [mm]
        self.BRICK_WIDTH = 97.5  # width of brick [mm]
        self.BRICK_LENGTH = 195  # length of brick [mm]
        self.BASE_HEIGHT = 262 - self.BRICK_STACKED_HEIGHT # z height of table surface, assuming gripper is holding brick

        # setup offsets
        self.X_OFFSET = -506.57
        self.Y_OFFSET = 110.53
        self.Z_OFFSET = 0
        self.Rx_OFFSET = 0
        self.Ry_OFFSET = 0
        self.Rz_OFFSET = 0

        # stacking offsets
        self.STACK_X_OFFSET_VALUE = 50
        self.STACK_Y_OFFSET_VALUE = 50
        self.STACK_Z_OFFSET_VALUE = 30
        self.stack_x_offset = 0
        self.stack_y_offset = 0
        self.stack_z_offset = 0

        # gripping offsets
        self.GRIP_Z_OFFSET_VALUE = 150
        self.grip_z_offset = 0

        # Z rotation values for x axis direction and y axis direction
        self.RZ_X_AXIS = np.pi * (1/4)  # down table
        self.RZ_Y_AXIS = np.pi * (3/4) # across table

        # home position - gripper up and out of view of camera
        self.HOME_POSITION = [200, -600, 300] + self.get_quaternion_from_euler(0,0,0)

        # command coordinates
        self.x = 0.75
        self.y = 0.0
        self.z = 0.35
        self.Rx = 0.0
        self.Ry = 0.0
        self.Rz = 0.0

        # state initialization
        self.state = "INIT"
        self.prev_state = "INIT"
        self.state_start_time = time.time()
        self.state_first_run = True

        # arduino
        # self.arduino = serial.Serial('COM3', 115200, timeout=1)
        # self.arduino.flushInput()
        # self.arduino.flushOutput()
        # time.sleep(1)
        self.grip_state = False
        self.force_reading_1 = 0 # load cell reading
        self.force_reading_2 = 0

        # number of bricks stacked (1 - 16)
        self.stack_number = stack_number

        # UR5 object
        self.UR5_object = UR5_object


    def state_step(self, next_state):
        self.prev_state = self.state
        self.state = next_state
        self.state_start_time = time.time()
        self.state_first_run = True
        print(self.state)

    def read_arduino(self):
        data = self. arduino.readline()
        try:
            data = data.decode()
            data = data.strip()
            data = data.split()
            if len(data) == 3:
                data = list(map(float, data))
                return data
        except:
            return None

    def next_stack_position(self):
        if self.stack_number == 1 or self.stack_number == 9:
            x = -97.5
            y = -48.75
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_X_AXIS
            if self.state == 9:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 2 or self.stack_number == 10:
            x = -48.75
            y = -195
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_Y_AXIS
            if self.state == 10:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 3 or self.stack_number == 11:
            x = -48.75
            y = -390
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_Y_AXIS
            if self.state == 11:
                z = z + self.BRICK_STACKED_HEIGHT


        elif self.stack_number == 4 or self.stack_number == 12:
            x = -97.5
            y = -536.25
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_X_AXIS
            if self.state == 12:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 5 or self.stack_number == 13:
            x = -292.5
            y = -536.25
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_X_AXIS
            if self.state == 13:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 6 or self.stack_number == 14:
            x = -341.25
            y = -390
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_Y_AXIS
            if self.state == 14:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 7 or self.stack_number == 15:
            x = -341.25
            y = -195
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_Y_AXIS
            if self.state == 15:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 8 or self.stack_number == 16:
            x = -292.5
            y = -48.75
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_X_AXIS
            if self.state == 16:
                z = z + self.BRICK_STACKED_HEIGHT

        # offset of coordinates
        x = x + self.X_OFFSET
        y = y + self.Y_OFFSET
        z = z + self.Z_OFFSET
        Rx = Rx + self.Rx_OFFSET
        Ry = Ry + self.Ry_OFFSET
        Rz = Rz + self.Rz_OFFSET

        # returns position of next stack
        return [x, y, z, Rx, Ry, Rz]

    def get_quaternion_from_euler(self, roll_in, pitch_in, yaw_in):
        roll = roll_in
        pitch = pitch_in + np.pi * 0.5
        yaw = yaw_in + np.pi * 0.25
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def pose_from_list(self, pose_list):
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.x = pose_list[3]
        wpose.orientation.y = pose_list[4]
        wpose.orientation.z = pose_list[5]
        wpose.orientation.w = pose_list[6]
        wpose.position.x = pose_list[0] * 0.001
        wpose.position.y = pose_list[1] * 0.001 
        wpose.position.z = pose_list[2] * 0.001

        return wpose

    def send_arm_pose_cmd(self, pose_list):
        # sends position target to arm
        pose = self.pose_from_list(pose_list)
        print('planning path')
        print(pose_list)
        print(pose)
        waypoints = []
        waypoints.append(copy.deepcopy(pose))
        cartesian_plan, fraction = self.UR5_object.plan_cartesian_path(waypoints=waypoints)
        raw_input()
        self.UR5_object.execute_plan(cartesian_plan)

    ### state machine ###
    def main(self):

        while True:

            # timer / loop rate control
            # get load cell inputs
            # get limit switch inputs
            # get current pose of gripper

            # if load detected on gripper over 'damage' threshold, ERROR, stop
            # if significant load detected on gripper and limit switches are not pressed, ERROR, stop

            if self.state == "INIT":
                # initializations, calibrations etc
                if self.state_first_run:
                    # move gripper to home position
                    print('home position')
                    raw_input()
                    self.send_arm_pose_cmd(self.HOME_POSITION)
                    self.x = self.HOME_POSITION[0]
                    self.y = self.HOME_POSITION[1]
                    self.z = self.HOME_POSITION[2]
                    self.Rx = 0
                    self.Ry = 0
                    self.Rz = 0
                    self.state_first_run = False

                self.x += 300
                self.y += 1100
                new_list = [self.x, self.y, self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                raw_input()
                self.send_arm_pose_cmd(new_list)

                self.z -= 200
                new_list = [self.x, self.y, self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                raw_input()
                self.send_arm_pose_cmd(new_list)

                self.z += 200
                new_list = [self.x, self.y, self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                raw_input()
                self.send_arm_pose_cmd(new_list)

                self.y -= 1100
                new_list = [self.x, self.y, self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                raw_input()
                self.send_arm_pose_cmd(new_list)

                self.z -= 200
                new_list = [self.x, self.y, self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                raw_input()
                self.send_arm_pose_cmd(new_list)

                self.z -= 50
                new_list = [self.x, self.y, self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                raw_input()
                self.send_arm_pose_cmd(new_list)

                self.z += 250
                new_list = [self.x, self.y, self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                raw_input()
                self.send_arm_pose_cmd(new_list)


                if time.time() >= self.state_start_time + 10:
                    #self.state_step('EMPTY_DWELL')
                    pass


            elif self.state == "EMPTY_DWELL":
                # gripper at home position with no brick
                if self.state_first_run:
                    self.state_first_run = False
                # take image(s) of workspace
                # determine location + orientation of brick to be picked up
                # set state to MOVE_XY_GRIP

                if time.time() >= self.state_start_time + 10:
                    #self.state_step('MOVE_XY_GRIP')
                    pass

            elif self.state == "MOVE_XY_GRIP":
                if self.state_first_run:

                    # self.x =
                    # self.y =
                    # self.z =
                    # self.Rx = 0
                    # self.Ry = 0
                    # self.Rz =
                    # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                    # self.send_arm_pose_cmd(pose)
                    self.state_first_run = False

            # move gripper to xy position of brick to be picked up
            # orient gripper along length of brick
            # if gripper in correct location, set state to DESCEND_GRIP
                if time.time() >= self.state_start_time + 10:
                    #self.state_step('MOVE_Z_GRIP_1')
                    pass

            elif self.state == "MOVE_Z_GRIP_1":
            # descend gripper to brick to be picked up
                if self.state_first_run:

                    # self.x = self.x
                    # self.y = self.y
                    # self.z =
                    # self.Rx = self.Rx
                    # self.Ry = self.Ry
                    # self.Rz = self.Rz
                    # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                    # self.send_arm_pose_cmd(pose)

                    self.state_first_run = False
            # slow descent when near brick
            # if gripper experiences significant load without limit switches being pressed, ERROR, stop
            # stop descent when limit switches depressed, check grip is engaged
                if time.time() >= self.state_start_time + 10:
                    #self.state_step('MOVE_Z_GRIP_2')
                    pass

            elif self.state == "MOVE_Z_GRIP_2":
            # descend gripper to brick to be picked up
                if self.state_first_run:

                    # self.x = self.x
                    # self.y = self.y
                    # self.z =
                    # self.Rx = self.Rx
                    # self.Ry = self.Ry
                    # self.Rz = self.Rz
                    # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                    # self.send_arm_pose_cmd(pose)

                    self.state_first_run = False
            # slow descent when near brick
            # if gripper experiences significant load without limit switches being pressed, ERROR, stop
            # stop descent when limit switches depressed, check grip is engaged
                if time.time() >= self.state_start_time + 10:
                    #self.state_step('MOVE_Z_HOME_GRIPPED')
                    pass

            elif self.state == "MOVE_Z_HOME_GRIPPED":
            # move gripper up to home position z plane
                if self.state_first_run:

                    # self.x = self.x
                    # self.y = self.y
                    # self.z = self.HOME_POSITION[2] * 1000
                    # self.Rx = self.Rx
                    # self.Ry = self.Ry
                    # self.Rz = self.Rz
                    # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                    # self.send_arm_pose_cmd(pose)

                    self.state_first_run = False
            # if switches still pressed (block not slipping out), set state to "MOVE_XY_HOME_GRIPPED"
                if time.time() >= self.state_start_time + 10:
                    #self.state_step('MOVE_XY_STACK_1')
                    pass

            # elif self.state == "MOVE_XY_HOME_GRIPPED":
            # # move gripper in xy to get to home position
            #     if self.state_first_run:
            #
            #         # self.x = self.HOME_POSITION[0] * 1000
            #         # self.y = self.HOME_POSITION[1] * 1000
            #         # self.z = self.z
            #         # self.Rx = 0
            #         # self.Ry = 0
            #         # self.Rz =
            #         # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
            #         # self.send_arm_pose_cmd(pose)
            #
            #         self.state_first_run = False
            # # if switches still pressed (block not slipping out), set state to "GRIPPED_DWELL"
            #     if time.time() >= self.state_start_time + 10:
            #         #self.state_step('MOVE_XY_STACK_1')
            #         pass

            elif self.state == "MOVE_XY_STACK_1":
            # move gripper to xy location of where brick is to be stacked, + offset
                if self.state_first_run:
                    stack_coordinates = self.next_stack_position()
                    # self.x = stack_coordinates[0]
                    # self.y = stack_coordinates[1]
                    # self.z = self.z
                    # self.Rx = 0
                    # self.Ry = 0
                    # self.Rz = stack_coordinates[5]
                    # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                    # self.send_arm_pose_cmd(pose)

                    self.state_first_run = False
            # orient gripper to correct stacking orientation
            # (if additional camera mounted on gripper) double check gripper position, adjust if needed
            # if gripper in correct location && switches pressed, set state to "MOVE_Z_STACK"
                if time.time() >= self.state_start_time + 10:
                    #self.state_step('MOVE_Z_STACK_1')
                    pass

            elif self.state == "MOVE_Z_STACK_1":
            # descend gripper to where brick is to be stacked, + offset
                if self.state_first_run:
                    stack_coordinates = self.next_stack_position()
                    # self.x = self.x
                    # self.y = self.y
                    # self.z = stack_coordinates[2]
                    # self.Rx = self.Rx
                    # self.Ry = self.Ry
                    # self.Rz = self.Rz
                    # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                    # self.send_arm_pose_cmd(pose)

                    self.state_first_run = False
                # slow descent when near
                # detect when brick has been successfully stacked based on load cell readings (look for "step")
                # stop descent after successful stack and grip is released
                if time.time() >= self.state_start_time + 10:
                    #self.stack_number = self.stack_number + 1
                    #self.state_step('MOVE_XY_STACK_2')
                    pass

            elif self.state == "MOVE_XY_STACK_2":
            # move gripper to xy location of where brick is to be stacked
                if self.state_first_run:
                    stack_coordinates = self.next_stack_position()
                    # self.x = stack_coordinates[0]
                    # self.y = stack_coordinates[1]
                    # self.z = self.z
                    # self.Rx = 0
                    # self.Ry = 0
                    # self.Rz = stack_coordinates[5]
                    # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                    # self.send_arm_pose_cmd(pose)

                    self.state_first_run = False
                # orient gripper to correct stacking orientation
                # (if additional camera mounted on gripper) double check gripper position, adjust if needed
                # if gripper in correct location && switches pressed, set state to "MOVE_Z_STACK"
                if time.time() >= self.state_start_time + 10:
                    #self.state_step('MOVE_Z_STACK_2')
                    pass

            elif self.state == "MOVE_Z_STACK_2":
            # descend gripper to where brick is to be stacked
                if self.state_first_run:
                    stack_coordinates = self.next_stack_position()
                    # self.x = self.x
                    # self.y = self.y
                    # self.z = stack_coordinates[2]
                    # self.Rx = self.Rx
                    # self.Ry = self.Ry
                    # self.Rz = self.Rz
                    # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                    # self.send_arm_pose_cmd(pose)

                    self.state_first_run = False
                # slow descent when near
                # detect when brick has been successfully stacked based on load cell readings (look for "step")
                # stop descent after successful stack and grip is released
                if time.time() >= self.state_start_time + 10:
                    #self.stack_number = self.stack_number + 1
                    #self.state_step('MOVE_Z_HOME_EMPTY')
                    pass


            elif self.state == "MOVE_Z_HOME_EMPTY":
            # move gripper up to home position z plane
                if self.state_first_run:

                    # self.x = self.x
                    # self.y = self.y
                    # self.z = self.HOME_POSITION[2] * 1000
                    # self.Rx = self.Rx
                    # self.Ry = self.Ry
                    # self.Rz = self.Rz
                    # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                    # self.send_arm_pose_cmd(pose)

                    self.state_first_run = False
            # set state to "MOVE_XY_HOME_EMPTY"
                if time.time() >= self.state_start_time + 10:
                    #self.state_step('MOVE_XY_HOME_EMPTY')
                    pass

            elif self.state == "MOVE_XY_HOME_EMPTY":
            # move gripper in xy to get to home position
                if self.state_first_run:

                    # self.x = self.HOME_POSITION[0] * 1000
                    # self.y = self.HOME_POSITION[1] * 1000
                    # self.z = self.z
                    # self.Rx = 0
                    # self.Ry = 0
                    # self.Rz =
                    # pose = [self.x, self.y ,self.z] + self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
                    # self.send_arm_pose_cmd(pose)
                    self.state_first_run = False
            # set state to "EMPTY_DWELL"
                if time.time() >= self.state_start_time + 10:
                    #self.state_step('EMPTY_DWELL')
                    pass

            elif self.state == "ERROR":
            # look at previous state before error
            # move back to home position
            # try previous action again for set number of retries
            # if still unsuccessful, exit and get human intervention
                pass



def test_pose_from_list(pose_list):
    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = pose_list[0] / 1000 # convert mm to m
    wpose.position.y = pose_list[1] / 1000
    wpose.position.z = pose_list[2] / 1000
    wpose.orientation.x = pose_list[3]
    wpose.orientation.y = pose_list[4]
    wpose.orientation.z = pose_list[5]
    wpose.orientation.w = pose_list[6]

    return wpose

def test_get_quaternion_from_euler(roll_in, pitch_in, yaw_in):
    roll = roll_in
    pitch = pitch_in + np.pi
    yaw = yaw_in
    print(roll, pitch, yaw)
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


if __name__ == "__main__":

    UR5 = MoveGroupPythonIntefaceTutorial()
    arm = Arm(COM_port='COM4', stack_number=1, UR5_object=UR5)
    arm.main()

    # wpose = geometry_msgs.msg.Pose()
    # wpose.orientation.x = 0.0
    # wpose.orientation.y = 0.7071
    # wpose.orientation.z = 0.0
    # wpose.orientation.w = 0.7071
    # wpose.position.x = 0.3
    # wpose.position.y = -0.075
    # wpose.position.z = 0.5


    # waypoints_1 = []
    # waypoints_1.append(copy.deepcopy(wpose))

    # cartesian_plan, fraction = UR5.plan_cartesian_path(waypoints=waypoints_1)
    # UR5.execute_plan(cartesian_plan)

    # test_list = [200, 300, 500] + test_get_quaternion_from_euler(0, 0, 0)
    # print(test_list)
    # print(test_list[0] / 1000)
    #

    # print('test')
    # raw_input()
    # UR5.execute_plan(cartesian_plan)
    # print('test2')
    # raw_input()


				
