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
from time import sleep
from move_group_python_interface_tutorial_w_gripper_wo_custom_topic import all_close, MoveGroupPythonIntefaceTutorial

import numpy as np
import keyboard
import time

class Arm(object):
    """Robot arm"""

    def __init__(self, COM_port, stack_number, UR5_object, calibration):
        ### instance variables ###

        # constants
        self.FORCE_THRESHOLD = 100

        # dimensions
        self.BRICK_HEIGHT = 82.5  # total brick height [mm]
        self.BRICK_STACKED_HEIGHT = 64.8  # brick height not including protrusions [mm]
        self.BRICK_WIDTH = 97.5  # width of brick [mm]
        self.BRICK_LENGTH = 195  # length of brick [mm]
        self.BASE_HEIGHT = 200 #262 # z height of table surface, assuming gripper is holding brick

        # offsets for stacking bricks
        self.X_OFFSET = 701.57
        self.Y_OFFSET = -12.92
        self.Z_OFFSET = 0
        self.Rx_OFFSET = 0
        self.Ry_OFFSET = 0
        self.Rz_OFFSET = 0

        # stacking offsets
        self.STACK_XY_OFFSET_VALUE = 50
        self.STACK_Z_OFFSET_VALUE = 30
        self.stack_x_offset = 0
        self.stack_y_offset = 0

        # Z rotation values for x axis direction and y axis direction
        self.RZ_X_AXIS = 0  # down table
        self.RZ_Y_AXIS = - np.pi/2 # across table

        # home position - gripper up and out of view of camera
        self.HOME_POSITION = [200, -600, 300, 0, 0, 0]

        # calibration position - zero coordinate of CV function
        self.CAL_POSITION = [200, -100, 300, 0, 0, -(np.pi/2)]
        self.calibration_flag = calibration

        # command coordinates
        self.x = self.HOME_POSITION[0]
        self.y = self.HOME_POSITION[1]
        self.z = self.HOME_POSITION[2]
        self.Rx = self.HOME_POSITION[3]
        self.Ry = self.HOME_POSITION[4]
        self.Rz = self.HOME_POSITION[5]
        self.next_z = self.z
        self.stack_coordinates = self.next_stack_position()

        # state initialization
        self.state = "INIT"
        self.prev_state = "INIT"
        self.state_start_time = time.time()
        self.state_first_run = True

        # arduino
        # print('SETTING UP ARDUINO')
        # self.arduino = serial.Serial('COM3', baudrate=115200, timeout=1)
        # self.arduino.flushInput()
        # self.arduino.flushOutput()
        # time.sleep(10)
        # print('ARDUINO INITIALIZED')
        self.grip_state = False
        self.force_reading_1 = 0 # load cell reading
        self.force_reading_2 = 0
        #self.read_arduino()

        # number of bricks stacked (1 - 16)
        self.stack_number = stack_number

        # UR5 object
        self.UR5_object = UR5_object

        # manual control
        self.X_STEP = 1
        self.Y_STEP = 1
        self.Z_STEP = 1
        self.RZ_STEP = 0.01

    def state_step(self, next_state):
        print(next_state)
        print('^^ next state, press enter to continue')
        raw_input()
        self.prev_state = self.state
        self.state = next_state
        self.state_start_time = time.time()
        self.state_first_run = True

    # def read_arduino(self):
    #     data = self.arduino.readline()
    #     try:
    #         data = data.decode()
    #         data = data.strip()
    #         data = data.split()
    #         if len(data) == 3:
    #             data = list(map(float, data))
    #             self.force_reading_1 = data[0]
    #             self.force_reading_2 = data[1]
    #             self.grip_state = data[2]
    #             return True
    #     except:
    #         return False
    #
    # def write(self, x):
    #     self.arduino.write(str(x).encode())

    def next_stack_position(self):
        if self.stack_number == 1 or self.stack_number == 9:
            x = -97.5
            y = -48.75
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_X_AXIS
            self.stack_x_offset = 0
            self.stack_y_offset = 0
            if self.state == 9:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 2 or self.stack_number == 10:
            x = -48.75
            y = -195
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_Y_AXIS
            self.stack_x_offset = 0
            self.stack_y_offset = - self.STACK_XY_OFFSET_VALUE
            if self.state == 10:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 3 or self.stack_number == 11:
            x = -48.75
            y = -390
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_Y_AXIS
            self.stack_x_offset = 0
            self.stack_y_offset = - self.STACK_XY_OFFSET_VALUE
            if self.state == 11:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 4 or self.stack_number == 12:
            x = -97.5
            y = -536.25
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_X_AXIS
            self.stack_x_offset = 0
            self.stack_y_offset = - self.STACK_XY_OFFSET_VALUE
            if self.state == 12:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 5 or self.stack_number == 13:
            x = -292.5
            y = -536.25
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_X_AXIS
            self.stack_x_offset = - self.STACK_XY_OFFSET_VALUE
            self.stack_y_offset = 0
            if self.state == 13:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 6 or self.stack_number == 14:
            x = -341.25
            y = -390
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_Y_AXIS
            self.stack_x_offset = 0
            self.stack_y_offset = self.STACK_XY_OFFSET_VALUE
            if self.state == 14:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 7 or self.stack_number == 15:
            x = -341.25
            y = -195
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_Y_AXIS
            self.stack_x_offset = 0
            self.stack_y_offset = self.STACK_XY_OFFSET_VALUE
            if self.state == 15:
                z = z + self.BRICK_STACKED_HEIGHT

        elif self.stack_number == 8 or self.stack_number == 16:
            x = -292.5
            y = -48.75
            z = self.BASE_HEIGHT + self.BRICK_STACKED_HEIGHT
            Rx = 0
            Ry = 0
            Rz = self.RZ_X_AXIS
            self.stack_x_offset = - self.STACK_XY_OFFSET_VALUE
            self.stack_y_offset = self.STACK_XY_OFFSET_VALUE
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

    def send_arm_pose_cmd(self):
        # sends position target to arm
        # if len(pose_list) == 7:
        #     pose = self.pose_from_list(pose_list)

        # if len(pose_list) == 6:
        xyz = [self.x, self.y, self.z]
        quat = self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
        new_list = xyz + quat
        pose = self.pose_from_list(new_list)
        print('planning path, press enter to execute')
        print(pose)
        waypoints = []
        waypoints.append(copy.deepcopy(pose))
        cartesian_plan, fraction = self.UR5_object.plan_cartesian_path(waypoints=waypoints)
        raw_input()
        self.UR5_object.execute_plan(cartesian_plan)

    def manual_send_arm_pose_cmd(self):
        # sends position target to arm
        # if len(pose_list) == 7:
        #     pose = self.pose_from_list(pose_list)

        # if len(pose_list) == 6:
        xyz = [self.x, self.y, self.z]
        quat = self.get_quaternion_from_euler(self.Rx, self.Ry, self.Rz)
        new_list = xyz + quat
        pose = self.pose_from_list(new_list)
        print('planning path, press enter to execute')
        print(pose)
        waypoints = []
        waypoints.append(copy.deepcopy(pose))
        cartesian_plan, fraction = self.UR5_object.plan_cartesian_path(waypoints=waypoints)
        # raw_input()
        self.UR5_object.execute_plan(cartesian_plan)
        time.sleep(0.2)

    ### state machine ###
    def main(self):

        while True:

            # timer / loop rate control
            # get arduino readings
            # self.read_arduino()
            #
            # # if load detected on gripper over 'damage' threshold, ERROR, stop
            # if self.force_reading_1 > self.FORCE_THRESHOLD or self.force_reading_2 > self.FORCE_THRESHOLD:
            #     self.state_step('ERROR')

            # if significant load detected on gripper and limit switches are not pressed, ERROR, stop

            if keyboard.is_pressed('o'):
                self.state_step('MANUAL_CONTROL')

            if self.state == "INIT":
                # initializations, calibrations etc
                if self.state_first_run:
                    # move gripper to home position
                    print('INIT START, PRESS ENTER TO CONTINUE')
                    raw_input()
                    self.x = self.HOME_POSITION[0]
                    self.y = self.HOME_POSITION[1]
                    self.z = self.HOME_POSITION[2]
                    self.Rx = self.HOME_POSITION[3]
                    self.Ry = self.HOME_POSITION[4]
                    self.Rz = self.HOME_POSITION[5]
                    self.send_arm_pose_cmd()
                    self.state_first_run = False

                self.x += 300
                self.y += 1100
                raw_input()
                self.send_arm_pose_cmd()

                self.z -= 200
                raw_input()
                self.send_arm_pose_cmd()

                self.z += 200
                raw_input()
                self.send_arm_pose_cmd()

                self.y -= 1100
                raw_input()
                self.send_arm_pose_cmd()

                self.z -= 200
                raw_input()
                self.send_arm_pose_cmd()

                self.z -= 50
                raw_input()
                self.send_arm_pose_cmd()

                self.z += 250
                raw_input()
                self.send_arm_pose_cmd()


                if time.time() >= self.state_start_time + 1:
                    if self.calibration_flag:
                        self.state_step('CALIBRATION')
                    else:
                        self.state_step('EMPTY_DWELL')

            elif self.state == "CALIBRATION":
                # gripper in calibration position
                if self.state_first_run:
                    self.x = self.CAL_POSITION[0]
                    self.y = self.CAL_POSITION[1]
                    self.z = self.CAL_POSITION[2]
                    self.Rx = self.CAL_POSITION[3]
                    self.Ry = self.CAL_POSITION[4]
                    self.Rz = self.CAL_POSITION[5]
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
                # set state to MOVE_XY_GRIP
                if time.time() >= self.state_start_time + 1:
                    self.calibration_flag = False
                    self.state_step('INIT')

            elif self.state == "MANUAL CONTROL":
                if self.state_first_run:
                    self.state_first_run = False

                # keyboard control
                if keyboard.is_pressed('w'):
                    self.x -= self.X_STEP
                if keyboard.is_pressed('a'):
                    self.x -= self.Y_STEP
                if keyboard.is_pressed('s'):
                    self.x += self.X_STEP
                if keyboard.is_pressed('d'):
                    self.x += self.Y_STEP
                if keyboard.is_pressed('u'):
                    self.x += self.Z_STEP
                if keyboard.is_pressed('j'):
                    self.x -= self.Z_STEP
                if keyboard.is_pressed('q'):
                    self.x -= self.RZ_STEP
                if keyboard.is_pressed('e'):
                    self.x += self.RZ_STEP

                self.manual_send_arm_pose_cmd()


                if keyboard.is_pressed('p'):
                    self.state_step(self.prev_state)


            elif self.state == "EMPTY_DWELL":
                # gripper at home position with no brick
                if self.state_first_run:
                    if self.stack_number >= 16:
                        print('FINISHED')
                        time.sleep(99)
                    # take image(s) of workspace
                    # determine location + orientation of brick to be picked up
                    # self.x =
                    # self.y =
                    self.next_z = 150
                    # self.Rz =
                    self.x = 300
                    self.y = 500
                    self.Rz = np.pi/4
                    self.state_first_run = False
                # set state to MOVE_XY_GRIP
                if time.time() >= self.state_start_time + 5:
                    self.state_step('MOVE_XY_GRIP')


            elif self.state == "MOVE_XY_GRIP":
                if self.state_first_run:
                    # move gripper to xy position of brick to be picked up
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
            # if gripper in correct location, set state to descend
                if time.time() >= self.state_start_time + 1:
                    self.state_step('MOVE_Z_GRIP_1')


            elif self.state == "MOVE_Z_GRIP_1":
            # descend gripper to brick to be picked up + some offset
                if self.state_first_run:
                    self.z = self.next_z + self.BRICK_HEIGHT
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
                if time.time() >= self.state_start_time + 1:
                    self.state_step('MOVE_Z_GRIP_2')


            elif self.state == "MOVE_Z_GRIP_2":
            # descend gripper to brick to be picked up
                if self.state_first_run:
                    self.z = self.next_z
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
                if time.time() >= self.state_start_time + 1:
                    self.state_step('MOVE_Z_HOME_GRIPPED')


            elif self.state == "MOVE_Z_HOME_GRIPPED":
            # move gripper up to home position z plane
                if self.state_first_run:
                    self.z = self.HOME_POSITION[2]
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
            # if switches still pressed (block not slipping out), set state to "MOVE_XY_HOME_GRIPPED"
                if time.time() >= self.state_start_time + 1:
                    self.state_step('MOVE_XY_STACK_1')


            elif self.state == "MOVE_XY_STACK_1":
            # move gripper to xy location of where brick is to be stacked, + offset
                if self.state_first_run:
                    # orient gripper to correct stacking orientation
                    self.stack_coordinates = self.next_stack_position()
                    self.x = self.stack_coordinates[0] + self.stack_x_offset
                    self.y = self.stack_coordinates[1] + self.stack_y_offset
                    self.Rz = self.stack_coordinates[5]
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
                if time.time() >= self.state_start_time + 1:
                    self.state_step('MOVE_Z_STACK_1')


            elif self.state == "MOVE_Z_STACK_1":
            # descend gripper to where brick is to be stacked, + offset
                if self.state_first_run:
                    self.z = self.stack_coordinates[2] + self.STACK_Z_OFFSET_VALUE
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
                if time.time() >= self.state_start_time + 1:
                    self.state_step('MOVE_XY_STACK_2')


            elif self.state == "MOVE_XY_STACK_2":
            # move gripper to xy location of where brick is to be stacked
                if self.state_first_run:
                    self.x = self.stack_coordinates[0]
                    self.y = self.stack_coordinates[1]
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
                if time.time() >= self.state_start_time + 1:
                    self.state_step('MOVE_Z_STACK_2')


            elif self.state == "MOVE_Z_STACK_2":
            # descend gripper to where brick is to be stacked
                if self.state_first_run:
                    self.z = self.stack_coordinates[2]
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
                if time.time() >= self.state_start_time + 1:
                    self.stack_number = self.stack_number + 1
                    self.state_step('MOVE_Z_HOME_EMPTY')


            elif self.state == "MOVE_Z_HOME_EMPTY":
            # move gripper up to home position z plane
                if self.state_first_run:
                    self.z = self.HOME_POSITION[2]
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
            # set state to "MOVE_XY_HOME_EMPTY"
                if time.time() >= self.state_start_time + 1:
                    self.state_step('MOVE_XY_HOME_EMPTY')


            elif self.state == "MOVE_XY_HOME_EMPTY":
            # move gripper in xy to get to home position
                if self.state_first_run:
                    self.x = self.HOME_POSITION[0]
                    self.y = self.HOME_POSITION[1]
                    self.z = self.HOME_POSITION[2]
                    self.Rx = self.HOME_POSITION[3]
                    self.Ry = self.HOME_POSITION[4]
                    self.Rz = self.HOME_POSITION[5]
                    self.send_arm_pose_cmd()
                    self.state_first_run = False
            # set state to "EMPTY_DWELL"
                if time.time() >= self.state_start_time + 1:
                    self.state_step('EMPTY_DWELL')
                    pass

            elif self.state == "ERROR":
            # look at previous state before error
            # move back to home position
            # try previous action again for set number of retries
            # if still unsuccessful, exit and get human intervention
                pass



def test_pose_from_list(pose_list):
    wpose = geometry_msgs.msg.Pose()
    wpose.position.x = pose_list[0] * 0.001# convert mm to m
    wpose.position.y = pose_list[1] * 0.001
    wpose.position.z = pose_list[2] * 0.001
    wpose.orientation.x = pose_list[3]
    wpose.orientation.y = pose_list[4]
    wpose.orientation.z = pose_list[5]
    wpose.orientation.w = pose_list[6]

    return wpose

def test_get_quaternion_from_euler(roll_in, pitch_in, yaw_in):
    roll = roll_in
    pitch = pitch_in + np.pi
    yaw = yaw_in + np.pi * 0.25
    print(roll, pitch, yaw)
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


if __name__ == "__main__":

    UR5 = MoveGroupPythonIntefaceTutorial()
    arm = Arm(COM_port='COM4', stack_number=1, UR5_object=UR5, calibration=True)
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


				
