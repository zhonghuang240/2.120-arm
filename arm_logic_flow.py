import time
import serial
import numpy as np
import geometry_msgs.msg


class Arm:
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

		# setup errors
		self.X_OFFSET = -506.57
		self.Y_OFFSET = 110.53
		self.Z_OFFSET = 0
		self.Rx_OFFSET = 0
		self.Ry_OFFSET = 0
		self.Rz_OFFSET = 0

		# Z rotation values for x axis direction and y axis direction
		self.RZ_X_AXIS = np.pi * (1/4)  # down table
		self.RZ_Y_AXIS = np.pi * (3/4) # across table

		# home position - gripper up and out of view of camera
		self.HOME_POSITION = [0.75, 0.0, 0.15, 0.0, 0.7071, 0.0, 0.7071]

		# state initialization
		self.state = "INIT"
		self.prev_state = "INIT"
		self.state_start_time = time.time()

		# arduino
		self.arduino = serial.Serial(COM_port, 9800, timeout=1)
		self.grip_state = False
		self.force_reading = 0 # load cell reading

		# number of bricks stacked (1 - 16)
		self.stack_number = stack_number

		# UR5 object
		self.UR5_object = UR5_object

		# xy and height settings



	def state_step(self, next_state):
		self.prev_state = self.state
		self.state = next_state
		self.state_start_time = time.time()

	def read_arduino(self):
		# read grip state
		# read force error
		# read collision error

	def get_quaternion_from_euler(self, roll, pitch, yaw):
		"""
		Convert an Euler angle to a quaternion.

		Input
		  :param roll: The roll (rotation around x-axis) angle in radians.
		  :param pitch: The pitch (rotation around y-axis) angle in radians.
		  :param yaw: The yaw (rotation around z-axis) angle in radians.

		Output
		  :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
		"""
		qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
			yaw / 2)
		qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
			yaw / 2)
		qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
			yaw / 2)
		qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
			yaw / 2)

		return [qx, qy, qz, qw]

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
		return [x,y,z].extend(self.get_quaternion_from_euler(Rx, Ry, Rz))

	def pose_from_list(self, pose_list):
		wpose = geometry_msgs.msg.Pose()

		wpose.position.x = pose_list[0] /1000
		wpose.position.y = pose_list[1] /1000
		wpose.position.z = pose_list[2] /1000
		wpose.orientation.x = pose_list[3]
		wpose.orientation.y = pose_list[4]
		wpose.orientation.z = pose_list[5]
		wpose.orientation.w = pose_list[6]

		return wpose

	def send_arm_pose_cmd(self, pose_list, velocity):
		# sends position target to arm
		pose = self.pose_from_list(pose_list)
		self.UR5_object.go_to_pose_goal(pose)

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
				# move gripper to home position
				self.send_arm_pose_cmd(self.HOME_POSITION)

				if time.time() >= self.state_start_time + 10:
					# set state to EMPTY_DWELL
					self.state_step('EMPTY_DWELL')

			elif self.state == "EMPTY_DWELL":
				# gripper at home position with no brick
				# take image(s) of workspace
				# determine location + orientation of brick to be picked up
				# set state to MOVE_XY_GRIP

			elif self.state == "MOVE_XY_GRIP":
				# move gripper to xy position of brick to be picked up
				# orient gripper along length of brick
				# (if additional camera mounted on gripper) double check gripper position, adjust if needed
				# if gripper in correct location, set state to DESCEND_GRIP

			elif self.state == "MOVE_Z_GRIP":
				# descend gripper to brick to be picked up
				# slow descent when near brick
				# if gripper experiences significant load without limit switches being pressed, ERROR, stop
				# stop descent when limit switches depressed
				# if limit switches pressed with no significant load on gripper, set state to "ENGAGE_GRIP"

			elif self.state == "ENGAGE_GRIP":
				# actuate both linear actuators on gripper to grip block
				# lift gripper slightly in z to test if grip holds
				# if limit switches still pressed after lift (grip is secure), set state to "MOVE_Z_HOME_GRIPPED"

			elif self.state == "MOVE_Z_HOME_GRIPPED":
				# move gripper up to home position z plane
				# if switches still pressed (block not slipping out), set state to "MOVE_XY_HOME_GRIPPED"
			
			elif self.state == "MOVE_XY_HOME_GRIPPED":
				# move gripper in xy to get to home position
				# if switches still pressed (block not slipping out), set state to "GRIPPED_DWELL"

			elif self.state == "GRIPPED_DWELL":
				# gripper at home position with block gripped
				# take image(s) of workspace 
				# determine location + orientation of where to place brick
				# if switches still pressed (block not slipping out), set state to "MOVE_XY_STACK"

			elif self.state == "MOVE_XY_STACK":
				# move gripper to xy location of where brick is to be stacked
				# orient gripper to correct stacking orientation
				# (if additional camera mounted on gripper) double check gripper position, adjust if needed
				# if gripper in correct location && switches pressed, set state to "MOVE_Z_STACK"

			elif self.state == "MOVE_Z_STACK":
				# descend gripper to where brick is to be stacked 
				# slow descent when near
				# detect when brick has been successfully stacked based on load cell readings (look for "step")
				# stop descent after successful stack
				# set state to "RELEASE_GRIP"

			elif self.state == "RELEASE_GRIP":
				# actuate both linear actuators on gripper to release grip
				# lift gripper slightly in z to test if grip has been released
				# if limit switches are open with no load on gripper
				# register location of where block has been stacked
				# set state to "MOVE_Z_HOME_EMPTY"

			elif self.state == "MOVE_Z_HOME_EMPTY":
				# move gripper up to home position z plane
				# set state to "MOVE_XY_HOME_EMPTY"

			elif self.state == "MOVE_XY_HOME_EMPTY":
				# move gripper in xy to get to home position
				# set state to "EMPTY_DWELL"

			elif self.state == "ERROR"
				# look at previous state before error
				# move back to home position
				# try previous action again for set number of retries
				# if still unsuccessful, exit and get human intervention

if __name__ == "__main__":


	arm = Arm(COM_port='COM4', stack_number=1,)
	arm.main()

				
