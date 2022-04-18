class Arm:
	"""Robot arm"""

	def __init__(self):
		### instance variables ### 
		self.state = "INIT"

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
				# set state to EMPTY_DWELL

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



				
