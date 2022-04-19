Instructions to run the UR5

Run the folowing command in every terminal tab
source ~/catkin_ws/devel/setup.bash

Run each of the following commands in differnt terminal tabs
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=10.2.58.98 kinematics_config:='/home/ur5/Documents/my_robot_calibration.yaml' 

roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch

roslaunch ur5_e_moveit_config demo.launch

rosrun robotiq_2f_gripper_corol Robotiq2FGripperRtuNode.py /dev/ttyUSB1

rosrun moveit_tutorials move_group_python_interface_tutorial.py