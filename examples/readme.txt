Hi All,

Several resources have been added to the term project module to help get you started. This includes some reference code for the UR5 from last year’s term project.

Control the UR5 arm using the MoveIt1 (Links to an external site.) motion planning package for ROS Melodic

Move_group_python_interface_tutorial_template_2022.py
This file gives an example of how you can control the UR5 using MoveIt1 by commanding desired poses in task space and by commanding specific joint angles.

Create custom Boolean topic to help coordinate between UR5 and your gripper

Subscriber_UR_example_student.py
Keyboard_process.py
PublisherSubscriber_example.docx: this file describes the details of the above python scripts

The term project last year involved docking the mobile robot in a specified area. The UR5 had to wait until this docking operation was complete before starting its own task. The "/test/docking" topic was set to True once the docking was complete. The UR5 subscribed to this topic and waited for it to be set to True before starting its task.

Hough transform for circle detection using D435i cameras (Links to an external site.)

Bottle_detection.py: The cameras for the UR5 are Intel D435i cameras. This script reads the 2D camera data from the topic "/camera/color/image_raw" and implements the Hough Transform to detect circles.