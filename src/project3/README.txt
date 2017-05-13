------------
Introduction
------------
This program implements CS791 Project 3. It is capable of accepting a "Piece Command" message per the definition posted on slack. When it receives said message, it will use MoveIt! to attempt to actuate the Baxter's left arm and perform the following actions:
* Move the gripper 0.05 meters above "move_from"     (ready to pick up the piece)
* Move the gripper to touch the board at "move_from" ("pickup" the piece")
* Move the gripper 0.05 meters above "move_from"     (move away from other pieces)
* Move the gripper 0.05 meters above "move_to"       (prepare to set down the piece)
* Move the gripper to touch the board at "move_to"   (set down the piece)
* Move the gripper 0.05 meters above "move_to"       (move away from other pieces

-----------------
Quick Start Guide
-----------------
* Throughout this guide I assume you update the package and CMakeLists and source your ROS environment appropriately
* Copy project3-node.py from my submission to a ROS package, such as: ~/ros_ws/src/project3/scripts/project3-node.py (if your package has a different name you'll need to update lines 7 and 69 of the code to your package name)
* Define a message called "ChessCommand.msg" in your message folder, such as ~/ros_ws/src/project3/msg (if you use a different message name you'll have to update line 69 of the code to your message name).
* Make project3-node.py executable, for example run: chmod u+x project3-node.py
* Start the following existing ros nodes/launch files (details ommitted, I assume you know how to do this): 
** baxter_project3.launch
** baxter_interface:joint_trajectory_action_server.py
** baxter_moveit_config:move_group_hw3.launch
* Enable to robot (rosrun baxter_tools enable_robot.py -e)
* Untuck the robot's arms (rosrun baxter_tools tuck_arms.py -u)
* Start my node, if you are using the same project names from above: (rosrun project3 project3-node.py)
* My node will output the help text "Ready to receive commands" followed by a list of conventions when it is ready
* Send my node a "Piece Message" message on the channel "piece_movement" using the following terminal command example as a guide (update node names and source/goal as needed):
** rostopic pub /piece_movement project3/ChessCommand '{header: {seq: 0, stamp: 0, frame_id: 'none'},move_from: "b2", move_to: "g7"}'
* My node will command MoveIt! to execute the trajectory provided. It will print out any MoveIt! error messages
* After completing a trajectory, the code will indicate it it has finished and will be ready to accept a new command

-----------
Assumptions
-----------
* I assume the robot arms are in the "untucked" position when the node starts. If this criteria is not met the arm may pass through the checkerboard (no collision detection implemented as it is not required)
* I assume your installation of MoveIt! and all other tools are "fresh" i.e. up to date (on ROS Indigo on Ubuntu 14.04) and unmodified.

----------------------
Behavior Specification
----------------------
The following actions are considered abnormal and lead to special behavior:
* Commanding a piece outside of the range [a-h],[1-9] causes unspecified behavior
* Sending a new command before a prior command is completed causes undefined behavior
* Starting the node without the proper dependencies started (see quick start guide) causes undefined behavior

--------------
Known "issues"
--------------
There are a few things (mostly previously discussed in class) which the node cannot due because of the assignment requirements or MoveIt! limitations (not due to limitations in my code):
* The robot will not find a valid path to cells a1, b1, and c1 because they are not inside the robot's operational space. It will attempt to plan for 5 seconds (by default, your MoveIt! configuration could override this), output a MoveIt! error, and then continue to execute the rest of the trajectory
* My node does not manipulate the gripper (per the instruction that this is not required)
* Depending on factors such as CPU speed, MoveIt! may sometimes fail to execute a trajectory and will abort it, especially near the end of the path. This is a limitation of MoveIt! which may be addressed with different tolerances. The behavior has not been observed to significantly affect the output of the system.

------------
Dependencies
------------
My code depends on the following python packages:
* project3.msg
* sys
* copy
* rospy
* tf
* moveit_commander
* moveit_msgs.msg
* math
* threading
* geometry_msgs.msg
