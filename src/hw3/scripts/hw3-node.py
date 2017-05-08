#!/usr/bin/python
import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import math
import threading
import hw3.msg #you may need to replace this with the message name which defines ChessCommand


shutdown_threads = False
def publishTransforms(tf_broadcaster,current_piece_trans,above_current_piece_trans,above_desired_piece_trans,desired_piece_trans,piece_rot,now):
	while not shutdown_threads:
		tf_broadcaster.sendTransform(current_piece_trans,piece_rot,now, "/chesspiece","/chessboard")
		tf_broadcaster.sendTransform(above_current_piece_trans,piece_rot,now, "/above_chesspiece","/chessboard")
		tf_broadcaster.sendTransform(above_desired_piece_trans,piece_rot,now, "/above_chesspiece_target","/chessboard")
		tf_broadcaster.sendTransform(desired_piece_trans,piece_rot,now, "/chesspiece_target","/chessboard")
		now = rospy.Time.now()
		rospy.sleep(0.1)
		
def addPose(from_frame,to_frame,now,waypoints):
	pose_target = geometry_msgs.msg.Pose()
	tf_listener.waitForTransform(from_frame,to_frame, now, rospy.Duration(5.0))
	(pos,rot) = tf_listener.lookupTransform(from_frame,to_frame, now)
	pose_target.position.x = pos[0]
	pose_target.position.y = pos[1]
	pose_target.position.z = pos[2]
	pose_target.orientation.w = rot[3]
	pose_target.orientation.x = rot[0]
	pose_target.orientation.y = rot[1]
	pose_target.orientation.z = rot[2]
	waypoints.append(copy.deepcopy(pose_target))
	return waypoints

def ChessCommandCallback(msg,input_command):
	input_command[0] = msg.move_from
	input_command[1] = msg.move_to
	input_command[2] = True


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('CS791_Project_3', anonymous=True)

#sleep until rospy knows the time
now = rospy.Time.now()
while now == rospy.Time(0):
	now = rospy.Time.now()

#Setup tf and MoveIt!
tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=10)

#Setup subscriber for input commands
input_command = ["a1","a1",False]
rospy.Subscriber("piece_movement", hw3.msg.ChessCommand, ChessCommandCallback, callback_args=input_command)

#get the position of the chessboard with respect to the base (I assume it doesn't change during the simulation')
have_transform = False
while not have_transform:
	try:
		(board_trans, board_rot) = tf_listener.lookupTransform('base', 'chessboard', rospy.Time())
		have_transform = True
	except(tf.LookupException):
		continue
# the quaternion for the chessboard is pointing the opposite direction of where the hand needs to
#	point, so define the link from /chessboard to /chesspiece to be pitched 180 degrees
#The third parameter yaws
#The first paramter rotates about red (roll)
piece_rot = (1,0,0,0)
board_square_size = 0.02 # meters
print("")
print("")
print("Ready to receive commands")
print("Conventions: * You must publish a tf transform /chessboard which is located at the center of square a1")
print("             * The chessboard is facing toward +Z, so the gripper points toward -Z")
print("             * The chessboard is "+str(board_square_size)+" meters per square, so be sure baxter can reach it.")
print("               A good position is (0.5,0.3,0.0) at orientation (0,0,-sqrt(2)/2,sqrt(2)/2)")
print("             * Publish the ChessCommand message on topic piece_movement")
while not rospy.is_shutdown():
	#Wait for a new command
	if input_command[2] != True:
		continue
	now = rospy.Time.now()
	shutdown_threads = False
	input_command[2] = False
	print("Received command to: "+str(input_command[0:2]))
	waypoints = []
	#Parse the command
	current_piece_index = (ord(input_command[0][0])-ord("a"),ord(input_command[0][1])-ord("1"))
	desired_piece_index = (ord(input_command[1][0])-ord("a"),ord(input_command[1][1])-ord("1"))

	#The translation is just (size of board square) * (square index) in each direction
	current_piece_trans = (board_square_size * current_piece_index[0],board_square_size * current_piece_index[1],0)
	above_current_piece_trans = (board_square_size * current_piece_index[0],board_square_size * current_piece_index[1],0.1)
	above_desired_piece_trans = (board_square_size * desired_piece_index[0],board_square_size * desired_piece_index[1],0.1)
	desired_piece_trans = (board_square_size * desired_piece_index[0],board_square_size * desired_piece_index[1],0)
	#publish these transforms to tf (done on a separate thread so that they are available throughout the planning)
	t = threading.Thread(target=publishTransforms,args=(tf_broadcaster,current_piece_trans,above_current_piece_trans,above_desired_piece_trans,desired_piece_trans,piece_rot,now))
	t.start()

	#sleep for a bit to be sure the transforms publish
	rospy.sleep(0.2)
	now = rospy.Time.now()

	#Add a waypoint for each pose
	waypoints = addPose('/world','/above_chesspiece',        now+rospy.Duration(0.5),waypoints)
	waypoints = addPose('/world','/chesspiece',              now+rospy.Duration(0.5),waypoints)
	waypoints = addPose('/world','/above_chesspiece',        now+rospy.Duration(0.5),waypoints)
	waypoints = addPose('/world','/above_chesspiece_target', now+rospy.Duration(0.5),waypoints)
	waypoints = addPose('/world','/chesspiece_target',       now+rospy.Duration(0.5),waypoints)
	waypoints = addPose('/world','/above_chesspiece_target', now+rospy.Duration(0.5),waypoints)

	#Execute the movement
	for waypoint in waypoints:
		group.set_pose_target(waypoint)
		print("--Starting waypoint plan")
		plan1 = group.plan()
		print("----Starting waypoint command")
		group.go(wait=True)
		print("------Finished waypoint command")
	shutdown_threads = True
	print("Piece movement is complete")
shutdown_threads = True
moveit_commander.roscpp_shutdown()
print "Shutting down"
quit()