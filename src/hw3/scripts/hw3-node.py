#!/usr/bin/python
#import rospy
#from std_msgs.msg import String
#import numpy as np
#from visualization_msgs.msg import *
#import baxter_core_msgs.msg
#from sensor_msgs.msg import *
#import math
#import pudb
#import copy
#
#def matToQuat(mat):
#	#Reference: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
#	trace = mat[0,0] + mat[1,1] + mat[2,2]
#	if trace > 0:
#		s = math.sqrt(trace + 1.0) * 2
#		w = 0.25 * s
#		x = (mat[2,1] - mat[1,2]) / s
#		y = (mat[0,2] - mat[2,0]) / s
#		z = (mat[1,0] - mat[0,1]) / s
#	elif ((mat[0,0] > mat[1,1]) and (mat[0,0] > mat[2,2])):
#		s = math.sqrt(1.0 + mat[0,0] - mat[1,1] - mat[2,2]) * 2
#		w = (mat[2,1] - mat[1,2]) / s
#		x = 0.25 * s
#		y = (mat[0,1] + mat[1,0]) / s
#		z = (mat[0,2] + mat[2,0]) / s
#	elif (mat[1,1] > mat[2,2]):
#		s = math.sqrt(1.0 + mat[1,1] - mat[0,0] - mat[2,2]) * 2
#		w = (mat[0,2] - mat[2,0]) / s
#		x = (mat[0,1] + mat[1,0]) / s
#		y = 0.25 * s
#		z = (mat[1,2] + mat[2,1]) / s
#	else:
#		s = math.sqrt(1.0 + mat[2,2] - mat[0,0] - mat[1,1]) * 2
#		w = (mat[1,0] - mat[0,1]) / s
#		x = (mat[0,2] + mat[2,0]) / s
#		y = (mat[1,2] + mat[2,1]) / s
#		z = 0.25 * s
#	return [w,x,y,z]
#
#def GetHTranslateMatrix(x,y,z):
#	#Reference: http://web.cs.iastate.edu/~cs577/handouts/homogeneous-transform.pdf
#	h = np.eye(4,4)
#	h[0,3] += x
#	h[1,3] += y
#	h[2,3] += z
#	return h
#def GetHRotationMatrix(roll,pitch,yaw):
#	#Reference: https://en.wikipedia.org/wiki/Rotation_matrix
#	roll_mat = np.eye(4,4)
#	roll_mat[1,1] =  math.cos(roll)
#	roll_mat[1,2] = -math.sin(roll)
#	roll_mat[2,1] =  math.sin(roll)
#	roll_mat[2,2] =  math.cos(roll)
#
#	pitch_mat = np.eye(4,4)
#	pitch_mat[0,0] =  math.cos(pitch)
#	pitch_mat[0,2] =  math.sin(pitch)
#	pitch_mat[2,0] = -math.sin(pitch)
#	pitch_mat[2,2] =  math.cos(pitch)
#
#	yaw_mat = np.eye(4,4)
#	yaw_mat[0,0] =  math.cos(yaw)
#	yaw_mat[0,1] = -math.sin(yaw)
#	yaw_mat[1,0] =  math.sin(yaw)
#	yaw_mat[1,1] =  math.cos(yaw)
#
#	return np.dot(yaw_mat,np.dot(pitch_mat, roll_mat))
#ZERO_VEL_RETRANSMISSIONS = 5
#class MyRobot:
#
#	def __init__(self, filename, linear_scale):
#		f = open(filename)
#		botSpecs = eval(f.read())
#		f.close()
#		self.parseDHFile(botSpecs)
#		self.linear_scale = linear_scale
#		#verify the joints are in order.
#		for joint_index in range(0,len(self.jointNames)-1):
#			if self.jointNames[joint_index] != self.jointParent[joint_index+1]:
#				print "This program currently requires your joints to be in order. Please reorder your input file so that the parent of the N'th joint is the name of the N-1'th joint"
#				print self.jointNames[joint_index+1] + " did not match " + self.jointParent[joint_index]
#				exit()
#		self.desiredDhParams = copy.deepcopy(self.dhParams)
#		self.currentVelocity = [0] * len(self.dhParams)
#		self.lastDhParams = self.dhParams
#		self.a0 = [0] * len(self.dhParams)
#		self.a1 = [0] * len(self.dhParams)
#		self.a2 = [0] * len(self.dhParams)
#		self.a3 = [0] * len(self.dhParams)
#		self.tInitial = [0] * len(self.dhParams)
#		self.tFinal = [0] * len(self.dhParams)
#		self.time_per_movement = 5.0
#		self.supressCommand = 0
#
#	def parseDHFile(self, DHparams):
#		# list of 4-tuples (a, alpha, d, theta)
#		if type(DHparams) == dict:
#			self.setDHFromDict(DHparams)
#		elif type(DHparams) == list:
#			self.dhParams = DHparams
#		else:
#			throw("Invalid Format")
#
#
#		self.numLinks = len(self.dhParams)
#		self.q = np.zeros((self.numLinks, 1))
#
#	def setDHFromDict(self, DHdict):
#		self.dhParams = []
#		self.jointNames = []
#		self.jointParent = []
#		for joint in DHdict["Joints"]:
#			self.dhParams.append(joint["DH_parameters"])
#			#Get the joint names
#			self.jointNames.append(joint["joint_name"])
#			self.jointParent.append(joint["parent_name"])
#
#	def getTranslation(self, i,j):
#		return self.getT(i,j)[:-1,-1]
#
#	def getRotationMatrix(self, i,j):
#		return self.getT(i,j)[:3,:3]
#
#
#	# constructA: constructs the transform matrix for the ith joint using the D-H parameters
#	# i: the joint to construct the homogeneous transform for
#	# returns the transform from the ith to the i+1th
#	def constructA(self, i):
#		#d, theta, a ,alpha= self.dhParams[i]
#		a, alpha, d ,theta= self.dhParams[i]
#		h_transform = np.eye(4)
#		# Method:
#		#	Starting from the homogeneous matrix above, treat it as a vector
#		#	that points along x. Translate it by d units along z, Rotate it by theta
#		#	radians about z, translate that by a units along x, and rotate that by alpha units about x.
#		#	Note that this could be done more directly (i.e. instead of being
#		#	composed from elementry operations) but then it would be less clear
#		#reference for numpy.dot: https://docs.scipy.org/doc/numpy/reference/generated/numpy.dot.html
#		a *= self.linear_scale
#		d *= self.linear_scale
#		#h_transform = np.dot(GetHTranslateMatrix(0,0,d),h_transform)
#		#h_transform = np.dot(GetHRotationMatrix(0,0,theta),h_transform)
#		##rotate a to the new orientation
#		#a_rotated = np.dot(GetHRotationMatrix(0,0,theta),GetHTranslateMatrix(a,0,0))
#		#h_transform = np.dot(GetHTranslateMatrix(a_rotated[0,3] ,a_rotated[1,3],a_rotated[2,3]),h_transform)
#		#h_transform = np.dot(GetHRotationMatrix(alpha,0,0),h_transform)
#
#
#		#alternative method #2: the full matrix. This produces different results. I'm not sure why.
#		#Reference: http://web.aeromech.usyd.edu.au//MTRX4700/Course_Documents/material/lectures/L2_Kinematics_Dynamics_2013.pdf
#		h_transform[0,0] = math.cos(theta);
#		h_transform[0,1] = -math.sin(theta)*math.cos(alpha);
#		h_transform[0,2] = math.sin(theta)*math.sin(alpha);
#		h_transform[0,3] = a * math.cos(theta);
#		
#		h_transform[1,0] = math.sin(theta);
#		h_transform[1,1] = math.cos(theta)*math.cos(alpha);
#		h_transform[1,2] = -math.cos(theta)*math.sin(alpha);
#		h_transform[1,3] = a * math.sin(theta);
#		
#		h_transform[2,1] = math.sin(theta);
#		h_transform[2,2] = math.cos(theta);
#		h_transform[2,3] = d;
#
#
#		#alternative method #3: the simple revolute-only method from Dave's slides
#		#h_transform[0,0] = math.cos(theta);
#		#h_transform[0,1] = -math.sin(theta)
#		#h_transform[0,3] = a * math.cos(theta)
#		#
#		#h_transform[1,0] = math.sin(theta)
#		#h_transform[1,1] = math.cos(theta)
#		#h_transform[1,3] = a * math.sin(theta)
#
#
#		return h_transform
#
#
#	# getT: computes a homogeneous transform from the ith frame to the jth frame by iteratively multiplying each homogeneous transform from i to j
#	# i is the index of the starting coordinate frame
#	# j is the index of the ending coordinate frame
#	# returns: a homogeneous transform matrix, T
#	def getT(self, i, j):
#		h_cummulative = np.eye(4,4)
#		for index in range (i,j+1):
#			h_cummulative = np.dot(h_cummulative, self.constructA(index))
#		return h_cummulative
#
##reference joint state message: http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
#def JointStateCallback(msg, bot):
#	print("<new coefficients:>")
#	for input_joint_index, input_joint_name in enumerate(msg.name):
#		for joint_index, joint_name in enumerate(bot.jointNames):
#			if input_joint_name == joint_name:
#				#change this joint's position
#				bot.desiredDhParams[joint_index][3] = msg.position[input_joint_index]
#				bot.tInitial[joint_index] = rospy.get_rostime().to_sec()
#				bot.tFinal[joint_index] = bot.tInitial[joint_index] + bot.time_per_movement
#
#				#First, In the cubic motion equations, there are four parameters: a0, a1, a2, and a3, which are the coefficients of the polynomial, i.e.:
#				#	a3*t^3 + a2*t^2 + a1*t + a0=qf				(equation 1)
#				#Where qf is the final position of the system
#				#Second, qs is the initial position of the system, which can be proved to be equal to a0 (in equation 1 set t=0)
#				#Third, derivative_qi is the velocity at t=0, which is equivalent to a1 (set t=0 and compute the derivative equation 1)
#				#Finally, the derivative of equation 1 is:
#				#	3*a3*t^2 + 2*a2*t + a1 = derivative_qf		(equation 2)
#				#Where derivative_qf is the final velocity of the system, which is often desired to be 0
#				bot.a0[joint_index]=bot.dhParams[joint_index][3] #bot.dhParams stores the current believed state of the system
#				bot.a1[joint_index]= bot.currentVelocity[joint_index]
#				bot.qf=bot.desiredDhParams[joint_index][3]
#				bot.derivative_qf = 0
#
#				#Now we have two equations (equation 1 and equation 2) and two unknowns (a2 and a3)
#				#Numpy's linalg.solve expects the variables to be on the left, so format the equations as:
#				#	a3*t^3 + a2*t^2 = qf - a1*t - a0
#				#	3*a3*t^2 + 2*a2*t = derivative_qf - a1
#				motion_equation_lhs = np.array([[1*math.pow(bot.time_per_movement,3.0),1.0*math.pow(bot.time_per_movement,2.0)],[3.0*math.pow(bot.time_per_movement,2.0),2.0*bot.time_per_movement]])
#				motion_equation_rhs = np.array([bot.qf-bot.a1[joint_index]*bot.time_per_movement-bot.a0[joint_index],bot.derivative_qf-bot.a1[joint_index]])
#				motion_equation_result = np.linalg.solve(motion_equation_lhs, motion_equation_rhs)
#				bot.a3[joint_index] = motion_equation_result[0]
#				bot.a2[joint_index] = motion_equation_result[1]
#
#				print("a3["+str(joint_index)+"]="+str(bot.a3[joint_index])+" a2["+str(joint_index)+"]="+str(bot.a2[joint_index])+" a1["+str(joint_index)+"]="+str(bot.a1[joint_index])+" a0["+str(joint_index)+"]="+str(bot.a0[joint_index]))
#				bot.supressCommand = ZERO_VEL_RETRANSMISSIONS
#	print("<end new coefficients>")
#
#
#
#BAXTER_RIGHT_JOINT_NAMES = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
##BAXTER_RIGHT_JOINT_NAMES = ['right_s0']
#ENDING_POSITION_TOLERANCE = 0.001
#END_TIME_TOLERANCE = 0.5
#def arm_sim(bot, interploation_rate):
#	rospy.init_node('talker', anonymous=True)
#	joint_vel_pub = rospy.Publisher('/robot/limb/right/joint_command', baxter_core_msgs.msg.JointCommand, queue_size=10)
#	rate = rospy.Rate(1000)
#	rospy.Subscriber("joint_state", JointState, JointStateCallback, callback_args=bot)
#
#	#Time reference: http://wiki.ros.org/rospy/Overview/Time
#	#Give ROS time to connect and get the time
#	rate.sleep()
#	t_initial = rospy.get_rostime().to_sec() - 0.001
#	#sleep a bit so that our first time step isn't at t=0 because that will make the motion equation singluar
#	rate.sleep()
#	print "Ready"
#
#	while not rospy.is_shutdown():
#		joint_cmd = baxter_core_msgs.msg.JointCommand()
#		joint_cmd.mode = joint_cmd.VELOCITY_MODE
#		joint_still_moving = False
#		for joint_index,joint_name in enumerate(BAXTER_RIGHT_JOINT_NAMES):
#			joint_cmd.names.append(joint_name)
#			t_now = rospy.get_rostime().to_sec()
#			if (abs(bot.dhParams[joint_index][3] - bot.desiredDhParams[joint_index][3]) > ENDING_POSITION_TOLERANCE) and (t_now < (bot.tFinal[joint_index]+END_TIME_TOLERANCE)):
#				#We can use these equations to compute the required velocity of the system at any point in time
#				t = (t_now - bot.tInitial[joint_index])
#				velocity_this_step = 3*bot.a3[joint_index]*pow(t,2) + 2*bot.a2[joint_index]*t + bot.a1[joint_index]
#
#				#Store the new position of the joint by solving equation 1 for qf at the current time
#				bot.dhParams[joint_index][3] = bot.a3[joint_index]*math.pow(t,3) + bot.a2[joint_index]*math.pow(t,2) + bot.a1[joint_index]*t + bot.a0[joint_index]
#				joint_still_moving = True
#			else:
#				velocity_this_step = 0
#			#Store the new velocity of the joint by similarly solving equation 1
#			bot.currentVelocity[joint_index] = velocity_this_step
#			joint_cmd.command.append(velocity_this_step)
#			#print("Commanded joint "+joint_cmd.names[-1]+ " to velocity "+str(joint_cmd.command[-1]))
#		if joint_still_moving == True:
#			joint_vel_pub.publish(joint_cmd)
#			#print("Commanding"+str(joint_cmd.command))
#		elif bot.supressCommand != 0:
#			joint_vel_pub.publish(joint_cmd)
#			bot.supressCommand -= 1
#			#print("Commanding"+str(joint_cmd.command))
#		states = "Current joint state:"
#		for joint_index,name in enumerate(BAXTER_RIGHT_JOINT_NAMES):
#			states += " q"+str(joint_index)+"="+str(bot.dhParams[joint_index][3])
#		print(states)
#
#		rate.sleep()
#
#if __name__ == '__main__':
#	#during debugging, configure numpy to print H matrices in an easy to read format
#	np.set_printoptions(linewidth=150)
#	try:
#		#Scales the joints so they are actually visible in rviz
#		linear_scale = 5
#		#Set interpolation to 1.0 to disable interpolation
#		interploation_rate = 0.15
#		bot = MyRobot("baxter.json",linear_scale)
#		arm_sim(bot, interploation_rate)
#	except rospy.ROSInterruptException:
#		pass
import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import pudb
import threading

def quatmult(q0,q1):
	q_out = np.zeros([4,1])
	q_out[0] = q1[0]*q0[0] - q1[1]*q0[1] - q1[2]*q0[2] - q1[3]*q0[3];
	q_out[1] = q1[0]*q0[1] + q1[1]*q0[0] - q1[2]*q0[3] + q1[3]*q0[2];
	q_out[2] = q1[0]*q0[2] + q1[1]*q0[3] + q1[2]*q0[0] - q1[3]*q0[1];
	q_out[3] = q1[0]*q0[3] - q1[1]*q0[2] + q1[2]*q0[1] + q1[3]*q0[0];
	return q_out

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
	pose_target.orientation.w = rot[0]
	pose_target.orientation.x = rot[1]
	pose_target.orientation.y = rot[2]
	pose_target.orientation.z = rot[3]
	waypoints.append(copy.deepcopy(pose_target))
	return waypoints

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
#sleep until rospy knows the time
now = rospy.Time.now()
while now == rospy.Time(0):
	now = rospy.Time.now()
print now
tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
print "============ Starting tutorial "
print "============ Reference frame: %s" % group.get_planning_frame()
print "============ Reference frame: %s" % group.get_end_effector_link()
print "============ Robot Groups:"
print robot.get_group_names()
print "============ Printing robot state"
print robot.get_current_state()
waypoints = []
print "============"
print "============ Generating plan 1"
#get the position of the chessboard with respect to the base
have_transform = False
while not have_transform:
	try:
		(board_trans, board_rot) = tf_listener.lookupTransform('base', 'chessboard', rospy.Time())
		have_transform = True
	except(tf.LookupException):
		continue
# the quaternion for the chessboard is pointing the opposite direction of where the hand needs to
#	point, so define the link from /chessboard to /chesspiece to be pitched 180 degrees
piece_rot = (0,1,0,0)
#The translation is just (size of board square) * (square index) in each direction
current_piece_index = (1,1) #arbitrary for now
desired_piece_index = (7,7)
board_square_size = 0.025 # meters
current_piece_trans = (board_square_size * current_piece_index[0],board_square_size * current_piece_index[1],0)
above_current_piece_trans = (board_square_size * current_piece_index[0],board_square_size * current_piece_index[1],0.1)
above_desired_piece_trans = (board_square_size * desired_piece_index[0],board_square_size * desired_piece_index[1],0.1)
desired_piece_trans = (board_square_size * desired_piece_index[0],board_square_size * desired_piece_index[1],0)
#publish these transforms to tf
t = threading.Thread(target=publishTransforms,args=(tf_broadcaster,current_piece_trans,above_current_piece_trans,above_desired_piece_trans,desired_piece_trans,piece_rot,now,))
t.start()

print("Transforms sent at "+str(now))
#Move the arm to each of these locationss


now = rospy.Time(0)
#group.set_start_state_to_current_state()
#group.set_num_planning_attempts(1000)
#group.set_planning_time(100000)
group.set_planner_id("RRTConnectkConfigDefault")
#waypoints = addPose('/world','/left_gripper', now,waypoints)
waypoints = addPose('/world','/above_chesspiece', now,waypoints)
waypoints = addPose('/world','/chesspiece', now,waypoints)
waypoints = addPose('/world','/above_chesspiece', now,waypoints)
waypoints = addPose('/world','/above_chesspiece_target', now,waypoints)
waypoints = addPose('/world','/chesspiece_target', now,waypoints)
waypoints = addPose('/world','/above_chesspiece_target', now,waypoints)
#fraction = 0
#(chess_plan,fraction) = group.compute_cartesian_path(waypoints, 0.1, 0.0)
#robot= moveit_commander.RobotCommander()
#chess_plan= group.retime_trajectory(robot.get_current_state(), chess_plan, 1.0)
#print(str(fraction)+" of the path was planned")

for waypoint in waypoints:
	group.set_pose_target(waypoint)
	print("Starting plan")
	plan1 = group.plan()
	#rospy.sleep(10)
	print("Starting command")
	group.go(wait=True)
	#rospy.sleep(10)
	print("Finished command")

#print "============ Waiting while RVIZ displays plan1..."
#rospy.sleep(5)
#print "============ Executing chess_plan..."
#group.execute(chess_plan,True)
#group.execute(chess_plan,True)
#group.go(wait=True)
moveit_commander.roscpp_shutdown()
print "Completed movement, shutting down"
shutdown_threads = True
quit()