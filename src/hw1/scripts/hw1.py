#!/usr/bin/python
import rospy
from std_msgs.msg import String
import numpy as np
from visualization_msgs.msg import *
from sensor_msgs.msg import *
import math
import pudb
import copy

def matToQuat(mat):
	#Reference: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	trace = mat[0,0] + mat[1,1] + mat[2,2]
	if trace > 0:
		s = math.sqrt(trace + 1.0) * 2
		w = 0.25 * s
		x = (mat[2,1] - mat[1,2]) / s
		y = (mat[0,2] - mat[2,0]) / s
		z = (mat[1,0] - mat[0,1]) / s
	elif ((mat[0,0] > mat[1,1]) and (mat[0,0] > mat[2,2])):
		s = math.sqrt(1.0 + mat[0,0] - mat[1,1] - mat[2,2]) * 2
		w = (mat[2,1] - mat[1,2]) / s
		x = 0.25 * s
		y = (mat[0,1] + mat[1,0]) / s
		z = (mat[0,2] + mat[2,0]) / s
	elif (mat[1,1] > mat[2,2]):
		s = math.sqrt(1.0 + mat[1,1] - mat[0,0] - mat[2,2]) * 2
		w = (mat[0,2] - mat[2,0]) / s
		x = (mat[0,1] + mat[1,0]) / s
		y = 0.25 * s
		z = (mat[1,2] + mat[2,1]) / s
	else:
		s = math.sqrt(1.0 + mat[2,2] - mat[0,0] - mat[1,1]) * 2
		w = (mat[1,0] - mat[0,1]) / s
		x = (mat[0,2] + mat[2,0]) / s
		y = (mat[1,2] + mat[2,1]) / s
		z = 0.25 * s
	return [w,x,y,z]

def GetHTranslateMatrix(x,y,z):
	#Reference: http://web.cs.iastate.edu/~cs577/handouts/homogeneous-transform.pdf
	h = np.eye(4,4)
	h[0,3] += x
	h[1,3] += y
	h[2,3] += z
	return h
def GetHRotationMatrix(roll,pitch,yaw):
	#Reference: https://en.wikipedia.org/wiki/Rotation_matrix
	roll_mat = np.eye(4,4)
	roll_mat[1,1] =  math.cos(roll)
	roll_mat[1,2] = -math.sin(roll)
	roll_mat[2,1] =  math.sin(roll)
	roll_mat[2,2] =  math.cos(roll)

	pitch_mat = np.eye(4,4)
	pitch_mat[0,0] =  math.cos(pitch)
	pitch_mat[0,2] =  math.sin(pitch)
	pitch_mat[2,0] = -math.sin(pitch)
	pitch_mat[2,2] =  math.cos(pitch)

	yaw_mat = np.eye(4,4)
	yaw_mat[0,0] =  math.cos(yaw)
	yaw_mat[0,1] = -math.sin(yaw)
	yaw_mat[1,0] =  math.sin(yaw)
	yaw_mat[1,1] =  math.cos(yaw)

	return np.dot(yaw_mat,np.dot(pitch_mat, roll_mat))
class MyRobot:

	def __init__(self, filename, linear_scale):
		f = open(filename)
		botSpecs = eval(f.read())
		f.close()
		self.parseDHFile(botSpecs)
		self.linear_scale = linear_scale
		#verify the joints are in order.
		for joint_index in range(0,len(self.jointNames)-1):
			if self.jointNames[joint_index] != self.jointParent[joint_index+1]:
				print "This program currently requires your joints to be in order. Please reorder your input file so that the parent of the N'th joint is the name of the N-1'th joint"
				print self.jointNames[joint_index+1] + " did not match " + self.jointParent[joint_index]
				exit()
		self.desiredDhParams = copy.deepcopy(self.dhParams)

	def parseDHFile(self, DHparams):
		# list of 4-tuples (a, alpha, d, theta)
		if type(DHparams) == dict:
			self.setDHFromDict(DHparams)
		elif type(DHparams) == list:
			self.dhParams = DHparams
		else:
			throw("Invalid Format")

		print self.dhParams

		self.numLinks = len(self.dhParams)
		self.q = np.zeros((self.numLinks, 1))

	def setDHFromDict(self, DHdict):
		self.dhParams = []
		self.jointNames = []
		self.jointParent = []
		for joint in DHdict["Joints"]:
			self.dhParams.append(joint["DH_parameters"])
			#Get the joint names
			self.jointNames.append(joint["joint_name"])
			self.jointParent.append(joint["parent_name"])

	def getTranslation(self, i,j):
		return self.getT(i,j)[:-1,-1]

	def getRotationMatrix(self, i,j):
		return self.getT(i,j)[:3,:3]


	# constructA: constructs the transform matrix for the ith joint using the D-H parameters
	# i: the joint to construct the homogeneous transform for
	# returns the transform from the ith to the i+1th
	def constructA(self, i):
		#d, theta, a ,alpha= self.dhParams[i]
		a, alpha, d ,theta= self.dhParams[i]
		h_transform = np.eye(4)
		# Method:
		#	Starting from the homogeneous matrix above, treat it as a vector
		#	that points along x. Translate it by d units along z, Rotate it by theta
		#	radians about z, translate that by a units along x, and rotate that by alpha units about x.
		#	Note that this could be done more directly (i.e. instead of being
		#	composed from elementry operations) but then it would be less clear
		#reference for numpy.dot: https://docs.scipy.org/doc/numpy/reference/generated/numpy.dot.html
		a *= self.linear_scale
		d *= self.linear_scale
		#h_transform = np.dot(GetHTranslateMatrix(0,0,d),h_transform)
		#h_transform = np.dot(GetHRotationMatrix(0,0,theta),h_transform)
		##rotate a to the new orientation
		#a_rotated = np.dot(GetHRotationMatrix(0,0,theta),GetHTranslateMatrix(a,0,0))
		#h_transform = np.dot(GetHTranslateMatrix(a_rotated[0,3] ,a_rotated[1,3],a_rotated[2,3]),h_transform)
		#h_transform = np.dot(GetHRotationMatrix(alpha,0,0),h_transform)


		#alternative method #2: the full matrix. This produces different results. I'm not sure why.
		#Reference: http://web.aeromech.usyd.edu.au//MTRX4700/Course_Documents/material/lectures/L2_Kinematics_Dynamics_2013.pdf
		h_transform[0,0] = math.cos(theta);
		h_transform[0,1] = -math.sin(theta)*math.cos(alpha);
		h_transform[0,2] = math.sin(theta)*math.sin(alpha);
		h_transform[0,3] = a * math.cos(theta);
		
		h_transform[1,0] = math.sin(theta);
		h_transform[1,1] = math.cos(theta)*math.cos(alpha);
		h_transform[1,2] = -math.cos(theta)*math.sin(alpha);
		h_transform[1,3] = a * math.sin(theta);
		
		h_transform[2,1] = math.sin(theta);
		h_transform[2,2] = math.cos(theta);
		h_transform[2,3] = d;


		#alternative method #3: the simple revolute-only method from Dave's slides
		#h_transform[0,0] = math.cos(theta);
		#h_transform[0,1] = -math.sin(theta)
		#h_transform[0,3] = a * math.cos(theta)
		#
		#h_transform[1,0] = math.sin(theta)
		#h_transform[1,1] = math.cos(theta)
		#h_transform[1,3] = a * math.sin(theta)


		return h_transform


	# getT: computes a homogeneous transform from the ith frame to the jth frame by iteratively multiplying each homogeneous transform from i to j
	# i is the index of the starting coordinate frame
	# j is the index of the ending coordinate frame
	# returns: a homogeneous transform matrix, T
	def getT(self, i, j):
		h_cummulative = np.eye(4,4)
		for index in range (i,j+1):
			h_cummulative = np.dot(h_cummulative, self.constructA(index))
		return h_cummulative

#reference joint state message: http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
def JointStateCallback(msg, bot):
	for input_joint_index, input_joint_name in enumerate(msg.name):
		for joint_index, joint_name in enumerate(bot.jointNames):
			if input_joint_name == joint_name:
				#change this joint's position
				bot.desiredDhParams[joint_index][3] = msg.position[input_joint_index]

def arm_sim(bot, interploation_rate):
	rospy.init_node('talker', anonymous=True)
	#publish to the marker channel(visualization_marker)
	#reference: http://wiki.ros.org/rviz/DisplayTypes/Marker
	#reference: http://answers.ros.org/question/11135/plotting-a-markerarray-of-spheres-with-rviz/
	array_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
	marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
	rate = rospy.Rate(10)
	rospy.Subscriber("joint_state", JointState, JointStateCallback, callback_args=bot)

	#setup the stuff in a marker that doesn't change
	marker_ref = Marker()
	marker_ref.header.frame_id = "/base"
	marker_ref.type = marker_ref.CUBE
	marker_ref.action = marker_ref.ADD
	marker_ref.scale.x = 0.1
	marker_ref.scale.y = 0.1
	marker_ref.scale.z = 0.1
	marker_ref.color.r = 1.0
	marker_ref.color.g = 0.0
	marker_ref.color.b = 1.0
	marker_ref.color.a = 1.0

	while not rospy.is_shutdown():
		all_markers = MarkerArray()
		all_lines = Marker()
		all_lines.header.frame_id = '/base'
		all_lines.id = len(bot.dhParams)+1
		all_lines.type = Marker.LINE_STRIP
		all_lines.action = Marker.ADD
		all_lines.scale.x = 0.1
		all_lines.color.r = 1.0
		all_lines.color.g = 1.0
		all_lines.color.b = 0.0
		all_lines.color.a = 1.0
		#push the origin to the visualization lists
		all_lines.points.append(geometry_msgs.msg.Point(0,0,0))
		
		#Render each joint
		for joint_index in range(0,len(bot.dhParams)):
			marker = copy.deepcopy(marker_ref)
			marker.id = joint_index
			marker.color.r = 1.0 - (float(joint_index) / len(bot.dhParams)) # gives joints a gradient
			marker.color.b = 0.0 + (float(joint_index) / len(bot.dhParams)) # gives joints a gradient

			#interpolate the current and desired params to make visualization easier
			for param_index in range(0,4):
				param_diff = bot.dhParams[joint_index][param_index] - bot.desiredDhParams[joint_index][param_index]
				if abs(param_diff) < 0.01 and param_diff != 0:
					bot.dhParams[joint_index][param_index] = bot.desiredDhParams[joint_index][param_index]
				elif param_diff != 0:
					bot.dhParams[joint_index][param_index] -= param_diff * interploation_rate
			#get a transform from the base to this joint
			h_transform = bot.getT(0,joint_index)
			#Put the transform into the marker message
			[marker.pose.orientation.w,marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z] = matToQuat(h_transform[0:3,0:3])
			marker.pose.position.x = h_transform[0,3]
			marker.pose.position.y = h_transform[1,3]
			marker.pose.position.z = h_transform[2,3]
			#store marker
			all_markers.markers.append(marker)
			#store line segment
			all_lines.points.append(marker.pose.position)
		#publish the lines and markers
		array_pub.publish(all_markers)
		marker_pub.publish(all_lines)
		rate.sleep()

if __name__ == '__main__':
	#during debugging, configure numpy to print H matrices in an easy to read format
	np.set_printoptions(linewidth=150)
	try:
		#Scales the joints so they are actually visible in rviz
		linear_scale = 10
		#Set interpolation to 1.0 to disable interpolation
		interploation_rate = 0.15
		bot = MyRobot("robot_test.json",linear_scale)
		arm_sim(bot, interploation_rate)
	except rospy.ROSInterruptException:
		pass
