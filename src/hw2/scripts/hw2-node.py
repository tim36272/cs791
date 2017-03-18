#!/usr/bin/python
import rospy
from std_msgs.msg import String
import numpy as np
from visualization_msgs.msg import *
import baxter_core_msgs.msg
import hw2.msg
import math
import pudb
import copy

#begin edits by Chad Adams
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
#end edits

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
ZERO_VEL_RETRANSMISSIONS = 5
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
		self.nextDesiredDhParams = []
		self.currentVelocity = [0] * len(self.dhParams)
		self.lastDhParams = self.dhParams
		self.a0 = [0] * len(self.dhParams)
		self.a1 = [0] * len(self.dhParams)
		self.a2 = [0] * len(self.dhParams)
		self.a3 = [0] * len(self.dhParams)
		self.nexta0 = []
		self.nexta1 = []
		self.nexta2 = []
		self.nexta3 = []
		self.tInitial = [0] * len(self.dhParams)
		self.nextTInitial = []
		self.tFinal = [0] * len(self.dhParams)
		self.nextTFinal = []
		self.supressCommand = 0
		self.nextTKPrime = []
		self.tKPrime = []
	def parseDHFile(self, DHparams):
		# list of 4-tuples (a, alpha, d, theta)
		if type(DHparams) == dict:
			self.setDHFromDict(DHparams)
		elif type(DHparams) == list:
			self.dhParams = DHparams
		else:
			throw("Invalid Format")


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
		
		h_transform[2,1] = math.sin(alpha);
		h_transform[2,2] = math.cos(alpha);
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


def TrajectoryMultiCommandCallback(msg,bot):
	#setup data structures
	start_time = rospy.get_rostime().to_sec()
	bot.nextDesiredDhParams = []
	bot.nextTFinal = []
	bot.nextTInitial = []
	bot.nexta0 = []
	bot.nexta1 = []
	bot.nexta2 = []
	bot.nexta3 = []
	bot.nextTKPrime = []
	for path_index in range(0,len(msg.points)):
		bot.nextDesiredDhParams.append(copy.deepcopy(bot.desiredDhParams))
		bot.nextTFinal.append([0] * len(msg.points[path_index].names))
		bot.nextTInitial.append([0] * len(msg.points[path_index].names))
		bot.nexta0.append([0] * len(msg.points[path_index].names))
		bot.nexta1.append([0] * len(msg.points[path_index].names))
		bot.nexta2.append([0] * len(msg.points[path_index].names))
		bot.nexta3.append([0] * len(msg.points[path_index].names))

	previous_position = []
	for param in bot.dhParams:
		previous_position.append(param[3])

	for path_index,point in enumerate(msg.points):
		bot.nextTKPrime.append(point.t_k_prime.data.to_sec())
		time_per_movement = point.t_k.data.to_sec()
		bot.nextDesiredDhParams[path_index] = copy.deepcopy(bot.desiredDhParams)
		for input_joint_index, input_joint_name in enumerate(point.names):
			for joint_index, joint_name in enumerate(bot.jointNames):
				if input_joint_name == joint_name:
					desired_position = point.q_final[input_joint_index]
					if joint_name == 'joint_2':
						desired_position += 1.5707963267948966
					bot.nextTInitial[path_index][joint_index] = start_time + time_per_movement * path_index
					bot.nextTFinal[path_index][joint_index] = bot.nextTInitial[path_index][joint_index] + time_per_movement
					
					#For a linear trajectory, the equations of motion are simply:
					#	a1*t + a0 = qf
					#	a1 = derivative_qf
					#Where a1 is the linear velocity and a0 is the starting position
					bot.nexta0[path_index][joint_index] = previous_position[joint_index]
					bot.nexta1[path_index][joint_index] = (desired_position - previous_position[joint_index]) / time_per_movement
					previous_position[joint_index] = desired_position
					
					#Set this last for pseudo thread safety
					bot.nextDesiredDhParams[path_index][joint_index][3] = desired_position
					bot.supressCommand = ZERO_VEL_RETRANSMISSIONS


def TrajectoryCommandCallback(msg, bot):
	print("<new coefficients:>")
	time_per_movement = msg.t_k.data.to_sec()
	for input_joint_index, input_joint_name in enumerate(msg.names):
		for joint_index, joint_name in enumerate(bot.jointNames):
			if input_joint_name == joint_name:
				#change this joint's position
				bot.desiredDhParams[joint_index][3] = msg.q_final[input_joint_index]
				bot.tInitial[joint_index] = rospy.get_rostime().to_sec()
				bot.tFinal[joint_index] = bot.tInitial[joint_index] + time_per_movement
				velocityFinal = msg.qdot_final[input_joint_index]
				if joint_name == 'joint_2':
					bot.desiredDhParams[joint_index][3] += 1.5707963267948966
				#First, In the cubic motion equations, there are four parameters: a0, a1, a2, and a3, which are the coefficients of the polynomial, i.e.:
				#	a3*t^3 + a2*t^2 + a1*t + a0=qf				(equation 1)
				#Where qf is the final position of the system
				#Second, qs is the initial position of the system, which can be proved to be equal to a0 (in equation 1 set t=0)
				#Third, derivative_qi is the velocity at t=0, which is equivalent to a1 (set t=0 and compute the derivative equation 1)
				#Finally, the derivative of equation 1 is:
				#	3*a3*t^2 + 2*a2*t + a1 = derivative_qf		(equation 2)
				#Where derivative_qf is the final velocity of the system, which is often desired to be 0
				bot.a0[joint_index]=bot.dhParams[joint_index][3] #bot.dhParams stores the current believed state of the system
				bot.a1[joint_index]= bot.currentVelocity[joint_index]
				bot.qf=bot.desiredDhParams[joint_index][3]
				bot.derivative_qf = velocityFinal

				#Now we have two equations (equation 1 and equation 2) and two unknowns (a2 and a3)
				#Numpy's linalg.solve expects the variables to be on the left, so format the equations as:
				#	a3*t^3 + a2*t^2 = qf - a1*t - a0
				#	3*a3*t^2 + 2*a2*t = derivative_qf - a1
				motion_equation_lhs = np.array([[1*math.pow(time_per_movement,3.0),1.0*math.pow(time_per_movement,2.0)],[3.0*math.pow(time_per_movement,2.0),2.0*time_per_movement]])
				motion_equation_rhs = np.array([bot.qf-bot.a1[joint_index]*time_per_movement-bot.a0[joint_index],bot.derivative_qf-bot.a1[joint_index]])
				motion_equation_result = np.linalg.solve(motion_equation_lhs, motion_equation_rhs)
				bot.a3[joint_index] = motion_equation_result[0]
				bot.a2[joint_index] = motion_equation_result[1]

				print("a3["+str(joint_index)+"]="+str(bot.a3[joint_index])+" a2["+str(joint_index)+"]="+str(bot.a2[joint_index])+" a1["+str(joint_index)+"]="+str(bot.a1[joint_index])+" a0["+str(joint_index)+"]="+str(bot.a0[joint_index]))
				bot.supressCommand = ZERO_VEL_RETRANSMISSIONS
	print("<end new coefficients>")



BAXTER_RIGHT_JOINT_NAMES = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
#BAXTER_RIGHT_JOINT_NAMES = ['right_s0']
ENDING_POSITION_TOLERANCE = 0.001
END_TIME_TOLERANCE = 0.5
def arm_sim(bot, interploation_rate):
	rospy.init_node('talker', anonymous=True)
	joint_vel_pub = rospy.Publisher('/robot/limb/right/joint_command', baxter_core_msgs.msg.JointCommand, queue_size=10)
	jacobian_pub = rospy.Publisher('/hw2/jacobian', hw2.msg.Jacobian, queue_size=10)
	rate = rospy.Rate(1000)
	rospy.Subscriber("trajectory_command", hw2.msg.TrajectoryCommand, TrajectoryCommandCallback, callback_args=bot)
	rospy.Subscriber("trajectory_multi_command", hw2.msg.TrajectoryMultiCommand, TrajectoryMultiCommandCallback, callback_args=bot)

	num_joints = len(BAXTER_RIGHT_JOINT_NAMES)

	#Time reference: http://wiki.ros.org/rospy/Overview/Time
	#Give ROS time to connect and get the time
	rate.sleep()
	t_initial = rospy.get_rostime().to_sec() - 0.001
	#sleep a bit so that our first time step isn't at t=0 because that will make the motion equation singluar
	rate.sleep()
	print "Ready"

	while not rospy.is_shutdown():
		joint_cmd = baxter_core_msgs.msg.JointCommand()
		joint_cmd.mode = joint_cmd.VELOCITY_MODE
		joint_still_moving = False
		for joint_index,joint_name in enumerate(BAXTER_RIGHT_JOINT_NAMES):
			joint_cmd.names.append(joint_name)
			t_now = rospy.get_rostime().to_sec()
			if (abs(bot.dhParams[joint_index][3] - bot.desiredDhParams[joint_index][3]) > ENDING_POSITION_TOLERANCE) and (t_now < (bot.tFinal[joint_index]+END_TIME_TOLERANCE)):
				#We can use these equations to compute the required velocity of the system at any point in time
				t = (t_now - bot.tInitial[joint_index])
				#check if we need to blend this velocity
				blended_a3 = bot.a3[joint_index]
				blended_a2 = bot.a2[joint_index]
				blended_a1 = bot.a1[joint_index]
				blended_a0 = bot.a0[joint_index]

				if bot.nextDesiredDhParams != []:
					#There is another trajectory coming up, so blend to it
					t_blend = (bot.tFinal[joint_index] - t_now) # this gives seconds until changeover
					if t_blend > bot.nextTKPrime[0]:
						t_blend = bot.nextTKPrime[0]
					alpha = t_blend / bot.nextTKPrime[0]
					blended_a3 = bot.a3[joint_index]*alpha + bot.nexta3[0][joint_index]*(1.0-alpha)
					blended_a2 = bot.a2[joint_index]*alpha + bot.nexta2[0][joint_index]*(1.0-alpha)
					blended_a1 = bot.a1[joint_index]*alpha + bot.nexta1[0][joint_index]*(1.0-alpha)
					blended_a0 = bot.a0[joint_index]*alpha + bot.nexta0[0][joint_index]*(1.0-alpha)
				velocity_this_step = 3*blended_a3*pow(t,2) + 2*blended_a2*t + blended_a1

				#Store the new position of the joint by solving equation 1 for qf at the current time
				bot.dhParams[joint_index][3] = blended_a3*math.pow(t,3) + blended_a2*math.pow(t,2) + blended_a1*t + blended_a0
				joint_still_moving = True
			else:
				velocity_this_step = 0
			#Store the new velocity of the joint by similarly solving equation 1
			bot.currentVelocity[joint_index] = velocity_this_step
			joint_cmd.command.append(velocity_this_step)
			#print("Commanded joint "+joint_cmd.names[-1]+ " to velocity "+str(joint_cmd.command[-1]))
		if joint_still_moving == True:
			joint_vel_pub.publish(joint_cmd)
			#print("Commanding"+str(joint_cmd.command))
		elif bot.supressCommand != 0:
			joint_cmd.command = [0 for x in range(0,len(BAXTER_RIGHT_JOINT_NAMES))]
			#print joint_cmd
			joint_vel_pub.publish(joint_cmd)
			bot.supressCommand -= 1
			#print("Commanding"+str(joint_cmd.command))
		elif bot.nextDesiredDhParams != []:
				#apply these new params
				bot.desiredDhParams = bot.nextDesiredDhParams[0]
				bot.a0 = bot.nexta0[0]
				bot.a1 = bot.nexta1[0]
				bot.a2 = bot.nexta2[0]
				bot.a3 = bot.nexta3[0]
				bot.tFinal = bot.nextTFinal[0]
				bot.tKPrime = bot.nextTKPrime[0]
				bot.tInitial = bot.nextTInitial[0]
				
				bot.nextDesiredDhParams = bot.nextDesiredDhParams[1:]
				bot.nexta0 = bot.nexta0[1:]
				bot.nexta1 = bot.nexta1[1:]
				bot.nexta2 = bot.nexta2[1:]
				bot.nexta3 = bot.nexta3[1:]
				bot.nextTFinal = bot.nextTFinal[1:]
				bot.nextTKPrime = bot.nextTKPrime[1:]
				bot.nextTInitial = bot.nextTInitial[1:]
				print "Executing next trajectory, "+str(len(bot.nextDesiredDhParams))+" remain"
				bot.supressCommand = ZERO_VEL_RETRANSMISSIONS

		states = "State:"
		#compute the position of the end efector
		#This is easy given the Project 1 code: just compute the homogeneous transform of the end effector and take the rightmost column
		h_transform_end_effector = bot.getT(0,num_joints-1)
		
		jacobian_msg = hw2.msg.Jacobian()
		jp = np.zeros((3,num_joints))
		jo = np.zeros((3,num_joints))
		
		for joint_index,name in enumerate(BAXTER_RIGHT_JOINT_NAMES):
			states += " q"+str(joint_index)+"="+str(bot.dhParams[joint_index][3])
			#compute the jacobian for this joint
			#compute the position of this joint
			h_transform_this_joint = bot.getT(0,joint_index-1)
			#The equation for the first three rows of the i'th column of the jacobian is:
			#	Jp_i=z_{i-1} x (P_e-P_i-1)
			joint_position = (h_transform_end_effector - h_transform_this_joint)[0:3,-1]
			#The z axis of the previous joint is just the third column (column 2) of the homogeneous matrix
			z_vector_previous_joint = h_transform_this_joint[0:3,2]			
			jp[0:3,joint_index] = np.cross(z_vector_previous_joint,joint_position)
			jo[0:3,joint_index] = z_vector_previous_joint

			#Format this as ROS wants it
			jp_col = np.round(jp[0:3,joint_index],15)
			jp_vector = Vector3(jp_col[0],jp_col[1],jp_col[2])
			jacobian_msg.JP.append(jp_vector)

			jo_col = np.round(jo[0:3,joint_index],15)
			jo_vector = Vector3(jo_col[0],jo_col[1],jo_col[2])
			jacobian_msg.JO.append(jo_vector)

		
		jacobian_msg.header.stamp = rospy.Time.now()
		jacobian_msg.header.frame_id = "0"
		jacobian_msg.names = BAXTER_RIGHT_JOINT_NAMES
		
		jacobian_pub.publish(jacobian_msg)
		print(states)

		rate.sleep()

if __name__ == '__main__':
	#during debugging, configure numpy to print H matrices in an easy to read format
	np.set_printoptions(linewidth=150)
	try:
		#Scales the joints so they are actually visible in rviz
		linear_scale = 1
		#Set interpolation to 1.0 to disable interpolation
		interploation_rate = 0.15
		bot = MyRobot("baxter.json",linear_scale)
		arm_sim(bot, interploation_rate)
	except rospy.ROSInterruptException:
		pass
