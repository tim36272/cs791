#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        try:
            #upper_shoulder > lower_shoulder > upper_elbow > lower_elbow > upper_forearm > lower_forearm > wrist
            (trans,rot) = listener.lookupTransform('/right_upper_shoulder', '/right_lower_shoulder', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

	print "Trans:"
	print trans
	print
	print "Rot: "
	print rot
        rate.sleep()
