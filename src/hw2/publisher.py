#!/usr/bin/python



import rospy

from math import sin, cos, pi

from sensor_msgs.msg import JointState



def joints():

    jsPub = rospy.Publisher("joint_state", JointState, queue_size = 10)

    rospy.init_node('joint_publisher')

    rate = rospy.Rate(2)

    jsmsg = JointState()



    jsmsg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

    jsmsg.position = [0, 0, 0,0,0,0,0]

    i = 0

    while not rospy.is_shutdown():



        jsmsg.header.frame_id = "/base_link"

        jsmsg.header.stamp = rospy.Time.now()



        jsmsg.position[i] = 3.1415927 / 2



        print jsmsg



        jsPub.publish(jsmsg)

        rate.sleep()



        jsmsg.position[i] = 3.1415927



        jsPub.publish(jsmsg)

        rate.sleep()



        jsmsg.position[i] = 0

        jsPub.publish(jsmsg)

        rate.sleep()



        i = (i + 1) % 7



if __name__ == '__main__':

    try:

        joints()

    except rospy.ROSInterruptException:

        pass
