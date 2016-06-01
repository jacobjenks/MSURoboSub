#!/usr/bin/env python
import rospy
import tf
from common_msgs.msg import Pose

def handle_pose(msg):

if __name__ == '__main__':
	rospy.init_node('robosub_tf_broadcaster')
	rospy.Subscriber('/robosub/pose', Pose, handle_pose)
	rospy.spin()
