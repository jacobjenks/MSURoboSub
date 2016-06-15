#!/usr/bin/env python
import rospy
from msurobosub.msg import Depth
from nav_msgs.msg import Odometry 

odomMsg = None
pubDepthOdom = None

def depthCallback(msg):
	global odomMsg, pubDepthOdom
	odomMsg.header.stamp = rospy.get_rostime() 
	odomMsg.header.seq += 1

	odomMsg.pose.pose.position.z = msg.depth * -1

	pubDepthOdom.publish(odomMsg)


def depthOdom():
	global odomMsg, pubDepthOdom
	pubDepthOdom = rospy.Publisher('odometry/depth', Odometry, queue_size=10)
	rospy.init_node('depthOdom')
	rospy.Subscriber("sensors/depth", Depth, depthCallback)

	odomMsg = Odometry()
	odomMsg.header.seq = 0
	odomMsg.header.frame_id = "depth"
	odomMsg.child_frame_id = "depth" # not really sure what this is for

	identity = 0
	for i in range(0, 36):
		if i == identity:
			identity = identity + 7
			odomMsg.pose.covariance[i] = .02298
		else:
			odomMsg.pose.covariance[i] = 0

	rospy.spin()

if __name__ == '__main__':
	try:
		depthOdom()
	except rospy.ROSInterruptException:
		pass
