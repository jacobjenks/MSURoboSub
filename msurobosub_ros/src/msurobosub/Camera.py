#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

camFConnected = True 
camDConnected = True 
cvBridge = None

#Publish images from forward and bottom camera
def main():
	global cvBridge
	rospy.init_node('camera')
	pubCamF = rospy.Publisher('sensor_msgs/CameraForward', Image, queue_size=10)
	pubCamD = rospy.Publisher('sensor_msgs/CameraDown', Image, queue_size=10)
	rate = rospy.Rate(30)#30 FPS

	cvBridge = CvBridge()
	camF = cv2.VideoCapture(0)
	camD = cv2.VideoCapture(1)
	if camF.isOpened() == False:
		camFConneced = False
		rospy.logerr("Failed to open forward camera")	
	if camD.isOpened() == False:
		camDConneced = False
		rospy.logerr("Failed to open downward camera")	

	while not rospy.is_shutdown():
		if camFConnected: 
			pubImage(pubCamF, camF)
		if camDConnected:
			pubImage(pubCamD, camD)

def pubImage(publisher, camera):
	global cvBridge
	ret, image = camera.read()
	if ret:
		publisher.publish(cvBridge.cv2_to_imgmsg(image))
	#else:
		#rospy.logerr("Failed to read from camera")

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
