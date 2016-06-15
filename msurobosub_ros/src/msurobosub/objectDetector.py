#!/usr/bin/env python
import rospy
from hector_object_tracker.msgs import ImagePercept
from sensor_msgs.msgs import Image

msgImagePercept = None
pubImagePercept = None
latestImage = None
gpuReady = True

def subImageCB(imageMsg):
	global gpuReady
	
	if !gpuReady:
		return

	gpuReady = False	
	#Run faster RCNN
	gpuReady = True

def fasterRCNN(imageMsg):
	
def objectDetector():
	pubMot = rospy.Publisher('worldmodel/image_percept', ImagePercept, queue_size=10)
	rospy.init_node('objectDetector')
	rospy.Subscriber("sensors/camF/image_raw", Image, subCamCB)

	msgImagePercept = ImagePercept()
	msgMot.header.seq = 0
	msgMot.header.frame_id = ""

	rospy.spin()

if __name__ == '__main__':
	try:
		objectDetector()
	except rospy.ROSInterruptException:
		pass
