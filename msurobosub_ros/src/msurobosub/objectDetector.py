#!/usr/bin/env python
import rospy
from hector_object_tracker.msgs import ImagePercept
from sensor_msgs.msgs import Image

msgImagePercept = None
pubImagePercept = None
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
	msgImagePercept.header.seq = 0
	msgImagePercept.header.frame_id = ""

	rospy.spin()

if __name__ == '__main__':
	try:
		objectDetector()
	except rospy.ROSInterruptException:
		pass
