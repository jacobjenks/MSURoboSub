#!/usr/bin/env python

import rospy
import math
# from msurobosub.msg import VisualCoordinates, MotorCommand, Depth
from std_msgs.msg import String, Bool # Don't know if we will actually need this
from heapq import heappop, heappush, heapify 


# Status Enums
class Status():
	waiting = 1 # Need to find
	ready = 2 # Found it
	complete = 3 # Done with this (should have been popped off of the heap)

# Task Enums
class Priority():
	qual_gate = 0
	buoy = 1
	path_marker = 2

	@staticmethod
	def get_priority(self, name):
		if name in globals():
			return globals()[name]

class Mission():

	def __init__(self):
		self.queue = []
		self.visual = VisualCoordinates()
		self.current_task = None

		rospy.init_node('Task', anonymous=True)
		self.pubControl = rospy.Publisher('visual/moveTo', VisualCoordinates, queue_size=5)
		self.visual = rospy.Subscriber('visual/coordinates', VisualCoordinates, get_object)

		rate = rospy.Rate(10)
		self.queue.heapify()
		while not rospy.is_shutdown():
			maintain_depth()		
			rate.sleep()

	def maintain_depth(self):
		pass

	def get_object(self, obj):
		priority = Priority.get_priority(obj.name)
		if not priority:
			return
		self.queue.heappush({"oject": obj.name, "priority": priority, "x": obj.x, "y": obj.y, "distance": obj.distance})


	def move_to(self):
		self.current_task = self.queue.heappop()
		self.pubControl.publish(self.visual)


if __name__ == "__main__":
	try:
		mission = Mission()
	except rospy.ROSIntrerruptException:
		pass
