#!/usr/bin/env python
import rospy
import math
from msurobosub.msg import Depth, MotorCommand
from geometry_msgs.msg import Pose, PoseWithCovariance, Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
# from msurobosub.msg import VisualCoordinates

#Enum
class TaskStatus:
	waiting = 1		#Waiting for prerequisites to be fullfilled
	ready = 2		#Ready to run
	complete = 3	#Task complete

class Task:
	priority = 0 # Task priority, tasks are executed in order of priority. Priority 0 is root task that runs all others
	subTasks = None	# List of subtasks that must be completed before this task can start
	targetObject = None  
	state = TaskStatus.waiting # Tasks sit in waiting state until monitorStatus() decides they are available to run
	pubControl = None # Tasks publish a desired Pose as a target for the controller
	pubVisiControl = None # Tasks publish desired object coordinates in the camera frame - normalized (-1 <= x,y <= 1)
	visiCoordindates = None # Callback data for object_detector publisher
	pubStatus = None # Channel for publishing task state/debug info
	name = "Task"
	runAlways = False # Some tasks should always run, regardless of status
	runMission = True# Whether or not to run at all - allows us to pause mission

	def __init__(self, priority = 0, name = "Task", subTasks = None):
		self.status = TaskStatus.waiting
		self.priority = priority
		self.name = name
		if subTasks == None:
			self.subTasks = list()
		elif not isinstance(subTasks, list):
			rospy.logError("Invalid subTask list")
		else:
			self.subTasks = subTasks

		rospy.init_node('Task', anonymous=True)
		self.pubControl = rospy.Publisher('controller/moveTo', Odometry, queue_size=5)
		self.pubStatus = rospy.Publisher('mission/status', String, queue_size=5)
		# self.pubVisiControl = rospy.Publisher('visual/moveTo', VisualCoordinates, queue_size=5)		

		if self.priority == 0:
			rate = rospy.Rate(10)
			rospy.Subscriber("command/missionToggle", Bool, self.runMissionCallback)
			while not rospy.is_shutdown():	
				if self.runMission:
					self.run()
				rate.sleep()

	def runMissionCallback(self, msg):
		self.runMission = msg.data

	#Callback function for any subscribers the task uses to monitor its own status
	def monitorStatus():
		pass
	
	#Run subtasks, then run self task
	def run(self):
		subTasksDone = True 

		#Run subtasks in order
		for task in self.subTasks:
			if task.status == TaskStatus.waiting:
				subTasksDone = False
			elif task.status == TaskStatus.ready:
				subTasksDone = False
				task.run()
				break
			elif task.runAlways == True:
				task.run()

		#Run this task if all subtasks are complete
		if subTasksDone:	
			self.runSelf()

	def runSelf(self):
		pass


def MaintainDepthTask(Task):
	depthSub = None 
	motorPub = None 
	depthThreshold = 1#Depth in meters at which this node will activate
	currentDepth = 0
	motorCommand = MotorCommand()

	def __init__(self):
		Task.__init__(self, 1, "MaintainDepth")
		self.depthSub = rospy.Subscriber("sensors/depth", Depth, monitorStatus)
		self.status = TaskStatus.complete
		self.runAlways = True

	def monitorStatus(self, depth):
		if depth.depth < depthThreshold:
			self.state = TaskStatus.ready
		self.currentDepth = depth.depth

	def runSelf(self):
		if currentDepth < depthThreshold:
			self.targetPose.position.z -= 1# Set target pose 1 meter down


class MoveToTarget:

	def __init__(self):
		self.pose = Odometry()
		#self.targetSub = rospy.Subscriber("worldmodel/object/pose", , self.monitorStatus)

	def getTarget(self):
		return self.pose

	def isFound():
		return True


class MoveToTargetLocation(MoveToTarget):
	
	def __init__(self, odom):
		self.pose = odom
		

class MoveToTask(Task):
	currentOdom = None		#base_link Odometry 
	targetOdom = None		#nav_msgs/Odometry we want to move to
	targetFound = False		#Whether or not we know where the target is yet
	targetDistance = 0		#Current distance to target
	speed = 0				#Speed %
	proximityThresh = .3	#Distance (meters) threshold for arriving at target
	bumpThresh = 0 			#Acceleration threshold (m/s^2) for detecting a collision
	steadyThresh = 0		#Acceleration threshold (m/s^2) for determining sub to be stationary
	targetFrame = ""		#Frame ID of the object we're looking for
	movingFrame = ""		#Frame ID we would like to move to target location ie "base_link" or "camF"
	
	#These are the three potential conditions necessary for this task to be complete
	distanceStatus = False
	bumpStatus = False
	steadyStatus = False

	imuSub = None
	odomSub = None
	targetSub = None
	
	def __init__(self, priority, name, subTasks, target, bumpThresh = 0, steadyThresh = 0, movingFrame = "base_link"):
		"""
		@param target: MoveToTarget
		"""
		Task.__init__(self, priority, name, subTasks)
		self.imuSub = rospy.Subscriber("sensors/imu", Imu, self.monitorStatus)
		self.odomSub = rospy.Subscriber("odometry/filtered", Odometry, self.monitorStatus)
		# self.visiCoordinates = rospy.Subscriber('visual/coordinates', VisualCoordinates, self.monitorStatus)

		self.target = target
		self.bumpThresh = bumpThresh
		self.steadyThresh = steadyThresh
		self.movingFrame = movingFrame
		self.status = TaskStatus.ready

		if bumpThresh == 0:
			bumpStatus = True
		if steadyThresh == 0:
			steadyStatus = True

	#Get 3d vector magnitude
	def getMagnitude(self, msg):
		return math.sqrt(math.pow(msg.x, 2) 
								+ math.pow(msg.y, 2)
								+ math.pow(msg.z, 2))

	def monitorStatus(self, msg):
		if type(msg).__name__ == "Imu":
			magnitude = self.getBumpMagnitude(msg.linear_acceleration)
			if not self.bumpStatus and self.bumpThresh > 0 and magnitude > self.bumpThresh: 
				self.bumpStatus = True
				self.pubStatus.publish(self.name + ": bump detected " + magnitude)

			if self.distanceStatus and not self.steadyStatus and self.steadyThresh > 0 and self.steadyThresh > magnitude:
				self.steadyStatus = True
				self.pubStatus.publish(self.name + ": sub is steady")

		elif type(msg).__name__ == "Odometry":
			self.currentOdom = msg
			point = Point()
			targetOdom = self.target.getTarget()
			point.x = targetOdom.pose.pose.x - currentOdom.pose.pose.x
			point.y = targetOdom.pose.pose.y - currentOdom.pose.pose.y
			point.z = targetOdom.pose.pose.z - currentOdom.pose.pose.z
			self.targetDistance = getMagnitude(point)

			self.pubStatus.publish("MoveTo target distance: " + self.targetDistance)

			#TODO: Check orientation as well as distance
			if self.targetDistance < self.proximityThresh:
				self.distanceStatus = True
				self.pubStatus.publish(self.name + ": arrived at target")
		# elif type(msg).__name__ == "VisualCoordinates":
			# Do Something
		
		elif type(msg).__name__ == "Object":
			if msg.header.frame_id == targetFrame:
				if self.targetFound == False:
					self.targetFound == True
					self.status = TaskStatus.ready
					self.pubStatus.publish(self.name + ": target found")
				targetOdom = msg
		

		if self.distanceStatus and self.steadyStatus and self.bumpStatus:
			self.status = TaskStatus.complete
			self.pubStatus.publish(self.name + ": task complete")
			
			
	def runSelf(self):
		if self.status == TaskStatus.ready:
			targetOdom = self.target.getTarget()
			targetOdom.header.frame_id = self.movingFrame
			self.pubControl.publish(targetOdom)


class PneumaticTask(Task):

	pneuPub = None
	pneuMsg = None

	def __init__(self, priority, name, subTasks, pneuID):
		'''
		@param pneuID: ID of the pneumatic device to activate
		'''
		Task.__init__(self, name, priority, subTasks)	
		self.pneuPub = rospy.Publisher("command/pneumatic", PneumaticCommand, queue_size=5)
		self.pneuMsg = PneumaticCommand()
		self.pneuMsg.command = pneuID

	def runSelf(self):
		self.pneuMsg.header.seq = 0
		self.pneuMsg.header.frame_id = "base_link"
		self.pneuMsg.header.stamp = rospy.get_rostime()
		pneuPub.publish(self.pneuMsg)

class MotorCommandTask(Task):

	def __init__(self, priority, name, subTasks, motorPower, timeout):
		Task.__init__(self, priority, name, subTasks)
		self.motorCommand = MotorCommand()
		self.motorCommand.power = motorPower
		self.timeout = timeout
		self.started = False
		self.pubMotor = rospy.Publisher("command/motor", MotorCommand, queue_size=16)
		self.status = TaskStatus.ready

	def runSelf(self):
		print "Running" + self.name
		if not self.started:
			self.start = rospy.get_time()
			self.started = True
		if self.timeout is not 0 and self.start + self.timeout < rospy.get_time():
			self.status = TaskStatus.complete
			return

		self.motorCommand.header.seq += 1
		self.motorCommand.header.stamp = rospy.get_rostime()
		self.motorCommand.header.frame_id = "base_link"
		self.pubMotor.publish(self.motorCommand)


class TestTask(Task):

	def __init__(self):
		Task.__init__(self, 0, "Test Task")

	def runSelf(self):
		self.pubStatus.publish("Running")

class TestMaintainDepthMission(Task):

	def __init__(self):
		subTasks = []
		subTasks.append(MaintainDepthTask())

		Task.__init__(self, 0, "Test Depth Mission", subTasks)


class TestBuoyMission(Task):

	def __init__(self):
		subTasks = []
		subTasks.append(MaintainDepthTask())

		subTasks.append(MoveToTask(2, "Red Buoy", [], 1, 1, .5, "buoy_red"))

		Task.__init__(self, 0, "Test Buoy Mission", subTasks)


class TestControllerMission(Task):

	def __init__(self):
		subTasks = [] 
		msg = Odometry()
		msg.pose.pose.position.x = 10
		subTasks.append(MoveToTask(2, "Point 1", [], MoveToTargetLocation(msg)))
		
		msg = Odometry()
		msg.pose.pose.position.x = 0 
		subTasks.append(MoveToTask(2, "Point 2", [], MoveToTargetLocation(msg)))

		msg = Odometry()
		msg.pose.pose.position.x = 10
		subTasks.append(MoveToTask(2, "Point 3", [], MoveToTargetLocation(msg)))

		msg = Odometry()
		msg.pose.pose.position.x = 0
		subTasks.append(MoveToTask(2, "Point 4", [], MoveToTargetLocation(msg)))
		
		Task.__init__(self, 0, "Test Controller Mission", subTasks)


class QualifyMission(Task):
	
	def __init__(self):
		subTasks = []
		subTasks.append(MotorCommandTask(1, "Wait", [], [0, 0, 0, 0, 0, 0, 0, 0], 25))
		subTasks.append(MotorCommandTask(1, "Down", [], [0, 0, 0, 0, -1, -1, -1, -1], 2))
		subTasks.append(MotorCommandTask(1, "Go", [], [1, 1, 0, 0, -0.34, -0.34, -0.34, -0.34], 400))

		subTasks.append(MotorCommandTask(2, "Stop", [], [0, 0, 0, 0, 0, 0, 0, 0], 0))
		Task.__init__(self, 0, "Qualify Mission", subTasks)



#TODO: Create competition mission, and specific testing missions

'''
class CompetitionMission(Task):
	def __init__(self):
		

'''

if __name__ == '__main__':
	try:
		#test = TestControllerMission()	
		QualifyMission()
	except rospy.ROSInterruptException:
		pass
