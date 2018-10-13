#!/usr/bin/env python

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import GetJointProperties
import rospy
import time

class gazebo_robot:

	def __init__(self, node_name, name, X, Y, Z): #initial robot conditions

		self._name = name
		self._x = X
		self._y = Y
		self._z = Z
		self._roll = 0.0
		self._pitch = 0.0
		self._yaw = 0.0

		rospy.init_node(node_name, anonymous=True)

		self._modelStateSetter = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10, latch=True)
		rospy.sleep(1)
		self._jointPropertiesGetter = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		rospy.sleep(1)
		self._jointEffortApplier = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
		rospy.sleep(1)

		self._msg = ModelState()

		self._msg.model_name = name
		
		self._msg.pose.position.x = X
		self._msg.pose.position.y = Y
		self._msg.pose.position.z = Z
		self._msg.pose.orientation.x = 0.0
		self._msg.pose.orientation.y = 0.0
		self._msg.pose.orientation.z = 0.0
		self._msg.pose.orientation.w = 0.0
		self._msg.twist.linear.x = 0.0
		self._msg.twist.linear.y = 0.0
		self._msg.twist.linear.z = 0.0
		self._msg.twist.angular.x = 0.0
		self._msg.twist.angular.y = 0.0
		self._msg.twist.angular.z = 0.0
		self._msg.reference_frame = "world"

		self._modelStateSetter.publish(self._msg)

	def setPosition(self, X, Y, Z):

		self._msg.pose.position.x = X
		self._msg.pose.position.y = Y
		self._msg.pose.position.z = Z

		self._modelStateSetter.publish(self._msg)

	def getJointProperties(self, joint_name):

		joints = self._jointPropertiesGetter(joint_name)
		return joints

	def applyJointEffort(self, joint_name, effort, duration):

		# duration_diff = rospy.Duration(0.01)
		duration_zero = rospy.Duration(0)

		# while duration >= duration_zero:

			# duration = duration - duration_diff
		start_time = rospy.get_rostime()
		response = self._jointEffortApplier(joint_name, effort, start_time, duration)
		rospy.sleep(2)
		start_time = rospy.get_rostime()
		self._jointEffortApplier(joint_name, 0, start_time, duration_zero)
			# print(duration)