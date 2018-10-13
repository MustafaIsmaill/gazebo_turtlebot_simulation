#!/usr/bin/env python

from gazebo_msgs.msg import ModelState
import rospy

class gazebo_robot:

	def __init__(self, name, X, Y, Z): #initial robot conditions

		self._name = name
		self._x = X
		self._y = Y
		self._z = Z
		self._roll = 0.0
		self._pitch = 0.0
		self._yaw = 0.0

		rospy.init_node('turtlebot_sim', anonymous=True)

		self._modelStateSetter = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10, latch=True)
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
