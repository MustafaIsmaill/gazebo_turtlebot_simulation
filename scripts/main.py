#!/usr/bin/env python

from gazebo_robot import *
import rospy
import numpy as np
from math import pi

if __name__ == '__main__':
	try:

		turtlebot = gazebo_robot("turtlebot_move", "turtlebot", 0.0, 0.0, 0.0)
		turtlebot.getModelState()

		rospy.sleep(1)

		turtlebot.setPosition(5, 5, 10)
		turtlebot.getModelState()

	except rospy.ROSInterruptException:
		print("exception occured")
