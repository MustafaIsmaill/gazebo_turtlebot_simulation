#!/usr/bin/env python

from gazebo_robot import *
import rospy

if __name__ == '__main__':
    try:

        turtlebot = gazebo_robot("turtlebot", 0.0, 0.0, 0.0)
    
    except rospy.ROSInterruptException:
        print("exception occured")
