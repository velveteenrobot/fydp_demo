#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

import numpy as np
import scipy as sp
import scipy.ndimage as spimage
import scipy.misc
import matplotlib
import matplotlib.pyplot as plt

import vision_test
import utils
import astar
import Robot

from numpy import int64
from numpy import int16



            
m = vision_test.Maps()
m.load_map()

robot = Robot.Robot(m)
robot.test_load()

#robot.waypoints.append((100,300))

#robot.current_vision = vision_test.get_robot_vision(m, robot.loc[0], robot.loc[1], robot.srad)
#robot.all_vision = robot.current_vision
def plan():
    pub = rospy.Publisher('waypoints', Pose)
    pub2 = rospy.Publisher('waypoints_done', Bool)
    rospy.init_node('global_planner')
    done = False
    while not (done or rospy.is_shutdown()):
        robot.update_vision(m)
        robot.past_loc.append(robot.loc)

        waypoint = Pose()
        waypoint.position.x = robot.loc[1]
        waypoint.position.y = robot.loc[0]
        pub.publish(waypoint)
        
        robot.loc = robot.pick_target(m)
        #robot.loc = robot.dsample_and_pick(m)
        if robot.is_at_current_goal():
            if robot.progress + 1 == len(robot.waypoints):
                waypoints_done = Bool()
                waypoints_done.data = True
                pub2.publish(waypoints_done)
                done = True
            else:
                robot.progress = robot.progress + 1


if __name__ == '__main__':
    try:
        plan()
    except rospy.ROSInterruptException:
        pass

    


        
