#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool

import numpy as np
import scipy as sp
import scipy.ndimage as spimage
import scipy.misc
import matplotlib
from matplotlib import animation
import matplotlib.pyplot as plt

import vision_test
import utils
import Robot

from numpy import int64
from numpy import int16

m = vision_test.Maps(dsample_scale=1.00)
m.load_map('IGVCmap3.tif')


robot = Robot.Robot(m)
robot.test_load()
robot.load_planner()

#robot.waypoints.append((100,300))

#robot.current_vision = vision_test.get_robot_vision(m, robot.loc[0], robot.loc[1], robot.srad)
#robot.all_vision = robot.current_vision

cnt = 0
done = False

record = []
record2 = []
record3 = []
record4 = []

pub = rospy.Publisher('waypoints', PoseArray)
pub2 = rospy.Publisher('waypoints_done', Bool)
rospy.init_node('global_planner')

while not done:
    #raw_input('CLICK TO GLOBAL PLAN!')
    robot.test_step()
    
    if robot.path.shape[0] < 11:
        robot.loc = robot.path[-1,:]
    else:
        robot.loc = robot.path[10,:]
        
    """record.append(robot.planner.key_record)
    record2.append(robot.planner.g_score)
    record3.append(robot.loc)
    record4.append(robot.planner.rhs)
    """

    waypoint_array = PoseArray();

    for i in range(robot.path.shape[0]):
        waypoint = Pose()
        waypoint.position.x = robot.path[i,1]
        print "x: " + str(robot.path[i,1])
        waypoint.position.y = robot.path[i,0]
        print "y: " + str(robot.path[i,0])
        waypoint_array.poses.append(waypoint)

    pub.publish(waypoint_array)
    rospy.sleep(3)

    for i in waypoint_array.poses:
        print i.position.x, i.position.y

    #
    if robot.is_at_current_goal():
        if robot.progress + 1 == len(robot.waypoints):
            waypoints_done = Bool()
            waypoints_done.data = True
            pub2.publish(waypoints_done)
            print "Done"
            done = True
        else:
            robot.progress = robot.progress + 1
            robot.move_to_next_waypoint()
    
#ani = animation.ArtistAnimation(fig, ims, interval=250, blit=True, repeat_delay=1000)
    
        
