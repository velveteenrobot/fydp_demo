#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from fydp_demo.msg import ips_msg

import numpy as np
import scipy as sp
import scipy.ndimage as spimage
import scipy.misc
import matplotlib
from matplotlib import animation
import matplotlib.pyplot as plt

import mtRobot as mtRobot

from numpy import int64
from numpy import int16

global map_ready
global pose_ready
pose_ready = False
map_ready = False
global map_size
global robot
global map_res

def map_callback(data):
    global map_ready
    global robot
    global map_size
    global map_res
    if map_ready:
        grid = np.zeros((data.info.height, data.info.width))
        for i in range(data.info.width):
            for j in range(data.info.height):
                if data.data[data.info.width*j + i] < 0.5:
                    grid[i,j] = 0
                elif data.data[data.info.width*j + i] == -1:
                    grid[i,j] = 0
                else:
                    grid[i,j] = 1
        robot.update_vision_gazebo(np.flipud(grid.T))
        robot.debug_map = np.flipud(grid.T)
    else:
        map_size = (data.info.height, data.info.width)
        map_ready = True
        map_res = float(data.info.resolution)

def position_callback(data):
    global pose_ready
    global map_ready
    global robot
    global map_res
    if map_ready:
        location = []
        # coordinates in the gazebo frame of reference
        gazebo_y = data.Y;  gazebo_x = data.X
        # refer to turtlebot_example where we get shifted_point for the inverse math
        # now convert to planner frame of reference
        planner_x = int(np.round(gazebo_x / map_res + float(map_size[1]) / 2) )
        planner_y = int(np.round(float(map_size[0]) / 2 - gazebo_y / map_res) )

        #print gazebo_x, gazebo_y, planner_x, planner_y

        location.append(planner_y); location.append(planner_x)
        robot.update_position(location)
        pose_ready = True
    else:
        pass

if __name__ == '__main__':

    global robot
    robot = mtRobot.mtRobot(simulation=False)

    MAP_TOPIC = '/map'

    pub = rospy.Publisher('waypoints', PoseArray)
    pub2 = rospy.Publisher('waypoints_done', Bool)

    map_sub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, map_callback)
    position_sub = rospy.Subscriber('/indoor_pos', ips_msg, position_callback)
    rospy.init_node('global_planner')
    while map_ready is False:
        rospy.sleep(1)
    
    robot.initialize_robot()
    # This is a dummy sensor, it gets completly ignord
    robot.add_simulated_sensor('IGVC-lane.tif', 25, 0, 'lanes', occluded=False)
    #robot.add_simulated_sensor('IGVC-obs.tif', 25, 0, 'obstacles', occluded=True)
    robot.sensors[0]._map = np.zeros(map_size)
    robot.combined_map = np.zeros(map_size)
    robot.load_planner()

    while pose_ready is False:
        rospy.sleep(1)

    cnt = 0
    done = False

    while not done:
        print "IN MAIN LOOP ", cnt
        robot.test_step()

        plt.figure()
        name = "debug/vidout-%03d.png" %cnt
        if cnt == 1:
            pass
            #import pdb; pdb.set_trace()
        debug_map = robot.planner._map
        plt.imshow(debug_map)
        plt.scatter(robot.loc[1], robot.loc[0], c='b')
        plt.scatter(robot.path[:,1], robot.path[:,0], c='r', s=20, linewidths=0)
        plt.savefig(name)
        plt.close()

        waypoint_array = PoseArray();
        print robot.loc, robot.current_goal()

        for i in range(robot.path.shape[0]):
            waypoint = Pose()
            waypoint.position.x = robot.path[i,1]
            #print "x: " + str(robot.path[i,1])
            waypoint.position.y = robot.path[i,0]
            #print "y: " + str(robot.path[i,0])
            waypoint_array.poses.append(waypoint)

        for i in waypoint_array.poses:
            #print i.position.x, i.position.y
            pass
        pub.publish(waypoint_array)
        rospy.sleep(0.5)
        
        if robot.is_at_current_goal():
            if robot.progress + 1 == len(robot.waypoints):
                waypoints_done = Bool()
                waypoints_done.data = True
                pub2.publish(waypoints_done)
                done = True
            else:
                robot.progress = robot.progress + 1
                robot.move_to_next_waypoint()
        
        cnt += 1
    
#ani = animation.ArtistAnimation(fig, ims, interval=250, blit=True, repeat_delay=1000)
    
