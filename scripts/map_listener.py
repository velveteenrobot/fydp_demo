#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np

def callback(data):
    print type(data)
    print data.info.width
    print data.info.height
    grid = np.zeros((data.info.height, data.info.width))
    for i in range(data.info.width):
        for j in range(data.info.height):
            grid[j,i] = data.data[data.info.width*j + i]
    print grid
          

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/mapping/driveable", OccupancyGrid, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
