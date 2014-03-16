import numpy as np
import scipy as sp
import scipy.ndimage as spimage
import scipy.misc
import matplotlib
import matplotlib.pyplot as plt

import RobotSimVision
import backoff_dstarlite as dstarlite
import time

class mtRobot:
    """
    Robot for multi-tier sensor and planner. Can load in arbitray number of sensor feeds and will feed into planner.
    """
    def __init__(self, simulation=True):
        self.loc = np.array((0,0))
        self.waypoints = []
        self.progress = 0
        self.past_loc = 0
        
        self.planner = None
        self.path = []
        self.sensors = []
        self.combined_map = []
        
        self.simulation = simulation

        self.prev_map = None

        self.t = time.clock()
        self.cnt = 0

        self.temp_stupid = None
        self.DEBUG_PRINT = False

    def add_prev_map(self, p_map):
        self.prev_map = p_map
        
    def add_simulated_sensor(self, map_string, max_sight, min_sight, name, occluded=True, vflipped=False):
        rawmap = matplotlib.image.imread(map_string)
        _map = (0.1*rawmap[:,:,0] + 0.1*rawmap[:,:,1] + 0.1*rawmap[:,:,2]) > 50
        if vflipped:
            _map = np.flipud(_map)
        s = RobotSimVision.RobotSimVision(_map, max_sight, min_sight, name, 1.0, occluded)
        self.sensors.append(s)
  
    def test_step(self):
        if self.simulation:
            self.update_vision()
            self.update_planner_vision_map()
            self.update_planner_location(self.loc)
            self.run_planner()
        else:
            self.cnt += 1
            print "in test step"
            self.update_planner_vision_map()
            self.update_planner_location(self.loc)
            self.run_planner()

        if self.DEBUG_PRINT:
                plt.figure()
                plt.imshow(self.planner._map)
                plt.scatter(self.planner.s_start[1], self.planner.s_start[0])
                plt.scatter(self.path[:,1], self.path[:,0])
                name = "blehhhh-%03d.png" %self.cnt
                plt.savefig(name)

        
    def load_params(self, s_start, waypoints):
        self.o_start = s_start
        self.loc = s_start
        self.o_waypoints = np.asarray(waypoints)
        if np.ndim(self.o_waypoints) == 1:
            self.o_waypoints = self.o_waypoints[None,...]
        
    def load_planner(self):
        self.planner = dstarlite.dlite(drive_mode='External', _map = np.ones(self.sensors[0]._map.shape), 
            start = self.o_start, goal=self.o_waypoints[0,:]    )
        
    def update_planner_vision_map(self):
        # Pushes the robot's current vision to planner
        print "in update_planner_vision_map"
        self.planner.extdrive_buffer_vis(self.combined_map)
        
    def update_planner_location(self, new_pos):
        # gets current robot position and passes into planner
        # planner will interpolate from its last position to keep the algorithm happy
        print "update_planner_location"
        self.planner.extdrive_buffer_pos(new_pos)
        
    def update_planner_goal(self, new_goal):
        print "In robot.update_planner_goal()"
        self.planner.extdrive_buffer_newgoal(new_goal)
       
    def move_to_next_waypoint(self):
        self.update_planner_goal(self.current_goal())
       
    def run_planner(self):
        status, path = self.planner.extdrive_compute()
        self.path = path
        
    def current_goal(self):
        """Get current coords to chase after."""
        return self.waypoints[self.progress]
        
    def is_at_current_goal(self):
        threshold = 0.1
        if (self.loc[0] - self.waypoints[self.progress][0])**2 + (self.loc[1] - self.waypoints[self.progress][1])**2 < threshold **2:
            return True
        else:
            return Falset 
        
    def initialize_robot(self, start_loc = None, waypoints = None):
        if start_loc is None:
            start_config_file = open("../../config/planner_start.txt", "r")
            l = start_config_file.readline().split(",")
            self.loc = (int(l[0].strip()), int(l[1].strip()))
            #self.loc = (0,0)
        else:
            self.loc = start_loc
        if waypoints is None:
            self.waypoints = []
            waypoint_config_file = open("../../config/waypoints_start.txt", "r")
            for line in waypoint_config_file:
                l = [x.strip() for x in line.split(",")]
                self.waypoints.append((int(l[0]), int(l[1])))
            #self.waypoints.append((9*10,19*10))
            #self.waypoints.append((5*10,2*10)) 
        else:
            self.waypoints = waypoints
        self.load_params( np.asarray(self.loc), np.asarray(self.waypoints))

    def update_position(self, location):
        """ 
        Reality/ROS hooks into here
        TODO: Mocked out for now, will implement later
        """
        self.loc[0] = location[0]
        self.loc[1] = location[1]

    def update_vision_gazebo(self, new_vision):
        """ Takes a threshold version of the ROS occupancy grid, expects 1 for obstacle, 0 for free """
        if not self.simulation:
            self.combined_map = np.copy(new_vision)
        
    def update_vision(self, data=None):
        """
        This is probably the best place to hook into ROS
        """

        if self.simulation:
            self.combined_map = np.zeros(self.sensors[0]._map.shape)

            for sensor in self.sensors:
                sensor.update_simulated_vision(self.loc[0], self.loc[1])
                self.combined_map = np.logical_or(self.combined_map, sensor.current_bound_edges)
        
        