import numpy as np
import scipy as sp
import scipy.ndimage as spimage
import scipy.misc
import matplotlib
import matplotlib.pyplot as plt


import vision_test
import utils
import dstarlite

import time

class Robot:
    def __init__(self, map_strc=None):
        self.loc = np.array((0,0))
        self.srad = 0
        self.waypoints = []
        self.progress = 0
        self.past_loc = 0
        
        self.planner = None
        self.path = []
        
        if map_strc is None:
            pass
        else:
            self.map_strc = map_strc
            
        self.current_vision = np.zeros(map_strc.obstacle.shape)
        self.all_vision = np.zeros(map_strc.obstacle.shape)
        self.traversible = np.zeros(map_strc.obstacle.shape)
        self.delta_vision = np.zeros(map_strc.obstacle.shape)
            
        self.free_edges = np.zeros(map_strc.obstacle.shape)
        self.free_edges_history = np.zeros(map_strc.obstacle.shape)
        self.bound_edges = np.zeros(map_strc.obstacle.shape)
        self.free_points = []
        self.delta_bound_edges = np.zeros(map_strc.obstacle.shape)
        
        dsample = False
        self.dsample_scale = map_strc.dsample_scale
        self.dsample_bound_edges = np.zeros(map_strc.dsample_obstacle.shape)
        self.dsample_free_points = []         

        self.planner_timer = 0
            
    def test_step(self):
        self.update_vision()
        self.update_planner_vision_map(self.delta_bound_edges)
        self.update_planner_location(self.loc)
        self.run_planner()
        
    def load_params(self, s_start, waypoints):
        self.o_start = s_start
        self.loc = s_start
        self.o_waypoints = np.asarray(waypoints)
        if np.ndim(self.o_waypoints) == 1:
            self.o_waypoints = self.o_waypoints[None,...]
        
    def load_planner(self):
        self.planner = dstarlite.dlite(drive_mode='External', _map = np.ones(self.map_strc.obstacle.shape), 
            start = self.o_start, goal=self.o_waypoints[0,:]    )
        self.planner_last_map = np.ones((self.map_strc.obstacle.shape))
        
    def update_planner_vision_map(self, map_delta):
        # gets a map delta and passes to planner
        self.planner.extdrive_buffer_vis(map_delta)
        
    def update_planner_location(self, new_pos):
        # gets current robot position and passes into planner
        # planner will interpolate from its last position to keep the algorithm happy
        self.planner.extdrive_buffer_pos(new_pos)
        
    def update_planner_goal(self, new_goal):
        print "In robot.update_planner_goal()"
        self.planner.extdrive_buffer_newgoal(new_goal)
       
    def move_to_next_waypoint(self):
        self.update_planner_goal(self.current_goal())
       
    def run_planner(self):
        stime = time.clock()
        status, path = self.planner.extdrive_compute()
        self.planner_last_map = np.copy(self.planner._map)
        self.path = path
        self.planner_timer += time.clock() - stime
        print ("Planner time: ", time.clock() - stime)
        print ("Robot at: ", self.planner.s_start)
        
    def current_goal(self):
        """Get current coords to chase after."""
        return self.waypoints[self.progress]
        
    def is_at_current_goal(self):
        threshold = 0.1
        if (self.loc[0] - self.waypoints[self.progress][0])**2 + (self.loc[1] - self.waypoints[self.progress][1])**2 < threshold **2:
            return True
        else:
            return False
        
    def test_load(self):
        self.loc = (650/4,400/4)
        #self.loc = (703,210)

        self.srad = 25
        self.waypoints.append((650/4,100/4))
        #self.waypoints.append((500,100))
        self.waypoints.append((100,25))
        
        self.load_params( np.asarray(self.loc), np.asarray(self.waypoints))
         
        
        # self.waypoints.append((40,125))
        # self.waypoints.append((200,200))
        # self.waypoints.append((20,360))
        # self.waypoints.append((280, 250))

    def update_position(self, data=None):
        """ 
        Reality/ROS hooks into here
        TODO: Mocked out for now, will implement later
        """
        pass
        
    def update_vision(self, data=None):
        """
        This is probably the best place to hook into ROS
        """
        self.current_vision = vision_test.get_robot_vision(self.map_strc, self.loc[0], self.loc[1], self.srad)
        self.all_vision = np.logical_or(self.all_vision, self.current_vision)
        self.update_traversible()
        self.update_edges()
        
    def update_edges(self):
        """Finds free and bound points on the edge of traversible region."""
        prev = self.free_edges
        e, self.free_edges, self.bound_edges = self.get_vision_edges(self.traversible)
        m = np.max(self.free_edges_history)
        new = np.logical_and(self.free_edges - prev, self.free_edges)
        new = (new).astype(np.int16) * (m+1)
        self.free_edges_history = self.free_edges_history + new
        self.free_points = np.asarray(np.nonzero(self.free_edges)).T
        
    def get_vision_edges(self, region_of_interest, dsample=False):
        # region_of_interest will typically be the traversible region
        if dsample:
            edges = region_of_interest - utils.simple_erode(region_of_interest, _mask=None, connectivity=4)
            bound_edges = self.map_strc.dsample_obstacle - utils.simple_erode(self.map_strc.dsample_obstacle, edges, connectivity=4)
        else:
            prev_bound = np.copy(self.bound_edges)
            edges = region_of_interest - utils.simple_erode(region_of_interest)
            bound_edges = self.map_strc.obstacle - utils.obstacle_erode(self.map_strc.obstacle, edges)
        free_edges = edges - bound_edges
        self.delta_bound_edges = bound_edges - prev_bound
        return (edges, free_edges, bound_edges)  

    def update_traversible(self):
        """Finds the directly traversible region.
        
        Finds all points which the robot can move to with 100% certainty.
        """
        areas, num = spimage.measurements.label(np.logical_and(self.all_vision, self.map_strc.obstacle))
        cur_label = areas[self.loc[0], self.loc[1]]
        self.traversible = areas == cur_label
        
