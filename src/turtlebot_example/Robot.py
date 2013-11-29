"""
TODO: 
    Pull out all the maps, and make it work with both standalone testing, and embedding into ROS
"""
import numpy as np
import scipy as sp
import scipy.ndimage as spimage
import scipy.misc
import matplotlib
import matplotlib.pyplot as plt

import vision_test
import utils
import astar

from numpy import int64
from numpy import int16

class Robot:
    def __init__(self, _map):
        """Initialize crap."""
        # _map really only exists as a figment of running the simulation purely in python
        self.loc = (0,0)
        self.srad = 0
        self.waypoints = []
        self.progress = 0
        self.past_loc = []
        
        self.mapsize = (0,0)
        self.current_vision = np.zeros(_map.obstacle.shape)
        self.all_vision = np.zeros(_map.obstacle.shape)
        self.traversible = np.zeros(_map.obstacle.shape)
        
        self.connectivity = 4
        
        self.free_edges = np.zeros(_map.obstacle.shape)
        self.bound_edges = np.zeros(_map.obstacle.shape)
        self.free_points = []
        
        self.map = _map
        self.dsample_scale = _map.dsample_scale
        self.dsample_trav = []
        self.dsample_free_edge = []
        self.dsample_edge = []
        self.dsample_vision = []
    
    def current_goal(self):
        """Get current coords to chase after."""
        return self.waypoints[self.progress]
        
    def update_vision(self, _map):
        """
        This is probably the best place to hook into ROS
        """
        self.current_vision = vision_test.get_robot_vision(_map, self.loc[0], self.loc[1], self.srad)
        self.all_vision = np.logical_or(self.all_vision, self.current_vision)
        self.update_traversible(_map)
        self.update_edges(_map)
    
    def update_traversible(self, _map):
        """Finds the directly traversible region.
        
        Finds all points which the robot can move to with 100% certainty.
        """
        areas, num = spimage.measurements.label(np.logical_and(self.all_vision, _map.obstacle))
        cur_label = areas[self.loc[0], self.loc[1]]
        self.traversible = areas == cur_label
        
    def update_edges(self, _map):
        """Finds free and bound points on the edge of traversible region."""
        e, self.free_edges, self.bound_edges = self.get_vision_edges(_map, self.traversible)
        self.free_points = np.nonzero(self.free_edges)
        
    def get_vision_edges(self, _map, region_of_interest, dsample=False):
        # region_of_interest will typically be the traversible region
        if dsample:
            edges = region_of_interest - utils.simple_erode(region_of_interest, _mask=None, connectivity=4)
            bound_edges = _map.dsample_obstacle - utils.simple_erode(_map.dsample_obstacle, edges, connectivity=4)
        else:
            edges = region_of_interest - utils.simple_erode(region_of_interest)
            bound_edges = _map.obstacle - utils.obstacle_erode(_map.obstacle, edges)
        free_edges = edges - bound_edges
        return (edges, free_edges, bound_edges)
        
    def pick_target(self, _map):
        """Fall back dead simple search for best point on the free edge."""
        if self.traversible[self.current_goal()] == 1:
            return self.current_goal()
        else:   
            points = np.transpose(np.array(np.nonzero(self.free_edges)))
            d1 = self.current_goal() - points
            d2 = points - self.loc
            d1 = np.apply_along_axis(np.linalg.norm, 1, d1)
            d2 = np.apply_along_axis(np.linalg.norm, 1, d2)
            d = d1+d2
            idx = np.argmin(d)
            return points[idx,:]
        
    def is_at_current_goal(self):
        threshold = 0.1
        if (self.loc[0] - self.waypoints[self.progress][0])**2 + (self.loc[1] - self.waypoints[self.progress][1])**2 < threshold **2:
            return True
        else:
            return False

    def dsample_and_pick(self, _map):
        """Run AStar to find best point on free edge to move to.
        
        Downsamples the map, gets downsampled version of free and bound edge. A* search through
        the bound edge (this will prevent you from moving through obstacles). Gets the path, and 
        intersects with the free_edge to find the free_edge point to move to.
        
        Running using 4 connectivity as it has the least problem dealing with down sampling artefacts.
        
        Still completely unusable. Not stable - will miss certain free-edges in teh course of downsampling
        and is also stupidly slow.
        
        """
        if self.traversible[self.current_goal()] == 1:
            return self.current_goal()
        else:          
            scale = _map.dsample_scale
            dsample_traversible, dsample_all_vision, e,f,b,dsample_free_points = self.dsample(_map)
            goal = self.current_goal()
            goal = np.round(np.array([goal[0], goal[1]]) * scale)
            start = np.round(np.array([self.loc[0], self.loc[1]]) * scale)
            path = astar.astar(np.logical_not(b), start, goal, connectivity = 4)
            path = path.astype(np.int64)
            intersection = utils.intersect2d(path, dsample_free_points, dsample_traversible.shape)
            target = intersection[0,:]
            target = np.round(target / scale).astype(np.int16)
            return target
        
    def dsample(self, _map):
        scale = self.dsample_scale
        dsample_traversible = sp.misc.imresize(self.traversible, scale, interp='nearest').view(dtype=bool)
        dsample_all_vision = sp.misc.imresize(self.all_vision, scale, interp='nearest').view(dtype=bool)
        
        e,f,b = self.get_vision_edges(_map, dsample_traversible, dsample=True)
        dsample_free_points = np.asarray(np.nonzero(f)).T
        
        self.dsample_trav = dsample_traversible
        self.dsample_vision = dsample_all_vision
        self.dsample_free_edge = f
        self.dsample_edge = e
        
        return dsample_traversible, dsample_all_vision, e,f,b,dsample_free_points
        
    def test_load(self):
        self.loc = (650, 400)
        self.srad = 100
        self.waypoints.append((650,100))
        self.waypoints.append((100,300))
