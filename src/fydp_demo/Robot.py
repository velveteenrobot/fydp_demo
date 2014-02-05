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
        self.dsample_bound_edges = np.zeros(_map.dsample_obstacle.shape)
        self.dsample_free_points = []
        
        self.move_cnt = 0
    
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
        self.free_points = np.asarray(np.nonzero(self.free_edges)).T
        
    def get_vision_edges(self, _map, region_of_interest, dsample=False):
        # region_of_interest will typically be the traversible region
        if dsample:
            edges = region_of_interest - utils.simple_erode(region_of_interest, _mask=None, connectivity=4)
            bound_edges = _map.dsample_obstacle - utils.simple_erode(_map.dsample_obstacle, edges, connectivity=4)
        else:
            edges = region_of_interest - utils.simple_erode(region_of_interest)
            bound_edges = _map.obstacle - utils.obstacle_erode(_map.obstacle, edges)
        free_edges = edges - bound_edges
        free_edges = np.logical_and(free_edges, _map.expanded_obstacle)
        return (edges, free_edges, bound_edges)
        
    def get_edges_from_occupany_grid(occupancy_grid, robot_location,obstacle_thres=30):
        unknown_symbol = -1     # symbol used to represent unknown cells in the occupancy grid
        
        traversible = np.logical_and(np.greater(occupancy_grid, unknown_symbol), np.less(occupancy_grid, obstacle_thres))
        # Getting only the immediately traversible area - basically, using paint bucket on the robot's current position
        traversible_areas, num_areas = scipy.ndimage.measurements.label(traversible)
        current_area = traversible_areas[robot_location[0], robot_location[1]]
        traversible = (traversible_areas == current_area)
        
        obstacle = np.greater_equal(occupancy_grid, obstacle_thres)
        unknown = np.equal(occupancy_grid, unknown_symbol)
        
        # grow the unknown region into the traversible region. Intersect with the traversible region to get the free_edge
        g = scipy.ndimage.morphology.binary_dilation(unknown, iterations=1, border_value=0)
        free_edge = np.logical_and(g, traversible)
        
        # Same as above, but with obstacles to get the bound_edge
        g = scipy.ndimage.morphology.binary_dilation(obstacle, iterations=1, border_value=0)
        bound_edge = np.logical_and(g, traversible)
        
        # resolve conflicts in favor of free. only way to garuntee finding solutions
        bound_edge[free_edge] = False
            
        return free_edge, bound_edge, traversible
    
    def pick_target(self, _map):
        """Fall back dead simple search for best point on the free edge."""
        if self.traversible[self.current_goal()] == 1:
            return self.current_goal()
        else:   
            points = np.transpose(np.array(np.nonzero(self.free_edges)))
            #d1 = self.current_goal() - points
            #d2 = points - self.loc
            #d1 = np.apply_along_axis(np.linalg.norm, 1, d1)
            #d2 = np.apply_along_axis(np.linalg.norm, 1, d2)
            
            endlist = np.tile(self.current_goal(), (points.shape[0],1))
            curlist = np.tile(self.loc,(points.shape[0],1))
            d1 = np.sum(np.abs(curlist-points)**2, axis=-1)**(1./2)
            d2 = np.sum(np.abs(endlist-points)**2, axis=-1)**(1./2)
            
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
            dsample_traversible, dsample_bound_edges, dsample_free_points = self.dsample(_map)
            goal = self.current_goal()
            goal = np.round(np.array([goal[0], goal[1]]) * scale).astype(np.int16)
            start = np.round(np.array([self.loc[0], self.loc[1]]) * scale).astype(np.int16)
            
            path = astar.astar(dsample_bound_edges, start, goal, connectivity = 4)
            path = path.astype(np.int64)
            intersection = utils.intersect2d(path, dsample_free_points, dsample_traversible.shape)
            target = intersection[0,:]
            target = np.round(target / scale).astype(np.int16)
            #target = self.usample_target(target, _map)
            return target
        
    def usample_target(self, target, _map):
        search_grid_size = int(1 / self.dsample_scale)
        # naive_target is the top-left element of the originally sampled positions
        naive_target = np.floor(target / self.dsample_scale).astype(np.int16)
        
        for j in range(search_grid_size):
            for i in range(search_grid_size):
                if self.free_edges[naive_target[0]+j,naive_target[1]+i] == 1:
                    return (naive_target + np.array([j,i]))
        
    def dsample_edges(self, _map):
        scale = self.dsample_scale

        temp_bound = np.ones(_map.dsample_obstacle.shape)
        
        free_points = self.free_points
        bound_points = np.asarray(np.nonzero(self.bound_edges)).T
        
        free_points = np.floor(free_points * scale).astype(np.int16)
        bound_points = np.floor(bound_points * scale).astype(np.int16)
        
        
        temp_bound[ bound_points[:,0], bound_points[:,1] ] = 0
        temp_bound[ free_points[:,0], free_points[:,1] ] = 1
    
        return temp_bound, free_points
        
    def dsample(self, _map):
        scale = self.dsample_scale
        dsample_traversible = sp.misc.imresize(self.traversible, scale, interp='nearest').view(dtype=bool)
        dsample_bound_edges, dsample_free_points = self.dsample_edges(_map)
        
        return dsample_traversible, dsample_bound_edges, dsample_free_points
        
    def test_load(self):
        #self.loc = (650, 400)
        self.srad = 100
        #self.waypoints.append((650,100))
        #self.waypoints.append((100,300))
        self.loc = (950, 400)
        self.waypoints.append((950,100))
        self.waypoints.append((400,300))
