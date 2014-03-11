import numpy as np
import scipy as sp
import scipy.ndimage as spimage
import scipy.misc
import matplotlib
import matplotlib.pyplot as plt

import utils

def test():
    _m = np.ones((50,50))
    _m[0:40, 20:23] = 0
    _m[10:50, 32:35] = 0
    sim = RobotSimVision(_m, 15, 4, 1.0, True)
    sim.update_simulated_vision(25, 25)
    return sim

class RobotSimVision:
    """
    Wrapper class for all the vision simulation that might be required
    """
    def __init__(self, underlying_map, max_sight_rad, min_sight_rad, name, dsample_scale, occluded_mode=True, ):
        # internal map format has free/traversible cells as True/1 and obstacles/occlusions as False/0
        self._map = underlying_map
        self.rmax = max_sight_rad
        self.rmin = min_sight_rad
        self.dsample_scale = dsample_scale
        self.occluded_mode = occluded_mode  # if self.occluded_made, we are dealing with solid obstacles that occlude vision, else we're dealing with lanes
        
        self.current_obstacle_map = np.zeros(self._map.shape) # cells that are traversible within current sight field are true/1
        self.current_vision = np.zeros(self._map.shape) # cells that we have vision of at this moment are true/1
        self.delta_bound_edges = np.zeros(self._map.shape)  # all new cells that are bound by obstacles found in this time step
        self.current_bound_edges = np.zeros(self._map.shape) # cells that are bound by obstacle are true/1
        
        self.memory_obstacle_map = np.zeros(self._map.shape)
        self.memory_vision = np.zeros(self._map.shape) # cells that we have had vision of at some point (including this moment) are true/1
        self.previous_bound_edges = np.zeros(self._map.shape) # all the bound edges at the last time step
        
        self.name = name
    
    def get_mask(self, y, x):
        # true/1 where we would have vision if unobstructed
        y,x = np.ogrid[-y:self._map.shape[0]-y, -x:self._map.shape[1]-x]
        mask = (x*x + y*y <= self.rmax**2) & (x*x + y*y >= self.rmin**2)
        return mask
        
    def get_vision_disc(self, y, x):
        # true/1 where we both would have vision, and not an obstacle (occluded or not)
        return np.logical_and(self.get_mask(y,x), self._map)
    
    def get_pretty_vision_disc(self, y, x):
        a = self.get_vision_disc(y,x).astype(np.int8)
        a[self.get_mask(y,x) == 0] = -1
        return a
        
    def update_simulated_vision(self, y,x):
        self.simulate_current_vision(y,x)   # update current vision and obstacle map
        self.memory_vision = np.logical_or(self.memory_vision, self.current_vision) # store to memory
        self.memory_obstacle_map = np.logical_or(self.memory_obstacle_map, self.current_obstacle_map)    # store to memory
        self.previous_bound_edges = np.logical_or(self.previous_bound_edges, self.current_bound_edges)
        self.simulate_current_edges(y,x)

    def simulate_current_edges(self, y,x):
        if self.occluded_mode:
            test_point = np.transpose(np.nonzero(self.current_vision))[0]
            areas, num = spimage.measurements.label(np.logical_and(self.memory_vision, self._map))
            cur_label = areas[test_point[0], test_point[1]]
            traversible = areas == cur_label

            edges = traversible - utils.simple_erode(traversible)
            self.current_bound_edges = self._map - utils.obstacle_erode(self._map, edges)
            self.delta_bound_edges = self.current_bound_edges - self.previous_bound_edges
        else:
            self.current_bound_edges = np.logical_and(self.current_vision, np.logical_not(self._map))
            self.delta_bound_edges = np.logical_and(self.current_bound_edges, np.logical_not(self.previous_bound_edges))
    
    def simulate_current_vision(self,y,x):
        if self.occluded_mode:
            # occluding obstacles - simulating lidar and laser
            # vision and obstacle map are the same! that is obstacles do not directly appear in the map
            vision = self.simulate_occluded_vision(y,x)
            self.current_obstacle_map = vision
            self.current_vision = vision
        else:
            # non-occluding obstacles - ie simulating lane detection
            # vision and obstacle map are not the same!
            self.current_vision = self.get_mask(y,x)
            self.current_obstacle_map = self.get_vision_disc(y,x)
         
         
    def get_sight_map(self, y, x):
        ntheta = 360.
        dtheta = 2.*np.pi/ntheta
        num_steps = np.ceil(self.rmax - self.rmin)
        sight_map = np.zeros((ntheta,1))
        angle_list = np.zeros(sight_map.shape)
        dvector = np.array([0,0])
        for i in range(int(ntheta)):
            angle = i * dtheta
            dvector = (self.rmax - self.rmin) * np.array([np.cos(angle), np.sin(angle)]) / num_steps
            min_step = self.rmin * np.array([np.cos(angle), np.sin(angle)])
            
            looking = True
            j = 0
            furthest_los = 0
            while looking:
                if j > num_steps:
                    looking = False
                    furthest_los = num_steps
                test_point = np.round(np.array([y,x]) + min_step + dvector * j)
                if self.is_map_occluded(test_point[0],test_point[1]):
                    furthest_los = j-1
                    looking = False
                j = j + 1
            angle_list[i] = angle
            sight_map[i] = furthest_los * (self.rmax - self.rmin) / num_steps
        
        return (sight_map, angle_list)
            
    def get_vision_bounds(self, y, x):
        x_bound = np.array([x-self.rmax,x+self.rmax])
        y_bound = np.array([y-self.rmax,y+self.rmax])
        if x_bound[0] < 0:
            x_bound[0] = 0
        if x_bound[1] > self._map.shape[1]-1:
            x_bound[1] = self._map.shape[1]-1
        if y_bound[0] < 0:
            y_bound[0] = 0
        if y_bound[1] > self._map.shape[0]-1:
            y_bound[1] = self._map.shape[0]-1
        return (y_bound, x_bound)
        
    def simulate_occluded_vision(self, y, x):
        # for when dealing with occluding obstacles
        # in the occluding case, vision and obstacle map are the same
        vision = np.zeros(self._map.shape)
        mask = self.get_mask(y, x)
        sight_map, angle_list = self.get_sight_map(y,x)
        coses = np.cos(angle_list) * (sight_map + self.rmin)
        sines = np.sin(angle_list) * (sight_map + self.rmin)
        s = np.abs(np.hstack((coses,sines)))
        
        y_bound, x_bound = self.get_vision_bounds(y,x)
        
        vector = np.array((0,0))
        
        for j in range(y_bound[0], y_bound[1]):
            for i in range(x_bound[0], x_bound[1]):
                if mask[j,i]:
                    vector[0] = j-y   
                    vector[1] = i-x
                    angle = np.arctan2(vector[1], vector[0])
                    if angle < 0:
                        angle = angle + 2.*np.pi
                    idx1 = np.argmin(np.abs(angle_list-angle))
                    
                    if angle_list[idx1] > angle:
                        idx2 = idx1 -1
                    else:
                        idx2 = idx1 + 1
                        
                    if idx1 == 0:
                        idx2 == angle_list[-1]
                    elif idx1 == len(angle_list)-1:
                        idx2 = 0
                        
                    vector = np.abs(vector)
                    if (vector[0] < s[idx1,0] and vector[1] < s[idx1,1]) or (vector[0] < s[idx2,0] and vector[1] < s[idx2,1]):
                        vision[j,i] = 1
        return vision   

    def is_map_occluded(self, y, x):
        if y < 0 or y >= self._map.shape[0] or x < 0 or x >= self._map.shape[1]:
            return False
        else:
            return not self._map[y,x]
    
        
    