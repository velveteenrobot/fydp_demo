import roslib
import numpy as np
import scipy as sp
import scipy.ndimage
import scipy.misc
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import mapload 
import time

class Maps:
    def __init__(self, dsample_scale = None):
        """Initialize Maps object with a down sampling factor."""
        if dsample_scale is None:
            dsample_scale = 0.125
        self.obstacle = []
        self.occlusion = []
        self.dsample_obstacle = []
        self.dsample_occlusion = []
        self.res = 1
        self.dsample_scale = dsample_scale
        
    def load_map(self, map_string=None):
        """Loading obstacle and occlusion maps and applies downsampling."""
        if map_string is None:
            path = roslib.packages.get_pkg_dir("fydp_demo")
            map_string = path.rstrip("/") + "/src/fydp_demo/IGVCmap.tif"
            
        obs_weights = np.array([1.,1.,1.])/3.
        obstacle_threshold = 0.5
        occ_weights = np.array([1.,-1.,-1.])
        occlusion_threshold = 0.5

        rawmap = matplotlib.image.imread(map_string)

        self.obstacle = (obs_weights[0]*rawmap[:,:,0] + obs_weights[1]*rawmap[:,:,1] + obs_weights[2]*rawmap[:,:,2]) > (255*obstacle_threshold)
        occlusion_map = (occ_weights[0]*rawmap[:,:,0] + occ_weights[1]*rawmap[:,:,1] + occ_weights[2]*rawmap[:,:,2]) > (255*occlusion_threshold)
        self.occlusion = np.logical_not(occlusion_map)
        self.dsample_obstacle = sp.misc.imresize(self.obstacle, self.dsample_scale, interp='nearest').view(dtype=bool)
        self.dsample_occlusion = sp.misc.imresize(self.occlusion, self.dsample_scale, interp='nearest').view(dtype=bool)

        
def get_mask(_map,y,x,r):
    """Gives boolean array of visible region"""
    if isinstance(_map, Maps):
        _map = _map.obstacle
    y,x = np.ogrid[-y:_map.shape[0]-y, -x:_map.shape[1]-x]
    mask = x*x + y*y <= r*r
    return mask
    
def get_vision_disc(_map, y,x,r):
    if isinstance(_map, Maps):
        m = _map.obstacle
    else:
        m = _map
    mask = get_mask(m,y,x,r)
    return np.logical_and(m, mask)
    
def get_robot_vision(_map, y,x,r):
    if isinstance(_map, Maps):
        vision = np.zeros(_map.obstacle.shape)
    else:
        vision = np.zeros(_map.shape)
        
    mask = get_mask(_map, y, x, r)
    sight_map, angle_list = get_sight_map(_map,y,x,r)
    coses = np.cos(angle_list) * sight_map
    sines = np.sin(angle_list) * sight_map
    s = np.abs(np.hstack((coses,sines)))
    
    y_bound, x_bound = get_vision_bounds(_map, y,x,r)
    
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
                
def get_vision_bounds(_map, y, x,r):
    if isinstance(_map, Maps):
        _map = _map.obstacle
    x_bound = np.array([x-r,x+r])
    y_bound = np.array([y-r,y+r])
    if x_bound[0] < 0:
        x_bound[0] = 0
    if x_bound[1] > _map.shape[1]-1:
        x_bound[1] = _map.shape[1]-1
    if y_bound[0] < 0:
        y_bound[0] = 0
    if y_bound[1] > _map.shape[0]-1:
        y_bound[1] = _map.shape[0]-1
    return (y_bound, x_bound)
            
def get_sight_map(_map, y, x, r):
    stime = time.clock()
    ntheta = 360.
    dtheta = 2.*np.pi/ntheta
    num_steps = np.ceil(r)
    
    sight_map = np.zeros((ntheta,1))
    angle_list = np.zeros(sight_map.shape)
    dvector = np.array([0,0])
    
    for i in range(int(ntheta)):
        angle = i * dtheta
        dvector = r * np.array([np.cos(angle), np.sin(angle)]) / num_steps
        
        looking = True
        j = 0
        furthest_los = 0
        while looking:
            j = j + 1
            if j > num_steps:
                looking = False
                furthest_los = num_steps
            test_point = np.round(np.array([y,x]) + dvector * j)
            if is_map_occluded(_map,test_point[0],test_point[1]):
                furthest_los = j-1
                looking = False
        angle_list[i] = angle
        sight_map[i] = furthest_los * r / num_steps
    
    print (time.clock() - stime)
    
    return (sight_map, angle_list)
            
def is_map_occupied(_map, y,x):
    if isinstance(_map, Maps):
        _map = _map.obstacle
    if y < 0 or y >= _map.shape[0] or x < 0 or x >= _map.shape[1]:
        return False
    else:
        return not _map[y,x]
        
def is_map_occluded(_map, y, x):
    if isinstance(_map, Maps):
        _map = _map.occlusion
    if y < 0 or y >= _map.shape[0] or x < 0 or x >= _map.shape[1]:
        return False
    else:
        return not _map[y,x]
       
