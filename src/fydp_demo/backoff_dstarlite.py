import numpy as np
import scipy.misc

import time

import matplotlib.pyplot as plt
import matplotlib
import copy
import utils

from dstarKey import dstarKey
from OpenSet import OpenSet

class dlite:
    def __init__(self, _map=None, start=None, goal=None, test_case=None, srad=None, drive_mode='internal'):
        if _map is None:
            (_map, start, goal) = self.deal_with_test_cases(test_case)

        # used in internal mode only
        self.real_map = _map
        # self._map is what the algorithm actually computes over. It is not a binary map. 
        #It is a float, with 1. being the lowest possible weight, representing fully traversible
        # when used in back_off mode, it will contain the smeared out values to keep the robot 
        # away from obstacles
        self._map = np.ones(_map.shape)     
        # self.prev_map stores the map form the prior planning cycle, used to compared edge weights 
        self.prev_map = None
        # self.pure_map should contain whatever the robot is feeding it. It should be in a 0. to 1.
        # float format, with 1. being certain of obstacle
        # self.pure_map should be considered of the same type as self.vis_buffer
        self.pure_map = np.ones(_map.shape)
            
        if srad is None:
            self.srad = 5
        else:
            self.srad = srad
            
        if drive_mode == 'internal':
            self.drive_mode = drive_mode
        elif drive_mode == 'external':
            self.drive_mode = drive_mode
            
        # Initializing helper constants and records
        self.narray = np.array([[1,0], [0,1], [-1, 0], [0, -1], [1,1], [1,-1], [-1,1], [-1,-1]])
        self.loop_flag = np.array([[10,10], [-10,-10]])
        self.true_count = 0
        self.pos_record = []
        
        # External drive buffers and flags
        self.newpos_flag = False
        self.pos_buffer = np.array((-1,-1))
        self.newvis_flag = False
        self.vis_buffer = np.zeros(_map.shape)		# considered paired with self.pure_map
        self.newgoal_flag = False
        self.newgoal_buffer = np.array((0,0))
        self.delta_vision_update = False
        
        # Initialize() from the paper
        self.s_goal = goal
        self.s_start = start
        self.U = OpenSet()
        self.k_m = 0
        self.rhs = np.zeros(self._map.shape)
        self.g_score = np.zeros(self._map.shape)
        self.rhs[self.rhs == 0] = np.inf
        self.g_score[self.g_score == 0] = np.inf
        self.rhs[self.s_goal[0], self.s_goal[1]] = 0
        k = self.calc_key(self.s_goal)
        self.U.add(k,self.s_goal)
        
        self.key_record = []
        
    def reset(self):
        """ Resets all memory effects, EXCEPT for map vision. 
        Used for rebuilds after loops, and for goal changes
        """
        self.U = OpenSet()
        self.k_m = 0
        self.rhs = np.zeros(self._map.shape)
        self.g_score = np.zeros(self._map.shape)
        self.rhs[self.rhs == 0] = np.inf
        self.g_score[self.g_score == 0] = np.inf
        self.rhs[self.s_goal[0], self.s_goal[1]] = 0
        k = self.calc_key(self.s_goal)
        self.U.add(k,self.s_goal)
    
    def sequential_deal(self):
        rolling_map = np.copy(self.prev_map)
        self.k_m = self.k_m + self.h_score(self.last, self.s_start)
        
        for i in np.arange(self.changed_nodes.shape[0]):
            p = self.changed_nodes[i,:]
            old_map = np.copy(rolling_map)
            rolling_map[p[0], p[1]] = self._map[ p[0], p[1] ]
            succ = self.get_pred(p)
            new_cost_list = self.gen_cost_list(p, succ, rolling_map)
            old_cost_list = self.gen_cost_list(p, succ, old_map)
            
            for j in np.arange(succ.shape[0]):
                if old_cost_list[j] > new_cost_list[j]:
                    # then we look at p to succ(j,:) as well as succ(j,:) to p
                    if not self.is_goal(p):
                        next_rhs = min(self.get_rhs(p), new_cost_list[j] + self.get_g_score(succ[j,:]))
                        self.set_rhs(p, next_rhs)
                    if not self.is_goal(succ[j,:]):
                        next_rhs = min(self.get_rhs(succ[j,:]), new_cost_list[j] + self.get_g_score(p))
                        self.set_rhs(succ[j,:], next_rhs)
                elif self.is_close(self.get_rhs(p), old_cost_list[j] + self.get_g_score(succ[j,:])):
                # If p is the destination after node succ[j,:]
                    if not self.is_goal(p):
                        succ = self.get_succ(p)
                        cost_list = self.gen_cost_list(p,succ,rolling_map) + self.rhs[succ[:,0], succ[:,1]]
                        self.set_rhs(p, np.min(cost_list))
                elif self.is_close(self.get_rhs(succ[j,:]), old_cost_list[j] + self.get_g_score(p)):
                # If succ[j,:] is the destination after node p
                    if not self.is_goal(succ[j,:]):
                        succ2 = self.get_succ(succ[j,:])
                        cost_list = self.gen_cost_list(succ[j,:], succ2, rolling_map) + self.rhs[succ2[:,0], succ2[:,1]]
                        self.set_rhs(succ[j,:], np.min(cost_list))
                

        for i in np.arange(self.changed_nodes.shape[0]):
            p = self.changed_nodes[i,:]
            succ = self.get_pred(p)
            for j in np.arange(succ.shape[0]):
                self.update_vertex(succ[j,:])
            self.update_vertex(p)
            
    def compute_shortest_path(self):
        self.key_record = []
        top = self.U.peek()
    
        while top[0] < self.calc_key(self.s_start) or (self.rhs[self.s_start[0], self.s_start[1]] > self.g_score[self.s_start[0], self.s_start[1]]):
            u = top
            k_old = u[0]
            coord = u[1]
            k_new = self.calc_key(coord)
            self.key_record.append(u[1])

            if k_old < k_new:
                self.U.remove(coord)
                self.U.add(k_new, coord)
            elif (self.get_g_score(coord) >= self.get_rhs(coord)):    
                self.set_g_score(coord, self.get_rhs(coord))
                self.U.remove(coord)
                pred = self.get_pred(coord)
                vals = np.minimum(self.rhs[pred[:,0], pred[:,1]], self.cost_list(coord, pred) + self.get_g_score(coord))
                
                for i in np.arange(pred.shape[0]):
                    if not self.is_goal(pred[i,:]):
                        self.set_rhs(pred[i,:], vals[i])
                    self.update_vertex(pred[i,:])
            else:
                g_old = self.get_g_score(coord)
                self.set_g_score(coord, np.inf)
                
                pred = self.get_pred(coord)
                for i in np.arange(pred.shape[0]):
                    p = pred[i,:]
                    if self.is_close(self.get_rhs(p), self.gen_cost(p, coord, self._map) + g_old):
                        if not self.is_goal(p):
                            succ = self.get_succ(p)
                            cost_list = self.cost_list(p, succ) + self.g_score[ succ[:,0], succ[:,1]]
                            self.set_rhs(p, np.min(cost_list))
                    self.update_vertex(pred[i,:])
                
                if self.is_close(self.get_rhs(coord), g_old):
                    if not self.is_goal(coord):
                        succ = self.get_succ(coord)
                        cost_list = self.cost_list(coord, succ) + self.g_score[ succ[:,0], succ[:,1]]
                        self.set_rhs(coord, np.min(cost_list))
                self.update_vertex(coord)
            top = self.U.peek()
            
    def update_vertex(self, coord):
        if self.get_g_score(coord) != self.get_rhs(coord):
            if self.U.contains(coord):
                self.U.remove(coord)
                self.U.add(self.calc_key(coord), coord)
            else:
                self.U.add(self.calc_key(coord), coord)
        else:
            if self.U.contains(coord):
                self.U.remove(coord)
                
    def extdrive_buffer_pos(self, new_pos):
        """ Buffers the robot position from reality/ROS and sets flag"""
        self.pos_buffer = np.copy(new_pos)
        self.newpos_flag = True
        
    def extdrive_buffer_vis(self, vision_update):
        """ Copies the robot's current vision into .vis_buffer. Should be a binary map with 1 as 
        obstacle.
        """
        self.vis_buffer = np.copy(vision_update)
        self.newvis_flag = True
        
    def extdrive_buffer_newgoal(self, new_goal):
        self.next_goal = np.asarray(new_goal)
        print "In extdrive_buffer_new_goal", self.next_goal
        self.newgoal_flag = True
        
    def extdrive_compute(self):
        """ Trigger from ROS/Robot to compute a path """
        self.pos_record.append(self.s_start)
        self.last = self.s_start
        self.s_start = self.pos_buffer
        print self.s_start, self.s_goal
        
        if self.newvis_flag:
            self.external_update_vision()
            if self.changed_nodes.shape[0] > 0:
                self.sequential_deal()
            self.newvis_flag = False
            
        if self.newgoal_flag:
            print "For new goal"
            self.reset()
            self.s_goal = self.next_goal
            self.reset()
            self.newgoal_flag = False
            self.compute_shortest_path()
            status = "Reset for new goal"
            path = self.get_path()        
        else:
            self.reset()
            self.compute_shortest_path()
            path = self.get_path()
            status = "Fine"
            if np.array_equal(path, self.loop_flag):
                # we have a loop
                s_time = time.clock()
                self.reset()
                self.compute_shortest_path()
                status = "Rebuild"
                path = self.get_path()       
            self.newpos_flag = False
        return (status, path)

    def external_update_vision(self):
        if self.newvis_flag:
            self.prev_map = np.copy(self._map)
            self.pure_map = np.copy(self.vis_buffer)
            changed_in_vision = self.vis_buffer - self.pure_map
            # new_backedoff_map is self._map compatible
            new_backedoff_map = self.get_backedoff_map(self.vis_buffer)	
            delta_nav_map = new_backedoff_map - self._map
            self._map = new_backedoff_map
            self.changed_nodes = np.transpose(np.nonzero(delta_nav_map))
            self.sort_changed_nodes()
            self.vis_buffer = np.zeros(self.vis_buffer.shape)
            
            self.newvis_flag = False
        
    def get_backedoff_map(self, binary_map, rad=3):
        """Takes binary map (1 occupied), returns something that can go right into self._map"""
        """Basically, will apply a 'potential' field of radius rad around all obstacles to prevent
        the planner from planning too close to obstacles """
        
        dim = binary_map.shape
        max_cost = np.sqrt(dim[0]**2 + dim[1]**2)
        if rad == 0:
            m = np.ones(dim)
            m += binary_map * max_cost
            return m
        else:
            m = np.ones(dim) + binary_map * max_cost
            smeared = np.copy(binary_map)
            for i in range(rad):
                r = i + 1
                smeared = utils.simple_erode(smeared)
                diff = np.logical_xor(smeared, binary_map)
                
                cost = max_cost * (1. - (float(r) / float(rad)))**2 / 100
                m += cost
            return m
            
    def get_pred(self, coord):
        pred = np.zeros([8,2])
        pred_cnt = 0
        if coord[0] >=1 and coord[0] < self._map.shape[0]-1 and coord[1] >=1 and coord[1] < self._map.shape[1]-1:
            return np.tile(coord, (8,1)) + self.narray
        else:
            for i in range(0,8):
                p = coord + self.narray[i,:]
                if p[0] < 0 or p[0] >= self._map.shape[0] or p[1] < 0 or p[1] >= self._map.shape[1]:
                    pass
                else:
                    pred[pred_cnt,:] = p
                    pred_cnt = pred_cnt + 1
            pred = pred[0:pred_cnt,:]
        return pred.astype(np.int16)
        
    def get_succ(self, coord):            
        if self._map[ coord[0], coord[1]] == 0:
            return []
        else:
            if coord[0] >=1 and coord[0] < self._map.shape[0]-1 and coord[1] >=1 and coord[1] < self._map.shape[1]-1:
                return np.tile(coord, (8,1)) + self.narray
            else:
                succ = np.zeros([8,2])
                succ_cnt = 0
                for i in range(0,8):
                    p = coord + self.narray[i,:]
                    if p[0] < 0 or p[0] >= self._map.shape[0] or p[1] < 0 or p[1] >= self._map.shape[1]:
                        pass
                    else:
                        succ[succ_cnt,:] = p
                        succ_cnt = succ_cnt + 1
                succ = (succ[0:succ_cnt,:])
                return succ.astype(np.int16)
    
    def get_path(self):
        cur = self.s_start
        path = []
        path.append(cur)
        while not cur[0] == self.s_goal[0] or not cur[1] == self.s_goal[1]:
            succ = self.get_succ(cur).astype(np.int16)
            l = self.cost_list(cur, succ) + self.g_score[succ[:,0], succ[:,1]]
            smallest_el = np.argmin(l)
            cur = succ[smallest_el,:]
            path.append(cur)
            if self.detect_loop(path, cur):
                return self.loop_flag
        return np.asarray(path)
            
    def detect_loop(self, path, last):
        path = np.asarray(path)
        for i in np.arange(len(path)-1):
            if self.same_point(last, path[i,:]):
                return True
        return False
            
    def is_close(self, p1,p2):
        if p1 == np.inf and p2 == np.inf:
            return True
        return np.abs(p1-p2) < 0.00001
        
    def same_point(self, p1, p2):
        return (p1[0] == p2[0] and p1[1] == p2[1])
        
    def calc_key(self, s):
        k1 = min(    self.g_score[s[0], s[1]], self.rhs[s[0], s[1]]) + self.h_score(s, self.s_start) + self.k_m
        k2 = min(    self.g_score[s[0], s[1]], self.rhs[s[0], s[1]])
        return dstarKey(k1,k2)   
            
    def h_score(self, p1, p2):
        #return np.linalg.norm(p1-p2) * 2
        return self.eight_connected_distance(p1,p2) * 2
        
    def eight_connected_distance(self, p1,p2):
        diff = np.abs(p1-p2)
        return (np.sqrt(2)-1)*np.min(diff) + np.max(diff)
        
    def cost_list(self, point, list):
        return self.gen_cost_list(point, list, self._map)
        
    def cost(self, p1, p2):
        return self.gen_cost(p1,p2, self._map)
        
    def gen_cost_list(self, point, list, weight_map):
        costs = (weight_map[ list[:,0], list[:,1] ] + weight_map[ point[0], point[1]] )* self.euclidean(point, list)
        return costs
        
    def gen_cost(self, p1, p2, weight_map):
        distance = np.sum(np.abs(p1-p2)**2, axis=-1)**(1./2)
        cost = (weight_map[p1[0], p1[1]] + weight_map[p2[0], p2[1]]) * distance
        return cost
        
    def euclidean(self, point, list):
        p1 = np.tile(point, (list.shape[0],1))
        distances = np.sum(np.abs(p1-list)**2, axis=-1)**(1./2)
        return distances
        
    def get_g_score(self, coord):
        return self.g_score[coord[0], coord[1]]
    
    def get_rhs(self, coord):
        return self.rhs[coord[0], coord[1]]
    
    def set_g_score(self, coord, val):
        self.g_score[coord[0], coord[1]] = val
        
    def set_rhs(self, coord, val):
        self.rhs[coord[0], coord[1]] = val           
        
    def is_goal(self, coord):
        return (coord[0] == self.s_goal[0] and coord[1] == self.s_goal[1])
        

        
    def sort_changed_nodes(self):
        # reorders from furthest to closest... seems to fix a lot of issues
        if self.changed_nodes.shape[0] > 0:
            d = self.euclidean(self.s_start, self.changed_nodes)
            #self.changed_nodes = np.flipud(self.changed_nodes[ d.argsort()])
        

        
    """INTERNAL DRIVE (ie: testing) FUNCTIONS
    The following functions allow dstarlite to be tested without being strapped into external classes / ROS
    Useful for testing and optimization
    """
        
    def internal_drive_main_loop(self):
        """ Main entry point for internal tests, mirrors extdrive_compute()
        """
        self.last = self.s_start
        self.pos_record.append(self.s_start)
        self.internal_update_vision()
        s_time = time.clock()
        self.compute_shortest_path()
        print self.s_start, "Initial Build: ", time.clock() - s_time

        path = np.asarray(self.get_path())
        while not self.is_goal(self.s_start):
            self.true_count = self.true_count + 1
            s_time = time.clock()
            self.last = self.s_start
            self.s_start = path[1,:]
            self.pos_record.append(self.s_start)
            self.internal_update_vision()
            if self.changed_nodes.shape[0] > 0:
                self.sequential_deal()
            self.compute_shortest_path()
            print self.s_start, time.clock() - s_time
            path = self.get_path()
            if np.array_equal(path, self.loop_flag):
                # we have a loop
                s_time = time.clock()
                self.reset()
                self.compute_shortest_path()
                print "Rebuild: ", time.clock() - s_time
                path = path = self.get_path()
        
    def internal_update_vision(self):
        srad = self.srad
        vision_disc = self.get_vision_disc(self.real_map, self.s_start[0], self.s_start[1], srad)
        mask = self.get_mask(self.real_map, self.s_start[0], self.s_start[1], srad)
        v = np.copy(vision_disc)
        v[v==0] = 1
        delta_vision = np.logical_and(mask,np.logical_not(np.isclose(self._map, v)))
        # delta_vision: all points that have changed
        new_vision = delta_vision * vision_disc
        self.prev_map = np.copy(self._map)
        self.changed_nodes = np.transpose(np.nonzero(delta_vision))
        self.sort_changed_nodes()
        self._map[delta_vision] = vision_disc[delta_vision]
        
    def get_mask(self,_map,y,x,r):
        """Gives boolean array of visible region"""
        y,x = np.ogrid[-y:_map.shape[0]-y, -x:_map.shape[1]-x]
        mask = x*x + y*y <= r*r
        return mask
        
    def get_vision_disc(self,_map, y,x,r):
        m = _map
        mask = self.get_mask(m,y,x,r)
        return np.logical_and(m, mask) * _map
        
    def deal_with_test_cases(self, test_case):
        if test_case is None or test_case == 1:
            # Original test case
            _map = np.ones((100,100))
            _map[0:70,30:35] = 100
            _map[30:100,75:80] = 100
            start = np.array((5,10))
            goal = np.array((95,90)) 
        elif test_case == 2:
            # Inverted (across horizontal axis) of test_case 1
            _map = np.ones((100,100))
            _map[30:100,30:35] = 100
            _map[0:70,75:80] = 100
            start = np.array((95,10))
            goal = np.array((5,90)) 
        elif test_case == 3:
            # Small scale test case, shows looping problem as it begins moving up after clearing the first wall
            _map = np.ones((20,20))
            _map[0:15, 5:6] = 100
            _map[5:20, 11:12] = 100
            start = np.array((2,2))
            goal = np.array((19,19)) 
        elif test_case == 4:
            # Case designed to isolate the looping behavoir
            _map = np.ones((10,10))
            _map[3:10, 4:5] = 100
            start = np.array((8,1))
            goal = np.array((8,8))
        elif test_case == 5:
            _map = matplotlib.image.imread('IGVCmap.jpg')
            _map = scipy.misc.imresize(_map, 0.25, interp='nearest')
            _map = 256 - _map
            _map[_map == 0] = 1
            _map[_map > 1 ] = 100
            # from the perspective of imshow, the coordinates are given as x,y from the top-left corner
            start = np.array((100,160 ))
            goal = np.array((30,160))
        return (_map, start, goal)
        
def test(case=None):
    if case is None:
        case = 1
    a = dlite(test_case=case)
    a.reset()
    start_time = time.clock()
    a.internal_drive_main_loop()
    return a
    
def simple_test():
    _map = np.ones((5,7))
    _map[0:4, 4] = 100
    start = np.array((2,0))
    goal = np.array((2,6))
    a = dlite(_map = _map, start=start, goal=goal)
    a._map = _map
    a.compute_shortest_path()
    a.last = a.s_start
    path = np.asarray(a.get_path())
    a.s_start = path[1,:]
    return a
    
def reduce_weight_simple_test(dlite):
    dlite.prev_map = np.copy(dlite._map)
    dlite._map[0:4, 4] = 1
    d = []
    d.append((0,4))
    d.append((1,4))
    d.append((2,4))
    d.append((3,4))
    dlite.changed_nodes = np.asarray(d)
    dlite.sequential_deal()
    dlite.compute_shortest_path()
    