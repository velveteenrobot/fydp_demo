"""
astar.py
Bastard implementation of A* search. NOT very efficient.
"""
import numpy as np

import heapq
from sets import Set

def hscore(a,b):
    """Heuristic score between two points (Euclidean distance here)."""
    return np.linalg.norm(a-b)

def totup(coord):
    """Convert 2d vector to tuple."""
    return (coord[0], coord[1])
    
def toarray(coord):
    """Convert tuple to 2d vector."""
    return np.array((coord[0], coord[1]))
    
def get_neighbours(coord, _map, narray, connectivity):
    """Get neighbouring nodes/pixels, taking map shape into account.
    
    Keywords:
    coord: 2d position (y,x) of node/pixel to get neighbour of
    _map: map from which nodes are drawn
    narray -- (connectivity)x2 array of ints representing displacements from coord for each neighbour
    connectivity -- The connectivity of neighbours to select. 4 connectivity are only does with faces touching.
                    8 connectivity include corners touching.
    """
    if connectivity == 8:
        if coord[0] >= 1 and coord[0] < _map.shape[0]-1 and coord[1] >= 1 and coord[1] < _map.shape[1] -1:
            return (np.tile(coord, (8,1)) + narray)
        else:
            neighbours = np.zeros((8,2))
            cnt = 0
            for i in range(0,8):
                p = coord + narray[i,:]
                if p[0] < 0 or p[0] >= _map.shape[0] or p[1] < 0 or p[1] >= _map.shape[1]:
                    pass
                else:
                    neighbours[cnt,:] = p
                    cnt = cnt + 1
            return (neighbours[0:cnt-1,:])
    elif connectivity == 4:
        if coord[0] >= 1 and coord[0] < _map.shape[0]-1 and coord[1] >= 1 and coord[1] < _map.shape[1] -1:
            return (np.tile(coord, (4,1)) + narray)
        else:
            neighbours = np.zeros((4,2))
            cnt = 0
            for i in range(0,4):
                p = coord + narray[i,:]
                if p[0] < 0 or p[0] >= _map.shape[0] or p[1] < 0 or p[1] >= _map.shape[1]:
                    pass
                else:
                    neighbours[cnt,:] = p
                    cnt = cnt + 1
            return (neighbours[0:cnt-1,:])
            
def reconstruct_path(came_from, current_node):
    """ Traces through the came_from matrix from goal to start."""
    prev = came_from[totup(current_node)]
    # p is a 1x2 np array
    path = prev
    done = False
    while not done:
        prev = came_from[totup(prev)]
        if np.array_equal(prev, np.array([0,0])):
            done = True
        else:
            path = np.vstack((prev, path))
    return path

class OpenSet:
    """Combination priority queue and set for quick membership testing."""
    def __init__(self):
        self.heap = []
        self.set = Set()
        
    def push(self, priority, item):
        self.add(priority, item)
    
    def add(self, priority, item):
        heapq.heappush(self.heap, (priority, item))
        self.set.add(item)
        
    def peek(self):
        return self.heap[0][1]
    
    def pop(self):
        item = heapq.heappop(self.heap)
        self.set.remove(item[1])
        return item
    
    def exists(self, item):
        return item in self.set

    def  __len__(self):
        return len(self.set)

def astar(_map=None, start=None, end=None, connectivity=8):
    """A grid based A* search algorithm
    
    If called without a map, will run a default test map
    
    _map: mxn boolean array. True for passable and False for unpassable points
    start/end: 2d vector of start an dstop positions
    connectivity: 4 or 8 connectivity
    
    """
    if _map is None:
        m = np.ones((100,100))
        m[0:70,30:35] = 0
        m[30:100,75:80] = 0
        start = np.array((5,10))
        end = np.array((95,90)) 
    else:
        m = _map

    closed_set = Set()
    open_set = OpenSet()
    came_from = np.zeros(m.shape, dtype=(np.int16, 2))
    # came_from is now a 100x100x2 integer array
    # you can access it the following ways:
    # came_from[0,0] = [10,20] 
    # came_from[0,10,2] = 20
    # came_from[0,10][1] = 10

    if connectivity == 8:
        neighbour_array = np.array([[1,0], [0,1], [-1, 0], [0, -1], [1,1], [1,-1], [-1,1], [-1,-1]])
    elif connectivity == 4:
        neighbour_array = np.array([ [1,0], [0,1], [-1,0], [0,-1]])
    gscore = np.zeros(m.shape)
    fscore = np.zeros(m.shape)

    gscore[totup(start)] = 0
    fscore[totup(start)] = gscore[totup(start)]  + hscore(start, end)

    open_set.push(fscore[totup(start)], totup(start))
    path = []
    
    while len(open_set) > 0:
        current = open_set.pop()[1]
        cur_gscore = gscore[totup(current)]
        if np.array_equal(current, end):
            path = reconstruct_path(came_from, toarray(current))
            
            
        closed_set.add(totup(current))
        neighbours  = get_neighbours(current,m, neighbour_array, connectivity)
        
        for n in neighbours:
            ntup = totup(n)
            if m[ntup] == 0:
                temp_g = 9999999999
            else:
                # Evaluating temp_g and temp_f here are the two slowest parts
                # Together, they take up ~45% of runtime
                temp_g = cur_gscore + hscore(current, n)
                temp_f = temp_g + hscore(n, end)
                
                if (ntup in closed_set and temp_f >= fscore[ntup]):
                    pass
                elif temp_f <= fscore[ntup]:
                    came_from[ntup] = current
                    gscore[ntup] = temp_g
                    fscore[ntup] = temp_f
                elif not open_set.exists(ntup):
                    came_from[ntup] = current
                    gscore[ntup] = temp_g
                    fscore[ntup] = temp_f
                    open_set.push(temp_f, ntup)
    
    return path