import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import time

import heapq
from sets import Set

def norm (vector):
    return np.sqrt(np.dot(vector, vector))
    
def hscore(p1, p2):
    return norm(p1-p2)
    
def totup(coord):
    return (coord[0], coord[1])
    
def toarray(coord):
    return np.array((coord[0], coord[1]))
    
def get_neighbours(coord, map):
    d = np.array([[1,0], [0,1], [-1, 0], [0, -1], [1,1], [1,-1], [-1,1], [-1,-1]])
    if coord[0] >= 1 and coord[0] < map.shape[0]-1 and coord[1] >= 1 and coord[1] < map.shape[1] -1:
        return (np.tile(coord, (8,1)) + d)
    else:
        neighbours = np.zeros((8,2))
        cnt = 0
        for i in range(0,8):
            p = coord + d[i,:]
            if p[0] < 0 or p[0] >= map.shape[0] or p[1] < 0 or p[1] >= map.shape[1]:
                pass
            else:
                neighbours[cnt,:] = p
                cnt = cnt + 1
        return (neighbours[0:cnt-1,:])

def reconstruct_path(came_from, current_node):
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

# wrapping a priority queue and set together to check for membership in constant time
class OpenSet:
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
        
    
# making a map of sorts
# generating the toy map. Coordinates are in y,x
m = np.ones((100,100))
m[0:70,30:35] = 0
m[30:100,75:80] = 0
start = np.array((5,10))
end = np.array((95,90))

closed_set = Set()
open_set = OpenSet()
came_from = np.zeros(m.shape, dtype=(np.int16, 2))
# came_from is now a 100x100x2 integer array
# you can access it the following ways:
# came_from[0,0] = [10,20] 
# came_from[0,10,2] = 20
# came_from[0,10][1] = 10

gscore = np.zeros(m.shape)
fscore = np.zeros(m.shape)

gscore[totup(start)] = 0
fscore[totup(start)] = gscore[totup(start)]  + hscore(start, end)

open_set.push(fscore[totup(start)], totup(start))

stime = time.clock()
while len(open_set) > 0:
    current = open_set.pop()[1]    # peaking
    if np.array_equal(current, end):
        path = reconstruct_path(came_from, toarray(current))
        pass
        
    closed_set.add(totup(current))
    neighbours  = get_neighbours(current,m)
    for n in neighbours:
        ntup = totup(n)
        if m[ntup] == 0:
            temp_g = 9999999999
        else:
            temp_g = gscore[totup(current)] + norm(current-n)
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
    
elapsed = (time.clock() - stime)
print elapsed

fig,ax = plt.subplots()
im = plt.imshow((m), cmap='gray')
im.set_interpolation('Nearest')
ax.autoscale(False)
ax.scatter(path[:,1], path[:,0])
ax.scatter(start[1], start[0], s=40, c='g', marker='x')
ax.scatter(end[1], end[0], s=40,c='r', marker='x')    
plt.show()