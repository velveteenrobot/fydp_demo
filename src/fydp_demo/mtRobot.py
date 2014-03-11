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
        self.combined_delta_bound_edges = []
        
        self.simulation = simulation

        self.prev_map = None

        self.t = time.clock()
        self.cnt = 0

        self.temp_stupid = None

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
            #if time.clock() - self.t > 10.0:
            #    import pdb; pdb.set_trace()
            print "in test step"
            self.update_planner_vision_map()
            self.update_planner_location(self.loc)
            self.run_planner()

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
        self.planner_last_map = np.ones((self.sensors[0]._map.shape))
        
    def update_planner_vision_map(self):
        # gets a map delta and passes to planner
        print "in robot delta: ", np.count_nonzero(self.combined_delta_bound_edges)
        self.planner.extdrive_buffer_vis(np.copy(np.asarray(self.combined_delta_bound_edges)))
        self.combined_delta_bound_edges = np.zeros(self.combined_delta_bound_edges.shape)
        
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
#        stime = time.clock()
        status, path = self.planner.extdrive_compute()
        self.planner_last_map = np.copy(self.planner._map)
        self.path = path
#        self.planner_timer += time.clock() - stime
#        print ("Planner time: ", time.clock() - stime)
#        print ("Robot at: ", self.planner.s_start)
        
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
        self.loc = (0,0)
        #self.loc = (703,210)

        self.waypoints.append((9*20,1*20))
        self.waypoints.append((5*20,2*20))
       
        
        self.load_params( np.asarray(self.loc), np.asarray(self.waypoints))
      
        
        
        # self.waypoints.append((40,125))
        # self.waypoints.append((200,200))
        # self.waypoints.append((20,360))
        # self.waypoints.append((280, 250))

    def update_position(self, location):
        """ 
        Reality/ROS hooks into here
        TODO: Mocked out for now, will implement later
        """
        self.loc[0] = location[0]
        self.loc[1] = location[1]

    def update_vision_gazebo(self, delta):
        if self.combined_delta_bound_edges == []:
            self.combined_delta_bound_edges = np.zeros(self.prev_map.shape)
        if not self.simulation:
            self.combined_delta_bound_edges = np.logical_or(delta, self.combined_delta_bound_edges)
        
    def update_vision(self, data=None):
        """
        This is probably the best place to hook into ROS
        """

        if self.simulation:
            self.combined_delta_bound_edges = np.zeros(self.sensors[0]._map.shape)

            for sensor in self.sensors:
                sensor.update_simulated_vision(self.loc[0], self.loc[1])
                self.combined_delta_bound_edges = np.logical_or(self.combined_delta_bound_edges, sensor.delta_bound_edges)
        
        