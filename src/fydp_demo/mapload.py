import roslib
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

obs_weights = np.array([1.,1.,1.])/3.
obstacle_threshold = 0.5
occ_weights = np.array([1.,-1.,-1.])
occlusion_threshold = 0.5

path = roslib.packages.get_pkg_dir('fydp_demo')

rawmap = matplotlib.image.imread(path.rstrip('/') + '/src/fydp_demo/IGVC_square.png')

obstacle_map = (obs_weights[0]*rawmap[:,:,0] + obs_weights[1]*rawmap[:,:,1] + obs_weights[2]*rawmap[:,:,2]) > (255*obstacle_threshold)
occlusion_map = (occ_weights[0]*rawmap[:,:,0] + occ_weights[1]*rawmap[:,:,1] + occ_weights[2]*rawmap[:,:,2]) > (255*occlusion_threshold)
