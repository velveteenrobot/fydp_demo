import matplotlib
import matplotlib.pyplot as plt
import numpy as np

def load_map():
    obs_weights = np.array([1.,1.,1.])/3.
    obstacle_threshold = 0.5
    occ_weights = np.array([1.,-1.,-1.])
    occlusion_threshold = 0.5

    rawmap = matplotlib.image.imread('IGVCmap.tif')

    obstacle_map = (obs_weights[0]*rawmap[:,:,0] + obs_weights[1]*rawmap[:,:,1] + obs_weights[2]*rawmap[:,:,2]) > (255*obstacle_threshold)
    occlusion_map = (occ_weights[0]*rawmap[:,:,0] + occ_weights[1]*rawmap[:,:,1] + occ_weights[2]*rawmap[:,:,2]) > (255*occlusion_threshold)
    occlusion_map = np.logical_not(occlusion_map)
    
    return (obstacle_map, occlusion_map)
    
def quick_plot(map):
    fig,ax = plt.subplots()
    im = ax.imshow(map, cmap='gray')
    im.set_interpolation('Nearest')
    ax.autoscale(False)
    plt.show()