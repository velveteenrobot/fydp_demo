"""
utils.py
Random helper functions. Mostly wrappers around numpy/scipy functions
"""

import numpy as np
import scipy as sp
import scipy.ndimage as spimage

import matplotlib
import matplotlib.pyplot as plt

from numpy import int64
from numpy import int16

large_struct = np.ones( (5,5))
large_struct[0,0] = 0
large_struct[0,4] = 0
large_struct[4,0] = 0
large_struct[4,4] = 0

def simple_dilate(_map, _mask=None, connectivity=8):
    """Wrapper around scipy's dilation with useful defaults."""
    if connectivity == 4:
        erosion_struc = np.array([ [0,1,0], [1,1,1], [0,1,0]])
    elif connectivity == 8:
        erosion_struc = np.array([ [1,1,1], [1,1,1], [1,1,1]])
    return sp.ndimage.morphology.binary_dilation(_map, structure=erosion_struc, border_value = 1, mask = _mask)

def simple_erode(_map, _mask=None, connectivity=8):
    """Wrapper around scipy's erosion with useful defaults."""
    if connectivity == 4:
        erosion_struc = np.array([ [0,1,0], [1,1,1], [0,1,0]])
    elif connectivity == 8:
        erosion_struc = np.array([ [1,1,1], [1,1,1], [1,1,1]])
    return sp.ndimage.morphology.binary_erosion(_map, structure=erosion_struc, border_value = 1, mask = _mask)
    
def obstacle_erode(_map, _mask=None, strc=None):
    """Wrapper around scipy's erosion with useful defaults for finding obstacles."""
    if strc is None:
        strc = large_struct
    return spimage.morphology.binary_erosion(_map, structure=strc, border_value=1, mask=_mask)
    
def quick_plot(map):
    fig,ax = plt.subplots()
    im = ax.imshow(map, cmap='gray')
    im.set_interpolation('Nearest')
    ax.autoscale(False)
    plt.show()
    
def quick_plt_loc(robot, map):
    plt.imshow(map, cmap='gray')
    plt.autoscale(False)
    a = np.asarray(robot.past_loc)
    plt.scatter(a[:,1], a[:,0])
    plt.plot(a[:,1], a[:,0])
    plt.show()

def intersect2d(arr1, arr2, _mapshape):
    """Finds set-theory intersection of two arrays of coordinates
    Crams 2d coordinates into numpy's intersect1d by converting 2d coords to unique indices based on map size
    Converts back to 2d coordinates when returning
    
    Keyword Arguments:
    arr1 -- nx2 integer array representing n 2d coordinates.
    arr2 -- mx2 integer array representing m 2d coordinates.
    _mapshape -- the shape of the map/matrix that the coordinates are drawn from. Needed for unique mapping. 
        Can fill with like _map.shape
    """
    arr1v = coord2idx(arr1[:,0], arr1[:,1], _mapshape)
    arr2v = coord2idx(arr2[:,0], arr2[:,1], _mapshape)
    
    #arr1_view = arr1.view([('', arr1.dtype)])#*arr1.shape[1])
    #arr2_view = arr2.view([('', arr2.dtype)])#*arr2.shape[1])
    intersect = np.intersect1d(arr1v, arr2v)
    intersect = idx2coord(intersect, _mapshape)
    #return intersect.view(arr1.dtype).reshape(-1, arr1.shape[1])
    return intersect
    
def coord2idx(y,x,_mapshape):
    """Convert y,x coordinate to unique index based on mapsize."""
    return x*_mapshape[0]+y

def idx2coord(idx,_mapshape):
    """Convert unique index to y,x coordinates based on mapsize."""
    x = np.mod(idx, _mapshape[0])
    y = (idx-x) / _mapshape[0]
    return np.array((x,y)).T