import numpy as np
import scipy as sp
import scipy.ndimage as spn
import cv2
import matplotlib.pyplot as plt

class LaneDetector:
    def __init__(self):
        self.min_pix_count = 400
        self.saliency_threshold_static = 0.400
        self.saliency_subwindows_static_x = 4
        self.saliency_subwindows_static_y = 4
        self.intensity_threshold = 100
        self.v_threshold = 0.7
        self.s_threshold = 0.1
        
    def compute_saliency(self, lab_image):
        lab_image = np.float32(lab_image) / 255.0
        m = cv2.mean(lab_image)
        saliency = lab_image[:,:,0] + lab_image[:,:,1] + lab_image[:,:,2] - m[0] - m[1] - m[2]
        b, saliency = cv2.threshold(saliency, 0.0, 1.0, 3)
        minVal, maxval, minLoc, maxLoc = cv2.minMaxLoc(saliency)
        return saliency / maxval

    def saliency_lanes_static(self, img, win_ratio_x, win_ratio_y):
        saliency_map_out = np.empty((img.shape[0], img.shape[1]))
        saliency_bin_out = np.empty((img.shape[0], img.shape[1]))
        
        nominal_win_size_x = int(round(img.shape[0] / win_ratio_x))
        nominal_win_size_y = int(round(img.shape[1] / win_ratio_y))
        
        lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        
        for j in range(1, win_ratio_x + 1):
            for i in range(1, win_ratio_y + 1):
                top = (i-1) * nominal_win_size_x
                bottom = min(i*nominal_win_size_x, img.shape[0])
                left = (j-1) * nominal_win_size_y
                right = min(j*nominal_win_size_y, img.shape[1])
                
                subimage = lab_img[top:bottom, left:right, :]
                submap_out = self.compute_saliency(subimage)
                sub_bin_out = submap_out > self.saliency_threshold_static;
                
                saliency_map_out[top:bottom, left:right] = submap_out
                saliency_bin_out[top:bottom, left:right] = sub_bin_out
                
        return saliency_bin_out
        
    def cluster_check(self, subwindow, submap):
        verified_map_out = np.zeros((subwindow.shape[0], subwindow.shape[1]))
        temp = np.uint8(submap)
        dsubwindow = np.float64(subwindow)
        
        cluster_map, cluster_num = spn.measurements.label(temp) #default mode is 4-connected
        #counts = np.bincount(cluster_map.ravel())
        
        for i in range(1, cluster_num+1):
            if np.sum(cluster_map == i) > self.min_pix_count / 10.0:
            #if counts[i] > self.min_pix_count / 10.0:
                mean_cluster = np.mean(dsubwindow[cluster_map  == i])
                mean_else = np.mean(dsubwindow[cluster_map != i])
                
                if mean_cluster > mean_else:
                    verified_map_out[cluster_map == i] = 1
        return verified_map_out
        
    def lane_verify(self, gray_image, lane_mask, win_ratio_x, win_ratio_y):
        lane_map = np.empty(gray_image.shape).astype(np.float64)
        lane_map_out = np.empty(gray_image.shape).astype(np.float64)
        
        nominal_win_size_x = int(round(gray_image.shape[0] / win_ratio_x))
        nominal_win_size_y = int(round(gray_image.shape[1] / win_ratio_y))

        for j in range(1, win_ratio_x + 1):
            for i in range(1, win_ratio_y+1):
                top = (i-1) * nominal_win_size_x
                bottom = min(i*nominal_win_size_x, gray_image.shape[0])
                left = (j-1) * nominal_win_size_y
                right = min(j*nominal_win_size_y, gray_image.shape[1])      
                
                subwindow = gray_image[top:bottom, left:right]
                submap = lane_mask[top:bottom, left:right]
                submap_out = self.cluster_check(subwindow, submap)
                lane_map[top:bottom, left:right] = submap_out

        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        lane_map = cv2.morphologyEx(lane_map, cv2.MORPH_CLOSE, element)
        
        cluster_map, cluster_num = spn.measurements.label(np.uint8(lane_map))
        
        for i in range(1, cluster_num+1):
            area = np.sum(cluster_map == i)
            if area > self.min_pix_count:
                if np.sum(gray_image[cluster_map == i])/area > self.intensity_threshold:
                    lane_map_out[cluster_map == i] = 10
                    
        return lane_map_out
        
    def hsv_verify(self, image, v_thres, s_thres):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        vMask = hsv_image[:,:,2] >= v_thres
        sMask = hsv_image[:,:,1] >= s_thres
        mask = np.logical_and(vMask, sMask)
        return mask
        
    def getLane(self, image):
        image_blurred = cv2.GaussianBlur(image, (7,7), 0)
        gray_image = cv2.cvtColor(image_blurred, cv2.COLOR_BGR2GRAY)
        
        saliency_initial_lane_map_static = self.saliency_lanes_static(image_blurred, self.saliency_subwindows_static_x, self.saliency_subwindows_static_y)
        saliency_verified_lane_map_static = self.lane_verify(gray_image, saliency_initial_lane_map_static, self.saliency_subwindows_static_x, self.saliency_subwindows_static_y)
        hsv_mask = self.hsv_verify(image, self.v_threshold, self.s_threshold)
        hsv_lane_map = hsv_mask * saliency_verified_lane_map_static
        
        #image[hsv_lane_map,0] = 0
        #image[hsv_lane_map,1] = 255
        #image[hsv_lane_map,2] = 255
        
        return hsv_lane_map
    
    
    
        
    
        
        