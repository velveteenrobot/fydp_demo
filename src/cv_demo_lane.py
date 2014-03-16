import roslib
import rospy
import cv2
import numpy as np
import scipy.ndimgae as spn

import lane_detector

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

global bridge
global pub
global window_name = "LANE MASK"

def image_callback(ros_image):
    global bridge
    global window_name
    try:
        cvImage = bridge.imgmsg_to_cv2(ros_image, "passthrough")
    except CvBridgeError, e:
        print e
    
    detector = lane_detector.LaneDetector()
    proc_img = detector.getLane(cvImage)
    cv2.imshow(window_name, proc_image)
    
    try:
        pub.publish(bridge.cv2_to_imgmsg(proc_image, "passthrough"))
    except CvBridgeError, e:
        print e
    

if __name__ == '__main__':
    global bridge
    global pub
    
    
    rospy.init_node('image_converter')
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, image_callback)
    pub = rospy.Publisher('lane_mask', Image)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()