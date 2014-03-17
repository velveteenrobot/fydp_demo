import roslib
import rospy
import cv2
import numpy as np
import scipy.ndimage as spn

import lane_detector

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

global bridge
global pub
global window_name

def image_callback(ros_image):
    global bridge
    try:
        cvImage = bridge.imgmsg_to_cv2(ros_image, "passthrough")
    except CvBridgeError, e:
        print e
    
    detector = lane_detector.LaneDetector()
    proc_image = detector.getLane(cvImage)
    bleh = np.empty((proc_image.shape[0], proc_image.shape[1], 3))
    bleh[:,:,0] = proc_image; bleh[:,:,1] = proc_image; bleh[:,:,2] = proc_image
    cv2.imshow("LANE MASK", bleh)
    cv2.waitKey(5)
    try:
        pub.publish(bridge.cv2_to_imgmsg(bleh, "passthrough"))
    except CvBridgeError, e:
        print e
    

if __name__ == '__main__':
    global bridge
    global pub

    
    rospy.init_node('image_converter')
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, image_callback)
    pub = rospy.Publisher('lane_mask', Image)
    cv2.namedWindow("LANE MASK")    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
