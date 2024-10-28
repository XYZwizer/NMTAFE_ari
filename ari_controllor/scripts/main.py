#!/usr/bin/env python3
from node_manager import ari_mover

import rospy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def show_image(img):
    cv2.imshow("live feed", img)
    cv2.waitKey(1)


def image_callback(img_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except (CvBridgeError, e):
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    show_image(cv_image)

if __name__ == "__main__":
	ari = ari_mover()
	sub_image = rospy.Subscriber("/torso_front_camera/color/image_raw", Image, image_callback)
	cv2.namedWindow("live feed", 1)
	while not rospy.is_shutdown():
		ari.do()
		rospy.spin()
		
