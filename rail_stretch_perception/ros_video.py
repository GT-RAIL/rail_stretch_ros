#!/usr/bin/env python2
import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image, Joy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/camera/convert/image", Image, queue_size=1)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.depth_pub = rospy.Publisher("/camera/convert/depth", Image, queue_size=1)

    def depth_callback(self, data):
        depth_data = self.bridge.imgmsg_to_cv2(data, "passthrough")
        depth_data = depth_data.astype("uint8").flatten()
        # print(depth_data.dtype)
        depth = Image()
        depth.data = depth_data.tolist()
        print(type(depth.data[0]))
        self.depth_pub.publish(depth)


    def image_callback(self,data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.cv_image = cv.filter2D(self.cv_image, -1, sharpen_kernel)
        self.cv_image = self.cv_image.flatten()
        self.cv_image = self.cv_image.tolist()
        image = Image()
        image.data = self.cv_image
        # print(image.data[0])
        self.image_pub.publish(image)

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")