#!/usr/bin/env python3

import rospy
import sys
import os
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CaptureImage:
    """
    A class that converts a subscribed ROS image to a OpenCV image and saves
    the captured image to a predefined directory.
    """
    def __init__(self):
        """
        A function that initializes a CvBridge class, subscriber, and save path.
        :param self: The self reference.
        """
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback, queue_size=1)
        self.save_path = './'

    def callback(self, msg):
        """
        A callback function that converts the ROS image to a CV2 image and stores the
        image.
        :param self: The self reference.
        :param msg: The ROS image message type.
        """
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logwarn('CV Bridge error: {0}'.format(e))

        file_name = 'camera_image.jpeg'
        completeName = os.path.join(self.save_path, file_name)
        cv2.imwrite(completeName, image)
        rospy.signal_shutdown("done")
        sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('capture_image', argv=sys.argv)
    CaptureImage()
    rospy.spin()
