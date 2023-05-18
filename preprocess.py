#!/usr/bin/env python3
"""
Author:
    Jinrae Kim, kjl950403@gmail.com
"""
import time
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header


class PCDPublisher:
    def __init__(self, image_file):
        self.image_cv2 = cv2.imread(image_file)
        self.bridge = CvBridge()

        self.pcd_sub = rospy.Subscriber(
            "cloud_pcd",
            PointCloud2,
            self.callback,
        )

        self.pcd_pub = rospy.Publisher(
            "spot1/camera_front/depth/color/points",
            PointCloud2,
            queue_size=1,
        )
        self.image_detector_pub = rospy.Publisher(
            "spot1/camera_front/color/image_raw",
            Image,
            queue_size=1,
        )
        self.image_localizer_pub = rospy.Publisher(
            "spot1/stair_localizer/color_image",
            Image,
            queue_size=1,
        )


    def callback(self, pcd_msg):
        print("Preprocessing color image and point cloud...")
        try:
            image_msg = self.bridge.cv2_to_imgmsg(self.image_cv2, encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("no image subscribed")
            print(e)
        image_msg.header = pcd_msg.header
        self.image_detector_pub.publish(image_msg)
        self.image_localizer_pub.publish(image_msg)
        # ordered pc
        pcd_msg.height = image_msg.height
        pcd_msg.width = image_msg.width
        pcd_msg.row_step = int(len(pcd_msg.data) / pcd_msg.height)
        self.pcd_pub.publish(pcd_msg)


if __name__ == "__main__":
    rospy.init_node("pcd_publisher")
    image_file = sys.argv[1]
    pp = PCDPublisher(image_file)
    rospy.spin()
    cv2.destroyAllWindows()
