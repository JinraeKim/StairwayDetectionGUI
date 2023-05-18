#!/usr/bin/env python3
"""
Author:
    Jinrae Kim, {jinrae.kim@jpl.nasa.gov,kjl950403@gmail.com}
Note:
    Generate virtual frame.
"""
import rospy
import tf
import math


if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    rospy.loginfo("Virtual base_link running ...")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100.0)  # 100Hz
    while not rospy.is_shutdown():
        trans = (0.0, 0.0, 0.0)
        quat = tf.transformations.quaternion_from_euler(
            0,
            0,
            -math.pi/2,
            "rzyx",  # rotating frame
        )
        br.sendTransform(
            trans,
            quat,
            rospy.Time.now(),
            "spot1/camera_front_color_optical_frame",  # child
            "spot1/base_link",  # parent
        )
        rate.sleep()
