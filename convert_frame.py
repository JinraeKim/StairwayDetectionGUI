import rospy
import sys
import tf2_py as tf2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2


class PCDConverter:
    def __init__(self, target_frame):
        self.pcd_sub = rospy.Subscriber(
            "cloud_pcd",
            PointCloud2,
            self.callback,
        )
        self.pcd_pub = rospy.Publisher(
            "cloud_pcd_converted",
            PointCloud2,
            queue_size=1,
        )
        self.target_frame = target_frame

    def callback(self, pcd_msg):
        try:
            trans = tf_buffer.lookup_transform(self.target_frame, pcd_msg.header.frame_id,
                                               pcd_msg.header.stamp,
                                               rospy.Duration(1.00))
        except tf2.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return
        pcd_out = do_transform_cloud(pcd_msg, trans)  # it is quite slow; like for each 0.4 s
        self.pcd_pub.publish(pcd_msg)
        print(f"Converting pcd... (frame: {pcd_msg.header.frame_id} to {self.target_frame})")


if __name__ == "__main__":
    target_frame = sys.argv[1]
    rospy.init_node("pcd_converter")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pp = PCDConverter(target_frame)
    rospy.spin()
