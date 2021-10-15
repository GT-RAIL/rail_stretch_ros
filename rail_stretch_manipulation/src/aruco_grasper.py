#!/usr/bin/env python
import rospy
import math

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from joint_controller import JointController
from visualization_msgs.msg import MarkerArray

class ArucoGrasper(object):
    MARKER_ID = 0

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.joint_controller = JointController()
        self.joint_controller.set_cmd(joint_names=['joint_wrist_yaw', 'joint_head_pan', 'joint_head_tilt'], values=[0, -math.pi / 3, -math.pi / 6], wait=True)
        rospy.Subscriber('/aruco/marker_array', MarkerArray, self.aruco_detected_callback)

    def get_displacement(self, marker):
        try:
            transform = self.tf_buffer.lookup_transform('link_grasp_center', marker.header.frame_id[1:], rospy.Time(0), rospy.Duration(1.0))
            p = PoseStamped()
            p.header.frame_id = marker.header.frame_id
            p.header.stamp = rospy.Time(0)
            p.pose = marker.pose

            p_in_grasp_frame = tf2_geometry_msgs.do_transform_pose(p, transform)

            return p_in_grasp_frame.pose.position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr_throttle_identical(5, 'aruco_grasper: ERROR')

    def aruco_detected_callback(self, msg):
        for marker in msg.markers:
            if marker.id == ArucoGrasper.MARKER_ID:
                displacement = self.get_displacement(marker)
                self.joint_controller.set_cmd(joint_names=[
                        'joint_lift',
                        'wrist_extension'
                    ],
                    values=[
                        self.joint_controller.joint_states.position[1] + displacement.z,
                        self.joint_controller.joint_states.position[0] + displacement.x
                    ],
                    wait=False)


if __name__ == '__main__':
    rospy.init_node('aruco_grasper')
    aruco_grasper = ArucoGrasper()
    rospy.spin()