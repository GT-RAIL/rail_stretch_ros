#!/usr/bin/env python
import rospy
import math

import tf2_ros
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from joint_controller import JointController, Joints
from visualization_msgs.msg import MarkerArray

class ArucoGrasper(object):
    MARKER_ID = 0
    MIN_LIFT = 0.3
    MAX_LIFT = 1.09
    MIN_WRIST_EXTENSION = 0.01
    MAX_WRIST_EXTENSION = 0.5

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.joint_controller = JointController()
        self.joint_controller.set_cmd(joints=[
            Joints.joint_wrist_yaw,
            Joints.joint_head_pan,
            Joints.joint_head_tilt,
            Joints.gripper_aperture,
            Joints.wrist_extension,
            Joints.joint_lift
            ],
            values=[math.pi, 0, 0, 0, ArucoGrasper.MIN_WRIST_EXTENSION, ArucoGrasper.MIN_LIFT], # gripper stowed, camera facing forward, camera horizontal to floor, gripper close
            wait=True)
        self.joint_controller.set_cmd(joints=[
            Joints.joint_wrist_yaw,
            Joints.joint_head_pan,
            Joints.joint_head_tilt,
            Joints.gripper_aperture
            ],
            values=[0, -math.pi / 2, -math.pi/6, 0.0445], # gripper facing right, camera facing right, camera tilted towards floor, gripper open
            wait=True)
        rospy.Subscriber('/aruco/marker_array', MarkerArray, self.aruco_detected_callback)
        self.switch_base_to_manipulation = rospy.ServiceProxy('/switch_to_manipulation_mode', Trigger)
        self.switch_base_to_manipulation()

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
                self.joint_controller.set_cmd(joints=[
                        Joints.joint_lift,
                        Joints.wrist_extension,
                        Joints.joint_mobile_base_translation
                    ],
                    values=[
                        self.joint_controller.joint_states.position[Joints.joint_lift.value] + displacement.z,
                        self.joint_controller.joint_states.position[Joints.wrist_extension.value] + displacement.x,
                        displacement.y + 0.03
                    ],
                    wait=False)


if __name__ == '__main__':
    rospy.init_node('aruco_grasper')
    aruco_grasper = ArucoGrasper()
    rospy.spin()