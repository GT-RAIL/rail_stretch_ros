#!/usr/bin/env python3
import rospy
import math

import tf2_ros
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from rail_stretch_manipulation.joint_controller import JointController, Joints
from visualization_msgs.msg import MarkerArray
from rail_stretch_navigation.srv import GraspAruco

class ArucoGrasper(object):
    LIFT_HEIGHT = 0.05

    def get_bounded_lift(self, lift_value):
        if lift_value > JointController.MAX_LIFT:
            return JointController.MAX_LIFT
        elif lift_value < JointController.MIN_LIFT:
            return JointController.MIN_LIFT

        return lift_value

    def get_bounded_extension(self, extension_value):
        if extension_value > JointController.MAX_WRIST_EXTENSION:
            return JointController.MAX_WRIST_EXTENSION
        elif extension_value < JointController.MIN_WRIST_EXTENSION:
            return JointController.MIN_WRIST_EXTENSION

        return extension_value

    def __init__(self):
        self.rate = 10
        self.grasp_aruco_service = rospy.Service('/aruco_grasper/grasp_aruco', GraspAruco, self.grasp_aruco)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.joint_controller = JointController()
        self.markers = []

        self.joint_controller.stow()

        rospy.Subscriber('/aruco/marker_array', MarkerArray, self.aruco_detected_callback)

    def get_displacement(self, marker):
        try:
            transform = self.tf_buffer.lookup_transform('link_grasp_center', marker.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            p = PoseStamped()
            p.header.frame_id = marker.header.frame_id
            p.header.stamp = rospy.Time(0)
            p.pose = marker.pose

            p_in_grasp_frame = tf2_geometry_msgs.do_transform_pose(p, transform)

            return p_in_grasp_frame.pose.position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('aruco_grasper: ERROR')

    def grasp_aruco(self, req):
        self.switch_base_to_manipulation = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_base_to_manipulation()

        self.joint_controller.set_cmd(joints=[
            Joints.joint_wrist_yaw,
            Joints.joint_head_pan,
            Joints.joint_head_tilt,
            Joints.gripper_aperture
            ],
            values=[0, -math.pi / 2, -math.pi/6, 0.0445], # gripper facing right, camera facing right, camera tilted towards floor, gripper open
            wait=True)


        # wait for aruco detection
        rospy.sleep(rospy.Duration(5))

        for marker in self.markers:
            if marker.text == req.aruco_name:
                height_offset = rospy.get_param('/aruco_marker_info/{}/height_offset'.format(marker.id), default=0)
                depth_offset = rospy.get_param('/aruco_marker_info/{}/depth_offset'.format(marker.id), default=0)
                

                displacement = self.get_displacement(marker)
                print(self.joint_controller.joint_states.position[Joints.joint_lift.value] + displacement.z + height_offset)
                self.joint_controller.set_cmd(joints=[
                        Joints.joint_lift,
                        Joints.translate_mobile_base
                    ],
                    values=[
                        self.get_bounded_lift(self.joint_controller.joint_states.position[Joints.joint_lift.value] + displacement.z + height_offset),    
                        displacement.y
                    ],
                    wait=True)

                self.joint_controller.set_cmd(joints=[Joints.wrist_extension],
                    values=[self.get_bounded_extension(self.joint_controller.joint_states.position[Joints.wrist_extension.value] + displacement.x + depth_offset)],
                    wait=True)

                self.joint_controller.set_cmd(joints=[Joints.gripper_aperture], values=[0.01], wait=True)

                rospy.sleep(rospy.Duration(1))

                self.joint_controller.set_cmd(joints=[Joints.joint_lift],
                    values=[self.get_bounded_lift(self.joint_controller.joint_states.position[Joints.joint_lift.value] + ArucoGrasper.LIFT_HEIGHT)],
                    wait=True)

                self.joint_controller.set_cmd(joints=[Joints.wrist_extension],
                    values=[JointController.MIN_WRIST_EXTENSION],
                    wait=True)

                return True

    def aruco_detected_callback(self, msg):
        self.markers = msg.markers

if __name__ == '__main__':
    rospy.init_node('aruco_grasper')
    aruco_grasper = ArucoGrasper()
    rospy.spin()