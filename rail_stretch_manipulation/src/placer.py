#!/usr/bin/env python3
import rospy

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from rail_stretch_manipulation.joint_controller import JointController, Joints
from rail_stretch_navigation.srv import GraspAruco

class Placer(object):
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
        self.place_service = rospy.Service(
            '/placer/place', GraspAruco, self.place)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.joint_controller = JointController()

        self.aruco_data = rospy.get_param('/aruco_marker_info')
        self.aruco_names = {}

        for aruco_id, aruco_datum in self.aruco_data.items():
            self.aruco_names[aruco_datum['name']] = aruco_id

    def get_aruco_param(self, param_name, aruco_id, default=None):
        if not default:
            return rospy.get_param('/aruco_marker_info/{}/{}'.format(aruco_id, param_name))

        return rospy.get_param('/aruco_marker_info/{}/{}'.format(aruco_id, param_name), default=default)

    def place(self, req):
        aruco_id = self.aruco_names[req.aruco_name]
        y_place_offset = rospy.get_param(
            '/aruco_marker_info/{}/y_place_offset'.format(aruco_id), default=0)

        print('y', y_place_offset)
        z_place_offset = rospy.get_param(
            '/aruco_marker_info/{}/z_place_offset'.format(aruco_id), default=0)

        print('z', z_place_offset)

        transform = self.tf_buffer.lookup_transform(
            'link_grasp_center', req.aruco_name + '_static', rospy.Time(0), rospy.Duration(10.0))

        p = PointStamped()
        p.header.frame_id = req.aruco_name
        p.header.stamp = rospy.Time(0)
        p.point.y = y_place_offset
        p.point.z = z_place_offset

        p_in_grasp_frame = tf2_geometry_msgs.do_transform_point(p, transform)

        self.joint_controller.set_cmd(
            joints=[Joints.joint_wrist_yaw],
            values=[0],
            wait=True
        )

        self.joint_controller.set_cmd(
            joints=[
                Joints.joint_lift,
            ],
            values=[
                self.get_bounded_lift(
                    self.joint_controller.joint_states.position[Joints.joint_lift.value] + p_in_grasp_frame.point.z),
            ],
            wait=True
        )

        self.joint_controller.set_cmd(
            joints=[Joints.wrist_extension],
            values=[self.get_bounded_extension(
                self.joint_controller.joint_states.position[Joints.wrist_extension.value] + p_in_grasp_frame.point.x)],
            wait=True
        )

        self.joint_controller.set_cmd(
            joints=[
                Joints.joint_lift,
            ],
            values=[
                self.get_bounded_lift(
                    self.joint_controller.joint_states.position[Joints.joint_lift.value] - 0.08),
            ],
            wait=True
        )

        self.joint_controller.place()

        rospy.sleep(rospy.Duration(1))

        self.joint_controller.retract_arm()

        self.joint_controller.stow()

        return True


if __name__ == '__main__':
    rospy.init_node('placer')
    aruco_grasper = Placer()
    rospy.spin()
