#! /usr/bin/env python3

import rospy
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rail_stretch_manipulation.joint_controller import JointController, Joints

class ArucoManager:
  MIN_LIFT = 0.3
  MAX_LIFT = 1.09
  MIN_WRIST_EXTENSION = 0.01
  MAX_WRIST_EXTENSION = 0.5

  def __init__(self) -> None:
    self.joint_controller = JointController()
    self.joint_controller.set_cmd(joints=[
            Joints.joint_wrist_yaw,
            Joints.joint_head_pan,
            Joints.joint_head_tilt,
            Joints.gripper_aperture,
            Joints.wrist_extension,
            Joints.joint_lift
            ],
            values=[math.pi, 0, 0, 0, ArucoManager.MIN_WRIST_EXTENSION, ArucoManager.MIN_LIFT], # gripper stowed, camera facing forward, camera horizontal to floor, gripper close
            wait=True)

    self.joint_controller.set_cmd([Joints.joint_head_tilt], [-math.pi / 6], wait=True)

    self.rate = rospy.Rate(10)
    self.broadcaster = tf2_ros.StaticTransformBroadcaster()
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    self.aruco_data = rospy.get_param('/aruco_marker_info')
    self.aruco_names = []

    for aruco_id, aruco_datum in self.aruco_data.items():
      self.aruco_names.append(aruco_datum['name'])

  def broadcast_static_transform(self, transform: TransformStamped) -> None:
    transform.child_frame_id = transform.child_frame_id + '_static'
    self.broadcaster.sendTransform(transform)

  def run(self) -> None:
    while not rospy.is_shutdown():
      for aruco_name in self.aruco_names:
        if aruco_name != 'unknown':
          if self.tf_buffer.can_transform('map', aruco_name, rospy.Time(0)):
            transform = self.tf_buffer.lookup_transform('map', aruco_name, rospy.Time(0))

            self.broadcast_static_transform(transform)

      self.rate.sleep()

if __name__ == '__main__':
  rospy.init_node('aruco_manager')
  aruco_manager = ArucoManager()
  aruco_manager.run()