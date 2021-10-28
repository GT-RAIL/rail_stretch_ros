#! /usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class ArucoManager:
  def __init__(self) -> None:
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
        if self.tf_buffer.can_transform('map', aruco_name, rospy.Time(0)):
          transform = self.tf_buffer.lookup_transform('map', aruco_name, rospy.Time(0))

          self.broadcast_static_transform(transform)

      self.rate.sleep()

if __name__ == '__main__':
  rospy.init_node('aruco_manager')
  aruco_manager = ArucoManager()
  aruco_manager.run()