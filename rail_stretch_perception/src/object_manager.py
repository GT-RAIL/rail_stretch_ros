#!/usr/bin/env python
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

def position_distance(pos1, pos2):
  return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)

class ObjectManager():
  POSITION_TOLERANCE = 2

  def __init__(self):
    self.object_dict = {}
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    rospy.Subscriber('/objects/marker_array', MarkerArray, self.objects_detected_callback)

  def get_position_in_map(self, marker):
    try:
      transform = self.tf_buffer.lookup_transform('map', marker.header.frame_id[1:], rospy.Time(0), rospy.Duration(1.0))
      p = PoseStamped()
      p.header.frame_id = marker.header.frame_id
      p.header.stamp = rospy.Time(0)
      p.pose = marker.pose
      p_in_map = tf2_geometry_msgs.do_transform_pose(p, transform)

      return p_in_map.pose.position
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass

  def objects_detected_callback(self, msg): 
    print(self.object_dict)
    for marker in msg.markers:
      if marker.text not in self.object_dict.keys():
        self.object_dict[marker.text] = [self.get_position_in_map(marker)]
      else:
        for i in range(len(self.object_dict[marker.text])):
          p_in_map = self.get_position_in_map(marker)
          if position_distance(self.object_dict[marker.text][i], p_in_map) < ObjectManager.POSITION_TOLERANCE:
            self.object_dict[marker.text][i] = p_in_map
          else:
            self.object_dict[marker.text].append(p_in_map)
        

if __name__ == '__main__':
  rospy.init_node('object_manager')
  object_manager = ObjectManager()
  rospy.spin()