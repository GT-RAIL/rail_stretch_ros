#! /usr/bin/env python2.7
import rospy
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, PointStamped
from rail_stretch_perception.msg import ObjectNames
from rail_stretch_perception.srv import GetObjectPosition, GetObjectPositionResponse

def position_distance(pos1, pos2):
  return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)

class ObjectManager():
  POSITION_TOLERANCE = 2

  def __init__(self):
    self.object_dict = {}
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    rospy.Subscriber('/objects/marker_array', MarkerArray, self.objects_detected_callback)
    self.object_names_pub = rospy.Publisher('/object_manager/object_names', ObjectNames, queue_size=10, latch=True)
    self.get_object_position_service = rospy.Service('/object_manager/get_object_position', GetObjectPosition, self.get_object_position_handler)
    
  def get_point_in_map(self, marker):
    try:
      transform = self.tf_buffer.lookup_transform('map', marker.header.frame_id[1:], rospy.Time(0), rospy.Duration(1.0))
      p = PoseStamped()
      p.header.frame_id = marker.header.frame_id
      p.header.stamp = rospy.Time(0)
      p.pose = marker.pose
      p_in_map = tf2_geometry_msgs.do_transform_pose(p, transform)

      point_stamped = PointStamped()
      point_stamped.header = p_in_map.header
      point_stamped.point = p_in_map.pose.position

      return point_stamped
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.logerr_throttle_identical(5, 'object_manager: Cannot get point in map frame.')

  def objects_detected_callback(self, msg):
    for marker in msg.markers:
      if marker.text not in self.object_dict.keys():
        point_stamped = self.get_point_in_map(marker)
        if point_stamped is not None:
          self.object_dict[marker.text] = [point_stamped]
      else:
        point_stamped = self.get_point_in_map(marker)
        replaced = False
        for i in range(len(self.object_dict[marker.text])):
          if position_distance(self.object_dict[marker.text][i].point, point_stamped.point) < ObjectManager.POSITION_TOLERANCE:
            self.object_dict[marker.text][i] = point_stamped
            replaced = True
            break
        
        if not replaced:
          self.object_dict[marker.text].append(point_stamped)

    object_names = ObjectNames()
    object_names.object_names = self.object_dict.keys()
    self.object_names_pub.publish(object_names)

  def get_object_position_handler(self, req):
    if req.object_name in self.object_dict:
      response = GetObjectPositionResponse()
      response.object_position = self.object_dict[req.object_name][0]

      return response
        

if __name__ == '__main__':
  rospy.init_node('object_manager')
  object_manager = ObjectManager()
  rospy.spin()