#! /usr/bin/env python3
import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from rail_stretch_navigation.srv import NavigateToAruco

class NavigateToArucoAction:
  def __init__(self):
    self.navigate_to_object_service = rospy.Service('/navigate_to_aruco_action/navigate_to_aruco', NavigateToAruco, self.navigate_to_aruco)
    self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    self.move_base_client.wait_for_server()

  def get_aruco_param(self, param_name, aruco_id, default=None):
    if not default:
      return rospy.get_param('/aruco_marker_info/{}/{}'.format(aruco_id, param_name))

    return rospy.get_param('/aruco_marker_info/{}/{}'.format(aruco_id, param_name), default=default)

  def navigate_to_aruco(self, req):
    rospy.logwarn('NAVIGATE TO ARUCO WITH ID: {}'.format(req.aruco_id))
    params = rospy.get_param('/aruco_marker_info/{}'.format(req.aruco_id))

    if params:
      x_offset = self.get_aruco_param('x_offset', req.aruco_id, default=0)
      y_offset = self.get_aruco_param('y_offset', req.aruco_id, default=0)
      name = self.get_aruco_param('name', req.aruco_id)

      try:
        transform = self.tf_buffer.lookup_transform('map', name, rospy.Time(0), rospy.Duration(10.0))
        p = PoseStamped()
        p.header.frame_id = name
        p.header.stamp = rospy.Time(0)
        p.pose.position.x = x_offset
        p.pose.position.y = y_offset
        p.pose.orientation.w = 1

        p_in_map_frame = tf2_geometry_msgs.do_transform_pose(p, transform)
        p_in_map_frame.pose.position.z = 0

        goal = MoveBaseGoal()
        goal.target_pose = p_in_map_frame

        self.move_base_client.send_goal(goal)
        return True

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)

    return False

if __name__ == '__main__':
  rospy.init_node('navigate_to_aruco_action')
  navigate_to_aruco_action = NavigateToArucoAction()
  rospy.spin()
