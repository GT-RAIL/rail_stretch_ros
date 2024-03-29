#! /usr/bin/env python3
import math
import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from rail_stretch_navigation.srv import NavigateToAruco
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def euler_from_ros_quaternion(quaterion):
  return euler_from_quaternion([quaterion.x, quaterion.y, quaterion.z, quaterion.w])

def ros_quaternion_from_euler(x, y, z):
  (x, y, z, w) = quaternion_from_euler(x, y, z)

  quaternion = Quaternion()
  quaternion.x = x
  quaternion.y = y
  quaternion.z = z
  quaternion.w = w

  return quaternion

class NavigateToArucoAction:
  def __init__(self):
    self.navigate_to_object_service = rospy.Service('/navigate_to_aruco_action/navigate_to_aruco', NavigateToAruco, self.navigate_to_aruco)
    self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    self.move_base_client.wait_for_server()

    self.aruco_data = rospy.get_param('/aruco_marker_info')
    self.aruco_names = {}

    for aruco_id, aruco_datum in self.aruco_data.items():
      self.aruco_names[aruco_datum['name']] = aruco_id

  def get_aruco_param(self, param_name, aruco_id, default=None):
    if not default:
      return rospy.get_param('/aruco_marker_info/{}/{}'.format(aruco_id, param_name))

    return rospy.get_param('/aruco_marker_info/{}/{}'.format(aruco_id, param_name), default=default)

  def navigate_to_aruco(self, req):
    aruco_id = self.aruco_names[req.aruco_name]
    rospy.logwarn('NAVIGATE TO ARUCO WITH ID: {}'.format(aruco_id))
    params = rospy.get_param('/aruco_marker_info/{}'.format(aruco_id))

    if params:
      x_offset = self.get_aruco_param('x_offset', aruco_id, default=0)
      z_offset = self.get_aruco_param('z_offset', aruco_id, default=0)
      name = self.get_aruco_param('name', aruco_id)

      try:
        alignment_axis = self.get_aruco_param('alignment_axis', aruco_id, 'x')
        flip_alignment = self.get_aruco_param('flip_alignment', aruco_id, default=False)
        
        transform = self.tf_buffer.lookup_transform('map', name + '_static', rospy.Time(0), rospy.Duration(10.0))
        p = PoseStamped()
        p.header.frame_id = name
        p.header.stamp = rospy.Time(0)
        p.pose.position.x = x_offset
        p.pose.position.z = z_offset
        p.pose.orientation.w = 1
        
        if alignment_axis == 'x':
          if flip_alignment:
            p.pose.orientation = ros_quaternion_from_euler(math.pi / 2, 0, math.pi)
          else:
            p.pose.orientation = ros_quaternion_from_euler(math.pi / 2, 0, 0)
        else:
          if flip_alignment:
            p.pose.orientation = ros_quaternion_from_euler(math.pi / 2, 0, math.pi / 2)
          else:
            p.pose.orientation = ros_quaternion_from_euler(math.pi / 2, 0, 1.5 * math.pi)

        p_in_map_frame = tf2_geometry_msgs.do_transform_pose(p, transform)
        p_in_map_frame.pose.position.z = 0

        (e_x, e_y, e_z) = euler_from_ros_quaternion(p_in_map_frame.pose.orientation)

        p_in_map_frame.pose.orientation = ros_quaternion_from_euler(0, 0, e_z)

        goal = MoveBaseGoal()
        goal.target_pose = p_in_map_frame

        state = self.move_base_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(200), preempt_timeout=rospy.Duration(200))

        if state == GoalStatus.SUCCEEDED:
          return True

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)

    return False

if __name__ == '__main__':
  rospy.init_node('navigate_to_aruco_action')
  navigate_to_aruco_action = NavigateToArucoAction()
  rospy.spin()
