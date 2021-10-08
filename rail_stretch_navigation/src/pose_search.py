#! /usr/bin/env python2.7
import rospy
import numpy as np
from math import cos, sin, atan2
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from rail_stretch_navigation.srv import GetValidPose, GetValidPoseResponse
from occupancy_grid_python import OccupancyGridManager

class PoseSearch():
  SEARCH_DIRECTIONS = 8
  MAX_ITERATIONS = 20
  SEARCH_STEP_DISTANCE = 0.05 # meters
  SEARCH_START_DISTANCE = 0.05 # meters
  FOOTPRINT = 0.2 # meters

  def __init__(self):
    self.search_service = rospy.Service('/pose_search/get_valid_pose', GetValidPose, self.search_handler)
    self.ogm = OccupancyGridManager('/move_base/global_costmap/costmap', subscribe_to_updates=True)

  def search_handler(self, req):
    search_poses = self.get_search_poses(req.target_point)

    response = GetValidPoseResponse()

    best_pose = self.get_valid_pose(search_poses)
    
    if best_pose is not None:
      response.goal_pose = best_pose # pose_stamped
      return response
    
    return False

  def get_search_poses(self, target_point):
    search_step_deg = 360 / PoseSearch.SEARCH_DIRECTIONS
    search_step_rad = np.deg2rad(search_step_deg)

    search_poses = []

    for iteration in range(1, PoseSearch.MAX_ITERATIONS + 1):
      length_v = np.array([0, PoseSearch.SEARCH_START_DISTANCE + PoseSearch.SEARCH_STEP_DISTANCE * iteration])
      for direction in range(PoseSearch.SEARCH_DIRECTIONS):
        theta = search_step_rad * direction
        rot_mat = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

        search_point = Point()
        search_v = np.dot(rot_mat, length_v)

        search_point.x, search_point.y = search_v
        
        angle_to_target = atan2(target_point.y - search_point.y, target_point.x - search_point.x)
        
        quat = quaternion_from_euler(0, 0, angle_to_target)
        ros_quat = Quaternion()
        ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w = quat

        search_pose = PoseStamped()
        search_pose.header.frame_id = 'map'
        search_pose.header.stamp = rospy.Time(0)
        search_pose.pose.position = search_point
        search_pose.pose.orientation = ros_quat

        search_poses.append(search_pose)
        search_poses.reverse()
    return search_poses

  def get_valid_pose(self, search_poses):
    for search_pose in search_poses:
      print(type(search_pose.pose.position).__name__)
      if self.ogm.get_cost_from_world_x_y(search_pose.pose.position.x, search_pose.pose.position.y) == 0:
        return search_pose

if __name__ == '__main__':
  rospy.init_node('pose_search')
  pose_search = PoseSearch()
  rospy.spin()