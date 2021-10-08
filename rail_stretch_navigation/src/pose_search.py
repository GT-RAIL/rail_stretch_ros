#! /usr/bin/env python2.7
import rospy
import numpy as np
from math import cos, sin, atan2, inf
from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import quaternion_from_euler
from rail_stretch_navigation.srv import PoseSearch, PoseSearchResponse

class PoseSearch():
  SEARCH_DIRECTIONS = 8
  MAX_ITERATIONS = 3
  SEARCH_STEP_DISTANCE = 0.5 # meters

  def __init__(self):
    self.search_service = rospy.Service('/pose_search/search', PoseSearch, self.search_handler)

  def search_handler(self, req):
    search_poses = self.get_search_poses(req.target_point)
    costs = []

    for search_pose in search_poses:
      costs.append(self.check_pose_cost(search_pose))

    min_cost = inf
    best_pose = None
    for i in range(len(costs)):
      if costs[i] < min_cost:
        min_cost = costs[i]
        best_pose = search_pose[i]

    response = PoseSearchResponse()
    response.goal_pose = best_pose

  def get_search_poses(self, target_point: Point):
    search_step_deg = 360 / PoseSearch.SEARCH_DIRECTIONS
    search_step_rad = np.deg2rad(search_step_deg)

    search_poses = []

    for iteration in range(1, PoseSearch.MAX_ITERATIONS + 1):
      length_v = np.array([0, PoseSearch.SEARCH_STEP_DISTANCE * iteration])
      for direction in range(PoseSearch.SEARCH_DIRECTIONS):
        theta = search_step_rad * direction
        rot_mat = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

        search_point = Point()
        search_point.x, search_point.y = np.dot(rot_mat, length_v)
        
        angle_to_target = atan2(target_point.y - search_point.y, target_point.x - search_point.x)
        
        quat = quaternion_from_euler(0, 0, angle_to_target)

        search_pose = PoseStamped()
        search_pose.header.frame_id = '/map'
        search_pose.header.stamp = rospy.Time(0)
        search_pose.pose.position = search_point
        search_pose.pose.orientation = quat

        search_poses.append(search_pose)

    return search_poses

  def check_pose_cost(self, search_pose: PoseStamped):
    pass

if __name__ == '__main__':
  debug = Point()
  debug.x = 0
  debug.y = 0
  PoseSearch().get_search_poses(debug)