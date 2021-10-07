#!/usr/bin/env python
import rospy
import numpy as np
from math import cos, sin, pi
from geometry_msgs.msg import PointStamped

class PoseSearch():
  SEARCH_DIRECTIONS = 8
  MAX_ITERATIONS = 3
  SEARCH_STEP_DISTANCE = 0.5 # meters

  def __init__(self):
    pass

  def get_search_poses(self, point):
    search_step_deg = 360 / PoseSearch.SEARCH_DIRECTIONS
    search_step_rad = np.deg2rad(search_step_deg)
    
    for iteration in range(1, PoseSearch.MAX_ITERATIONS + 1):
      length_v = np.array([0, PoseSearch.SEARCH_STEP_DISTANCE * iteration])
      for direction in range(PoseSearch.SEARCH_DIRECTIONS):
        theta = search_step_rad * direction
        rot_mat = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
        print(np.dot(rot_mat, length_v))

if __name__ == '__main__':
  debug = PointStamped()
  debug.point.x = 0
  debug.point.y = 0
  debug.point.z = 0
  PoseSearch().get_search_poses(debug)