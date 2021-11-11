#!/usr/bin/env python3
import rospy
from rail_stretch_manipulation.joint_controller import JointController, Joints

class Stower:
  def __init__(self) -> None:
      self.joint_controller = JointController()
      self.joint_controller.stow()

if __name__ == '__main__':
  rospy.init_node()
  stower = Stower()