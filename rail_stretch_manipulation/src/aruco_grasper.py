#!/usr/bin/env python
import rospy
import math

from joint_controller import JointController

class ArucoGrasper(object):

    def __init__(self):
        self.joint_controller = JointController()
        self.joint_controller.set_cmd(joint_names=['joint_head_pan', 'joint_head_tilt'], values=[-math.pi / 2, 0], wait=True)


if __name__ == '__main__':
    rospy.init_node('aruco_grasper')
    aruco_grasper = ArucoGrasper()