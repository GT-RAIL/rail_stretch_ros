#!/usr/bin/env python

from enum import Enum
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
import actionlib

import rospy

class Joints(Enum):
    wrist_extension = 0
    joint_lift = 1
    joint_arm_l3 = 2
    joint_arm_l2 = 3
    joint_arm_l1 = 4
    joint_arm_l0 = 5
    joint_head_pan = 6
    joint_head_tilt = 7
    joint_wrist_yaw = 8
    joint_gripper_finger_left = 9
    joint_gripper_finger_right = 10
    joint_mobile_base_translation = 11
    gripper_aperture = None


class JointController(object):

    def __init__(self):
        self.joint_states = JointState()

        self.trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.trajectory_client.wait_for_server()

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, data):
        self.joint_states = data

    def set_cmd(self, joints, values, wait):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(0.0)
        point.positions = values

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.goal_time_tolerance = rospy.Time(2.0)

        joint_names = []

        for joint in joints:
            joint_names.append(Joints(joint).name)

        trajectory_goal.trajectory.joint_names = joint_names

        trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        
        self.trajectory_client.send_goal(trajectory_goal)

        if (wait):
            self.trajectory_client.wait_for_result()