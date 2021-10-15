#!/usr/bin/env python

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
import actionlib

import rospy

class JointController(object):

    def __init__(self):
        self.joint_states = JointState()

        self.trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.trajectory_client.wait_for_server()

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, data):
        self.joint_states = data

    def set_cmd(self, joint_names, values, wait):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(0.0)
        point.positions = values

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.goal_time_tolerance = rospy.Time(2.0)
        trajectory_goal.trajectory.joint_names = joint_names

        trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        
        self.trajectory_client.send_goal(trajectory_goal)

        if (wait):
            self.trajectory_client.wait_for_result()