#!/usr/bin/env python

import rospy
import actionlib

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from rail_stretch_manipulation.msg import jointPosition
from rail_stretch_manipulation.srv import set_joint_state,set_joint_stateResponse

class JointController(object):

    def __init__(self):
        self.trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.trajectory_client.wait_for_server()

    def set_cmd(self, joint_name, value, wait):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(0.0)
        point.positions = [value]

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
        trajectory_goal.trajectory.joint_names = [joint_name]

        trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        
        self.trajectory_client.send_goal(trajectory_goal)

class jointControl(object):

    def __init__(self):
        rospy.Service("set_joint_state", set_joint_state, self.service_callback)
        self.joint_controller = JointController()

    def service_callback(self, request):
        self.joint_controller.set_cmd(request.joint_name, request.joint_value, False)
        return set_joint_stateResponse(True)

if __name__ == "__main__":
    rospy.init_node('joint_control')
    joint_control = jointControl()
    rospy.spin()