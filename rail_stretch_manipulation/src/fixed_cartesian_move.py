#!/usr/bin/env python

import rospy
import actionlib

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Twist, Point
from trajectory_msgs.msg import JointTrajectoryPoint

import math


class DriveController(object):

    def __init__(self):
        self.enabled = False
        self.rotation_effort = 0.0
        self.rotation_goal = 0.0

        self.linear_effort = 0.0
        self.linear_goal = 0.0

        self.linear_deadband = 0.01 # 1 cm
        self.odom = Odometry()

        # Publishers
        self.rotation_setpoint = rospy.Publisher('/fixed_cartesian_move/rotation/setpoint', Float64, queue_size=10)

        self.rotation_state = rospy.Publisher('/fixed_cartesian_move/rotation/state', Float64, queue_size=10)

        self.linear_setpoint = rospy.Publisher('/fixed_cartesian_move/linear/setpoint', Float64, queue_size=10)

        self.linear_state = rospy.Publisher('/fixed_cartesian_move/linear/state', Float64, queue_size=10)

        self.drive_cmd = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.Subscriber('/fixed_cartesian_move/rotation/control_effort', Float64, self.rotation_effort_callback)
        rospy.Subscriber('/fixed_cartesian_move/linear/control_effort', Float64, self.linear_effort_callback)

        self.rotation_setpoint.publish(Float64(self.rotation_goal))
        self.linear_setpoint.publish(Float64(self.linear_goal))

    def laser_callback(self, data):
        # Length of array is 720
        # rospy.loginfo("length" + str(len(data.ranges)))

        # Front of the robot.
        # rospy.loginfo(data.ranges[0])

        # Right Front of the robot.
        # rospy.loginfo(data.ranges[540])

        front_right = 0
        count = 1
        for i in range(540, 550):
            if not math.isnan(data.ranges[i]):
                front_right += data.ranges[i]
                count += 1
        
        front_right = front_right / count

        back_right = 0
        count = 1

        for i in range(530, 540):
            if not math.isnan(data.ranges[i]):
                back_right += data.ranges[i]
                count += 1

        back_right = back_right / count

        laser_difference = front_right - back_right

        self.rotation_state.publish(Float64(laser_difference))

    def odom_callback(self, data):
        self.odom = data
        self.linear_state.publish(Float64(data.pose.pose.position.x))

    def rotation_effort_callback(self, data):
        if not math.isnan(data.data):
            self.rotation_effort = data.data
        else:
            self.rotation_effort = 0.0

    def linear_effort_callback(self, data):
        if (self.enabled):
            self.linear_effort = data.data
            cmd_vel = Twist()
            cmd_vel.angular.z = self.rotation_effort
            cmd_vel.linear.x = self.linear_effort

            self.drive_cmd.publish(cmd_vel)

    def get_linear_error(self):
        current_x = self.odom.pose.pose.position.x
        
        return self.linear_goal - current_x

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def set_cmd(self, value, wait):
        self.linear_goal = self.odom.pose.pose.position.x + value
        
        self.linear_setpoint.publish(Float64(self.linear_goal))
        self.rotation_setpoint.publish(Float64(self.rotation_goal))

        if (wait): 
            while (abs(self.get_linear_error()) > self.linear_deadband):
                self.linear_setpoint.publish(Float64(self.linear_goal))
                self.rotation_setpoint.publish(Float64(self.rotation_goal))
        
class JointController(object):

    def __init__(self):
        self.joint_states = JointState()

        self.trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.trajectory_client.wait_for_server()

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, data):
        self.joint_states = data

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

        if (wait):
            self.trajectory_client.wait_for_result()


class FixedCartersianMoveAction(object):

    def __init__(self):
        rospy.Subscriber('goal', Point, self.fixed_cartesian_move_callback)
        self.drive_controller = DriveController()
        self.joint_controller = JointController()

    def fixed_cartesian_move_callback(self, data):
        self.joint_controller.set_cmd("joint_lift", data.z, False)
        self.drive_controller.enable()
        self.drive_controller.set_cmd(data.x, True)
        self.joint_controller.set_cmd("wrist_extension", data.y, False)


class AlignWallAction(object):

    def __init__(self):
        pass

if __name__ == "__main__":
    rospy.init_node('controller')
    fixed_cartesian_move_action = FixedCartersianMoveAction()
    align_wall_action = AlignWallAction()

    
    rospy.spin()