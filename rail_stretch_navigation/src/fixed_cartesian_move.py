#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2, LaserScan, JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Twist, Point
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rotation_difference = 0
rotation_effort = 0

rotation_setpoint = rospy.Publisher('/fixed_cartesian_move/setpoint', Float64, queue_size=10)
rotation_state = rospy.Publisher('/fixed_cartesian_move/state', Float64, queue_size=10)

drive = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=10)

joint_states = JointState()

trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)


move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

def laser_callback(data):
    # rospy.loginfo(data.ranges)
    # rospy.loginfo("min" + str(data.angle_min))
    # rospy.loginfo("max" + str(data.angle_max))
    # rospy.loginfo("increment" + str(data.angle_increment))

    # Length of array is 720
    # rospy.loginfo("length" + str(len(data.ranges)))

    # Front of the robot.
    # rospy.loginfo(data.ranges[0])

    # Right Front of the robot.
    rospy.loginfo(data.ranges[550])

    rospy.loginfo(data.ranges[530])

    laser_difference = data.ranges[550] - data.ranges[530]
    rotation_setpoint.publish(Float64(0))
    rotation_state.publish(Float64(laser_difference))

def rotation_effort_callback(data):
    # cmd_vel = Twist()
    # cmd_vel.angular.z = data.data

    # drive.publish(cmd_vel)
    rotation_effort = data.data

def joint_states_callback(data):
    joint_states = data

def cartesian_move_callback(data):
    move_base_goal = MoveBaseGoal()
    move_base_goal.target_pose.header.frame_id = "base_link"
    move_base_goal.target_pose.header.stamp = rospy.Time.now()
    move_base_goal.target_pose.pose.position.x = data.x
    move_base_goal.target_pose.pose.orientation.w = 1.0

    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(0.0)
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.goal_time_tolerance = rospy.Time(1.0)

    joint_name = "joint_lift"
    trajectory_goal.trajectory.joint_names = [joint_name]

    point.positions = [data.y]
    trajectory_goal.trajectory.points = [point]
    trajectory_goal.trajectory.header.stamp = rospy.Time.now()

    move_base_client.send_goal(move_base_goal)
    trajectory_client.send_goal(trajectory_goal)

    move_base_client.wait_for_result()


    joint_name = "wrist_extension"
    trajectory_goal.trajectory.joint_names = [joint_name]

    point.positions = [data.z]
    trajectory_goal.trajectory.points = [point]
    trajectory_goal.trajectory.header.stamp = rospy.Time.now()

    trajectory_client.send_goal(trajectory_goal)

if __name__ == "__main__":
    rospy.init_node('drive_align')

    trajectory_client.wait_for_server()
    move_base_client.wait_for_server()

    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber('/fixed_cartesian_move/control_effort', Float64, rotation_effort_callback)

    rospy.Subscriber('/fixed_cartesian_move/goal', Point, cartesian_move_callback)

    rospy.Subscriber('/stretch/joint_states', JointState, joint_states_callback)

    rospy.spin()