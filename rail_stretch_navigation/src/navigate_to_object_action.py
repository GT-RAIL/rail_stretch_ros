#! /usr/bin/env python2.7
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rail_stretch_navigation.srv import NavigateToObject
from rail_stretch_perception.srv import GetObjectPosition
from rail_stretch_navigation.srv import GetValidPose

class NavigateToObjectAction:
  def __init__(self):
    rospy.wait_for_service('/object_manager/get_object_position')
    self.navigate_to_object_service = rospy.Service('/navigate_to_object_action/navigate_to_object', NavigateToObject, self.navigate_to_object)
    self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.move_base_client.wait_for_server()
    self.get_object_position = rospy.ServiceProxy('/object_manager/get_object_position', GetObjectPosition)
    self.get_valid_pose = rospy.ServiceProxy('/pose_search/get_valid_pose', GetValidPose)
  
  def navigate_to_object(self, req):
    resp = self.get_object_position(req.object_name)
    if resp:
      goal = MoveBaseGoal()
      goal.target_pose = self.get_valid_pose(resp.object_position.point).goal_pose

      if goal.target_pose:
        self.move_base_client.send_goal(goal)
        return True

    return False

if __name__ == '__main__':
  rospy.init_node('navigate_to_object_action')
  navigate_to_object_action = NavigateToObjectAction()
  rospy.spin()
