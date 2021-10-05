import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class NavigateToObjectAction:
  def __init__(self) -> None:
    self.navigate_to_object_service = rospy.Service('/object_manager/navigate_to_object', NavigateToObject, self.navigate_to_object)
    self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.move_base_client.wait_for_server()
  
  def navigate_to_object(self, req) -> bool:
    if req.object_name in self.object_dict:
      goal = MoveBaseGoal()
      goal.target_pose = PoseStamped()
      goal.target_pose.pose = self.object_dict[req.object_name][0]
      goal.target_pose.header.stamp = rospy.Time(0)
      goal.target_pose.header.frame_id = 'map'
      goal.target_pose.pose.position.z = 0
      goal.target_pose.pose.orientation.x = 0
      goal.target_pose.pose.orientation.y = 0
      goal.target_pose.pose.orientation.z = 0
      goal.target_pose.pose.orientation.w = 1
      self.move_base_client.send_goal(goal)
      return True
    
    return False