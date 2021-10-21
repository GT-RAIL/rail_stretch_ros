#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
class stretchState():
    def __init__(self):
        self.STATE = "WAITING"
        self.rate = 10.0

        rospy.wait_for_service('/grasp_object/trigger_grasp_object')
        rospy.loginfo('Connected to /grasp_object/trigger_grasp_object')
        self.trigger_grasp_object_service = rospy.ServiceProxy('/grasp_object/trigger_grasp_object', Trigger)
        
        rospy.wait_for_service('/funmap/trigger_lower_until_contact')
        rospy.loginfo(' Connected to /funmap/trigger_lower_until_contact.')
        self.trigger_lower_until_contact_service = rospy.ServiceProxy('/funmap/trigger_lower_until_contact', Trigger)

        rospy.wait_for_service('/switch_to_navigation_mode')
        rospy.loginfo('Connected to /switch_to_navigation_mode')
        self.switch_to_navigation_mode_service = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)

        rospy.wait_for_service('/switch_to_position_mode')
        rospy.loginfo(' Connected to /switch_to_position_mode')
        self.switch_to_position_mode_service = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        wait = self.move_base_client.wait_for_server(rospy.Duration(5.0))
        rospy.loginfo("Waiting for move_base action server...")
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            #return
        rospy.loginfo("Connected to move base server")
        self.i = 1
        self.timer = rospy.Timer(rospy.Duration(0.1), self.state_callback)
        #Service server for input pose and target pose with respect to aruco
    
    def done_cb(self,status,result):
        rospy.loginfo("Action server done executing")
    
    def active_cb(self):
        rospy.loginfo("Running action")
    
    def feedback_cb(self,feedback):
        print("Feedback" + str(feedback))

    def state_callback(self, timer):
        
        
        if self.i==1:
            trigger_request = TriggerRequest() 
            trigger_result = self.trigger_grasp_object_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

            trigger_request = TriggerRequest() 
            trigger_result = self.switch_to_navigation_mode_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))
            
            goal = MoveBaseGoal()
            self.i = 2 
            goal.target_pose = PoseStamped()
            goal.target_pose.header.stamp = rospy.Time(0)
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = 0.5
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.x = 0
            goal.target_pose.pose.orientation.y = 0
            goal.target_pose.pose.orientation.z = 0
            goal.target_pose.pose.orientation.w = 1
            self.move_base_client.send_goal(goal,self.done_cb, self.active_cb, self.feedback_cb)
        else:
            return

if __name__ == '__main__':
    try:
        rospy.init_node('stretch_state')
        node = stretchState()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('shutting down')