#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped
class stretchState():
    def __init__(self):
        self.STATE = "WAITING"
        self.rate = 10.0

        #rospy.wait_for_service('/grasp_object/trigger_grasp_object')
        rospy.loginfo('Connected to /grasp_object/trigger_grasp_object')
        #self.trigger_grasp_object_service = rospy.ServiceProxy('/grasp_object/trigger_grasp_object', Trigger)
        
        #rospy.wait_for_service('/funmap/trigger_lower_until_contact')
        rospy.loginfo(' Connected to /funmap/trigger_lower_until_contact.')
        #self.trigger_lower_until_contact_service = rospy.ServiceProxy('/funmap/trigger_lower_until_contact', Trigger)

        self.move_base_goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1,latch=True)
        
        #Service server for input pose and target pose with respect to aruco
    
    def run(self):
        rate = rospy.Rate(self.rate)
        i=1
        while not rospy.is_shutdown():
            '''
            goal = PoseStamped()
            rospy.loginfo('hello')
            goal.header.frame_id = 'map'
            goal.header.stamp = rospy.Time(0)
            goal.pose.position.x = 1.0
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            print(goal)
            self.move_base_goal_pub.publish(goal)
            '''
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('stretch_state')
        node = stretchState()
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo('shutting down')