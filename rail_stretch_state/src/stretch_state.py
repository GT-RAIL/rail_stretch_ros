#!/usr/bin/env python3
from enum import Enum
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rail_stretch_navigation.srv import NavigateToArucoRequest
import actionlib
from actionlib_msgs.msg import GoalStatus
from rail_stretch_manipulation.joint_controller import JointController, Joints
from sensor_msgs.msg import JointState
from rail_stretch_state.srv import stretch_mission, stretch_missionResponse

class States(Enum):
    WAITING = 0
    NAVIGATING = 1
    GRASPING = 2
    PLACING = 3

class StretchState():
    def __init__(self):
        self.STATE = States.WAITING
        self.rate = 10.0
        self.joint_controller = JointController()
        self.stretch_mission_service = rospy.Service("stretch_mission", stretch_mission, self.service_server)

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
        rospy.loginfo('Connected to /switch_to_position_mode')
        self.switch_to_position_mode_service = rospy.ServiceProxy('/switch_to_position_mode', Trigger)

        rospy.wait_for_service('/navigate_to_aruco_action/navigate_to_aruco')
        rospy.loginfo('Connected to /navigate_to_aruco_action/navigate_to_aruco')
        self.navigate_to_aruco_service = rospy.ServiceProxy('/navigate_to_aruco_action/navigate_to_aruco', Trigger)
        
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
    
    def service_server(self, request):
        server_response = stretch_missionResponse()
        self.STATE = States.NAVIGATING
        trigger_request = TriggerRequest() 
        trigger_result = self.switch_to_navigation_mode_service(trigger_request)
        rospy.loginfo('trigger_result = {0}'.format(trigger_result))
        navigate_to_aruco_goal = NavigateToArucoRequest()
        navigate_to_aruco_goal.aruco_name = request.aruco_name
        trigger_result = self.navigate_to_aruco_service(navigate_to_aruco_goal)
        rospy.loginfo('trigger_result = {0}'.format(trigger_result))
        if trigger_result.sent_goal == False:
            server_response.response = "Couldn't navigate to goal"
            server_response.result = False
            self.STATE = States.WAITING
            return server_response
        rospy.loginfo("Action server done executing")
        if request.mode == "grasping":
            trigger_request = TriggerRequest() 
            trigger_result = self.switch_to_position_mode_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))
            self.STATE = States.GRASPING
            trigger_request = TriggerRequest() 
            trigger_result = self.trigger_grasp_object_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))
            if trigger_result.success == False:
                server_response.response = "Couldn't grasp object"
                server_response.result = False
                self.STATE = States.WAITING
                return server_response
            self.STATE = States.WAITING
        elif request.mode == "placing":
            trigger_request = TriggerRequest() 
            trigger_result = self.switch_to_position_mode_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))
            self.STATE = States.PLACING
            self.joint_controller.extend_arm()
            trigger_request = TriggerRequest() 
            trigger_result = self.trigger_lower_until_contact_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))
            if trigger_result.success == False:
                server_response.response = "Couldn't find a contact to place object"
                server_response.result = False
                self.STATE = States.WAITING
                return server_response
            self.joint_controller.place_object()
            self.STATE = States.WAITING
        server_response.response = "Mission completed successfully"
        server_response.result = True
        return server_response
    
    def joint_states_callback(self,joint_states):
        self.lift_state = joint_states.position[1]

if __name__ == '__main__':
    try:
        rospy.init_node('stretch_state')
        node = StretchState()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('shutting down')