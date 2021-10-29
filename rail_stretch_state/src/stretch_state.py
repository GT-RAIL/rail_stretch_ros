#!/usr/bin/env python3
from enum import Enum
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from rail_stretch_navigation.srv import NavigateToAruco, NavigateToArucoRequest
from rail_stretch_manipulation.joint_controller import JointController, Joints
from rail_stretch_state.srv import ExecuteStretchMission, ExecuteStretchMissionResponse

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

        rospy.wait_for_service('/grasp_object/trigger_grasp_object')
        rospy.loginfo('Connected to /grasp_object/trigger_grasp_object')
        self.trigger_grasp_object_service = rospy.ServiceProxy('/grasp_object/trigger_grasp_object', Trigger)
        
        rospy.wait_for_service('/funmap/trigger_lower_until_contact')
        rospy.loginfo(' Connected to /funmap/trigger_lower_until_contact.')
        self.trigger_lower_until_contact_service = rospy.ServiceProxy('/funmap/trigger_lower_until_contact', Trigger)

        rospy.wait_for_service('/funmap/trigger_align_with_nearest_cliff')
        rospy.loginfo(' Connected to /funmap/trigger_align_with_nearest_cliff')
        self.trigger_align_with_nearest_cliff_service = rospy.ServiceProxy('/funmap/trigger_align_with_nearest_cliff', Trigger)

        rospy.wait_for_service('/switch_to_navigation_mode')
        rospy.loginfo('Connected to /switch_to_navigation_mode')
        self.switch_to_navigation_mode_service = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)

        rospy.wait_for_service('/switch_to_position_mode')
        rospy.loginfo('Connected to /switch_to_position_mode')
        self.switch_to_position_mode_service = rospy.ServiceProxy('/switch_to_position_mode', Trigger)

        rospy.wait_for_service('/navigate_to_aruco_action/navigate_to_aruco')
        rospy.loginfo('Connected to /navigate_to_aruco_action/navigate_to_aruco')
        self.navigate_to_aruco_service = rospy.ServiceProxy('/navigate_to_aruco_action/navigate_to_aruco', NavigateToAruco)

        self.stretch_mission_service = rospy.Service("stretch_mission", ExecuteStretchMission, self.stretch_mission_handler)
    
    def stretch_mission_handler(self, request):
        trigger_request = TriggerRequest() 
        server_response = ExecuteStretchMissionResponse()

        '''
        NAVIGATE TO FIRST ARUCO MARKER
        '''
        self.STATE = States.NAVIGATING
        
        self.switch_to_navigation_mode_service(trigger_request)

        navigate_to_aruco_goal = NavigateToArucoRequest()
        navigate_to_aruco_goal.aruco_name = request.from_aruco_name
        result = self.navigate_to_aruco_service(navigate_to_aruco_goal)

        if result.navigation_succeeded == False:
            server_response.response = "Couldn't navigate to goal"
            server_response.mission_success = False
            self.STATE = States.WAITING
            return server_response

        '''
        BEGIN GRASPING
        '''
        self.STATE = States.GRASPING
        self.switch_to_position_mode_service(trigger_request)

        result = self.trigger_grasp_object_service(trigger_request)
        
        if result.success == False:
            server_response.response = "Couldn't grasp object"
            server_response.mission_success = False
            self.STATE = States.WAITING
            return server_response

        '''
        NAVIGATE TO SECOND ARUCO MARKER
        '''
        self.STATE = States.NAVIGATING
        
        self.switch_to_navigation_mode_service(trigger_request)

        navigate_to_aruco_goal = NavigateToArucoRequest()
        navigate_to_aruco_goal.aruco_name = request.to_aruco_name
        result = self.navigate_to_aruco_service(navigate_to_aruco_goal)

        if result.navigation_succeeded == False:
            server_response.response = "Couldn't navigate to goal"
            server_response.mission_success = False
            self.STATE = States.WAITING
            return server_response

        '''
        BEGIN PLACING
        '''
        self.STATE = States.PLACING

        self.switch_to_position_mode_service(trigger_request)

        self.joint_controller.extend_arm()
        
        result = self.trigger_lower_until_contact_service(trigger_request)
        
        if result.success == False:
            server_response.response = "Couldn't find a contact to place object"
            server_response.mission_success = False
            self.STATE = States.WAITING
            return server_response
            
        self.joint_controller.place_object()

        '''
        MISSION SUCCESS
        '''
        self.STATE = States.WAITING
        server_response.response = "Mission completed successfully"
        server_response.mission_success = True
        return server_response

if __name__ == '__main__':
    try:
        rospy.init_node('stretch_state')
        node = StretchState()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('shutting down')