#!/usr/bin/env python3
from enum import Enum
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from rail_stretch_navigation.srv import NavigateToAruco, NavigateToArucoRequest, GraspAruco, GraspArucoRequest
from rail_stretch_manipulation.joint_controller import JointController, Joints
from rail_stretch_state.srv import ExecuteStretchMission, ExecuteStretchMissionResponse

class States(Enum):
    WAITING = 0
    NAVIGATING = 1
    GRASPING = 2
    PLACING = 3
    FAILED = 4

class StretchState():
    ATTEMPTS = 4

    def __init__(self):
        self.STATE = States.WAITING
        self.rate = 10.0
        self.joint_controller = JointController()

        rospy.wait_for_service('/aruco_grasper/grasp_aruco')
        rospy.loginfo('Connected to /aruco_grasper/grasp_aruco')
        self.trigger_grasp_object_service = rospy.ServiceProxy('/aruco_grasper/grasp_aruco', GraspAruco)

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

        self.aruco_data = rospy.get_param('/aruco_marker_info')
        self.aruco_names = []
        
        for aruco_id, aruco_datum in self.aruco_data.items():
            self.aruco_names.append(aruco_datum['name'])
    
    def stretch_mission_handler(self, request):
        trigger_request = TriggerRequest()
        server_response = ExecuteStretchMissionResponse()

        self.joint_controller.stow()

        '''
        NAVIGATE TO FIRST ARUCO MARKER
        '''
        self.STATE = States.NAVIGATING
        
        self.switch_to_navigation_mode_service(trigger_request)
        
        navigate_to_aruco_goal = NavigateToArucoRequest()
        navigate_to_aruco_goal.aruco_name = request.from_aruco_name

        result = None
        
        for i in range(StretchState.ATTEMPTS):
            result = self.navigate_to_aruco_service(navigate_to_aruco_goal)

            if result.navigation_succeeded == True:
                break

        if result.navigation_succeeded == False:
            server_response.response = "Couldn't navigate to goal"
            server_response.mission_success = False
            self.STATE = States.WAITING
            self.joint_controller.stow()
            return server_response

        '''
        BEGIN GRASPING
        '''
        self.STATE = States.GRASPING
        self.switch_to_position_mode_service(trigger_request)

        for i in range(StretchState.ATTEMPTS):
            result = self.trigger_grasp_object_service(GraspArucoRequest(self.aruco_names.index(request.object_name)))
            
            if result.grasp_finished == True:
                break
        
        if result.grasp_finished == False:
            server_response.response = "Couldn't grasp object"
            server_response.mission_success = False
            self.STATE = States.WAITING
            self.joint_controller.stow()
            return server_response

        '''
        NAVIGATE TO SECOND ARUCO MARKER
        '''
        self.STATE = States.NAVIGATING
        
        self.switch_to_navigation_mode_service(trigger_request)

        navigate_to_aruco_goal = NavigateToArucoRequest()
        navigate_to_aruco_goal.aruco_name = request.to_aruco_name

        for i in range(StretchState.ATTEMPTS):
            result = self.navigate_to_aruco_service(navigate_to_aruco_goal)

            if result.navigation_succeeded == True:
                break

        if result.navigation_succeeded == False:
            server_response.response = "Couldn't navigate to goal"
            server_response.mission_success = False
            self.STATE = States.WAITING
            self.joint_controller.stow()
            return server_response

        '''
        BEGIN PLACING
        '''
        self.STATE = States.PLACING

        self.switch_to_position_mode_service(trigger_request)

        self.joint_controller.extend_arm()
        
        self.joint_controller.place_object()

        '''
        MISSION SUCCESS
        '''
        self.STATE = States.WAITING
        server_response.response = "Mission completed successfully"
        server_response.mission_success = True
        self.joint_controller.stow()
        return server_response

if __name__ == '__main__':
    try:
        rospy.init_node('stretch_state')
        node = StretchState()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('shutting down')