#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from rail_stretch_manipulation.joint_controller import JointController, Joints
from sensor_msgs.msg import JointState
from rail_stretch_state.srv import stretch_mission,stretch_missionResponse
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
        self.lift_state = None
        self.grasp_object = None
        self.drop_object = None
        self.joint_controller = JointController()
        #rospy.Subscriber('/stretch_state/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        service = rospy.Service("stretch_mission",stretch_mission,self.service_server)
        #self.timer = rospy.Timer(rospy.Duration(0.1), self.state_callback)
        #Service server for input pose and target pose with respect to aruco
    
    def service_server(self,request):
        if request.mode == "grasping":
            self.grasp_object = True
        elif request.mode == "dropping":
            self.drop_object = True
        self.STATE = "NAVIGATION"
        trigger_request = TriggerRequest() 
        trigger_result = self.switch_to_navigation_mode_service(trigger_request)
        rospy.loginfo('trigger_result = {0}'.format(trigger_result))
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose = request.goal
        self.move_base_client.send_goal(goal,self.done_cb, self.active_cb, self.feedback_cb)
        server_response = stretch_missionResponse()
        server_response.response = "Mission set successfully"
        server_response.result = True
        return server_response
    
    def done_cb(self,status,result):
        rospy.loginfo("Action server done executing")
        if self.grasp_object == True:
            trigger_request = TriggerRequest() 
            trigger_result = self.switch_to_position_mode_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))
            self.STATE = "GRASPING"
            trigger_request = TriggerRequest() 
            trigger_result = self.trigger_grasp_object_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))
            self.grasp_object = False
            self.STATE = "WAITING"
        elif self.drop_object == True:
            trigger_request = TriggerRequest() 
            trigger_result = self.switch_to_position_mode_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))
            self.STATE = "DROPPING"
            self.joint_controller.set_cmd(joints=[Joints.wrist_extension],values=[0.3],wait=True)
            #trigger_request = TriggerRequest() 
            #trigger_result = self.trigger_lower_until_contact_service(trigger_request)
            #rospy.loginfo('trigger_result = {0}'.format(trigger_result))
            
            #self.joint_controller.set_cmd(joints=[Joints.joint_lift],values=[self.lift_state + 0.1],wait=True)
            self.joint_controller.set_cmd(joints=[Joints.gripper_aperture],values=[0.0445],wait=True)
            #self.joint_controller.set_cmd(joints=[Joints.gripper_aperture],values=[0],wait=True)

            self.joint_controller.set_cmd(joints=[Joints.wrist_extension, Joints.joint_lift, Joints.gripper_aperture],values=[0,0.9,0],wait=True)
            #self.joint_controller.set_cmd(joints=[Joints.joint_gripper_finger_left],values=[-0.07],wait=True)            
            self.drop_object = False
            self.STATE = "WAITING"

    def joint_states_callback(self,joint_states):
        self.lift_state = joint_states.position[1]


    def active_cb(self):
        rospy.loginfo("Running action")
    
    def feedback_cb(self,feedback):
        rospy.logdebug(str(feedback))
    
if __name__ == '__main__':
    try:
        rospy.init_node('stretch_state')
        node = stretchState()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('shutting down')