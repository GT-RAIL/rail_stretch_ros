#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger

if __name__ == "__main__":
    initial_scan = False
    
    while not rospy.is_shutdown():
        if not initial_scan:
            rospy.wait_for_service('/funmap/trigger_head_scan')
            try:
                trigger_head_scan = rospy.ServiceProxy('/funmap/trigger_head_scan', Trigger)
                response = trigger_head_scan()
                initial_scan = response.success
                print(initial_scan)
                print("Initial scan complete.")
            except rospy.ServiceException as e:
                print(e)
                print("Trying again...")

        try:
            rospy.wait_for_service('/funmap/trigger_drive_to_scan')
            rospy.wait_for_service('/funmap/trigger_head_scan')
            trigger_drive_to_scan = rospy.ServiceProxy('funmap/trigger_drive_to_scan', Trigger)
            response = trigger_drive_to_scan()
            print(response.success)
            print("Arrived at new scan location.")

            trigger_head_scan = rospy.ServiceProxy('/funmap/trigger_head_scan', Trigger)
            response = trigger_head_scan()
            print(response.success)
            print("Did another scan.")
        except rospy.ServiceException as e:
            print(e)
            print("Trying again...")
            
        

