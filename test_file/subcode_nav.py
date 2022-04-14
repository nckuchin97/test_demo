#!/usr/bin/env python

import rospy
import json
import math
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

'''
Gives navigation goal
'''

# parameters
after_navigation_duration = 1

def nav_function(nav_x,nav_y,nav_z,nav_w):

    client_nav = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    rospy.loginfo("Wait for the waypoint server to come up")
    client_nav.wait_for_server(rospy.Duration(5))
    goal_sent = True
    goal_nav = MoveBaseGoal()
    goal_nav.target_pose.header.frame_id = 'map'
    goal_nav.target_pose.header.stamp = rospy.Time.now()
    goal_nav.target_pose.pose.position.x= nav_x
    goal_nav.target_pose.pose.position.y= nav_y
    goal_nav.target_pose.pose.orientation.z = nav_z
    goal_nav.target_pose.pose.orientation.w = nav_w
    client_nav.send_goal(goal_nav)
    wait = client_nav.wait_for_result(rospy.Duration(300))
    state = client_nav.get_state()

    if wait and state == GoalStatus.SUCCEEDED:
        return 'finished_nav'
    else:
        return 'failed_nav'