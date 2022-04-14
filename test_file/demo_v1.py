#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty,String, Int64, Int16
from smach import CBState
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
import json
from datetime import datetime
from smach import State, StateMachine, Concurrence
from pura_execution.srv import *
from pura_execution.msg import *
from pura_execution.msg import DockingAction,DockingGoal
from geometry_msgs.msg import * 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pura_execution.msg import Surface
import math

'''
Charging Station Loop tests
- navigation only
'''

global surface_flag,start_x, s 
target_dist = 0
start_x = 0
start_y = 0
surface_flag = True

# set parameter
ROTATION_SPEED = 0.1
FORWARD_SPEED = 0.1
PITCH_D_TOLERANCE = 5
X_DISTANCE_TOLERANCE = 0.08 
DOCKING_DISTANCE = 0.165

# read file
# simulation file location
myJsonFile = open(r'src/pura_cloud/scripts/arche_solution/create_task.json', 'r')
CSJsonFile = open(r'/home/tommy/PURA_ROS_WS-main/src/pura_execution/json/preset_point.json', 'r')

jsonData = myJsonFile.read()           
jsonObject = json.loads(jsonData)
after_navigation_duration = 1
sanitation_duration = 5

CSJsonData = CSJsonFile.read()
CSJsonObject = json.loads(CSJsonData)


@smach.cb_interface(input_keys=['json_col_in'],output_keys=['json_col_out'],outcomes=['send_json_col','finished_json'])
def json_cb(userdata):
    rospy.loginfo('running json server')
    json_length_no_x = len(jsonObject)
    array = [r for r in range(json_length_no_x)]
    if userdata.json_col_in in array:
        userdata.json_col_out = userdata.json_col_in
        return 'send_json_col'
    else:
        userdata.json_col_out = 0
        rospy.sleep(1)
        return 'finished_json'
        
@smach.cb_interface(input_keys=['nav_col_in'],output_keys=['nav_col_out'],outcomes=['finished_nav','failed_nav'])
def nav_cb(userdata):
    rospy.loginfo('running nav server')

    nav_x = float(jsonObject[userdata.nav_col_in]["Position_X"])
    nav_y = float(jsonObject[userdata.nav_col_in]["Position_Y"])
    nav_z = float(jsonObject[userdata.nav_col_in]["Quaternion_Z"])
    nav_w = float(jsonObject[userdata.nav_col_in]["Quaternion_W"])

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

        rospy.loginfo('Waypoint ' + str(userdata.nav_col_in) + 'is reached')
        rospy.sleep(after_navigation_duration)
        userdata.nav_col_out = userdata.nav_col_in+1
        return 'finished_nav'
    else:

        rospy.loginfo('robot failed to arrive')
        userdata.nav_col_out = userdata.nav_col_in + 1
        return 'failed_nav'
 
if __name__ == '__main__':
    rospy.init_node('pura_state_machine')
    

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['i_have_finished'])
    sm.userdata.sm_counter = 0
    sm.userdata.time = 0

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    

    # Open the container
    with sm:
        # Add states to the container

        # smach.StateMachine.add('Undocking_server', CBState(undocking_cb), 
        #                         transitions={'check_again':'Undocking_server',
        #                                     'finish_undocking':'Json_server'},
        #                         remapping = {'undocking_in':'sm_counter',
        #                                      'undocking_out':'sm_counter'})    

        smach.StateMachine.add('Json_server', CBState(json_cb),
                                transitions = {'send_json_col':'Nav_server', 
                                            'finished_json':'i_have_finished'},
                                remapping = {'json_col_in': 'sm_counter',
                                             'json_col_out':'sm_counter'})

        smach.StateMachine.add('Nav_server', CBState(nav_cb), 
                                transitions={'finished_nav':'Json_server',
                                             'failed_nav':'Json_server'},
                                remapping = {'nav_col_in': 'sm_counter',
                                             'nav_col_out':'sm_counter'})

        # smach.StateMachine.add('Charging_station', CBState(charging_station_cb), 
        #                         transitions={'arrive_charging_station': 'Stage1_server',
        #                                      'failed':'Charging_station'})

        # smach.StateMachine.add('Stage1_server', CBState(stage1_cb), 
        #                         transitions={'check_again': 'Stage1_server',
        #                                     'go_stage2':'Stage2_server'})

        # smach.StateMachine.add('Stage2_server', CBState(stage2_cb), 
        #                         transitions={'reset':'Reset_docking_server',
        #                                     'check_again':'Stage2_server',
        #                                     'finish':'Undocking_server'})

        # smach.StateMachine.add('Reset_docking_server', CBState(reset_docking_cb), 
        #                         transitions={'check_again':'Reset_docking_server',
        #                                     'finish':'Stage1_server'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
