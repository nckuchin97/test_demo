#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty,String, Int64, Int16,Int16MultiArray
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
from geometry_msgs.msg import * 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pura_execution.msg import Surface
from component_controller import *
from subcode_status import *
from subcode_nav import *
import math

'''
Charging Station Loop tests
- charging station
- led
- navigation only
- update cloud info
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
CSJsonFile = open(r'src/pura_execution/json/preset_point.json', 'r')

jsonData = myJsonFile.read()           
jsonObject = json.loads(jsonData)
CSJsonData = CSJsonFile.read()
CSJsonObject = json.loads(CSJsonData)

after_navigation_duration = 1
sanitation_duration = 5

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
    component_control('ghost_white')
    update_robot_status("Navigating to Point "+ str(userdata.nav_col_in))

    nav_x = float(jsonObject[userdata.nav_col_in]["Position_X"])
    nav_y = float(jsonObject[userdata.nav_col_in]["Position_Y"])
    nav_z = float(jsonObject[userdata.nav_col_in]["Quaternion_Z"])
    nav_w = float(jsonObject[userdata.nav_col_in]["Quaternion_W"])

    if nav_function(nav_x,nav_y,nav_z,nav_w) == 'finished_nav':

        update_robot_status('Arriving Point '+ str(userdata.nav_col_in))
        rospy.loginfo('Waypoint ' + str(userdata.nav_col_in) + 'is reached')
        rospy.sleep(after_navigation_duration)
        
        component_control(jsonObject[userdata.nav_col_in]["Sanitation_Function"])
        rospy.sleep(5)
        component_control('Stop')

        userdata.nav_col_out = userdata.nav_col_in+1
        return 'finished_nav'
    else:
        update_robot_status('Failed on Point '+ str(userdata.nav_col_in))
        rospy.loginfo('robot failed to arrive')
        component_control('indian_red')
        userdata.nav_col_out = userdata.nav_col_in + 1
        return 'failed_nav'
        
@smach.cb_interface(input_keys=['cs_col_in'],output_keys=['cs_col_out'],outcomes=['finished_cs','failed_cs'])
def cs_cb(userdata):

    rospy.loginfo('running cs server')
    component_control('gold_yellow')

    nav_x = float(CSJsonObject["Position_X"])
    nav_y = float(CSJsonObject["Position_Y"])
    nav_z = float(CSJsonObject["Quaternion_Z"])
    nav_w = float(CSJsonObject["Quaternion_W"])

    if nav_function(nav_x,nav_y,nav_z,nav_w) == 'finished_nav':
        
        update_robot_status('Reaching Charging Station')
        rospy.loginfo('Reaching Charging Station ...... ')
        rospy.sleep(after_navigation_duration)
        userdata.cs_col_out = userdata.cs_col_in
        return 'finished_cs'
    else:
        update_robot_status('Failed to Charging Station')
        rospy.loginfo('robot failed to charging station')
        userdata.cs_col_out = userdata.cs_col_in
        return 'failed_cs'
 
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
                                            'finished_json':'Charging_station'},
                                remapping = {'json_col_in': 'sm_counter',
                                             'json_col_out':'sm_counter'})

        smach.StateMachine.add('Nav_server', CBState(nav_cb), 
                                transitions={'finished_nav':'Json_server',
                                             'failed_nav':'Json_server'},
                                remapping = {'nav_col_in': 'sm_counter',
                                             'nav_col_out':'sm_counter'})

        smach.StateMachine.add('Charging_station', CBState(cs_cb), 
                                transitions={'finished_cs': 'i_have_finished',
                                             'failed_cs':'Charging_station'},
                                remapping = {'cs_col_in': 'sm_counter',
                                             'cs_col_out':'sm_counter'})

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
