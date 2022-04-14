#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty,String, Int64, Int16MultiArray
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

def component_control(control):
    
    control = rospy.Publisher('Component_talker', Int16MultiArray, queue_size=10)
    control_input = Int16MultiArray()
    control_input.layout.dim = []
    control_input.layout.data_offset = 0
    control_input.data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    if control== 'gold_yellow':
        # gold yellow : charging
        control_input.data[0] = 1
        control_input.data[1] = 255
        control_input.data[2] = 215
        control_input.data[3] = 0

    elif control== 'light_green':
        # Light Green : fully charged
        control_input.data[0] = 1
        control_input.data[1] = 144
        control_input.data[2] = 238
        control_input.data[3] = 144

    elif control== 'ghost_white':
        # Ghost white : navigation
        control_input.data[0] = 1
        control_input.data[1] =  248
        control_input.data[2] = 248
        control_input.data[3] = 255

    elif control== 'indian_red':
        # Indian Red : error
        control_input.data[0] = 1
        control_input.data[1] = 205
        control_input.data[2] = 92
        control_input.data[3] = 92

    elif control== 'steelblue':
        # steelblue: floor sanitation
        control_input.data[0] = 1
        control_input.data[1] = 70
        control_input.data[2] = 130
        control_input.data[3] = 180

    elif control== 'pump_purple':
        # Plum Purple : air sanitation
        control_input.data[0] = 1
        control_input.data[1] = 221
        control_input.data[2] = 160
        control_input.data[3] = 221

    elif control== 'SS1':
        # Plum Purple : air sanitation
        control_input.data[0] = 1
        control_input.data[1] = 0
        control_input.data[2] = 255
        control_input.data[3] = 255
        control_input.data[8] = 1
        control_input.data[9] = 100

    elif control== 'Stop':
        # Plum Purple : air sanitation
        control_input.data[8] = 0
        control_input.data[9] = 0

    for i in range (0,10):
        control.publish(control_input)
 