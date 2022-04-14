#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty,String, Int64, Int16,Int16MultiArray
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
Docking + Undocking
'''

# set parameter
ROTATION_SPEED = 0.1
FORWARD_SPEED = 0.1
PITCH_D_TOLERANCE = 5
DOCKING_DISTANCE = 0.165 


def docking_1():

    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    rate = rospy.Rate(10)
    rospy.sleep(1)

    aruco_pitch_da =[]
    aruco_pos_xa = []
    aruco_pos_za = []

    for i in range(0,5):
        aruco_data = rospy.wait_for_message('/aruco_single/pose',PoseStamped)

        orientation_q = aruco_data.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        aruco_pitch_da.append(round(pitch*180/math.pi,5))
        aruco_pos_xa.append(round(aruco_data.pose.position.x,5))
        aruco_pos_za.append(round(aruco_data.pose.position.z,5))
        # print(aruco_pitch_da[i],aruco_pos_xa[i],aruco_pos_za[i])

    aruco_pitch_d = round(sum(aruco_pitch_da)/ len(aruco_pitch_da),4)
    aruco_pos_x = round(sum(aruco_pos_xa)/ len(aruco_pos_xa),4)
    aruco_pos_z = round(sum(aruco_pos_za)/ len(aruco_pos_za),4)
    print(aruco_pitch_d, aruco_pos_x,aruco_pos_z)

    cmd.linear.x = 0
    cmd.angular.z = 0
    vel_publisher.publish(cmd)
    t0 = 1
    rotate_angle = 0

    if aruco_pitch_d > PITCH_D_TOLERANCE :

        print('stage1 turn right')
        while (rotate_angle < abs(aruco_pitch_d)):
            cmd.angular.z = - ROTATION_SPEED
            vel_publisher.publish(cmd)
            # Calculate current angle
            rotate_angle = ROTATION_SPEED* (t0)*180/math.pi
            t0 = t0+0.5
            rospy.sleep(0.5)
        cmd.linear.x = 0
        cmd.angular.z = 0
        vel_publisher.publish(cmd)
        return 'check_again'

    elif aruco_pitch_d < -PITCH_D_TOLERANCE :

        print('stage1 turn left')
        while (rotate_angle < abs(aruco_pitch_d)):
            cmd.angular.z = ROTATION_SPEED
            vel_publisher.publish(cmd)
            # Calculate current angle
            rotate_angle = ROTATION_SPEED* (t0)*180/math.pi
            t0 = t0+0.5
            rospy.sleep(0.5)

        cmd.linear.x = 0
        cmd.angular.z = 0
        vel_publisher.publish(cmd)
        return 'check_again'

    else: 
        return 'go_stage2'

def docking_2():
 
    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    rate = rospy.Rate(10)

    aruco_data = rospy.wait_for_message('/aruco_single/pose',PoseStamped)

    orientation_q = aruco_data.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    aruco_pitch_d = round(pitch*180/math.pi,5)
    aruco_pos_x = round(aruco_data.pose.position.x,5)
    aruco_pos_z = round(aruco_data.pose.position.z,5)
    print(aruco_pitch_d,aruco_pos_x,aruco_pos_z)

    rotate_angle = 0
    t0 = 1

    if aruco_pos_z > 0.23 and aruco_pos_z < 0.28 and abs(aruco_pos_x) > 0.1:

        rotate_angle = 0
        t0 =1

        if aruco_pitch_d > 0 :
            print('stage2 turn right')
            while (rotate_angle < abs(aruco_pitch_d)):
                cmd.angular.z = - ROTATION_SPEED
                vel_publisher.publish(cmd)
                # Calculate current angle
                rotate_angle = ROTATION_SPEED* (t0)*180/math.pi
                t0 = t0+0.5
                
                rospy.sleep(0.5)

        elif aruco_pitch_d < 0 :
            print('stage1 turn left')
            while (rotate_angle < abs(aruco_pitch_d)):
                cmd.angular.z = ROTATION_SPEED
                vel_publisher.publish(cmd)
                # Calculate current angle
                rotate_angle = ROTATION_SPEED* (t0)*180/math.pi
                t0 = t0+0.5
                
                rospy.sleep(0.5)

        cmd.linear.x = 0
        cmd.angular.z = 0
        vel_publisher.publish(cmd)

        print('reset docking')
        return 'reset'


    # alumimiun z value : 0.175
    elif aruco_pos_z > 0.182:
        if aruco_pos_x > 0.004: 
            cmd.angular.z = -0.02
            rospy.sleep(0.5)
            # print('go right')
        elif aruco_pos_x < 0.001: 
            cmd.angular.z = 0.02
            rospy.sleep(0.5)
            # print('go left')
        vel_publisher.publish(cmd)
        cmd.linear.x = 0.05
        vel_publisher.publish(cmd)
        rospy.sleep(0.5)
        
        return 'check_again'

    elif aruco_pos_z < 0.182 :
        rotate_angle = 0
        t0 = 1

        cmd.linear.x = 0
        cmd.angular.z = 0
        vel_publisher.publish(cmd)

        aruco_data = rospy.wait_for_message('/aruco_single/pose',PoseStamped)

        orientation_q = aruco_data.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        aruco_pitch_d = round(pitch*180/math.pi,5)
        aruco_pos_x = round(aruco_data.pose.position.x,5)
        aruco_pos_z = round(aruco_data.pose.position.z,5)
        print(aruco_pitch_d,aruco_pos_x,aruco_pos_z)

        if aruco_pitch_d > PITCH_D_TOLERANCE :
            print('stage2 turn right')
            while (rotate_angle < abs(aruco_pitch_d)):
                cmd.angular.z = - ROTATION_SPEED
                vel_publisher.publish(cmd)
                # Calculate current angle
                rotate_angle = ROTATION_SPEED* (t0)*180/math.pi
                t0 = t0+1
                
                rospy.sleep(1)

            cmd.linear.x = 0
            cmd.angular.z = 0
            vel_publisher.publish(cmd)

        elif aruco_pitch_d < -PITCH_D_TOLERANCE :
            print('stage2 turn left')
            while (rotate_angle < abs(aruco_pitch_d)):
                cmd.angular.z = ROTATION_SPEED
                vel_publisher.publish(cmd)
                # Calculate current angle
                rotate_angle = ROTATION_SPEED* (t0)*180/math.pi
                t0 = t0+1
                
                rospy.sleep(1)

            cmd.linear.x = 0
            cmd.angular.z = 0
            vel_publisher.publish(cmd)     

        return 'finish'

def reset_docking(userdata):

    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    rate = rospy.Rate(10)

    aruco_data = rospy.wait_for_message('/aruco_single/pose',PoseStamped)

    orientation_q = aruco_data.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    aruco_pitch_d = round(pitch*180/math.pi,5)
    aruco_pos_x = round(aruco_data.pose.position.x,5)
    aruco_pos_z = round(aruco_data.pose.position.z,5)
    print(aruco_pitch_d,aruco_pos_x,aruco_pos_z)

    t0 = 1
    rotate_angle = 0

    if aruco_pos_z < 0.415  :

        if aruco_pitch_d > PITCH_D_TOLERANCE :
            print('stage2 turn right')
            while (rotate_angle < abs(aruco_pitch_d)):
                cmd.linear.x = 0
                cmd.angular.z = - ROTATION_SPEED
                vel_publisher.publish(cmd)
                # Calculate current angle
                rotate_angle = ROTATION_SPEED* (t0)*180/math.pi
                t0 = t0+0.5
                
                rospy.sleep(0.5)

            cmd.linear.x = 0
            cmd.angular.z = 0
            vel_publisher.publish(cmd)
            
        elif aruco_pitch_d < -PITCH_D_TOLERANCE :
            print('stage2 turn left')
            while (rotate_angle < abs(aruco_pitch_d)):
                cmd.linear.x=0
                cmd.angular.z = ROTATION_SPEED
                vel_publisher.publish(cmd)
                # Calculate current angle
                rotate_angle = ROTATION_SPEED* (t0)*180/math.pi
                t0 = t0+0.5
                
                rospy.sleep(0.5)

            cmd.linear.x = 0
            cmd.angular.z = 0
            vel_publisher.publish(cmd)
        
        cmd.linear.x = -0.01
        vel_publisher.publish(cmd)
        
        return 'check_again'

    if aruco_pos_z > 0.415:
        cmd.linear.x = 0
        cmd.angular.z = 0
        vel_publisher.publish(cmd)

        rospy.sleep(5)

        return 'finish'

def undocking():

    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    rate = rospy.Rate(10)

    aruco_data = rospy.wait_for_message('/aruco_single/pose',PoseStamped)
    orientation_q = aruco_data.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    aruco_pitch_d = round(pitch*180/math.pi,5)
    aruco_pos_x = round(aruco_data.pose.position.x,5) 
    aruco_pos_z = round(aruco_data.pose.position.z,5)
    print(aruco_pitch_d,aruco_pos_x,aruco_pos_z)

    rotate_angle = 0
    t0 = 1

    if aruco_pos_z < 0.415  :

        if aruco_pitch_d > PITCH_D_TOLERANCE :
            # print('stage2 turn right')
            while (rotate_angle < abs(aruco_pitch_d)):
                cmd.linear.x = 0
                cmd.angular.z = - ROTATION_SPEED
                vel_publisher.publish(cmd)
                # Calculate current angle
                rotate_angle = ROTATION_SPEED* (t0)*180/math.pi
                t0 = t0+0.5
                
                rospy.sleep(0.5)
            
        elif aruco_pitch_d < -PITCH_D_TOLERANCE :
            # print('stage2 turn left')
            while (rotate_angle < abs(aruco_pitch_d)):
                cmd.linear.x = 0
                cmd.angular.z = ROTATION_SPEED
                vel_publisher.publish(cmd)
                # Calculate current angle
                rotate_angle = ROTATION_SPEED* (t0)*180/math.pi
                t0 = t0+0.5
                
                rospy.sleep(0.5)

        cmd.angular.z= 0
        cmd.linear.x = -0.15
        vel_publisher.publish(cmd)
        
        return 'check_again'

    if aruco_pos_z > 0.415:

        cmd.linear.x = 0
        cmd.angular.z = 0
        vel_publisher.publish(cmd)
        print('finish')
        return 'finish_undocking'
 