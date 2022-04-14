#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray
import json

def testing():
    pub = rospy.Publisher('component_management', Int16MultiArray, queue_size=10)
    msg_example = Int16MultiArray()
    msg_example.layout.dim = []
    msg_example.layout.data_offset = 0
    msg_example.data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    # gold yellow : charging
    msg_example.data[0] = 1
    msg_example.data[1] = 255
    msg_example.data[2] = 215
    msg_example.data[3] = 0

    # Light Green : fully charged
    msg_example.data[0] = 1
    msg_example.data[1] = 144
    msg_example.data[2] = 238
    msg_example.data[3] = 144

    # Ghost white : navigation
    msg_example.data[0] = 1
    msg_example.data[1] =  248
    msg_example.data[2] = 248
    msg_example.data[3] = 255

    # Indian Red : error
    msg_example.data[0] = 1
    msg_example.data[1] = 205
    msg_example.data[2] = 92
    msg_example.data[3] = 92

    # steelblue: floor sanitation
    msg_example.data[0] = 1
    msg_example.data[1] = 70
    msg_example.data[2] = 130
    msg_example.data[3] = 180

    # aqua blue : surface sanitation
    msg_example.data[0] = 1
    msg_example.data[1] = 0
    msg_example.data[2] = 255
    msg_example.data[3] = 255

    # Plum Purple : air sanitation
    msg_example.data[0] = 1
    msg_example.data[1] = 221
    msg_example.data[2] = 160
    msg_example.data[3] = 221

    # Pump
    msg_example.data[4] = 1

    # Piezo
    msg_example.data [6]= 1

    # Fan
    msg_example.data[8] = 1
    msg_example.data[9] = 80

    # RGBLed_Enable_Order = msg_example.data[0]
	# RGBLed_Order_R = msg_example.data[1]
	# RGBLed_Order_G = msg_example.data[2]
	# RGBLed_Order_B = msg_example.data[3]
	# Pump_Enable_Order = msg_example.data[4];
	# Pump_PWM_Order = msg_example.data[5];
	# Piezo_Enable_Order =msg_example.data[6];
	# Piezo_PWM_Order = msg_example.data[7];
	# Fan_Enable_Order =msg_example.data[8];
	# Fan_PWM_Order =msg_example.data[9];
	# UVA_1_Enable_Order = msg_example.data[10];
	# UVA_1_PWM_Order = msg_example.data[11];
	# UVA_2_Enable_Order = msg_example.data[12];
	# UVA_2_PWM_Order = msg_example.data[13];
	# Motor_Enable_Order =msg_example.data[14];

    rospy.init_node('test_component', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo(msg_example)
    pub.publish(msg_example)
    rate.sleep()

if __name__ == '__main__':
    try:
        testing()
    except rospy.ROSInterruptException:
        pass