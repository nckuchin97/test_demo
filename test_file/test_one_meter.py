#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# subscribe encoder data 

def test_one_meter():

    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    rospy.init_node('test_one_meter', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    distance =0

    while distance < 1.0:
        time = 1.0
        cmd.linear.x = 0.1 
        distance = distance +0.1
        vel_publisher.publish(cmd)
        rospy.sleep(time)
        print(distance)

    cmd.linear.x = 0 
    print(distance)
    vel_publisher.publish(cmd)

    rate.sleep()

if __name__ == '__main__':
    try:
        test_one_meter()
    except rospy.ROSInterruptException:
        pass