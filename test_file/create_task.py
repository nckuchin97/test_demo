#!/usr/bin/env python
from ast import While
import rospy
from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import json

jsonFile = 'src/pura_cloud/scripts/arche_solution/create_task.json'
start_bool = True
listObj =[]
    

def task_generator():

    rospy.init_node('create_task_listener', anonymous=True)
    print("\n \n")
    print('Task Generator is launching ..... ')
    x=input("Input 'yes' to continue, 'no' to exit: ")
    while x =='yes' and start_bool == True:

        with open(jsonFile) as fp:
            listObj = json.load(fp)
            print(listObj)

        print("\n")

        waypoint_name =input("Enter Waypoint Name: ")
        print("Choose the waypoint on rviz by using 2D Pose Estimate")
        initialpose_data = rospy.wait_for_message('/initialpose',PoseWithCovarianceStamped)
        print("Waypoint is chosen")
        sanitation =input("Sanitation_Function : ")
        listObj.append({
            "Name": waypoint_name,
            "Position_X": initialpose_data.pose.pose.position.x,
            "Position_Y": initialpose_data.pose.pose.position.y,
            "Quaternion_Z": initialpose_data.pose.pose.orientation.z,
            "Quaternion_W": initialpose_data.pose.pose.orientation.w,
            "Sanitation_Function": sanitation

        })
        print(listObj)
        with open(jsonFile, 'w') as json_file:
            json.dump(listObj, json_file, 
                                indent=6  
                                )


        x=input("Continue? Yes/No : ")

    print('Task Generator is generated')
    rospy.spin()
    pass

if __name__ == '__main__':

    try:
        task_generator()
    except rospy.ROSInterruptException:
        pass
