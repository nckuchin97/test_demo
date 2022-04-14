#!/usr/bin/env python

import rospy
import json
import math

'''
Update robot status
'''

# read file

CPJsonFile = 'src/pura_cloud/scripts/arche_solution/database.json'

def update_robot_status(status):

    with open(CPJsonFile, "r+") as DataCollectorFile:
        data_collector = json.load(DataCollectorFile)
        data_collector["robot_status"] = str(status)
        DataCollectorFile.seek(0)
        json.dump(data_collector,DataCollectorFile)
        DataCollectorFile.truncate()