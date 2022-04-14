#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from rosgraph_msgs.msg import Log
import json

while True:
  jsonFile = 'src/pura_execution/scripts/behavior_engine/rosout_record.json'
  listObj =[]

  # def callback(data):
  #     if "/listener_11" in data.name:
  #         pass
  #     else: 
          
  #         with open(jsonFile, "r+") as DataCollectorFile:
  #             listObj = json.load(DataCollectorFile)

  #             y = {
  #                 "topic_name": data.name,
  #                 "topic_name": data.msg
  #             }

  #             listObj.append(y)
              

  #         json.dump(listObj,jsonFile,indent =4)

      
  # def listener():

  #     rospy.init_node('listener_11', anonymous=True)

  #     rospy.Subscriber("/rosout", Log, callback)

  #     rospy.spin()

  # if __name__ == '__main__':
  #     listener()

  # Check if file exists

  
  # Read JSON file
  rospy.init_node('listener_11', anonymous=True)
  with open(jsonFile) as fp:
    listObj = json.load(fp)
  
  # Verify existing list
  print(listObj)

  print(type(listObj))
  data = rospy.wait_for_message("/rosout", Log)
  
  listObj.append({
    "topic_name": data.name,
    "topic_msg": data.msg
  })
  
  # Verify updated list
  print(listObj)
  
  with open(jsonFile, 'w') as json_file:
      json.dump(listObj, json_file, 
                          indent=6  
                          )
  
  print('Successfully appended to the JSON file')
