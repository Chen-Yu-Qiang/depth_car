#!/usr/bin/wowpython
import os
import sys
from datetime import datetime
import rospy
from sensor_msgs.msg import NavSatFix

# a=os.path.expanduser('~')+"/20211206_16-17-41"
# rospy.set_param("date_time_folder",a)


a=rospy.get_param("date_time_folder",default="0")

a=a.replace("ncslaber","ursrobot")
os.system("mkdir -p "+a+"/shapefiles/neg")

import json
import numpy as np
treedata_str=rospy.get_param("lm_map",default="0")
if treedata_str=="0":
    print("NO tree list in rosparma!!!!")
    raw_input("===================")
list_obj=json.loads(treedata_str)
np_obj=np.array(list_obj)
np.save(a+"/shapefiles/neg/center_all",np_obj)




if a=="0":
    a=raw_input("Please enter the date-time folder of the waypoint: ~/202201")
    a=os.path.expanduser('~')+"/202201"+a
    rospy.set_param("date_time_folder",a)
b=raw_input("The date time folder is: {} \nPlease press Enter to agree or any key to reset".format(a))
if not b=='':
    a=raw_input("Please enter the date-time folder of the waypoint: ~/202201")
    a=os.path.expanduser('~')+"/202201"+a
    rospy.set_param("date_time_folder",a)


import TREEDATA
rospy.set_param("RADIUS_MAX",float(TREEDATA.R_MAX))
rospy.set_param("RADIUS_MIN",float(TREEDATA.R_MIN))

Init_tree_num=raw_input("Please enter the trunk number to initialize the GPS offset=> ")
rospy.set_param("Init_tree_num",Init_tree_num)

print("GOGO WayPoints !")
# os.system("roslaunch depth_car wow_waypoint.launch")
os.system("roslaunch depth_car wow_no_gen_scan.launch")
