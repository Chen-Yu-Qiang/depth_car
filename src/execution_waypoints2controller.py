#!/usr/bin/wowpython
import os
import sys
from datetime import datetime
import rospy
from sensor_msgs.msg import NavSatFix
import TREEDATA
if sys.version[0]=='2':
    a=raw_input("Please enter the date-time folder of the waypoint: ~/202201")
elif sys.version[0]=='3':
    a=input("Please enter the date-time folder of the waypoint: ~/202201")
a=os.path.expanduser('~')+"/202201"+a
rospy.set_param("date_time_folder",a)
rospy.set_param("RADIUS_MAX",TREEDATA.R_MAX)
rospy.set_param("RADIUS_MIN",TREEDATA.R_MIN)

Init_tree_num=input("Please enter the trunk number to initialize the GPS offset")
rospy.set_param("Init_tree_num",Init_tree_num)


while 1:
    mode=input("1: Create positive boundaries \n 2: Walking waypoints \n choose a mode => ")
    if mode=="1":
        os.system("roslaunch depth_car wow_no_gen_scan.launch.launch")
        break
    elif mode=="2":
        os.system("roslaunch depth_car wow_waypoint.launch")
        break
    else:
        print("I do not know what you're talking about")