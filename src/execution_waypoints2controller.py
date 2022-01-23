#!/usr/bin/wowpython
import os
import sys
from datetime import datetime
import rospy
from sensor_msgs.msg import NavSatFix


a=rospy.get_param("date_time_folder")
b=input("The datetime folder is: {} \nPlease press Enter to agree or any key to reset".format(a))
if not b=='':
    a=input("Please enter the date-time folder of the waypoint: ~/202201")
    a=os.path.expanduser('~')+"/202201"+a
    rospy.set_param("date_time_folder",a)


import TREEDATA
rospy.set_param("RADIUS_MAX",float(TREEDATA.R_MAX))
rospy.set_param("RADIUS_MIN",float(TREEDATA.R_MIN))

Init_tree_num=input("Please enter the trunk number to initialize the GPS offset")
rospy.set_param("Init_tree_num",Init_tree_num)


while 1:
    mode=input("1: Create positive boundaries \n2: Walking waypoints \nchoose a mode => ")
    if mode=="1":
        os.system("roslaunch depth_car wow_no_gen_scan.launch")
        break
    elif mode=="2":
        os.system("roslaunch depth_car wow_waypoint.launch")
        break
    else:
        print("I do not know what you're talking about")