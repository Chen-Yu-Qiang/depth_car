#!/usr/bin/wowpython
import os
import sys
from datetime import datetime
import rospy
from sensor_msgs.msg import NavSatFix
import follow_waypoints_lib
if sys.version[0]=='2':
    a=raw_input("Please enter the date-time folder of the waypoint: ~/202201")
elif sys.version[0]=='3':
    a=input("Please enter the date-time folder of the waypoint: ~/202201")
a=os.path.expanduser('~')+"/202201"+a
rospy.set_param("date_time_folder",a)

print("wait for gps")

os.system("roslaunch depth_car wow_waypoint.launch")