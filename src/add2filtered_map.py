#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

mag_ang_data=Imu()
def cb_ang(data):
    global mag_ang_data
    mag_ang_data=data

def cb_map(data):
    global mag_ang_data
    out_msg=data
    out_msg.pose.pose.orientation=mag_ang_data.orientation
    pub.publish(out_msg)




rospy.init_node('add2filtered_map')

rospy.Subscriber("/EKF/mag_ang",Imu,cb_ang)
rospy.Subscriber("/outdoor_waypoint_nav/odometry/filtered_map",Odometry,cb_map)
pub=rospy.Publisher("/my_filtered_map",Odometry,queue_size=1)

rospy.spin()