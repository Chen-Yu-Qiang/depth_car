#!/usr/bin/env python

import rospy
import sys

import time
import cv2
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

import EKF_localization
u_init=EKF_localization.set_u_init(-0.0138,-7.835,-1.017)
ekf=EKF_localization.EKF_localization(u_init)


def cb_array(data):
    ekf.update_landmark(EKF_localization.list_2_landmark_Z(data.data))
def cb_pos(data):
    ekf.update_positon(EKF_localization.Odom_2_position_Z(data))
v=0
omg=0
def cb_cmd(data):
    global v,omg
    v=data.linear.x
    omg=data.angular.z

if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("landmark_ekf", anonymous=True)
    rospy.Subscriber("/tree_data2", Float64MultiArray,cb_array)
    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist,cb_cmd)
    rospy.Subscriber("/my_filtered_map", Odometry, cb_pos)
    ekf_out=rospy.Publisher("landmark",Twist,queue_size=1)
    rate=rospy.Rate(10)

    
    while not rospy.is_shutdown():
        ekf.prediction(v,omg)

        ekf_out_msg=Twist()
        ekf_out_msg.linear.x=ekf.u[0][0]
        ekf_out_msg.linear.y=ekf.u[1][0]
        ekf_out_msg.angular.z=ekf.u[2][0]
        ekf_out.publish(ekf_out_msg)


        rate.sleep()


