#!/usr/bin/env python

import rospy
import sys

import time

import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

import EKF_localization
u_init=EKF_localization.set_u_init(-0.0138,-7.835,-1.017)
ekf=EKF_localization.EKF_localization(u_init)


def cb_array(data):
    a,b,c,d=ekf.update_landmark(EKF_localization.list_2_landmark_Z(data.data))
    if not a==-1:
        l=[a,b,c[0][0],c[1][0],c[2][0],d[0][0],d[1][0],d[2][0]]
        m=Float64MultiArray(data=l)
        ekf_out_landmark_z.publish(m)
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
    ekf_out_sigma=rospy.Publisher("landmark_sigma",Twist,queue_size=1)
    ekf_out_landmark_z=rospy.Publisher("landmark_z",Float64MultiArray,queue_size=1)
    rate=rospy.Rate(10)

    
    while not rospy.is_shutdown():
        ekf.prediction(v,omg)

        ekf_out_msg=Twist()
        ekf_out_msg.linear.x=ekf.u[0][0]
        ekf_out_msg.linear.y=ekf.u[1][0]
        ekf_out_msg.angular.z=ekf.u[2][0]
        ekf_out.publish(ekf_out_msg)
        ekf_out_sigma_msg=Twist()
        ekf_out_sigma_msg.linear.x=ekf.sigma[0][0]
        ekf_out_sigma_msg.linear.y=ekf.sigma[1][1]
        ekf_out_sigma_msg.angular.z=ekf.sigma[2][2]
        ekf_out_sigma.publish(ekf_out_sigma_msg)



        rate.sleep()


