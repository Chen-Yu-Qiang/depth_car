#!/usr/bin/env python

import rospy
import sys

import time

import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray

import EKF_localization 
# u_init=EKF_localization.set_u_init(0.191,0.366,1.902) # 1726
# u_init=EKF_localization.set_u_init(2767716.07,-352874.54,1.902) # 1726 in utm
# u_init=EKF_localization.set_u_init(-0.01105,-7.835,-1.017)  # 1900
u_init=EKF_localization.set_u_init(2767707.86,-352870.92,-1.017)  # 1900 in utm

ekf=EKF_localization.EKF_localization(u_init)
use_landmark=-1

def cb_array(data):
    global use_landmark
    j,max_j,Z,z_hat,delta_z=ekf.update_landmark(EKF_localization.list_2_landmark_Z(data.data))
    if j>0:
        l=[j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0],delta_z[0][0],delta_z[1][0],delta_z[2][0]]
        use_landmark=j

        print("is tree!",j,max_j,Z[0][0],Z[1][0],z_hat[0][0],z_hat[1][0])
        m=Float64MultiArray(data=(l+list(data.data)))
        ekf_out_landmark_z.publish(m)   
    else:
        l=[j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0],-1,-1,-1]
        print("no tree!",j,max_j,Z[0][0],Z[1][0],z_hat[0][0],z_hat[1][0])
 

t0=time.time()
def cb_pos(data):
    if time.time()-t0<60:
        pass
        # ekf.update_positon(EKF_localization.Odom_2_position_Z(data))
    ekf.update_angle(EKF_localization.Odom_2_angle_Z(data))
v=0
omg=0
def cb_cmd(data):
    global v,omg
    v=data.linear.x
    omg=data.angular.z

def cb_gps(data):
    z=EKF_localization.gps_2_utm_Z(data)
    if time.time()-t0<100:
        ekf.update_gps_utm(z)

    gps_utm_out_msg=Twist()
    gps_utm_out_msg.linear.x=z[0][0]
    gps_utm_out_msg.linear.y=z[1][0]
    gps_utm_out.publish(gps_utm_out_msg)


if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("landmark_ekf", anonymous=True)
    rospy.Subscriber("/tree_data", Float64MultiArray,cb_array)
    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist,cb_cmd)
    rospy.Subscriber("/my_filtered_map", Odometry, cb_pos)
    rospy.Subscriber("/navsat/fix", NavSatFix, cb_gps)
    gps_utm_out=rospy.Publisher("gps_utm",Twist,queue_size=1)
    ekf_out=rospy.Publisher("landmark",Twist,queue_size=1)
    ekf_out2=rospy.Publisher("landmark_odom",Odometry,queue_size=1)
    ekf_out_sigma=rospy.Publisher("landmark_sigma",Twist,queue_size=1)
    ekf_out_landmark_z=rospy.Publisher("landmark_z",Float64MultiArray,queue_size=1)
    rate=rospy.Rate(10)

    
    while not rospy.is_shutdown():
        ekf.prediction(v,omg)

        ekf_out_msg=Twist()
        ekf_out_msg.linear.x=ekf.u[0][0]
        ekf_out_msg.linear.y=ekf.u[1][0]
        ekf_out_msg.linear.z=use_landmark
        ekf_out_msg.angular.z=ekf.u[2][0]
        ekf_out.publish(ekf_out_msg)

        ekf_out2_msg=Odometry()
        ekf_out2_msg.pose.pose.position.x=ekf.u[0][0]
        ekf_out2_msg.pose.pose.position.y=ekf.u[1][0]
        ang_0_2pi=((ekf.u[2][0]+3.14159) % (6.28318))-3.14159
        ang_q=tf.transformations.quaternion_from_euler(0,0,ang_0_2pi)
        ekf_out2_msg.pose.pose.orientation.z=ang_q[2]
        ekf_out2_msg.pose.pose.orientation.w=ang_q[3]
        ekf_out2.publish(ekf_out2_msg)

        ekf_out_sigma_msg=Twist()
        ekf_out_sigma_msg.linear.x=ekf.sigma[0][0]
        ekf_out_sigma_msg.linear.y=ekf.sigma[1][1]
        ekf_out_sigma_msg.angular.z=ekf.sigma[2][2]
        ekf_out_sigma.publish(ekf_out_sigma_msg)

        use_landmark=-1

        rate.sleep()


