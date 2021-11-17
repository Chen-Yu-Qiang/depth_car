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
# u_init=EKF_localization.set_u_init(2767707.86,-352870.92,-1.017)  # 1900 in utm
u_init=EKF_localization.set_u_init(2767701,-352869,1.5)  # 1900 in utm

ekf=EKF_localization.EKF_localization(u_init)
ekf.tree_data=[[2767686.6016908274,-352842.33876166824,0.4],[2767679.3516908274,-352852.6887616683,0.3],[2767692.1516908277,-352870.48876166827,0.25],[2767691.5516908276,-352872.78876166826,0.25],[2767693.1516908277,-352874.6887616683,0.4],[2767691.6016908274,-352874.9387616683,0.15],[2767693.7516908273,-352878.08876166824,0.35],[2767685.1016908274,-352891.08876166824,0.3],[2767705.3016908276,-352847.9387616683,0.8],[2767726.7516908273,-352849.03876166826,0.3],[2767714.5016908273,-352850.48876166827,0.2],[2767715.8016908276,-352854.28876166826,0.2],[2767708.9016908277,-352855.38876166823,0.25],[2767713.5016908273,-352857.23876166827,0.35],[2767713.6016908274,-352859.4387616683,0.25],[2767725.4516908275,-352862.28876166826,0.15],[2767729.522274709,-352863.2814219437,0.2],[2767711.022274709,-352863.5314219437,0.2],[2767730.122274709,-352870.4314219437,0.4],[2767730.222274709,-352878.5314219437,0.2],[2767702.022274709,-352888.08142194373,0.3],[2767716.822274709,-352889.6314219437,0.8],[2767726.122274709,-352889.58142194373,0.3],[2767719.072274709,-352889.8814219437,0.35],[2767727.572274709,-352890.33142194373,0.4],[2767725.9222747087,-352891.1314219437,0.35],[2767707.822274709,-352891.7314219437,0.65],[2767727.872274709,-352892.3564219437,0.225],[2767727.272274709,-352899.1814219437,0.2],[2767707.9222747087,-352904.83142194373,0.15],[2767701.572274709,-352905.1814219437,0.3],[2767700.272274709,-352905.0314219437,0.15]]
use_landmark=-1

def cb_array(data):
    global use_landmark
    j,max_j,Z,z_hat,delta_z=ekf.update_landmark(EKF_localization.list_2_landmark_Z(data.data))
    if j>0:
        l=[j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0],delta_z[0][0],delta_z[1][0],delta_z[2][0]]
        use_landmark=j

        print("is tree!",j,max_j,Z[0][0],Z[1][0],z_hat[0][0],z_hat[1][0])
   
    else:
        l=[j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0],-1,-1,-1]
        print("no tree!",j,max_j,Z[0][0],Z[1][0],z_hat[0][0],z_hat[1][0])
    m=Float64MultiArray(data=(l+list(data.data)))
    ekf_out_landmark_z.publish(m) 

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
    # rospy.Subscriber("/outdoor_waypoint_nav/odometry/filtered_map", Odometry, cb_pos)
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


