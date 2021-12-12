#!/usr/bin/env python


import rospy
import sys

import time
import numpy as np
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import depth2map
import EKF_localization 
import TREEDATA

u_init=EKF_localization.set_u_init(2767701,-352869,1.5)  # 1900 in utm

ekf=EKF_localization.EKF_localization(u_init)
TREE_DATA=TREEDATA.TREE_DATA
ekf.tree_data=TREE_DATA
use_landmark=-1


ARRAY_LAY1=20
ARRAY_LAY2=40
def cb_array(data):
    global use_landmark
    d=list(data.data)
    n=int(len(d)/ARRAY_LAY1)
    d_out=[0 for i in range(n*ARRAY_LAY2)]
        

    dis=0
    n_in=0
    #     for i in range(min(n,1)):
    for i in range(n):

        if int(rospy.get_param('XZ_MODE')):
            j,max_j,Z,z_hat,delta_z=ekf.update_landmark_xz(EKF_localization.list_2_landmark_xz_Z_together(data.data,i))
        else:
            if n>=2 and (d[i*ARRAY_LAY1+11]>-1):
                j,max_j,Z,z_hat,delta_z=ekf.update_landmark_know_cor(EKF_localization.list_2_landmark_Z_together(data.data,i),d[i*ARRAY_LAY1+11]+1)
                print(d[i*ARRAY_LAY1+11]+1)
            else:
                # j,max_j,Z,z_hat,delta_z=ekf.update_landmark_know_cor(EKF_localization.list_2_landmark_Z_together(data.data,i),corr_list[i])
                j,max_j,Z,z_hat,delta_z=ekf.update_landmark(EKF_localization.list_2_landmark_Z_together(data.data,i))

        # if j>0 and (j in[13,15,16,17,18,19,21,23]):
        if j>0:
            use_landmark=j

            new_car_x=ekf.u[0][0]
            new_car_y=ekf.u[1][0]
            new_car_th=ekf.u[2][0]
            # ###########use use d th r
            # d_list=[Z[0][0]*1000]
            # th_list=[Z[1][0]]
            # r_list=[Z[2][0]]
            # centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
            # new_Z=depth2map.fromCar2World(centre_x_list,centre_z_list,new_car_x*1000,new_car_y*1000,new_car_th) # in UTM in mm
            # dd=EKF_localization.get_dis(new_Z[0,0]*0.001,new_Z[1,0]*(-0.001),TREE_DATA[int(j-1)][0],TREE_DATA[int(j-1)][1])

            # ###########use x z r
            new_Z=depth2map.fromCar2World([Z[0][0]*1000],[Z[1][0]*1000],new_car_x*1000,new_car_y*1000,new_car_th) # in UTM in mm
            dd=EKF_localization.get_dis(new_Z[0,0]*0.001,new_Z[1,0]*(-0.001),TREE_DATA[int(j-1)][0],TREE_DATA[int(j-1)][1])


            
            dis=dis+dd*dd
            n_in=n_in+1
            l=[j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0],delta_z[0][0],delta_z[1][0],delta_z[2][0],new_Z[0,0]*0.001,new_Z[1,0]*0.001,new_car_x,new_car_y,new_car_th]
            print("is tree!",j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0])
    
        else:
            l=[j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0],-1,-1,-1,-1,-1,-1,-1,-1]
            print("no tree!",j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0])
        d_out[ARRAY_LAY2*i:ARRAY_LAY2*i+11]=l
        d_out[ARRAY_LAY2*i+20:ARRAY_LAY2*i+20+ARRAY_LAY1]=d[ARRAY_LAY1*i:ARRAY_LAY1*(i+1)]
    

    # print("d_out---------------------")
    if n_in==0:
        mean_error=0
    else:
        mean_error=dis/n_in
    m=Float64MultiArray(data=[dis,np.sqrt(mean_error),n,n_in])
    ekf_out_landmark_error.publish(m)
    m=Float64MultiArray(data=d_out)
    ekf_out_landmark_z.publish(m) 

t0=time.time()

#=Auto first point=========================
x0,y0=0,0
#==========================================

#=for fixed point strat====================
# x0=2767671.53
# y0=-352851.478
#==========================================

x0_loc,y0_loc=0,0
def cb_pos(data):
    global x0_loc,y0_loc,x0,y0
    if x0==0 and y0==0:
       z=EKF_localization.Odom_2_position_Z(data,0,0)
       x0_loc=z[0][0]
       y0_loc=z[1][0]
    else:
        pass
        # ekf.update_positon(EKF_localization.Odom_2_position_Z(data,x0,y0))
        # ekf.update_positon(EKF_localization.Odom_2_position_Z(data,2767715.47,-352874.09))
    ekf.update_angle(EKF_localization.Odom_2_angle_Z(data))
v=0
omg=0
ekf.Qt[0][0]=10**(100)
ekf.Qt[1][1]=0.2*10**(100)
ekf.Qt[2][2]=10**(100)
ekf.max_j_th=2.0
def cb_cmd(data):
    global v,omg
    v=data.linear.x
    omg=data.angular.z
    # print("v= ",v,"  , omg= ",omg)
    # if abs(v)<(10**(-4)) and abs(omg)<(10**(-4)):
    #     ekf.Qt[0][0]=10**(-1)
    #     ekf.Qt[1][1]=0.2*10**(-1)
    #     ekf.Qt[2][2]=10**(-1)
    #     ekf.max_j_th=1

    # else:
    #     ekf.Qt[0][0]=10**(-1)
    #     ekf.Qt[1][1]=10**(-1)*0.2
    #     ekf.Qt[2][2]=10**(-1)
    #     ekf.max_j_th=1


    ekf.Qt[0][0]=10**(-2)
    ekf.Qt[1][1]=10**(-2)
    ekf.Qt[2][2]=10**(-1)
    ekf.max_j_th=0.45

def cb_gps(data):
    global x0_loc,y0_loc,x0,y0
    z=EKF_localization.gps_2_utm_Z(data)
    if time.time()-t0<100:
        ekf.update_gps_utm(z)
    if x0==0 and y0==0 and not(x0_loc==0 or y0_loc==0):
        x0=z[0][0]-x0_loc
        y0=z[1][0]-y0_loc
        print("x0="+str(x0)+" y0="+str(y0))
    gps_utm_out_msg=Twist()
    gps_utm_out_msg.linear.x=z[0][0]
    gps_utm_out_msg.linear.y=z[1][0]
    gps_utm_out.publish(gps_utm_out_msg)


if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("landmark_ekf", anonymous=True)
    rospy.Subscriber("/tree_data_together", Float64MultiArray,cb_array)
    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist,cb_cmd)
    # rospy.Subscriber("/my_filtered_map", Odometry, cb_pos)
    rospy.Subscriber("/outdoor_waypoint_nav/odometry/filtered_map", Odometry, cb_pos)
    # rospy.Subscriber("/navsat/fix", NavSatFix, cb_gps)
    rospy.Subscriber("/outdoor_waypoint_nav/gps/filtered", NavSatFix, cb_gps)
    gps_utm_out=rospy.Publisher("gps_utm",Twist,queue_size=1)
    ekf_out=rospy.Publisher("landmark",Twist,queue_size=1)
    ekf_out2=rospy.Publisher("landmark_odom",Odometry,queue_size=1)
    ekf_out3=rospy.Publisher("landmark_local",Twist,queue_size=1)
    ekf_out_sigma=rospy.Publisher("landmark_sigma",Twist,queue_size=1)
    ekf_out_landmark_z=rospy.Publisher("landmark_z",Float64MultiArray,queue_size=1)
    ekf_out_landmark_error=rospy.Publisher("landmark_error",Float64MultiArray,queue_size=1)
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

        if x0==0 and y0==0:
            pass
        else:
            ekf_out3_msg=Twist()
            ekf_out3_msg.linear.x=ekf.u[0][0]-x0
            ekf_out3_msg.linear.y=ekf.u[1][0]-y0
            ekf_out3_msg.angular.z=ekf.u[2][0]
            ekf_out3.publish(ekf_out3_msg)

        ekf_out_sigma_msg=Twist()
        ekf_out_sigma_msg.linear.x=ekf.sigma[0][0]
        ekf_out_sigma_msg.linear.y=ekf.sigma[1][1]
        ekf_out_sigma_msg.angular.z=ekf.sigma[2][2]
        ekf_out_sigma.publish(ekf_out_sigma_msg)

        use_landmark=-1

        rate.sleep()


