#!/usr/bin/wowpython


import rospy
import sys

import time
import numpy as np
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3Stamped
import depth2map
import EKF_localization 
import TREEDATA
from mapping_explorer.srv import InitOffset, InitOffsetResponse

print("[landmark_ekf_ros.py] Python version: "+str(sys.version))
rospy.init_node("landmark_ekf", anonymous=True)
print("[landmark_ekf_ros.py] wait for message /lm_ekf/gps/utm ")
gps_init=rospy.wait_for_message("/lm_ekf/gps/utm", Twist)
print("[landmark_ekf_ros.py] Get GPS init x = {} ,y = {}".format(gps_init.linear.x,gps_init.linear.y))


print("[landmark_ekf_ros.py] wait for service get_init_offset_in_map ")
if sys.version[0]=='3':
    rospy.wait_for_service('get_init_offset_in_map')
    try:
        get_init_offset_in_map = rospy.ServiceProxy('get_init_offset_in_map',InitOffset)
        resp = get_init_offset_in_map(int(rospy.get_param('Init_tree_num',default=0))) #identity: matched trunk index in the map
    except rospy.ServiceException as e:
        print("[landmark_ekf_ros.py] Service call failed: %s" % e)
else:
    resp=InitOffset()
    resp.offset_x=0
    resp.offset_y=0

print("[landmark_ekf_ros.py] Get GPS offset init x = {} ,y = {}".format(resp.offset_y,resp.offset_x*(-1.0)))


u_init=EKF_localization.set_u_init(gps_init.linear.x,gps_init.linear.y,1.5,resp.offset_y,resp.offset_x*(-1.0))

ekf=EKF_localization.EKF_localization(u_init)
TREE_DATA=TREEDATA.TREE_DATA
ekf.tree_data=TREE_DATA
use_landmark=-1


ARRAY_LAY1=20
ARRAY_LAY2=40
x0,y0=0,0
def cb_array(data):
    global use_landmark
    d=list(data.data)
    n=int(len(d)/ARRAY_LAY1)
    d_out=[0 for i in range(n*ARRAY_LAY2)]
        

    dis=0
    n_in=0
    #     for i in range(min(n,1)):
    for i in range(n):

        # tt1=time.time()
        if int(rospy.get_param('XZ_MODE'))==1:
            if n>=2 and (d[i*ARRAY_LAY1+11]>-1) and 0:
                j,max_j,Z,z_hat,delta_z=ekf.update_landmark_xz_know_cor(EKF_localization.list_2_landmark_xz_Z_together(data.data,i),d[i*ARRAY_LAY1+11]+1)
            else:
                j,max_j,Z,z_hat,delta_z=ekf.update_landmark_xz(EKF_localization.list_2_landmark_xz_Z_together(data.data,i))


        elif int(rospy.get_param('XZ_MODE'))==2:
            if d[i*ARRAY_LAY1+3]>6:
                j,max_j,Z,z_hat,delta_z=ekf.update_landmark(EKF_localization.list_2_landmark_Z_together(data.data,i))
            else:
                if n>=2 and (d[i*ARRAY_LAY1+11]>-1) and 0:
                    j,max_j,Z,z_hat,delta_z=ekf.update_landmark_xz_know_cor(EKF_localization.list_2_landmark_xz_Z_together(data.data,i),d[i*ARRAY_LAY1+11]+1)
                else:
                    j,max_j,Z,z_hat,delta_z=ekf.update_landmark_xz(EKF_localization.list_2_landmark_xz_Z_together(data.data,i))       
                    
                             
        else:
            if n>=2 and (d[i*ARRAY_LAY1+11]>-1):
                j,max_j,Z,z_hat,delta_z=ekf.update_landmark_know_cor(EKF_localization.list_2_landmark_Z_together(data.data,i),d[i*ARRAY_LAY1+11]+1)
            else:
                j,max_j,Z,z_hat,delta_z=ekf.update_landmark(EKF_localization.list_2_landmark_Z_together(data.data,i))

        # if j>0 and (j in[13,15,16,17,18,19,21,23]):
        # tt2=time.time()
        if j>0:
            use_landmark=j

            new_car_x=ekf.u[0][0]
            new_car_y=ekf.u[1][0]
            new_car_th=ekf.u[2][0]


            if int(rospy.get_param('XZ_MODE')):
                new_Z=depth2map.fromCar2World([Z[0][0]*1000],[Z[1][0]*1000],new_car_x*1000,new_car_y*1000,new_car_th) # in UTM in mm
                dd=EKF_localization.get_dis(new_Z[0,0]*0.001,new_Z[1,0]*(-0.001),TREE_DATA[int(j-1)][0],TREE_DATA[int(j-1)][1])
            else:
                d_list=[Z[0][0]*1000]
                th_list=[Z[1][0]]
                r_list=[Z[2][0]]
                centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
                new_Z=depth2map.fromCar2World(centre_x_list,centre_z_list,new_car_x*1000,new_car_y*1000,new_car_th) # in UTM in mm
                dd=EKF_localization.get_dis(new_Z[0,0]*0.001,new_Z[1,0]*(-0.001),TREE_DATA[int(j-1)][0],TREE_DATA[int(j-1)][1]) 


            
            dis=dis+dd*dd
            n_in=n_in+1
            l=[j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0],delta_z[0][0],delta_z[1][0],delta_z[2][0],new_Z[0,0]*0.001,new_Z[1,0]*0.001,new_car_x,new_car_y,new_car_th]
            # print("is tree!",j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0])
    
        else:
            l=[j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0],-1,-1,-1,-1,-1,-1,-1,-1]
            # print("no tree!",j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0])
        d_out[ARRAY_LAY2*i:ARRAY_LAY2*i+11]=l
        d_out[ARRAY_LAY2*i+20:ARRAY_LAY2*i+20+ARRAY_LAY1]=d[ARRAY_LAY1*i:ARRAY_LAY1*(i+1)]
        # print(time.time()-tt2,tt2-tt1)
    

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







x0_loc,y0_loc=0,0
x0y0_finish=0
def cb_pos(data):
    global x0,y0,t0
    if x0==0 and y0==0:
    #     x0_loc= data.pose.pose.position.x
    #     y0_loc= data.pose.pose.position.y
    #     x0y0_finish=1
        pass
    elif (time.time()-t0)<int(rospy.get_param("Use_GPS_time",default=10000)):
        z=EKF_localization.Odom_2_position_Z(data,x0,y0)
        ekf.update_positon(z)
        # ekf_out5_msg=Twist()
        # ekf_out5_msg.linear.x=z[0][0]-ekf.u[3][0]
        # ekf_out5_msg.linear.y=z[1][0]-ekf.u[4][0]
        # ekf_out5_msg.angular.z=z[2][0]
        # ekf_out5.publish(ekf_out5_msg)
        # print(time.time())
        # ekf_out6_msg=Twist()
        # ekf_out6_msg.linear.x=z[0][0]-ekf.u[3][0]-x0
        # ekf_out6_msg.linear.y=z[1][0]-ekf.u[4][0]-y0
        # ekf_out6_msg.angular.z=z[2][0]
        # ekf_out6.publish(ekf_out6_msg)

        # filtered_utm_msg=Twist()
        # filtered_utm_msg.linear.x=z[0][0]
        # filtered_utm_msg.linear.y=z[1][0]
        # filtered_utm_msg.angular.z=z[2][0]
        # # print(z)
        # filtered_utm.publish(filtered_utm_msg)
    
    ekf.update_angle(EKF_localization.Odom_2_angle_Z(data))
v=0
omg=0
ekf.Qt[0][0]=10**(-1)
ekf.Qt[1][1]=10**(-1)
ekf.Qt[2][2]=10**(-1)
ekf.max_j_th=0.8

def cb_cmd(data):
    global v,omg
    v=data.linear.x
    omg=data.angular.z


def cb_gps(data):
    global x0,y0,t0,x0y0_finish

    # z=EKF_localization.gps_2_utm_Z(data)
    z=EKF_localization.gps_utm_2_Z(data)
    # if x0==0 and y0==0 and x0y0_finish==1:
    #     x0=z[0][0]-x0_loc
    #     y0=z[1][0]-y0_loc
    #     print("!!!!x0=",x0,"!!!!y0=",y0)
    #     x0y0_finish=2

    if time.time()-t0<int(rospy.get_param("Use_GPS_time",default=10000)):
        # print("gps~~~~~~~~")
        ekf.update_gps_utm(z)


def cb_imu(data):
    ekf.update_angle(EKF_localization.Vector_2_angle_Z(data))


def cb_x0y0(data):
    global x0,y0
    x0=data.linear.x
    y0=data.linear.y



if __name__=="__main__":
    rospy.Subscriber("/tree/data/together", Float64MultiArray,cb_array, buff_size=2**20,queue_size=1)
    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist,cb_cmd, buff_size=2**20,queue_size=1)
    rospy.Subscriber("/outdoor_waypoint_nav/odometry/filtered_map", Odometry, cb_pos, buff_size=2**20,queue_size=1)
    # rospy.Subscriber("/lm_ekf/gps/utm", Twist, cb_gps, buff_size=2**20,queue_size=1)
    rospy.Subscriber("/lm_ekf/local_org/utm", Twist,cb_x0y0, buff_size=2**20,queue_size=1)
    # rospy.Subscriber("/imu_filter/rpy/filtered", Vector3Stamped, cb_imu, buff_size=2**20,queue_size=1)
    # if int(rospy.get_param("Enable_Fixed_Point_Strat",default=0))==0:
    # #     rospy.Subscriber("/lm_ekf/local_org/utm", Twist,cb_x0y0)
    #     x0=0
    #     y0=0
    # else:
    #     x0=2767671.53
    #     y0=-352851.478-1.0

    ekf_out=rospy.Publisher("/lm_ekf/raw/utm",Twist,queue_size=1)
    # ekf_out2=rospy.Publisher("/lm_ekf/gps_w_offset/utm_odom",Odometry,queue_size=1)
    ekf_out3=rospy.Publisher("/lm_ekf/raw/local",Twist,queue_size=1)
    # ekf_out5=rospy.Publisher("/lm_ekf/gps_w_offset/utm",Twist,queue_size=1)
    # ekf_out6=rospy.Publisher("/lm_ekf/gps_w_offset/local",Twist,queue_size=1)
    # filtered_utm=rospy.Publisher("/lm_ekf/filtered_map/utm",Twist,queue_size=1)
    ekf_out4=rospy.Publisher("/lm_ekf/offset",Twist,queue_size=1)
    ekf_out_sigma=rospy.Publisher("/lm_ekf/sigma",Twist,queue_size=1)
    ekf_out_landmark_z=rospy.Publisher("/lm_ekf/z",Float64MultiArray,queue_size=1)
    ekf_out_landmark_error=rospy.Publisher("/lm_ekf/error",Float64MultiArray,queue_size=1)
    rate=rospy.Rate(10)
    EKF_localization.DELTA_T=0.1

    
    while not rospy.is_shutdown():
        ekf.prediction(v,omg)

        ekf_out_msg=Twist()
        ekf_out_msg.linear.x=ekf.u[0][0]
        ekf_out_msg.linear.y=ekf.u[1][0]
        ekf_out_msg.linear.z=use_landmark
        ekf_out_msg.angular.z=ekf.u[2][0]
        ekf_out.publish(ekf_out_msg)

        # ekf_out2_msg=Odometry()
        # ekf_out2_msg.pose.pose.position.x=ekf.u[0][0]
        # ekf_out2_msg.pose.pose.position.y=ekf.u[1][0]
        # ang_0_2pi=((ekf.u[2][0]+3.14159) % (6.28318))-3.14159
        # ang_q=tf.transformations.quaternion_from_euler(0,0,ang_0_2pi)
        # ekf_out2_msg.pose.pose.orientation.z=ang_q[2]
        # ekf_out2_msg.pose.pose.orientation.w=ang_q[3]
        # ekf_out2.publish(ekf_out2_msg)

        if x0==0 and y0==0:
            pass
        else:
            ekf_out3_msg=Twist()
            ekf_out3_msg.linear.x=ekf.u[0][0]-x0
            ekf_out3_msg.linear.y=ekf.u[1][0]-y0
            ekf_out3_msg.angular.z=ekf.u[2][0]
            ekf_out3.publish(ekf_out3_msg)


        ekf_out4_msg=Twist()
        ekf_out4_msg.linear.x=ekf.u[3][0]
        ekf_out4_msg.linear.y=ekf.u[4][0]
        ekf_out4.publish(ekf_out4_msg)


        ekf_out_sigma_msg=Twist()
        ekf_out_sigma_msg.linear.x=ekf.sigma[0][0]
        ekf_out_sigma_msg.linear.y=ekf.sigma[1][1]
        ekf_out_sigma_msg.angular.z=ekf.sigma[2][2]
        ekf_out_sigma.publish(ekf_out_sigma_msg)

        use_landmark=-1

        rate.sleep()


