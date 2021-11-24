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

u_init=EKF_localization.set_u_init(2767701,-352869,1.5)  # 1900 in utm

ekf=EKF_localization.EKF_localization(u_init)
ekf.tree_data=[[2767694.6000474878,-352852.16121769784,0.3],[2767687.550047488,-352862.78621769784,0.35],[2767694.800047488,-352880.1862176978,0.4],[2767700.5000474877,-352880.1362176978,0.2],[2767698.510047488,-352881.72621769784,0.2],[2767699.6500474876,-352882.8862176978,0.3],[2767697.483380821,-352883.4362176978,0.2],[2767701.3750474877,-352884.3862176978,0.15],[2767699.300047488,-352884.9362176978,0.45],[2767701.7500474877,-352888.03621769784,0.4],[2767697.266584974,-352889.88638060197,0.2],[2767697.1665849737,-352892.43638060195,0.1],[2767707.6516908277,-352848.88876166823,0.75],[2767730.1016908274,-352849.9387616683,0.15],[2767717.2516908273,-352851.1887616683,0.1],[2767721.1016908274,-352853.78876166826,0.25],[2767710.1016908274,-352856.08876166824,0.2],[2767718.1516908277,-352857.08876166824,0.25],[2767718.4016908277,-352859.48876166827,0.2],[2767729.5516908276,-352862.63876166823,0.1],[2767711.5516908276,-352863.48876166827,0.1],[2767729.522274709,-352863.2814219437,0.2],[2767711.022274709,-352863.5314219437,0.2],[2767730.122274709,-352870.4314219437,0.4],[2767730.222274709,-352878.5314219437,0.2],[2767702.022274709,-352888.08142194373,0.3],[2767716.822274709,-352889.6314219437,0.8],[2767726.122274709,-352889.58142194373,0.3],[2767719.072274709,-352889.8814219437,0.35],[2767727.572274709,-352890.33142194373,0.4],[2767725.9222747087,-352891.1314219437,0.35],[2767707.822274709,-352891.7314219437,0.65],[2767727.872274709,-352892.3564219437,0.225],[2767727.272274709,-352899.1814219437,0.2],[2767707.9222747087,-352904.83142194373,0.15],[2767701.572274709,-352905.1814219437,0.3],[2767664.1878069476,-352882.288491822,0.25],[2767671.9378069476,-352882.938491822,0.25],[2767658.4378069476,-352886.638491822,0.25],[2767662.1878069476,-352889.138491822,0.3],[2767661.3878069473,-352892.98849182203,0.65],[2767667.9878069474,-352893.73849182203,0.25],[2767671.3378069475,-352895.688491822,0.2],[2767664.2878069477,-352898.138491822,0.3],[2767666.0878069475,-352901.338491822,0.25],[2767649.3600058393,-352846.57829085743,0.65],[2767651.1100058393,-352850.27829085744,0.3],[2767651.6600058395,-352863.8782908574,0.25],[2767648.2600058396,-352869.9782908574,0.25]]
use_landmark=-1


ARRAY_LAY1=20
ARRAY_LAY2=40
def cb_array(data):
    global use_landmark
    d=list(data.data)
    n=int(len(d)/ARRAY_LAY1)
    d_out=[0 for i in range(n*ARRAY_LAY2)]
    corr_list=[]

    # if n==1:
    #     print("test case 1")
    #     j,_,_,_,_=ekf.update_landmark_sim(EKF_localization.list_2_landmark_Z_together(data.data,0))
    #     corr_list=[j]
    #     print("test case 1")
    #     print("n=1")
    # elif n==2:
    #     print("test case 1")
    #     j11,score11,_,_,_=ekf.update_landmark_sim(EKF_localization.list_2_landmark_Z_together(data.data,0))
    #     j12,score12,_,_,_=ekf.update_landmark_sim(EKF_localization.list_2_landmark_Z_together(data.data,1),{j11})

    #     print("test case 1")
    #     print("test case 2")

    #     j21,score21,_,_,_=ekf.update_landmark_sim(EKF_localization.list_2_landmark_Z_together(data.data,1))
    #     j22,score22,_,_,_=ekf.update_landmark_sim(EKF_localization.list_2_landmark_Z_together(data.data,0),{j21})
    
    #     print("test case 2")
    #     if (score11+score12)>=(score21+score22):
    #         corr_list=[j11,j12]
    #         print("n=2! case 1")
    #     else:
    #         corr_list=[j22,j21]
    #         print("n=2 case 2")
    # if n>2:
    #     print("n>2!!!!!!!!!!!!!!")
        


    for i in range(n):
        if n>=2:
            j,max_j,Z,z_hat,delta_z=ekf.update_landmark_know_cor(EKF_localization.list_2_landmark_Z_together(data.data,i),d[i*ARRAY_LAY1+11]+1)
            print(d[i*ARRAY_LAY1+11]+1)
        else:
            # j,max_j,Z,z_hat,delta_z=ekf.update_landmark_know_cor(EKF_localization.list_2_landmark_Z_together(data.data,i),corr_list[i])
            j,max_j,Z,z_hat,delta_z=ekf.update_landmark(EKF_localization.list_2_landmark_Z_together(data.data,i))

        if j>0:
            l=[j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0],delta_z[0][0],delta_z[1][0],delta_z[2][0]]
            use_landmark=j

            print("is tree!",j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0])
    
        else:
            l=[j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0],-1,-1,-1]
            print("no tree!",j,max_j,Z[0][0],Z[1][0],Z[2][0],z_hat[0][0],z_hat[1][0],z_hat[2][0])
        d_out[ARRAY_LAY2*i:ARRAY_LAY2*i+11]=l
        d_out[ARRAY_LAY2*i+20:ARRAY_LAY2*i+20+ARRAY_LAY1]=d
        
    m=Float64MultiArray(data=d_out)
    ekf_out_landmark_z.publish(m) 

t0=time.time()
x0,y0=0,0
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
def cb_cmd(data):
    global v,omg
    v=data.linear.x
    omg=data.angular.z
    if abs(v)<(10**(-4)) and abs(omg)<(10**(-4)):
        ekf.Qt[0][0]=10**(-2)
        ekf.Qt[1][1]=10**(-2)*0.2
        ekf.Qt[2][2]=10**(-2)
        ekf.max_j_th=2
    else:
        ekf.Qt[0][0]=10**(0)
        ekf.Qt[1][1]=10**(0)*0.2
        ekf.Qt[2][2]=10**(0)
        ekf.max_j_th=6


def cb_gps(data):
    global x0_loc,y0_loc,x0,y0
    z=EKF_localization.gps_2_utm_Z(data)
    if time.time()-t0<10:
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


