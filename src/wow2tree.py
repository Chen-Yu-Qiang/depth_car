#!/usr/bin/wowpython

from numpy.lib.type_check import imag
import rospy
import sys
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
import tf
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray,MultiArrayDimension
import depth2map
import numpy as np
from mapping_explorer.msg import Trunkset, Trunkinfo

car_x=0
car_y=0
car_theta=0
map=depth2map.cerate_map()

car1=depth2map.trj()
AA=0
ARRAY_LAY1=20
image=np.zeros((480,640))
def list2ROSmsg_dthr_with_cor(d_list,th_list,r_list,car_x,car_y,car_theta,AA,puber,cor_list):
    centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
    NnW=depth2map.fromCar2World(centre_x_list,centre_z_list,car_x*1000,car_y*1000,car_theta)
    a=[0 for i in range(len(centre_x_list)*ARRAY_LAY1)]
    for i in range(len(centre_x_list)):
        a[i*ARRAY_LAY1]=centre_x_list[i]*0.001
        a[i*ARRAY_LAY1+1]=centre_z_list[i]*0.001
        a[i*ARRAY_LAY1+2]=radius_r_list[i]*0.001
        a[i*ARRAY_LAY1+3]=d_list[i]*0.001
        a[i*ARRAY_LAY1+4]=th_list[i]
        # a[i*ARRAY_LAY1+5]=car_x
        # a[i*ARRAY_LAY1+6]=car_y
        # a[i*ARRAY_LAY1+7]=car_theta
        a[i*ARRAY_LAY1+8]=AA
        a[i*ARRAY_LAY1+9]=NnW[0,i]
        a[i*ARRAY_LAY1+10]=NnW[1,i]
        a[i*ARRAY_LAY1+11]=cor_list[i]
    b=Float64MultiArray(data=a)
    
    b.layout.dim=[MultiArrayDimension()]
    b.layout.dim[0].stride=ARRAY_LAY1
    b.layout.dim[0].size=ARRAY_LAY1

    b.layout.dim[0].label="A_Tree"

    puber.publish(b)

def list2ROSmsg_dthr_each_with_cor(d_list,th_list,r_list,car_x,car_y,car_theta,AA,puber,cor_list):
    centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
    NnW=depth2map.fromCar2World(centre_x_list,centre_z_list,car_x*1000,car_y*1000,car_theta)

    for i in range(len(centre_x_list)):
        a=[0 for j in range(ARRAY_LAY1)]
        a[0]=centre_x_list[i]*0.001
        a[1]=centre_z_list[i]*0.001
        a[2]=radius_r_list[i]*0.001
        a[3]=d_list[i]*0.001
        a[4]=th_list[i]
        # a[5]=car_x
        # a[6]=car_y
        # a[7]=car_theta
        a[8]=AA
        a[9]=NnW[0,i]*0.001
        a[10]=NnW[1,i]*0.001
        a[11]=cor_list[i]

        
        b=Float64MultiArray(data=a)
        
        b.layout.dim=[MultiArrayDimension()]
        b.layout.dim[0].stride=ARRAY_LAY1
        b.layout.dim[0].size=ARRAY_LAY1

        b.layout.dim[0].label="A_Tree_each"
        puber.publish(b)


def move30cm(d,th):
    move_dis=300
    th_abs=abs(th)
    d2=np.sqrt(d*d+move_dis*move_dis-2.0*move_dis*d*np.cos(np.pi-th_abs))
    th2=np.arcsin(d/d2*np.sin(np.pi-th_abs))
    if th>0:
        return d2,th2
    else:
        return d2,-th2

def cbTrunkset(data):
    global car_x,car_y,car_theta,AA,image_org

    d_list=[]
    r_list=[]
    th_list=[]
    cor_list=[]
    org_in_img_x=320
    org_in_img_y=480
    # image=image_org.copy()
    n=len(data.aframe)
    for i in range(n):
        inAframe = data.aframe[i]
        distance = inAframe.d*1000.0
        theta = inAframe.t
        radius = inAframe.r*1000.0
        if n<2:
            cor=-100
        else:
            cor = int(data.match[i])

        distance,theta=move30cm(distance,theta)
        d_list.append(distance)
        r_list.append(radius)
        th_list.append(theta)
        cor_list.append(cor)

    #     cv2.line(image,(org_in_img_x,org_in_img_y) , (int(org_in_img_x-np.sin(theta)*distance/100), int(org_in_img_y-np.cos(theta)*distance/100)),30000, 5)
    #     cv2.circle(image, (int(org_in_img_x-np.sin(theta)*distance/100), int(org_in_img_y-np.cos(theta)*distance/100)), int(radius*0.01),60000, -1)
    #     if not cor==-100:
    #         cv2.putText(image,str(int(cor+1)),((int(org_in_img_x-np.sin(theta)*(distance+100)/100), int(org_in_img_y-np.cos(theta)*(distance+100)/100))), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    # print(cor_list)
    # t=time.time()

    # for i in range(len(d_list)):
    #     cv2.putText(image,str(int(d_list[i]))+","+str(int((th_list[i])*57.3))+","+str(int(r_list[i]))+","+str(int(cor_list[i]+1)),(0,400+i*30), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)

    if len(d_list)>=0:
        list2ROSmsg_dthr_with_cor(d_list,th_list,r_list,car_x,car_y,car_theta,AA,tree_data2_together_pub,cor_list)
        list2ROSmsg_dthr_each_with_cor(d_list,th_list,r_list,car_x,car_y,car_theta,AA,tree_data2_each_pub,cor_list)
    # map=depth2map.circle_to_world(map,centre_x_list,centre_z_list,radius_r_list,car_x*1000,car_y*1000,car_theta)

    # cv2.putText(image,str(int(car_x*1000)),(0,30), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    # cv2.putText(image,str(int(car_y*1000)),(0,60), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    # cv2.putText(image,str(int((car_theta)*57.3)),(0,90), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)

    # cv2.imshow("depth",image)
    # cv2.waitKey(1)

    
    AA+=1


def cbDepth(data):
    global image_org
    bridge = CvBridge()
    image_org = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def cbOdom(msg):
    global car_x,car_y,car_theta
    car_x=msg.pose.pose.position.x
    car_y=msg.pose.pose.position.y
    z=tf.transformations.euler_from_quaternion([0,0,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    car_theta=z[2]
    # print(car_x,car_y,car_theta*57.3)


if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("depth_to_tree", anonymous=True)
    rospy.Subscriber("/wow/trunk_info", Trunkset, cbTrunkset,queue_size=1, buff_size=2**24)
    # rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth,queue_size=1, buff_size=2**24)
        
    tree_data2_each_pub=rospy.Publisher("/tree_data2_each", Float64MultiArray,queue_size=1)
    tree_data2_together_pub=rospy.Publisher("/tree_data2_together", Float64MultiArray,queue_size=1)


    subOdom = rospy.Subscriber("/outdoor_waypoint_nav/odometry/filtered_map", Odometry, cbOdom)
    rospy.spin()



    

    
