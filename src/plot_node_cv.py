#!/usr/bin/wowpython

from numpy.lib.type_check import imag

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
import tf
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from mapping_explorer.msg import Trunkset, Trunkinfo
from std_msgs.msg import Float64MultiArray,MultiArrayDimension

import numpy as np

import random

from datetime import datetime

import rospy
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped
from mapping_explorer.msg import Trunkset, Trunkinfo, Correspondence

import TREEDATA
from collections import deque
'''
world
Y
^
|
|
o------> X


Numpy
o------> axis-2
|
|
V
axis-1


OpenCV
o------> axis-1
|
|
V
axis-2

'''

MAP_RESOLUTION=0.05  # m pre pixel
GRID_SIZE=1 # m
GRID_COLOR=(0, 0, 255)
TREE_COLOR=(102,255,178)
Z_COLOR=(102,178,255)
Z_HAT_COLOR=(0,76,153)


X_MIN=1.0*TREEDATA.X_MIN
X_MAX=1.0*TREEDATA.X_MAX
Y_MIN=1.0*TREEDATA.Y_MIN
Y_MAX=1.0*TREEDATA.Y_MAX

def CVpix2m(u,v):
    x=1.0*u*MAP_RESOLUTION+1.0*X_MIN
    y=1.0*Y_MAX-1.0*v*MAP_RESOLUTION
    return x,y

def NPpix2m(u,v):
    x=1.0*v*MAP_RESOLUTION+1.0*X_MIN
    y=1.0*Y_MAX-1.0*u*MAP_RESOLUTION
    return x,y

def m2NPpix(x,y):
    u=1.0*(Y_MAX-y)/MAP_RESOLUTION
    v=1.0*(x-X_MIN)/MAP_RESOLUTION
    return int(u),int(v)

def m2CVpix(x,y):
    v=1.0*(Y_MAX-y)/MAP_RESOLUTION
    u=1.0*(x-X_MIN)/MAP_RESOLUTION
    return int(u),int(v)
def new_map():
    x_len=int((X_MAX-X_MIN)/MAP_RESOLUTION)
    y_len=int((Y_MAX-Y_MIN)/MAP_RESOLUTION)
    m=np.zeros((y_len,x_len,3),dtype=np.uint8)    
    return m
def draw_grid(m):
    x_grid_len=int((X_MAX-X_MIN)/GRID_SIZE)
    y_grid_len=int((Y_MAX-Y_MIN)/GRID_SIZE)
    for i in range(1,x_grid_len+1):
        u1,v1=m2CVpix(X_MIN+i*GRID_SIZE,Y_MAX)
        u2,v2=m2CVpix(X_MIN+i*GRID_SIZE,Y_MIN+GRID_SIZE)
        m=cv2.line(m,(u1,v1),(u2,v2),GRID_COLOR)
        if i>0:
            u2,v2=m2CVpix(X_MIN+i*GRID_SIZE,Y_MIN)
            m=cv2.putText(m,str(int(X_MIN+i*GRID_SIZE)),(u2,v2-30),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.5, color=(255, 255, 255))

    for i in range(y_grid_len+1):
        u1,v1=m2CVpix(X_MIN+GRID_SIZE,Y_MIN+i*GRID_SIZE)
        u2,v2=m2CVpix(X_MAX,Y_MIN+i*GRID_SIZE)
        cv2.line(m,(u1,v1),(u2,v2),GRID_COLOR)
        if i>0:
            u1,v1=m2CVpix(X_MIN,Y_MIN+i*GRID_SIZE)
            m=cv2.putText(m,str(int(Y_MIN+i*GRID_SIZE)),(u1+15,v1),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.5, color=(255, 255, 255))

    return m

def draw_tree(m):
    for i in TREEDATA.TREE_DATA:
        u,v=m2CVpix(i[0],i[1])
        r=int(i[2]/MAP_RESOLUTION)
        m=cv2.circle(m,(u,v),r,TREE_COLOR,-1)
    return m

class car:
    def __init__(self):
        self.trj=new_map()
        self.trj_color=(255,255,255)
        self.car_color=(255,0,0)
        self.x=0
        self.y=0
        self.th=0
        self.car_l=5

    def new_trj_point(self,x,y):
        self.x=x
        self.y=y
        u,v=m2NPpix(x,y)
        self.trj[u][v]=self.trj_color

    def new_trj_line(self,x,y):
        u1,v1=m2CVpix(x,y)
        u2,v2=m2CVpix(self.x,self.y)
        self.trj=cv2.line(self.trj,(u1,v1),(u2,v2),self.trj_color)
        self.x=x
        self.y=y

    def draw_pose(self,m,x,y,th):
        u1,v1=m2CVpix(x,y)
        u2,v2=m2CVpix(x+np.cos(th)*self.car_l,y+np.sin(th)*self.car_l)
        m=cv2.arrowedLine(m,(u1,v1),(u2,v2),color=self.car_color,thickness=2)
        # self.new_trj_point(x,y)
        self.new_trj_line(x,y)
        self.th=th
        return m
        
    def draw_sense(self,m,d_list,th_list):
        if d_list ==-1:
            return m
        for i in range(len(d_list)):
            if int(rospy.get_param('XZ_MODE')):
                x=th_list[i]
                z=d_list[i]
                u1,v1=m2CVpix(self.x+np.cos(self.th)*x-np.sin(self.th)*z,self.y+np.sin(self.th)*x+np.cos(self.th)*z)
                u2,v2=m2CVpix(self.x,self.y)
                m=cv2.line(m,(u1,v1),(u2,v2),Z_COLOR)
    
        return m

    def draw_sense_map(self,m,d_list,th_list):
        if d_list ==-1:
            return m
        for i in range(len(d_list)):
            th=th_list[i]
            d=d_list[i]
            u1,v1=m2CVpix(self.x+np.cos(self.th+th)*d,self.y+np.sin(self.th+th)*d)
            u2,v2=m2CVpix(self.x,self.y)
            m=cv2.line(m,(u1,v1),(u2,v2),Z_HAT_COLOR)
    
        return m

    def draw_trj(self,bg):
        m=bg+self.trj
        return m

class data_set:
    def __init__(self):
        self.d={}
    
    def get(self,n):
        if n in self.d:
            return self.d[n]
        else:
            return -1

    def set(self,n,v):
        self.d[n]=v
    
    def append(self,n,v):
        if (n in self.d) and (type(self.d[n]) is list):
            self.d[n].append(v)
        else:
            self.d[n]=[v]

ARRAY_LAY2=40
ds=data_set()

def cb_landmark_z(data):
    d=list(data.data)
    n=int(len(d)/ARRAY_LAY2)


    ds.set("z_d",[])
    ds.set("z_th",[])
    ds.set("z_r",[])
    ds.set("z_hat_d",[])
    ds.set("z_hat_th",[])
    ds.set("z_hat_r",[])
    ds.set("z_hat_index",[])
    ds.set("x_pro",0)
    ds.set("y_pro",0)
    ds.set("th_pro",0)
    ds.set("z_time",time.time())




    # print("d:",d)
    for i in range(n):
        if d[ARRAY_LAY2*i]>0 and i<10:
            ds.append("z_d",d[ARRAY_LAY2*i+2])
            ds.append("z_th",d[ARRAY_LAY2*i+3])
            ds.append("z_r",d[ARRAY_LAY2*i+4])
            ds.append("z_hat_d",d[ARRAY_LAY2*i+5])
            ds.append("z_hat_th",d[ARRAY_LAY2*i+6])
            ds.append("z_hat_r",d[ARRAY_LAY2*i+7])
            ds.set("x_pro",d[ARRAY_LAY2*i+13])
            ds.set("y_pro",d[ARRAY_LAY2*i+14])
            ds.append("z_hat_index",int(d[ARRAY_LAY2*i]-1))

    for i in range(n):
        if d[ARRAY_LAY2*i]<0:
            ds.append("z_d",d[ARRAY_LAY2*i+2])
            ds.append("z_th",d[ARRAY_LAY2*i+3])
            ds.append("z_r",d[ARRAY_LAY2*i+4])




def cb_gps(data):
    ds.set("x_gps",data.linear.x)
    ds.set("y_gps",data.linear.y)
    ds.set("th_gps",data.angular.z)

def cb_lm(data):
    ds.set("x",data.linear.x)
    ds.set("y",data.linear.y)
    ds.set("th",data.angular.z)


def cbGoal(msg):
    ds.set("waypoint_utm_x",msg.pose.position.x)
    ds.set("waypoint_utm_y",msg.pose.position.y)

def cb_gps_offset(data):
    ds.set("x_gps_offset",data.linear.x)
    ds.set("y_gps_offset",data.linear.y)
    


if __name__ == '__main__':

    print("Python version: ",sys.version)
    rospy.init_node("plot_node_cv", anonymous=True)
    landmark_z_sub=rospy.Subscriber("/lm_ekf/z", Float64MultiArray,cb_landmark_z,buff_size=2**20,queue_size=1)
    gps_sub=rospy.Subscriber("/lm_ekf/gps/utm", Twist,cb_gps,buff_size=2**20,queue_size=1)
    lm_sub=rospy.Subscriber("/lm_ekf/raw/utm", Twist,cb_lm,queue_size=1, buff_size=2**20)
    gps_offset_sub=rospy.Subscriber("/lm_ekf/gps_w_offset/utm", Twist,cb_gps_offset,queue_size=1, buff_size=2**20)
    subGoal = rospy.Subscriber("/ctrl/wp/utm", PoseStamped, cbGoal,buff_size=2**20,queue_size=1)

    rate=rospy.Rate(0.1)

    map=new_map()
    map=draw_grid(map)
    map=draw_tree(map)


    c_gps=car()
    c_lm=car()
    c_lm2=car()
    while not rospy.is_shutdown():
        t=time.time()
        m=map.copy()

        x=ds.get("x")
        y=ds.get("y")
        th=ds.get("th")
        m=c_lm.draw_pose(m,x,y,th)

        x_gps=ds.get("x_gps")
        y_gps=ds.get("y_gps")
        m=c_gps.draw_pose(m,x_gps,y_gps,th)


        x_gps_offset=ds.get("x_gps_offset")
        y_gps_offset=ds.get("y_gps_offset")
        m=c_lm2.draw_pose(m,x_gps_offset,y_gps_offset,th)


        


        z_d=ds.get("z_d")
        z_th=ds.get("z_th")
        m=c_lm.draw_sense(m,z_d,z_th)


        
        # z_hat_d=ds.get("z_hat_d")
        # z_hat_th=ds.get("z_hat_th")
        # m=c_lm.draw_sense_map(m,z_hat_th,z_th)


        m=c_gps.draw_trj(m)
        m=c_lm.draw_trj(m)
        # m=c_lm2.draw_trj(m)

        cv2.imshow("aa",m)
        cv2.waitKey(1)

        time.sleep(0.1)
