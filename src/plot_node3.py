#!/usr/bin/wowpython

import time

import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
from datetime import datetime
import numpy as np

import rospy
import sys
import time
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import MagneticField,NavSatFix
import threading


from collections import deque

class data_set:
    def __init__(self):
        self.d={}
        self.lock=threading.Lock()

    def get(self,n):
        if n in self.d:
            return self.d[n]
        else:
            return -1
    def get_last(self,n):
        if not (type(self.d[n]) is list):
            return self.get(n)
        if n in self.d and len(self.d[n])>0:
            return self.d[n][-1]
        else:
            return -1
    def set(self,n,v):
        self.d[n]=v
    
    def append(self,n,v):
        if (n in self.d) and (type(self.d[n]) is list):
            self.d[n].append(v)
        else:
            self.d[n]=[v]


class a_plot:
    def __init__(self):
        if sys.version[0]=='2':
            self.fig, ((self.ax1,self.ax2),(self.ax3,self.ax4)) = plt.subplots(2, 2,dpi=70,figsize=(10,10))
        elif sys.version[0]=='3':
            self.fig, ((self.ax1,self.ax2),(self.ax3,self.ax4)) = plt.subplots(2, 2,dpi=120,figsize=(10,10))
        

        self.ax2.set_aspect('equal')


        self.ax2.set_xlim(-0.0003,0.0001)
        self.ax2.set_ylim(-0.00024,0.00016)
        self.ax3.set_xlim(-30,0)
        self.ax3.set_ylim(-1,1)
        self.ax4.set_xlim(-30,0)
        self.ax4.set_ylim(-5,10)
        self.ax1.set_title("Information")
        self.ax2.set_title("Magnetometer")
        self.ax3.set_title("Velocity command")
        self.ax4.set_title("Error")

        self.ax1.set_axis_off()
        self.ax2.set_xlabel("X-component")
        self.ax2.set_ylabel("Y-component")
        self.ax3.set_xlabel("Time (from now) [s]")
        self.ax3.set_ylabel("[m/s] [deg/s]")
        self.ax4.set_xlabel("Time (from now) [s]")
        self.ax4.set_ylabel("[m]")

        plt.ion()
        


        self.mag_line=self.ax2.plot([],[],'-', color='b')[0] 
        self.mag_dot=self.ax2.plot([],[],'o', color='r', label='Current')[0] 
        self.mag_org_dot=self.ax2.plot([],[],'o', color='g', label='Fitting Center')[0] 
        self.cmd_omg_line=self.ax3.plot([],[],'-', color='salmon', label='Angular')[0] 
        self.cmd_v_line=self.ax3.plot([],[],'-', color='b', label='Linear')[0] 
        # self.sigma_x_line=self.ax4.plot([],[],'-', color='b', label='X')[0] 
        # self.sigma_y_line=self.ax4.plot([],[],'-', color='r', label='Y')[0] 
        # self.sigma_th_line=self.ax4.plot([],[],'-', color='g', label='Theta')[0] 
        self.error_line=[None for i in range(4)]
        error_line_name=["x(local)","y(local)","x(global)","y(global)"]
        error_line_color=["red","blue","salmon","cornflowerblue"]
        for i in range(4):
            self.error_line[i]=self.ax4.plot([],[],'-', color=error_line_color[i], label=error_line_name[i])[0] 

        self.corres=self.ax1.text(0,0,'',fontsize=14, color='k')
        self.ax2.legend()
        self.ax3.legend()
        self.ax4.legend()


        self.cmd_v_data=deque([], maxlen=250)
        self.cmd_omg_data=deque([], maxlen=250)
        # self.sigma_x_data=deque([], maxlen=250)
        # self.sigma_y_data=deque([], maxlen=250)
        # self.sigma_th_data=deque([], maxlen=250)

        self.error_data=[deque([], maxlen=250) for i in range(4)]

        self.t_data=deque([], maxlen=250)


        plt.pause(0.1)


    def save_fig(self):

        if sys.version[0]=='2':
            self.fig.savefig('/home/yuqiang/211125/' + datetime.now().strftime("%d-%m-%Y %H:%M:%S.%f")+'.png',)
        elif sys.version[0]=='3':
            self.fig.savefig('/home/ncslaber/110-1/211125_localTest/z_hat/' + datetime.now().strftime("%d-%m-%Y_%H:%M:%S.%f")+'.png',)

    def car_position(self,ds):


        ds.lock.acquire()
        gps_lat=ds.get("gps_lat")
        gps_lon=ds.get("gps_lon")
        gps_x=ds.get("gps_x")
        gps_y=ds.get("gps_y")
        gps_qual=ds.get("gps_qual")
        cmd_v=ds.get("cmd_v")
        cmd_omg=ds.get("cmd_omg")
        # sigma_x=ds.get("sigma_x")
        # sigma_y=ds.get("sigma_y")
        # sigma_th=ds.get("sigma_th")
        mag_x=ds.get("mag_x")
        mag_y=ds.get("mag_y")

        mag_x_last=ds.get_last("mag_x")
        mag_y_last=ds.get_last("mag_y")
        mag_x_org=ds.get("mag_org_x")
        mag_y_org=ds.get("mag_org_y")
        error_x_loc=ds.get("error_x_loc")
        error_y_loc=ds.get("error_y_loc")
        error_x_glb=ds.get("error_x_glb")
        error_y_glb=ds.get("error_y_glb")
        ds.lock.release()

        self.cmd_v_data.append(cmd_v)
        self.cmd_omg_data.append(cmd_omg)
        self.t_data.append(time.time())
        # self.sigma_x_data.append(sigma_x)
        # self.sigma_y_data.append(sigma_y)
        # self.sigma_th_data.append(sigma_th)
        ee=[error_x_loc,error_y_loc,error_x_glb,error_y_glb]
        for i in range(4):
            self.error_data[i].append(ee[i])

        self.corres.set_text(datetime.now().strftime(\
            "%Y/%m/%d %H:%M:%S.%f")+\
            "\n\n===GPS==="+\
            "\n lat (WD) = "+str(round(gps_lat,8))+" deg"+\
            "\n lon (JD) = "+str(round(gps_lon,8))+" deg"+\
            "\n UTM x ="+str(round(gps_y,2)*(-1.0))+" m"+\
            "\n UTM y ="+str(round(gps_x,2))+" m"+\
            "\n Quality = "+str(gps_qual)+\
            "\n\n===Command==="+\
            "\n v = "+str(round(cmd_v,5))+" m/s"+\
            "\n omega = "+str(round(cmd_omg,5))+" deg/s"+\
            "\n ")
        
        self.mag_dot.set_data(mag_x_last,mag_y_last)
        self.mag_org_dot.set_data(mag_x_org,mag_y_org)
        self.mag_line.set_data(mag_x,mag_y)

        
        t0=time.time()
        cmd_t0=[x-t0 for x in self.t_data]


        self.cmd_omg_line.set_data(cmd_t0,self.cmd_omg_data)
        self.cmd_v_line.set_data(cmd_t0,self.cmd_v_data)
        # self.sigma_x_line.set_data(cmd_t0,self.sigma_x_data)
        # self.sigma_y_line.set_data(cmd_t0,self.sigma_y_data)
        # self.sigma_th_line.set_data(cmd_t0,self.sigma_th_data)
        for i in range(4):
            self.error_line[i].set_data(cmd_t0,self.error_data[i])


        

ds=data_set()
ds.lock.acquire()
ds.set("mag_x",deque([], maxlen=2500))
ds.set("mag_y",deque([], maxlen=2500))
ds.lock.release()


def cb_mag(data):
    ds.lock.acquire()
    ds.append("mag_x",data.magnetic_field.x)
    ds.append("mag_y",data.magnetic_field.y)
    ds.lock.release()

def cb_gps(data):
    
    ds.lock.acquire()
    ds.set("gps_lat",data.latitude)
    ds.set("gps_lon",data.longitude)
    ds.lock.release()

def cb_gps_utm(data):
    
    ds.lock.acquire()
    ds.set("gps_x",data.linear.x)
    ds.set("gps_y",data.linear.y)
    ds.lock.release()


def cb_gps_qual(data):
    ds.lock.acquire()
    ds.set("gps_qual",data.data)
    ds.lock.release()

def cb_cmd(data):
    ds.lock.acquire()
    ds.set("cmd_v",data.linear.x)
    ds.set("cmd_omg",data.angular.z)
    ds.lock.release()


def cb_sigma(data):
    ds.lock.acquire()
    ds.set("sigma_x",data.linear.x)
    ds.set("sigma_y",data.linear.y)
    ds.set("sigma_th",data.angular.z)
    ds.lock.release()

def cb_error(data):
    ds.lock.acquire()
    ds.set("error_x_loc",data.linear.x)
    ds.set("error_x_loc",data.linear.y)
    ds.set("error_x_glb",data.angular.x)
    ds.set("error_y_glb",data.angular.y)
    ds.lock.release()

def cb_ekf_org(data):
    ds.lock.acquire()
    ds.set("mag_org_x",data.magnetic_field.x)
    ds.set("mag_org_y",data.magnetic_field.y)
    ds.lock.release()    

if __name__ == '__main__':

    print("Python version: ",sys.version)
    rospy.init_node("plot_node3", anonymous=True)
    rospy.Subscriber("/imu/mag", MagneticField,cb_mag,queue_size=1)
    rospy.Subscriber("/outdoor_waypoint_nav/gps/filtered", NavSatFix,cb_gps,queue_size=1)
    rospy.Subscriber("gps_utm", Twist,cb_gps_utm,queue_size=1)
    rospy.Subscriber("/gps/qual", UInt8,cb_gps_qual,queue_size=1)
    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist,cb_cmd,queue_size=1)
    rospy.Subscriber("/wow/achieveGoal_error", Twist,cb_error,queue_size=1)
    rospy.Subscriber("/EKF/mag_org", MagneticField,cb_ekf_org,queue_size=1)
    # rospy.Subscriber("landmark_sigma",Twist,cb_sigma,queue_size=1)
    rate=rospy.Rate(10)
    a=a_plot()

    while not rospy.is_shutdown():
        a.car_position(ds)
        plt.pause(0.001)

        # a.save_fig()
        # print("plot time",time.time()-t)
        rate.sleep()
