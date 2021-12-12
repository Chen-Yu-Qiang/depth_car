#!/usr/bin/env python

import time

import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
from datetime import datetime

import rospy
import sys
import time
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import MagneticField,NavSatFix



from collections import deque

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


class a_plot:
    def __init__(self):
        if sys.version[0]=='2':
            self.fig, (self.ax1,self.ax2) = plt.subplots(1, 2,dpi=70,figsize=(10,5))
        elif sys.version[0]=='3':
            self.fig, (self.ax1,self.ax2) = plt.subplots(1, 2,dpi=120,figsize=(10,10))
        

        self.ax1.set_aspect('equal')
        self.ax2.set_aspect('equal')

        self.ax2.set_xlim(-0.0002,0)
        self.ax2.set_ylim(-0.00014,0.00002)

        plt.show(False)
        plt.draw()
        
        
        self.background1 = self.fig.canvas.copy_from_bbox(self.ax1.bbox)
        self.background2 = self.fig.canvas.copy_from_bbox(self.ax2.bbox)


        self.mag_line=self.ax2.plot([],[],'-', color='b')[0] 


        self.corres=self.ax1.text(0,0,'',fontsize=14, color='k')


    def save_fig(self):

        if sys.version[0]=='2':
            self.fig.savefig('/home/yuqiang/211125/' + datetime.now().strftime("%d-%m-%Y %H:%M:%S.%f")+'.png',)
        elif sys.version[0]=='3':
            self.fig.savefig('/home/ncslaber/110-1/211125_localTest/z_hat/' + datetime.now().strftime("%d-%m-%Y_%H:%M:%S.%f")+'.png',)

    def car_position(self,ds):
        
        mag_x=ds.get("mag_x")
        mag_y=ds.get("mag_y")
        gps_lat=ds.get("gps_lat")
        gps_lon=ds.get("gps_lon")
        gps_qual=ds.get("gps_qual")
        cmd_v=ds.get("cmd_v")
        cmd_omg=ds.get("cmd_omg")
        self.corres.set_text(datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")+\
            "\n\n GPS(lat WD) = "+str(gps_lat)+\
            "\n GPS(lon JD) = "+str(gps_lon)+\
            "\n GPS gual = "+str(gps_qual)+\
            "\n\n CMD v = "+str(cmd_v)+\
            "\n CMD omg = "+str(cmd_omg)+\
            "\n ")
        self.mag_line.set_data(mag_x,mag_y)


        # restore background
        self.fig.canvas.draw()
        self.fig.canvas.restore_region(self.background)
        self.fig.canvas.restore_region(self.background2)

        # redraw just the points
        self.ax1.draw_artist(self.corres)
        self.ax2.draw_artist(self.mag_line)


        # fill in the axes rectangle
        self.fig.canvas.blit(self.ax1.bbox)
        self.fig.canvas.blit(self.ax2.bbox)


ds=data_set()
ds.set("mag_x",deque([], maxlen=2500))
ds.set("mag_y",deque([], maxlen=2500))

def cb_mag(data):
    ds.append("mag_x",data.magnetic_field.x)
    ds.append("mag_y",data.magnetic_field.y)

def cb_gps(data):
    ds.set("gps_lat",data.latitude)
    ds.set("gps_lon",data.longitude)

def cb_gps_qual(data):
    ds.set("gps_qual",data.data)


def cb_cmd(data):
    ds.set("cmd_v",data.linear.x)
    ds.set("cmd_omg",data.angular.z)

if __name__ == '__main__':

    print("Python version: ",sys.version)
    rospy.init_node("plot_node2", anonymous=True)

    rospy.Subscriber("/imu/mag", MagneticField,cb_mag,queue_size=1)
    rospy.Subscriber("/outdoor_waypoint_nav/gps/filtered", NavSatFix,cb_gps,queue_size=1)
    rospy.Subscriber("/gps/qual", UInt8,cb_gps_qual,queue_size=1)
    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist,cb_cmd,queue_size=1)
    rate=rospy.Rate(10)
    a=a_plot()

    while not rospy.is_shutdown():

        try:
            a.car_position(ds)
        except:
            pass
        # a.save_fig()
        # print("plot time",time.time()-t)
        rate.sleep()
