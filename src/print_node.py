#!/usr/bin/wowpython
import time

from datetime import datetime
import numpy as np

import rospy
import sys
import time
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import MagneticField,NavSatFix
import threading


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


ds=data_set()


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
    ds.set("error_x",data.linear.x)
    ds.set("error_y",data.linear.y)
    ds.set("error_d",data.linear.z)
    ds.set("error_ang",data.angular.z)
    ds.set("error_s",data.angular.x)
    ds.lock.release()

  

if __name__ == '__main__':

    print("Python version: ",sys.version)
    rospy.init_node("print_node", anonymous=True)
    rospy.Subscriber("/outdoor_waypoint_nav/gps/filtered", NavSatFix,cb_gps,queue_size=1)
    rospy.Subscriber("/lm_ekf/gps/utm", Twist,cb_gps_utm,queue_size=1)
    rospy.Subscriber("/gps/qual", UInt8,cb_gps_qual,queue_size=1)
    rospy.Subscriber("/husky_velocity_controller/cmd_vel", Twist,cb_cmd,queue_size=1)
    # rospy.Subscriber("/ctrl/achieve_error", Twist,cb_error,queue_size=1)
    rospy.Subscriber("/ctrl/error", Twist,cb_error,queue_size=1)
    rospy.Subscriber("/lm_ekf/sigma",Twist,cb_sigma,queue_size=1)
    rate=rospy.Rate(10)
    import curses
    stdscr = curses.initscr()
    # curses.noecho()
    begin_x = 0
    begin_y = 0
    height = 20
    width = 40
    win1 = curses.newwin(height, width, begin_y, begin_x)
    begin_x = 40
    begin_y = 0
    height = 20
    width = 40
    win2 = curses.newwin(height, width, begin_y, begin_x)

    while not rospy.is_shutdown():
        

        ds.lock.acquire()
        gps_lat=ds.get("gps_lat")
        gps_lon=ds.get("gps_lon")
        gps_x=ds.get("gps_x")
        gps_y=ds.get("gps_y")
        gps_qual=ds.get("gps_qual")
        cmd_v=ds.get("cmd_v")
        cmd_omg=ds.get("cmd_omg")
        sigma_x=ds.get("sigma_x")
        sigma_y=ds.get("sigma_y")
        sigma_th=ds.get("sigma_th")

        error_x=ds.get("error_x")
        error_y=ds.get("error_y")
        error_d=ds.get("error_d")
        error_ang=ds.get("error_ang")
        error_s=ds.get("error_s")
        ds.lock.release()

        win1.addstr(0,0,datetime.now().strftime(\
            "%Y/%m/%d %H:%M:%S.%f")+\
            "\n"+\
            datetime.fromtimestamp(rospy.get_time()).strftime(\
            "%Y/%m/%d %H:%M:%S.%f")+\
            "\n\n===GPS==="+\
            "\n lat (WD) = "+str(round(gps_lat,8))+" deg"+\
            "\n lon (JD) = "+str(round(gps_lon,8))+" deg"+\
            "\n UTM x ="+str(round(gps_y,2)*(-1.0))+" m"+\
            "\n UTM y ="+str(round(gps_x,2))+" m"+\
            "\n Quality = "+str(gps_qual)+\
            "\n\n===Command==="+\
            "\n v = "+str(round(cmd_v,5))+" m/s"+\
            "\n omega = "+str(round(cmd_omg,5))+" deg/s\n")


        
        mode={-1:"",0:"Standby",1:"Get a New WayPoint and Rotating",2:"Reach the Direction and Moving"}
        win2.addstr(0,0,\
            "\n\n===Error==="+\
            "\n x = "+str(round(error_x,2))+" m"+\
            "\n y = "+str(round(error_y,2))+" m"+\
            "\n Dis. = "+str(round(error_d,2))+" m"+\
            "\n Angle = "+str(round(error_ang,2))+" rad"+\
            "\n Mode = "+str(int(error_s))+" "+mode[int(error_s)]+\
            "\n\n===Sigma==="+\
            "\n x = "+str(round(sigma_x,5))+" m/s"+\
            "\n y = "+str(round(sigma_y,5))+" m/s"+\
            "\n theta = "+str(round(sigma_th,5))+" m/s")
        win1.refresh()

        win2.refresh()
        rate.sleep()
    win1.nocbreak()
    win2.nocbreak()
    stdscr.keypad(False)
    win1.echo()
    win2.echo()