#!/usr/bin/wowpython
'''ros utils'''
import time
import rospy
from sensor_msgs.msg import NavSatFix

from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseStamped, Point

from geometry_msgs.msg import Twist

import numpy as np

import os

import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt

def div(file_name,file_new_name):
    if os.path.isfile(file_new_name):
        print("already div")
        return

    if os.path.isfile(file_name):
        print('waypoint_utm exist!')
        x_waypoints_list = []
        y_waypoints_list = []
        with open(file_name) as f:
            for line in f.readlines():
                s = line.split(' ')
                x_waypoints_list.append(float(s[0]))
                y_waypoints_list.append(float(s[1]))
        x_waypoints = x_waypoints_list
        y_waypoints = y_waypoints_list
    fig, ax = plt.subplots(figsize=(10,10))


    n_x=list()
    n_y=list()
    for i in range(len(x_waypoints)-1):
        dis=np.sqrt((x_waypoints[i]-x_waypoints[i+1])**2+(y_waypoints[i]-y_waypoints[i+1])**2)
        n=1.0*int(dis*0.0001)

        if n==0:
            n_x.append(x_waypoints[i])
            n_y.append(y_waypoints[i])
        else:
            for j in range(int(n)):
                n_x.append((x_waypoints[i+1]-x_waypoints[i])/n*j+x_waypoints[i])
                n_y.append((y_waypoints[i+1]-y_waypoints[i])/n*j+y_waypoints[i])

    n_x.append(x_waypoints[-1])
    n_y.append(y_waypoints[-1])

    for i in range(len(n_x)):
        with open(file_new_name,"a") as f:
            f.write(str(n_x[i])+" "+str(n_y[i])+"\n")

if __name__=="__main__":

    file_name="/home/yuqiang/20220110_17-32-20/shapefiles/waypoint/waypoint.txt"
    file_new_name="/home/yuqiang/20220110_17-32-20/shapefiles/waypoint/waypoint_new.txt"
    div(file_name,file_new_name)