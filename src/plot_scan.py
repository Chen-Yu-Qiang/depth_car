#!/usr/bin/wowpython

from re import L, TEMPLATE
import numpy as np
import time
import random
from sensor_msgs.msg import LaserScan
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
from datetime import datetime

import rospy
import sys
import time
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped
from mapping_explorer.msg import Trunkset, Trunkinfo, Correspondence

import TREEDATA
from collections import deque


laser_msg=LaserScan()
trunk_msg=Trunkset()
def cbLaser(msg):
    global laser_msg
    laser_msg=msg


def cbTrunkset(msg):
    global trunk_msg
    trunk_msg=msg


if __name__=='__main__':
        
    rospy.init_node("laser_ploter", anonymous=True)
    rospy.Subscriber("/scan", LaserScan, cbLaser, buff_size=2**20, queue_size=1)
    rospy.Subscriber("/tree/trunk_info", Trunkset, cbTrunkset, buff_size=2**20, queue_size=1)
    rate=rospy.Rate(15)


    fig, ax = plt.subplots(1, 1,dpi=70,figsize=(10,5))
    plt.ion()
    ax.set_aspect('equal')

    ax.set_xlim(-10,10)
    ax.set_ylim(-1,10)
    tree_plot_list=[ax.plot([2],[2], 'o', color='k', markersize=3)[0] for i in range(10)]
    tree_text_list=[ax.text(0,0,'',fontsize=14, color='k') for i in range(10)]
    ls_plot_obj=ax.plot([2],[1], 'o', color='r', markersize=2.5, label='Laser')[0]

    xminorLocator = MultipleLocator(1)
    yminorLocator = MultipleLocator(1)
    ax.xaxis.set_minor_locator(xminorLocator)
    ax.yaxis.set_minor_locator(yminorLocator)

    xmajorLocator = MultipleLocator(5)
    ymajorLocator = MultipleLocator(5)
    ax.xaxis.set_major_locator(xmajorLocator)
    ax.yaxis.set_major_locator(ymajorLocator)

    ax.xaxis.grid(True, which='minor',color="silver")
    ax.xaxis.grid(True, which='major',color="k")
    ax.yaxis.grid(True, which='minor',color="silver")
    ax.yaxis.grid(True, which='major',color="k")


    while not rospy.is_shutdown():
        point_x_list=[]
        point_z_list=[]


        for i in range(len(laser_msg.ranges)):
            ls_range = laser_msg.ranges[i]    
            theta = laser_msg.angle_max - laser_msg.angle_increment * i
            x_camera = ls_range * np.sin(theta)
            z_camera = ls_range * np.cos(theta)

            point_x_list.append(x_camera)
            point_z_list.append(z_camera)

        # print(point_x_list,point_z_list)
        ls_plot_obj.set_data(point_x_list,point_z_list)




        r_list=[]
        d_list=[]
        th_list=[]

        for i in range(len(trunk_msg.aframe)):
            r_list.append(trunk_msg.aframe[i].r)
            d_list.append(trunk_msg.aframe[i].d)
            th_list.append(trunk_msg.aframe[i].t)

        # print(len(r_list))


        number_of_point=50

        trunk_point=np.zeros((len(r_list),number_of_point))
        
        piece_rad = np.pi/(number_of_point/2)
        for i in range(len(r_list)):
            trunk_point=[]
            trunk_x=d_list[i]*np.cos(th_list[i])
            trunk_z=d_list[i]*np.sin(th_list[i])*(-1.0)


            for j in range(number_of_point):
                trunk_point.append((trunk_x+r_list[i]*np.cos(piece_rad*j), trunk_z+r_list[i]*np.sin(piece_rad*j)))
            trunk_point=np.asarray(trunk_point)
            tree_plot_list[i].set_visible(True)
            tree_plot_list[i].set_data(trunk_point[:,1], trunk_point[:,0])
            tree_text_list[i].set_visible(True)
            tree_text_list[i].set_position((max(min(trunk_z,10),-10),max(min(trunk_x,10),-1)))
            tree_text_list[i].set_text("d = "+str(round(d_list[i],3))+"\nr = "+str(round(r_list[i],3))+"\nth = "+str(round(th_list[i],3)))

        for i in range(len(r_list),10):
            tree_plot_list[i].set_visible(False)
            tree_text_list[i].set_visible(False)

        
        # print(2)

        plt.pause(0.01)

    rate.sleep()

