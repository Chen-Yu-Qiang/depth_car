#!/usr/bin/env python

from re import L
import numpy as np
import time
import random
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt


import rospy
import sys
import time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

from collections import deque

# q = Queue(maxsize=3)
q_x_gps, q_y_gps = deque([], maxlen=500), deque([], maxlen=500)
q_x, q_y = deque([], maxlen=2500), deque([], maxlen=2500)



TREE_DATA=[]
if sys.version[0]=='2':
    TREE_DATA=[[2767694.6000474878,-352852.16121769784,0.3],[2767687.550047488,-352862.78621769784,0.35],[2767694.800047488,-352880.1862176978,0.4],[2767700.5000474877,-352880.1362176978,0.2],[2767698.510047488,-352881.72621769784,0.2],[2767699.6500474876,-352882.8862176978,0.3],[2767697.483380821,-352883.4362176978,0.2],[2767701.3750474877,-352884.3862176978,0.15],[2767699.300047488,-352884.9362176978,0.45],[2767707.6516908277,-352848.88876166823,0.75],[2767730.1016908274,-352849.9387616683,0.15],[2767717.2516908273,-352851.1887616683,0.1],[2767721.1016908274,-352853.78876166826,0.25],[2767710.1016908274,-352856.08876166824,0.2],[2767718.1516908277,-352857.08876166824,0.25],[2767718.4016908277,-352859.48876166827,0.2],[2767729.522274709,-352863.2814219437,0.2],[2767711.022274709,-352863.5314219437,0.2],[2767730.122274709,-352870.4314219437,0.4],[2767730.222274709,-352878.5314219437,0.2],[2767702.022274709,-352888.08142194373,0.3],[2767716.822274709,-352889.6314219437,0.8],[2767726.122274709,-352889.58142194373,0.3],[2767719.072274709,-352889.8814219437,0.35],[2767727.572274709,-352890.33142194373,0.4],[2767725.9222747087,-352891.1314219437,0.35],[2767707.822274709,-352891.7314219437,0.65],[2767727.872274709,-352892.3564219437,0.225],[2767727.272274709,-352899.1814219437,0.2],[2767707.9222747087,-352904.83142194373,0.15],[2767701.572274709,-352905.1814219437,0.3],[2767670.0378069477,-352897.688491822,0.2],[2767664.7878069477,-352903.338491822,0.25],[2767649.7422344387,-352847.6228367216,0.75],[2767651.5922344383,-352851.3728367216,0.3],[2767652.3422344383,-352864.8728367216,0.35],[2767648.7422344387,-352871.07283672155,0.25],[2767697.109163938,-352891.04801889224,0.3],[2767696.909163938,-352893.5980188922,0.35],[2767652.6987745943,-352877.0904607297,0.23333333333333334],[2767654.3321079277,-352881.17379406304,0.5],[2767661.882107928,-352884.4737940631,0.55],[2767656.6821079277,-352888.62379406305,0.35],[2767660.3321079277,-352891.54879406304,0.3],[2767659.9321079277,-352895.2237940631,0.7],[2767665.3321079277,-352894.92379406304,0.35],[2767661.9321079277,-352899.92379406304,0.4]]
    
    tree_data_np=np.array(TREE_DATA)
    a=tree_data_np[:,1].copy()
    tree_data_np[:,1]=tree_data_np[:,0]
    tree_data_np[:,0]=-a
elif sys.version[0]=='3':
    tree_data_np = np.load('/home/ncslaber/center_list_all.npy')
TREE_DATA=tree_data_np
# print(tree_data_np)


class a_plot:
    def __init__(self):
        self.fig, self.ax = plt.subplots(1, 1,dpi=100,figsize=(10,10))
        self.ax.set_aspect('equal')

        gps_init=rospy.wait_for_message("/gps_utm", Twist)
        # print("!!!!!!!!!!!!!!!!!!1")
        if (gps_init.linear.y*(-1.0))<352875:
            # for H
            self.ax.set_xlim(352840,352870)
            self.ax.set_ylim(2767700,2767730)
            self.now_zone="h"
        else:
            # for B
            self.ax.set_xlim(352880,352910)
            self.ax.set_ylim(2767650,2767680)
            self.now_zone="b"
        # self.ax.hold(True)
        x, y, th= 352910,2767650,0
        
        self.ax.plot(TREE_DATA[:,0], TREE_DATA[:,1], 'o')[0]

        plt.show(False)
        plt.draw()
        
        
        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        # self.points = self.ax.plot(x, y, 'o', markersize=30)[0]


        r=0.1
        self.car_1 = self.ax.plot(x, y, 'o', markersize=5)[0]
        self.car_1_th = self.ax.plot([x,x+r*np.cos(th+np.pi*0.5)],[y,y+r*np.sin(th+np.pi*0.5)], '-')[0]
        self.car_2 = self.ax.plot(x, y, 'o', markersize=5)[0]

        r=0.1
        self.car_3 = self.ax.plot(x, y, 'o', markersize=4)[0]
        self.car_3_th = self.ax.plot([x,x+r*np.cos(th+np.pi*0.5)],[y,y+r*np.sin(th+np.pi*0.5)], '-')[0]

        self.trj_lm=self.ax.plot([],[],'-')[0]
        self.trj_gps=self.ax.plot([],[],'-')[0]


        self.obs_line=[None for i in range(10)]
        for i in range(10):
            self.obs_line[i]=self.ax.plot([0,1],[1,0],'--', color='k')[0]
        self.real_line=[None for i in range(10)]
        self.real_tree=[None for i in range(10)]
        for i in range(10):
            self.real_line[i]=self.ax.plot([0,1],[1,0],'-', color='b')[0] 
            self.real_tree[i]=self.ax.plot([0,1],[1,0],'o',markersize=10, color='r')[0] 
        self.now_zone="b"

    

    def car_position(self,x,y,th,x_pro,y_pro,th_pro,x_gps,y_gps,z_d,z_th,z_r,z_hat_d,z_hat_th,z_hat_r,q_x, q_y, q_x_gps, q_y_gps,z_hat_index):




        r=1
        self.car_1.set_data(x, y)
        self.car_1_th.set_data([x,x+r*np.cos(th+np.pi*0.5)],[y,y+r*np.sin(th+np.pi*0.5)])

        self.car_2.set_data(x_gps, y_gps)


        self.trj_lm.set_data(q_x, q_y)
        self.trj_gps.set_data(q_x_gps, q_y_gps)

        for i in range(len(z_hat_d)):
            self.obs_line[i].set_visible(True)
            self.real_line[i].set_visible(True)
            self.real_tree[i].set_visible(True)
            self.obs_line[i].set_data([x,x+z_d[i]*np.cos(z_th[i]+th+np.pi*0.5)],[y,y+z_d[i]*np.sin(z_th[i]+th+np.pi*0.5)])
            self.real_line[i].set_data([x,x+z_hat_d[i]*np.cos(z_hat_th[i]+th+np.pi*0.5)],[y,y+z_hat_d[i]*np.sin(z_hat_th[i]+th+np.pi*0.5)])
            print(z_hat_index[i])
            self.real_tree[i].set_data(TREE_DATA[z_hat_index[i],0],TREE_DATA[z_hat_index[i],1])


        for i in range(len(z_hat_d),len(z_d)):
            self.obs_line[i].set_visible(True)
            self.real_line[i].set_visible(False)
            self.real_tree[i].set_visible(False)
            self.obs_line[i].set_data([x,x+z_d[i]*np.cos(z_th[i]+th+np.pi*0.5)],[y,y+z_d[i]*np.sin(z_th[i]+th+np.pi*0.5)])


        for i in range(len(z_d),10):
            self.obs_line[i].set_visible(False)
            self.real_line[i].set_visible(False)
            self.real_tree[i].set_visible(False)

        if len(z_hat_d)==0:
            self.car_3.set_visible(False)
            self.car_3_th.set_visible(False)
        else:
            self.car_3.set_visible(True)
            self.car_3_th.set_visible(True)
            self.car_3.set_data(x_pro, y_pro)
            self.car_3_th.set_data([x_pro,x_pro+r*np.cos(th_pro+np.pi*0.5)],[y_pro,y_pro+r*np.sin(th_pro+np.pi*0.5)])

        # restore background
        self.fig.canvas.draw()
        self.fig.canvas.restore_region(self.background)

        # redraw just the points
        self.ax.draw_artist(self.car_1)
        self.ax.draw_artist(self.car_2)
        self.ax.draw_artist(self.car_3)
        self.ax.draw_artist(self.car_1_th)
        self.ax.draw_artist(self.car_3_th)
        for i in range(len(z_d)):
            self.ax.draw_artist(self.obs_line[i])
            self.ax.draw_artist(self.real_line[i])
            self.ax.draw_artist(self.real_tree[i])
        self.ax.draw_artist(self.trj_lm)
        self.ax.draw_artist(self.trj_gps)


        # fill in the axes rectangle
        self.fig.canvas.blit(self.ax.bbox)

    
    # def __del__(self):
    #     plt.close(self.fig)



ARRAY_LAY2=40
x_gps,y_gps=2767650,-352910
x,y,th,x_pro,y_pro,th_pro,z_d,z_th,z_r,z_hat_d,z_hat_th,z_hat_r=2767650,-352910,0,2767650,-352910,0,[],[],[],[],[],[]
z_hat_index=[]
def cb_landmark_z(data):
    global x,y,th,x_pro,y_pro,th_pro,z_d,z_th,z_r,z_hat_d,z_hat_th,z_hat_r, q_x, q_y,z_hat_index
    d=list(data.data)
    n=int(len(d)/ARRAY_LAY2)

    z_d=[]
    z_th=[]
    z_r=[]
    z_hat_d=[]
    z_hat_th=[]
    z_hat_r=[]
    z_hat_index=[]
    x_pro=0
    y_pro=0
    th_pro=0

    # print("d:",d)
    for i in range(n):
        if d[ARRAY_LAY2*i]>0:
            z_d.append(d[ARRAY_LAY2*i+2])
            z_th.append(d[ARRAY_LAY2*i+3])
            z_r.append(d[ARRAY_LAY2*i+4])
            z_hat_d.append(d[ARRAY_LAY2*i+5])
            z_hat_th.append(d[ARRAY_LAY2*i+6])
            z_hat_r.append(d[ARRAY_LAY2*i+7])
            x_pro=d[ARRAY_LAY2*i+13]
            y_pro=d[ARRAY_LAY2*i+14]
            th_pro=d[ARRAY_LAY2*i+15]
            z_hat_index.append(int(d[ARRAY_LAY2*i]-1))
    for i in range(n):
        if d[ARRAY_LAY2*i]<0:
            z_d.append(d[ARRAY_LAY2*i+2])
            z_th.append(d[ARRAY_LAY2*i+3])
            z_r.append(d[ARRAY_LAY2*i+4])

    if len(d):
        y=d[25]
        x=d[26]*(-1.0)
        th=d[27]
        q_x.append(x)
        q_y.append(y)


def cb_gps(data):
    global x_gps,y_gps, q_x_gps, q_y_gps
    x_gps=data.linear.y*(-1.0)
    y_gps=data.linear.x
    
    q_x_gps.append(x_gps)
    q_y_gps.append(y_gps)

def cb_lm(data):
    global x,y,th
    x=data.linear.y*(-1.0)
    y=data.linear.x
    th=data.angular.z
    q_x.append(x)
    q_y.append(y)

if __name__ == '__main__':

    print("Python version: ",sys.version)
    rospy.init_node("plot_node", anonymous=True)
    landmark_z_sub=rospy.Subscriber("/landmark_z", Float64MultiArray,cb_landmark_z,queue_size=1)
    gps_sub=rospy.Subscriber("/gps_utm", Twist,cb_gps,queue_size=1)
    lm_sub=rospy.Subscriber("/landmark", Twist,cb_lm,queue_size=1)
    rate=rospy.Rate(30)
    a=a_plot()
    while not rospy.is_shutdown():
        # print(x,y,th,x_pro,y_pro,th_pro,x_gps,y_gps,z_d,z_th,z_r,z_hat_d,z_hat_th,z_hat_r)
        t=time.time()
        a.car_position(x,y,th,x_pro,y_pro,th_pro,x_gps,y_gps,z_d,z_th,z_r,z_hat_d,z_hat_th,z_hat_r, q_x, q_y, q_x_gps, q_y_gps,z_hat_index)
        # print("plot time",time.time()-t)
        rate.sleep()
    # for i in range(10):
    #     x=random.random()*10+2767650
    #     y=random.random()*10-352910
    #     th=random.random()*2*np.pi
    #     a.car_position(x,y,th,[-1],[-1],[-1],[-1],[-1],[-1],[-1],[-1])
    #     plt.pause(1)
    # plt.pause(5)

    # world_bounds_x = [0,1]
    # world_bounds_y = [2,3]
    # # world_bounds_x = [352880,352910]
    # # world_bounds_y = [2767650,2767680]
    # fig = plt.figure(figsize=(18,6),dpi=120)


    # subplott1 = fig.add_subplot(131)
    # subplott1.set_xlim(0,2)
    # subplott1.set_ylim(0,10)
    # # subplott1.axis('equal')
    # subplott1.plot([1,2,3,4,5])


    # subplott2 = fig.add_subplot(132)
    # subplott2.set_xlim(0,3)
    # subplott2.set_ylim(0,10)
    # # subplott2.axis('equal')
    # subplott2.plot([1,2,3,4,5])

    # subplott3 = fig.add_subplot(133)
    # subplott3.set_xlim(0,10)
    # subplott3.set_ylim(0,10)
    # # subplott3.axis('equal')
    # subplott3.plot([1,2,3,4,5])

    # plt.show()