#!/usr/bin/wowpython

from re import L, TEMPLATE
import numpy as np
import time
import random
import matplotlib
# matplotlib.use('TKAgg')
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





TREE_DATA=[]
# if sys.version[0]=='2':
#     TREE_DATA=TREEDATA.TREE_DATA
#     tree_data_np=np.array(TREE_DATA)
#     a=tree_data_np[:,1].copy()
#     tree_data_np[:,1]=tree_data_np[:,0]
#     tree_data_np[:,0]=-a
# elif sys.version[0]=='3':
#     tree_data_np = np.load('/home/ncslaber/center_list_all.npy')




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

class car_obj:
    def __init__(self,plot_obj,r=1,ms=10,c="red",ec="k",trj_en=1,theta_en=1,car_name=""):
        self.x=359159.089156
        self.y=2774786.94243
        self.th=0
        self.trj_data_x=deque([], maxlen=2500)
        self.trj_data_y=deque([], maxlen=2500)
        self.trj_data_th=deque([], maxlen=2500)
        

        self.r=r
        self.trj_en=trj_en
        self.theta_en=theta_en
        self.ax_obj=plot_obj.ax.plot([359159.089156], [2774786], 'o',markersize=ms, color=c, markeredgecolor=ec,label=car_name)[0]

        if self.theta_en:        
            self.ax_th_obj=plot_obj.ax.plot([],[], '-',linewidth=5, color=c)[0]
        if trj_en:
            self.ax_trj_obj=plot_obj.ax.plot([],[],'-', color=c,label=car_name+" Trajectory")[0]


    def update(self,x,y,th):
        r=self.r
        self.ax_obj.set_data(x,y)

        if self.theta_en:
            self.ax_th_obj.set_data([x,x+r*np.cos(th+np.pi*0.5)],[y,y+r*np.sin(th+np.pi*0.5)])
        if self.trj_en and not(x==-1 or y==-1):
            self.trj_data_x.append(x)
            self.trj_data_y.append(y)
            self.trj_data_th.append(th)
            self.ax_trj_obj.set_data(self.trj_data_x,self.trj_data_y)


    def draw_artist(self,plot_obj):    
        plot_obj.ax.draw_artist(self.ax_obj)
        if self.theta_en:
            plot_obj.ax.draw_artist(self.ax_th_obj)
        if self.trj_en:
            plot_obj.ax.draw_artist(self.ax_trj_obj)

    def set_visible(self,v):
        self.ax_obj.set_visible(v)
        if self.theta_en:
            self.ax_th_obj.set_visible(v)
        if self.trj_en:
            self.ax_trj_obj.set_visible(v)

    def change_visible_trj(self):
        if self.trj_en:
            self.ax_trj_obj.set_visible(not self.ax_trj_obj.get_visible())


class a_plot:
    def __init__(self, mode=0):
        if sys.version[0]=='2':
            self.fig, self.ax = plt.subplots(1, 1,dpi=70,figsize=(10,10))
        elif sys.version[0]=='3':
            self.fig, self.ax = plt.subplots(1, 1,dpi=120,figsize=(10,10))
        self.ax.set_aspect('equal')
        self.mode=mode
        self.fig.canvas.mpl_connect('key_press_event', self.key_event)
        self.ax.set_title("Map")
        self.ax.set_xlabel("X (Weat--East) [m]")
        self.ax.set_ylabel("Y (South--North) [m]")
        plt.ion()

        # gps_init=rospy.wait_for_message("/lm_ekf/gps/utm", Twist)
        # print("!!!!!!!!!!!!!!!!!!1")
        # if (gps_init.linear.x)>2767690:
        #     # for H
        #     self.ax.set_xlim(352840,352875)
        #     self.ax.set_ylim(2767700,2767735)
        #     self.now_zone="h"
        # else:
        #     if (gps_init.linear.y)< (-352865):
        #         # for B
        #         self.ax.set_xlim(352860,352905)
        #         self.ax.set_ylim(2767645,2767690)
        #         self.now_zone="b"
        #     else:
        #         # for I
        #         self.ax.set_xlim(352835,352865)
        #         self.ax.set_ylim(2767655,2767690)
        #         self.now_zone="i"


        # self.ax.hold(True)
        # self.ax.set_xlim(352840,352910)
        # self.ax.set_ylim(2767650,2767745)      


        # self.ax.set_xlim(352840,352865)
        # self.ax.set_ylim(2767665,2767700)

        # self.ax.set_xlim(359190,359220)
        # self.ax.set_ylim(2774760,2774780)


        # self.ax.set_xlim(352840,352875)
        # self.ax.set_ylim(2767700,2767735)
        self.now_zone="h"
   
        # da hu 1 zone
        # self.ax.set_xlim(359220,359240)
        # self.ax.set_ylim(2774760,2774775)


        self.ax.set_xlim(TREEDATA.Y_MAX*(-1.0),TREEDATA.Y_MIN*(-1.0))
        self.ax.set_ylim(TREEDATA.X_MIN,TREEDATA.X_MAX)


        x, y, th= 352910,2767650,0
        
        self.ax.plot(TREE_DATA[:,0], TREE_DATA[:,1], 'x', color='g', markersize=5, label='Tree')[0]

        number_of_point=12
        piece_rad = np.pi/(number_of_point/2)
        for j in range( len(TREE_DATA[:,1]) ):
            neg_bd = []
            for i in range(number_of_point):
                neg_bd.append((TREE_DATA[j,0]+TREE_DATA[j,2]*np.cos(piece_rad*i), TREE_DATA[j,1]+TREE_DATA[j,2]*np.sin(piece_rad*i)))
            neg_bd=np.asarray(neg_bd)
            self.ax.plot(neg_bd[:,0], neg_bd[:,1], 'o', color='k', markersize=3)[0]
            self.ax.text(TREE_DATA[j,0]+1,TREE_DATA[j,1],str(j),fontsize=20, color='k')



        xminorLocator = MultipleLocator(1)
        yminorLocator = MultipleLocator(1)
        self.ax.xaxis.set_minor_locator(xminorLocator)
        self.ax.yaxis.set_minor_locator(yminorLocator)

        xmajorLocator = MultipleLocator(5)
        ymajorLocator = MultipleLocator(5)
        self.ax.xaxis.set_major_locator(xmajorLocator)
        self.ax.yaxis.set_major_locator(ymajorLocator)

        self.ax.xaxis.grid(True, which='minor',color="silver")
        self.ax.xaxis.grid(True, which='major',color="k")
        self.ax.yaxis.grid(True, which='minor',color="silver")
        self.ax.yaxis.grid(True, which='major',color="k")



        self.car2_obj=car_obj(self,c='red',theta_en=0,car_name="Car (w/ GPS)")
        self.car1_obj=car_obj(self,c='blue',car_name="Car (w/ Landmark)")
        self.car3_obj=car_obj(self,trj_en=0)
        self.car4_obj=car_obj(self,c='k',trj_en=1,theta_en=0,car_name="Way point")
        self.car5_obj=car_obj(self,c='green',trj_en=1,theta_en=0,car_name="gps offset")


        if self.now_zone=="h":
            self.corres=self.ax.text(352840,2767700,'',fontsize=14, color='k')
        elif self.now_zone=="b":
            self.corres=self.ax.text(352860,2767645,'',fontsize=14, color='k')
        elif self.now_zone=="i":
            self.corres=self.ax.text(352835,2767655,'',fontsize=14, color='k')
        else:
            self.corres=self.ax.text(352835,2767655,'',fontsize=14, color='k')


        self.obs_line=[None for i in range(10)]
        self.obs_line[0]=self.ax.plot([0,1],[1,0],'-', color='k', label='Observation tree')[0]
        for i in range(1,10):
            self.obs_line[i]=self.ax.plot([0,1],[1,0],'-', color='k')[0]


        self.real_line=[None for i in range(10)]
        self.real_tree=[None for i in range(10)]
        self.real_line[0]=self.ax.plot([0,1],[1,0],'--', color='b', label='Real tree')[0] 
        self.real_tree[0]=self.ax.plot([0,1],[1,0],'o',markersize=10, color='r', label='Matched')[0] 
        for i in range(1,10):
            self.real_line[i]=self.ax.plot([0,1],[1,0],'--', color='b')[0] 
            self.real_tree[i]=self.ax.plot([0,1],[1,0],'o',markersize=10, color='r')[0] 
        self.now_zone="b"
        self.ax.legend()
        plt.pause(0.1)

    def key_event(self,e):
        print("get",str(e.key))
        sys.stdout.flush()
        if e.key=='w':
            self.car2_obj.change_visible_trj()
        if e.key=='e':
            self.car1_obj.change_visible_trj()
        if e.key=='r':
            self.car5_obj.change_visible_trj()
        

    def save_fig(self):

        if sys.version[0]=='2':
            self.fig.savefig('/home/yuqiang/211125/' + datetime.now().strftime("%d-%m-%Y %H:%M:%S.%f")+'.png',)
        elif sys.version[0]=='3':
            self.fig.savefig('/home/ncslaber/110-1/211125_localTest/z_hat/' + datetime.now().strftime("%d-%m-%Y_%H:%M:%S.%f")+'.png',)

    def car_position(self,ds):
        tt0=time.time()
        x=ds.get("x")
        y=ds.get("y")
        th=ds.get("th")
        x_fm=ds.get("x_fm")
        y_fm=ds.get("y_fm")
        th_fm=ds.get("th_fm")
        x_pro=ds.get("x_pro")
        y_pro=ds.get("y_pro")
        th_pro=ds.get("th_pro")
        x_gps=ds.get("x_gps")
        y_gps=ds.get("y_gps")
        th_gps=ds.get("th_gps")
        if time.time()-ds.get("z_time")>0.1:
            ds.set("z_d",[])
            ds.set("z_th",[])
            ds.set("z_r",[])
            ds.set("z_hat_d",[])
            ds.set("z_hat_th",[])
            ds.set("z_hat_r",[])
        z_d=ds.get("z_d")
        z_th=ds.get("z_th")
        z_r=ds.get("z_r")
        z_hat_d=ds.get("z_hat_d")
        z_hat_th=ds.get("z_hat_th")
        z_hat_r=ds.get("z_hat_r")

        z_hat_index=ds.get("z_hat_index")
        correspondence=ds.get("correspondence")
        resid_scalar=ds.get("resid_scalar")
        mean_error=ds.get("mean_error")

        x_utm_waypoint=ds.get("waypoint_utm_x")
        y_utm_waypoint=ds.get("waypoint_utm_y")

        x_gps_offset=ds.get("x_gps_offset")
        y_gps_offset=ds.get("y_gps_offset")
        tt1=time.time()



               
        self.corres.set_text(str(correspondence)+"\nResid = "+str(round(resid_scalar,2))+" m\nMatch Error(RMS)= "+str(round(mean_error,3))+" m")

        r=1

        
        self.car2_obj.update(x_gps, y_gps, th_gps)
        self.car1_obj.update(x, y, th)
        self.car4_obj.update(x_utm_waypoint, y_utm_waypoint, 0)
        self.car5_obj.update(x_gps_offset, y_gps_offset, 0)
        tt2=time.time()




        for i in range(len(z_hat_d)):
            self.obs_line[i].set_visible(True)
            self.real_line[i].set_visible(True)
            self.real_tree[i].set_visible(True)
            if int(rospy.get_param('XZ_MODE')):
                x_w=z_d[i]*np.cos(-th)-z_th[i]*np.sin(-th)
                z_w=z_d[i]*np.sin(-th)+z_th[i]*np.cos(-th)
                self.obs_line[i].set_data([x,x-x_w],[y,y+z_w])
                x_w=z_hat_d[i]*np.cos(-th)-z_hat_th[i]*np.sin(-th)
                z_w=z_hat_d[i]*np.sin(-th)+z_hat_th[i]*np.cos(-th)
                self.real_line[i].set_data([x,x-x_w],[y,y+z_w])
            else:
                self.obs_line[i].set_data([x,x+z_d[i]*np.cos(z_th[i]+th+np.pi*0.5)],[y,y+z_d[i]*np.sin(z_th[i]+th+np.pi*0.5)])
                self.real_line[i].set_data([x,x+z_hat_d[i]*np.cos(z_hat_th[i]+th+np.pi*0.5)],[y,y+z_hat_d[i]*np.sin(z_hat_th[i]+th+np.pi*0.5)])
            # print(z_hat_index[i])
            self.real_tree[i].set_data(TREE_DATA[z_hat_index[i],0],TREE_DATA[z_hat_index[i],1])


        for i in range(len(z_hat_d),len(z_d)):
            self.obs_line[i].set_visible(True)
            self.real_line[i].set_visible(False)
            self.real_tree[i].set_visible(False)
            if int(rospy.get_param('XZ_MODE')):
                x_w=z_d[i]*np.cos(-th)-z_th[i]*np.sin(-th)
                z_w=z_d[i]*np.sin(-th)+z_th[i]*np.cos(-th)
                self.obs_line[i].set_data([x,x-x_w],[y,y+z_w])
            else:
                self.obs_line[i].set_data([x,x+z_d[i]*np.cos(z_th[i]+th+np.pi*0.5)],[y,y+z_d[i]*np.sin(z_th[i]+th+np.pi*0.5)])


        for i in range(len(z_d),10):
            self.obs_line[i].set_visible(False)
            self.real_line[i].set_visible(False)
            self.real_tree[i].set_visible(False)

        if len(z_hat_d)==0:
            self.car3_obj.set_visible(False)
        else:
            self.car3_obj.set_visible(False)
            # self.car3_obj.update(x_pro, y_pro, th_pro)
    

        # print("plot time",tt1-tt0,tt2-tt1,time.time()-tt2)

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

    # if len(d):

    #     ds.set("x",d[26]*(-1.0))
    #     ds.set("y",d[25])
    #     ds.set("th",d[27])
    #     ds.append("q_x",d[26]*(-1.0))
    #     ds.append("q_y",d[25])



def cb_gps(data):

    ds.set("x_gps",data.linear.y*(-1.0))
    ds.set("y_gps",data.linear.x)
    ds.set("th_gps",data.angular.z)

def cb_lm(data):

    ds.set("x",data.linear.y*(-1.0))
    ds.set("y",data.linear.x)
    ds.set("th",data.angular.z)
    ds.set("x_fm",data.linear.y*(-1.0)+1.0)
    ds.set("y_fm",data.linear.x+1.0)
    ds.set("th_fm",data.angular.z)
    # ds.append("q_x",data.linear.y*(-1.0))
    # ds.append("q_y",data.linear.x)


def cbTrunk(msg):
    ds.set("correspondence",msg.match)
    ds.set("resid_scalar",msg.residuals)

def cb_landmark_error(msg):
    d=list(msg.data)
    if len(d):
        ds.set("mean_error",d[1])
    

def cbGoal(msg):
    ds.set("waypoint_utm_x",msg.pose.position.y*(-1.0))
    ds.set("waypoint_utm_y",msg.pose.position.x)

def cb_gps_offset(data):

    ds.set("x_gps_offset",data.linear.y*(-1.0))
    ds.set("y_gps_offset",data.linear.x)
    


if __name__ == '__main__':

    print("Python version: ",sys.version)
    rospy.init_node("plot_node2", anonymous=True)
    landmark_z_sub=rospy.Subscriber("/lm_ekf/z", Float64MultiArray,cb_landmark_z,buff_size=2**20,queue_size=1)
    landmark_error_sub=rospy.Subscriber("/lm_ekf/error", Float64MultiArray,cb_landmark_error, buff_size=2**20,queue_size=1)
    gps_sub=rospy.Subscriber("/lm_ekf/gps/utm", Twist,cb_gps,buff_size=2**20,queue_size=1)
    # gps_sub=rospy.Subscriber("/lm_ekf/filtered_map/utm", Twist,cb_gps,queue_size=1)
    lm_sub=rospy.Subscriber("/lm_ekf/raw/utm", Twist,cb_lm,queue_size=1, buff_size=2**20)
    gps_offset_sub=rospy.Subscriber("/lm_ekf/gps_w_offset/utm", Twist,cb_gps_offset,queue_size=1, buff_size=2**20)
    subTrunk = rospy.Subscriber("/tree/trunk_info", Trunkset, cbTrunk,buff_size=2**20,queue_size=1)
    subGoal = rospy.Subscriber("/ctrl/wp/utm", PoseStamped, cbGoal,buff_size=2**20,queue_size=1)
    if rospy.get_param("Good_Computer",default="0")=='1':
        rate=rospy.Rate(20)
    else:
        rate=rospy.Rate(5)

    # from npy file
    # TREE_DATA=TREEDATA.TREE_DATA
    # tree_data_np=np.array(TREE_DATA)
    # a=tree_data_np[:,1].copy()
    # tree_data_np[:,1]=tree_data_np[:,0]
    # tree_data_np[:,0]=-a
    # TREE_DATA=tree_data_np
    # X_MIM,X_MAX=TREEDATA.X_MIN,TREEDATA.X_MAX
    # Y_MIN,Y_MAX=TREEDATA.Y_MAX*(-1.0),TREEDATA.Y_MIN*(-1.0)

    # print(tree_data_np)

    # from ROS topic
    print("[plot_node2.py] wait for Tree List msg")
    temp=rospy.wait_for_message("/tree/data/list", Float64MultiArray)
    # print(temp)
    temp=list(temp.data)
    TREE_DATA=[]
    for i in range(0,len(temp),3):
        TREE_DATA.append([temp[i+1]*(-1.0),temp[i],temp[i+2]])
        print("[plot_node2.py] Get tree X="+str(temp[i])+", Y="+str(temp[i+1]))
    TREE_DATA=np.array(TREE_DATA)
    print(TREE_DATA)
    X_MAX,X_MIN,Y_MIN,Y_MAX,_,_=TREEDATA.get_maxmin(TREE_DATA)
    Y_MIN=Y_MIN*(-1.0)
    Y_MAX=Y_MAX*(-1.0)

    a=a_plot()

    while not rospy.is_shutdown():
        t=time.time()

        try:
            
            a.car_position(ds)
            t2=time.time()
            plt.pause(0.001)

        except:
            pass
        # a.save_fig()
        # print("plot time",time.time()-t2,t2-t)
        rate.sleep()
