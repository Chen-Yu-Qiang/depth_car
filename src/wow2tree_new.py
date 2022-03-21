#!/usr/bin/wowpython


from numpy.lib.type_check import imag
import rospy
import sys

import time

from std_msgs.msg import Float64MultiArray,MultiArrayDimension

import numpy as np
from mapping_explorer.msg import Trunkset, Trunkinfo
import TREEDATA


AA=0
ARRAY_LAY1=20


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

    n=len(data.aframe)
    if n==0:
        return
    a=[0 for i in range(n*ARRAY_LAY1)]
    for i in range(n):
        inAframe = data.aframe[i]
        distance = inAframe.d
        theta = inAframe.t
        radius = inAframe.r
        radius = min(TREEDATA.R_MAX, max(TREEDATA.R_MIN,radius))
        if n<2:
            cor=-100
        else:
            if i<len(data.match):
                cor = int(data.match[i])
            else:
                cor=-100

        x=np.sin(theta)*distance
        z=np.cos(theta)*distance
        a[i*ARRAY_LAY1]=x
        a[i*ARRAY_LAY1+1]=z
        a[i*ARRAY_LAY1+2]=radius
        a[i*ARRAY_LAY1+3]=distance
        a[i*ARRAY_LAY1+4]=theta       
        a[i*ARRAY_LAY1+11]=cor


    b=Float64MultiArray(data=a)
    
    b.layout.dim=[MultiArrayDimension()]
    b.layout.dim[0].stride=ARRAY_LAY1
    b.layout.dim[0].size=ARRAY_LAY1

    b.layout.dim[0].label="A_Tree_each"
    tree_data2_together_pub.publish(b)



if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("depth_to_tree", anonymous=True)
    rospy.Subscriber("/tree/trunk_info", Trunkset, cbTrunkset,queue_size=1, buff_size=2**24)
    tree_data2_together_pub=rospy.Publisher("/tree/data/together", Float64MultiArray,queue_size=1)
    rospy.spin()



    

    
