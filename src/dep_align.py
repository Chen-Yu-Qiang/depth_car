#!/usr/bin/env python
from logging import info

import sys
import time
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from numpy.core.numeric import NaN
from scipy.sparse import csr_matrix
from scipy.interpolate import griddata
import numpy as np
import matplotlib.pyplot as plt

import cv2
grid_x, grid_y = np.mgrid[0:480, 0:640]
def align_dep(npDepth):
    t=time.time()
    cx_d = 320.6562194824219 #424
    cy_d = 241.57083129882812 #241
    fx_d = 384.31365966796875 #424
    fy_d = 384.31365966796875 #424

    fxc = 616.811767578125
    fyc = 616.7999267578125
    cxc = 327.9185485839844
    cyc = 241.32443237304688
    depth_to_color_extrinsics = np.array( [[0.9999837279319763, -0.005083103198558092, -0.0025966160465031862],
                                            [0.005084178410470486, 0.999987006187439, 0.000407703424571082],
                                            [0.0025945098605006933, -0.000420898461015895, 0.999996542930603],
                                            [0.014714013785123825, 0.00010518113413127139, 0.0004244096053298563]] )
    depth_to_color_extrinsics = depth_to_color_extrinsics.T
    row, column = npDepth.shape

    npPointX = np.asarray(range(640))-cx_d
    npPointX = np.diag(npPointX)
    npPointX = npDepth.dot(npPointX)/ fx_d
    npPointX = npPointX.astype('float64')

    npPointY = np.asarray(range(480))-cy_d
    npPointY = np.diag(npPointY)
    theta = 0/180*np.pi
    npPointY = npPointY.dot(npDepth)/ fy_d
    npPointY = npPointY*np.cos(theta) + npDepth * np.sin(theta)
    npPointY = npPointY.astype('float64')

    mask=np.ones((307200,))
    x_depth=np.reshape(npPointX,(307200,))
    y_depth=np.reshape(npPointY,(307200,))
    z_depth=np.reshape(npDepth,(307200,))
    xyz1=np.array([x_depth,y_depth,z_depth,np.ones((307200,))])
    world_color=np.matmul(depth_to_color_extrinsics,xyz1)
    uc=world_color[0,:]/world_color[2,:]*fxc+cxc
    vc=world_color[1,:]/world_color[2,:]*fyc+cyc
    val=world_color[2,:]
    
    mask[uc>=640]=0
    mask[uc<0]=0
    mask[vc>=480]=0
    mask[vc<0]=0
    mask[val<10]=0
    mask[val>30000]=0

    uc=uc[~(mask==0)]
    vc=vc[~(mask==0)]

    val=val[~(mask==0)]

    t2=time.time()

    # No interpolation
    vc=vc.astype("uint16")
    uc=uc.astype("uint16")
    m=csr_matrix((val,(vc,uc)))
    m=m.toarray()

    # Yes Interpolation    
    # m = griddata(np.asarray([vc,uc]).T, val, (grid_x, grid_y), method='nearest')

    print(time.time()-t2,t2-t)

    return m.astype('uint16')


def cbDepth(data):

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    after_image=align_dep(image)

    out_msg=bridge.cv2_to_imgmsg(after_image, '16UC1')
    out_msg.header=data.header
    pub.publish(out_msg)

if __name__=="__main__":
    # AA=500
    # file_path = "/home/yuqiang/catkin_car/src/depth_car/src/syn_rosbag1/"
    # npDepth = np.load(file_path+"depth/"+str(AA)+".npy")
    # npColor = np.load(file_path+"color/"+str(AA)+".npy")
    # align_depth_to_color=align_dep(npDepth)
    # plt.figure()
    # plt.imshow(npColor)
    # plt.show(block=False)
    # plt.figure()
    # plt.imshow(npDepth)
    # plt.show(block=False)
    # plt.figure()
    # plt.imshow(align_depth_to_color)
    # plt.show()


    print("Python version: ",sys.version)
    rospy.init_node("depth_align", anonymous=True)
    subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth,queue_size=1, buff_size=2**24)
    pub=rospy.Publisher("/camera/depth_aligned/image_rect_raw", Image,queue_size=1)
    rospy.spin()