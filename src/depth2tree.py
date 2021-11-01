#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
import tf
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import depth2map

import numpy as np

def cbDepth(data):
    global car_x,car_y,car_theta
    t=time.time()
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    npPointX,npHeight,npDepth=depth2map.depth_to_3DPoint(image)

    tm=depth2map.get_tree_mask(npPointX,npHeight,npDepth)
    f_img=depth2map.img_mask(image,tm)
    _,contours, _ = cv2.findContours(tm.astype('uint8'), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(len(contours))
    m=np.zeros((100,200))
    for i in range(0, len(contours)):
        x, y, w, h = cv2.boundingRect(contours[i])
        # print(x,y,w,h)
        image=cv2.rectangle(image,(x,y),(x+w,y+h),1000,2)


        a_tree_depth=np.nanmean(npDepth[y:y+h,x:x+w])
        a_tree_X=np.nanmean(npPointX[y:y+h,x:x+w])
        th=np.arctan2(a_tree_X,a_tree_depth)/4.0*9.0
        r=np.sqrt(a_tree_X*a_tree_X+a_tree_depth*a_tree_depth)
        org_in_img_x=320
        org_in_img_y=480

        cv2.line(image, (org_in_img_x, org_in_img_y), (int(org_in_img_x-np.sin(th)*r/100), int(org_in_img_y-np.cos(th)*r/100)), 60000, 5)

        print(a_tree_X,a_tree_depth,r,th*57)


    cv2.imshow("map",image)
    cv2.waitKey(1)
    out_msg=bridge.cv2_to_imgmsg(f_img, '16UC1')
    out_msg.header=data.header
    tree_depth_pub.publish(out_msg)



if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("depth_filter_tree", anonymous=True)
    subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth,queue_size=1, buff_size=2**24)
    tree_depth_pub=rospy.Publisher("/camera_only_tree/depth/image_rect_raw", Image,queue_size=1)

    rospy.spin()



    
