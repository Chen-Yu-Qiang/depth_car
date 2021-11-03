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
    r_list,th_list=depth2map.depth_dir_tree_rth(npPointX,npDepth,tm)
    centre_x_list,centre_y_list,radius_r_list=depth2map.rth2xyr(r_list,th_list)
    
    
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



    
