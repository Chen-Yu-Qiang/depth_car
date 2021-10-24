#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import depth2map

car_x=0
car_y=0
car_theta=0
map=depth2map.cerate_map()
car1=depth2map.trj()
AA=0
def cbDepth(data):
    global car_x,car_y,car_theta,map,AA
    t=time.time()
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    npPointX,npHeight,npDepth=depth2map.depth_to_3DPoint(image)
    tm=depth2map.get_tree_mask(npPointX,npHeight,npDepth)
    f_img=depth2map.img_mask(image,tm)
    z_depth, x_depth=depth2map.depth_filter(npPointX,npHeight,npDepth,tm)
    g = depth2map.depth_to_topView(z_depth, x_depth)

    centre_x_list,centre_y_list,radius_r_list, g2 = depth2map.topView_to_circle(g)

    centre_x_list,centre_y_list,radius_r_list = depth2map.circle_filter(centre_x_list,centre_y_list,radius_r_list)

    map=depth2map.into_world_map(map,z_depth, x_depth,car_x*1000,car_y*1000,car_theta-1.570796)
    map=depth2map.circle_to_world(map,centre_y_list,centre_x_list,radius_r_list,car_x*1000,car_y*1000,car_theta-1.570796)

    out_msg=bridge.cv2_to_imgmsg(f_img, '16UC1')
    out_msg.header=data.header
    tree_depth_pub.publish(out_msg)


    map_rgb=cv2.cvtColor(map*30, cv2.COLOR_GRAY2BGR)
    map_rgb=depth2map.draw_arrowed(car_x*1000,car_y*1000,car_theta,map_rgb)
    car1.add_car(car_x*1000,car_y*1000,car_theta)
    map_rgb=depth2map.draw_arrowed(car_x*1000,car_y*1000,car_theta,map_rgb)
    map_rgb=car1.add_img(map_rgb)    
    map_rgb=cv2.resize(map_rgb, (1000,1000))
    cv2.imshow("map",map_rgb)
    cv2.imshow("grid2",g2*255)
    cv2.imwrite("each/"+str(AA)+".png",g2*255)
    print(time.time()-t)
    cv2.waitKey(1)
    AA+=1


def cbIMU(msg):
    global car_theta
    car_theta=msg.vector.z*0.0174532
    
def cbOdom(msg):
    global car_x,car_y
    car_x=msg.pose.pose.position.x
    car_y=msg.pose.pose.position.y



if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("depth_to_map", anonymous=True)
    subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth,queue_size=1, buff_size=2**24)
    tree_depth_pub=rospy.Publisher("/camera_only_tree/depth/image_rect_raw", Image,queue_size=1)

    subIMU = rospy.Subscriber("/imu_filter/rpy/filtered", Vector3Stamped, cbIMU)
    subOdom = rospy.Subscriber("/outdoor_waypoint_nav/odometry/filtered_map", Odometry, cbOdom)
    rospy.spin()



    
