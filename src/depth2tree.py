#!/usr/bin/env python

from numpy.lib.type_check import imag
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
from std_msgs.msg import Float64MultiArray,MultiArrayDimension
import depth2map
import max_like_tree

car_x=0
car_y=0
car_theta=0
map=depth2map.cerate_map()

car1=depth2map.trj()
AA=0
ARRAY_LAY1=20
def list2ROSmsg_rth(r_list,th_list,car_x,car_y,car_theta,AA):
    centre_x_list,centre_z_list,radius_r_list=depth2map.rth2xyr(r_list,th_list)
    NnW=depth2map.fromCar2World(centre_x_list,centre_z_list,car_x,car_y,car_theta)
    a=[0 for i in range(len(centre_x_list)*ARRAY_LAY1)]
    for i in range(len(centre_x_list)):
        a[i*ARRAY_LAY1]=centre_x_list[i]
        a[i*ARRAY_LAY1+1]=centre_z_list[i]
        a[i*ARRAY_LAY1+2]=radius_r_list[i]
        a[i*ARRAY_LAY1+3]=r_list[i]
        a[i*ARRAY_LAY1+4]=th_list[i]
        a[i*ARRAY_LAY1+5]=car_x
        a[i*ARRAY_LAY1+6]=car_y
        a[i*ARRAY_LAY1+7]=car_theta
        a[i*ARRAY_LAY1+8]=AA
        a[i*ARRAY_LAY1+9]=NnW[0,i]
        a[i*ARRAY_LAY1+10]=NnW[1,i]
    b=Float64MultiArray(data=a)
    
    b.layout.dim=[MultiArrayDimension()]
    b.layout.dim[0].stride=ARRAY_LAY1
    b.layout.dim[0].size=ARRAY_LAY1

    b.layout.dim[0].label="A_Tree"

    return b

def list2ROSmsg_dthr(d_list,th_list,r_list,car_x,car_y,car_theta,AA):
    centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
    NnW=depth2map.fromCar2World(centre_x_list,centre_z_list,car_x,car_y,car_theta)
    a=[0 for i in range(len(centre_x_list)*ARRAY_LAY1)]
    for i in range(len(centre_x_list)):
        a[i*ARRAY_LAY1]=centre_x_list[i]
        a[i*ARRAY_LAY1+1]=centre_z_list[i]
        a[i*ARRAY_LAY1+2]=radius_r_list[i]
        a[i*ARRAY_LAY1+3]=d_list[i]
        a[i*ARRAY_LAY1+4]=th_list[i]
        a[i*ARRAY_LAY1+5]=car_x
        a[i*ARRAY_LAY1+6]=car_y
        a[i*ARRAY_LAY1+7]=car_theta
        a[i*ARRAY_LAY1+8]=AA
        a[i*ARRAY_LAY1+9]=NnW[0,i]
        a[i*ARRAY_LAY1+10]=NnW[1,i]
    b=Float64MultiArray(data=a)
    
    b.layout.dim=[MultiArrayDimension()]
    b.layout.dim[0].stride=ARRAY_LAY1
    b.layout.dim[0].size=ARRAY_LAY1

    b.layout.dim[0].label="A_Tree"

    return b

def list2ROSmsg_dthr_each(d_list,th_list,r_list,car_x,car_y,car_theta,AA,puber):
    centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
    NnW=depth2map.fromCar2World(centre_x_list,centre_z_list,car_x*1000,car_y*1000,car_theta)
    tree_data_1900=[[2.5,12.5,0.5],[3.5,5.0,0.5],[4.0,-1.0,0.5],[9.0,10.0,0.5],[9.5,6.0,0.5],[10.0,3.0,0.5],[13.5,8.0,0.5]]
    tree_data_1726=[[-4.58,25.74,0.5],[-3.27,19.23,0.5],[-3.36,11.97,0.5],[2.9,23.56,0.5],[2.6,19.29,0.5],[3.15,16.94,0.5],[7.27,21.5,0.5]]
    tree_data_1726_utm=[[2767711.3, -352849.18, 0.5], [2767712.61, -352855.69, 0.5], [2767712.52, -352862.95, 0.5], [2767718.78, -352851.36, 0.5], [2767718.48, -352855.63, 0.5], [2767719.03, -352857.98, 0.5], [2767723.15, -352853.42, 0.5]]
    tree_data_1900_utm=[[2767710.38, -352850.58, 0.5], [2767711.38, -352858.08, 0.5], [2767711.88, -352864.08, 0.5], [2767716.88, -352853.08, 0.5], [2767717.38, -352857.08, 0.5], [2767717.88, -352860.08, 0.5], [2767721.38, -352855.08, 0.5]]

    tree_data=tree_data_1900_utm
    for i in range(len(centre_x_list)):
        a=[0 for j in range(ARRAY_LAY1)]
        a[0]=centre_x_list[i]*0.001
        a[1]=centre_z_list[i]*0.001
        a[2]=radius_r_list[i]*0.001
        a[3]=d_list[i]*0.001
        a[4]=th_list[i]
        a[5]=car_x
        a[6]=car_y
        a[7]=car_theta
        a[8]=AA
        a[9]=NnW[0,i]*0.001
        a[10]=NnW[1,i]*0.001
        like_max_i,like_max=max_like_tree.max_like(car_x,car_y,car_theta,tree_data,d_list[i]*0.001,th_list[i],radius_r_list[i]*0.001)
        a[11]=like_max_i+1
        a[12]=like_max
        
        b=Float64MultiArray(data=a)
        
        b.layout.dim=[MultiArrayDimension()]
        b.layout.dim[0].stride=ARRAY_LAY1
        b.layout.dim[0].size=ARRAY_LAY1

        b.layout.dim[0].label="A_Tree_each"
        puber.publish(b)



def list2ROSmsg_xzr(centre_x_list,centre_z_list,radius_r_list,car_x,car_y,car_theta,AA):
    NnW=depth2map.fromCar2World(centre_x_list,centre_z_list,car_x,car_y,car_theta)
    a=[0 for i in range(len(centre_x_list)*ARRAY_LAY1)]
    for i in range(len(centre_x_list)):
        a[i*ARRAY_LAY1]=centre_x_list[i]
        a[i*ARRAY_LAY1+1]=centre_z_list[i]
        a[i*ARRAY_LAY1+2]=radius_r_list[i]
        a[i*ARRAY_LAY1+3]=0
        a[i*ARRAY_LAY1+4]=0
        a[i*ARRAY_LAY1+5]=car_x
        a[i*ARRAY_LAY1+6]=car_y
        a[i*ARRAY_LAY1+7]=car_theta
        a[i*ARRAY_LAY1+8]=AA
        a[i*ARRAY_LAY1+9]=NnW[0,i]
        a[i*ARRAY_LAY1+10]=NnW[1,i]
    b=Float64MultiArray(data=a)
    
    b.layout.dim=[MultiArrayDimension()]
    b.layout.dim[0].stride=ARRAY_LAY1
    b.layout.dim[0].size=ARRAY_LAY1

    b.layout.dim[0].label="A_Tree"

    return b

def cbDepth_2(data):
    global car_x,car_y,car_theta,map,AA
    t=time.time()
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    npPointX,npHeight,npDepth=depth2map.depth_to_3DPoint(image)

    tm=depth2map.get_tree_mask(npPointX,npHeight,npDepth)
    f_img=depth2map.img_mask(image,tm)

    d_list,th_list,r_list,xywh_list,image=depth2map.depth_dir_tree_dthr(npPointX,npDepth,tm,image)

    for i in xywh_list:
        tree_data_image_pub.publish(Float64MultiArray(data=i))
    cv2.putText(image,str(int(car_x*1000)),(0,30), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    cv2.putText(image,str(int(car_y*1000)),(0,60), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    cv2.putText(image,str(int((car_theta)*57.3)),(0,90), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    for i in range(len(d_list)):
        cv2.putText(image,str(int(d_list[i]))+","+str(int((th_list[i])*57.3))+","+str(int(r_list[i])),(0,400+i*30), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    cv2.imshow("rth",image)
    centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
    centre_x_list,centre_z_list,radius_r_list = depth2map.circle_filter(centre_x_list,centre_z_list,radius_r_list)

    if len(centre_x_list)>=0:
        list2ROSmsg_dthr_each(d_list,th_list,r_list,car_x,car_y,car_theta,AA,tree_data2_pub)
    # map=depth2map.circle_to_world(map,centre_x_list,centre_z_list,radius_r_list,car_x*1000,car_y*1000,car_theta)

    out_msg=bridge.cv2_to_imgmsg(f_img, '16UC1')
    out_msg.header=data.header
    tree_depth_pub.publish(out_msg)

    # print(time.time()-t)
    cv2.waitKey(1)

    
    AA+=1



    
def cbOdom(msg):
    global car_x,car_y,car_theta
    car_x=msg.pose.pose.position.x
    car_y=msg.pose.pose.position.y
    z=tf.transformations.euler_from_quaternion([0,0,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    car_theta=z[2]
    # print(car_x,car_y,car_theta*57.3)


if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("depth_to_tree", anonymous=True)
    subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth_2,queue_size=1, buff_size=2**24)
    tree_depth_pub=rospy.Publisher("/camera_only_tree/depth/image_rect_raw", Image,queue_size=1)
    # tree_data_pub=rospy.Publisher("/tree_data", Float64MultiArray,queue_size=1)
    tree_data2_pub=rospy.Publisher("/tree_data2", Float64MultiArray,queue_size=1)
    tree_data_image_pub=rospy.Publisher("/tree_image_data", Float64MultiArray,queue_size=1)




    # subIMU = rospy.Subscriber("/imu_filter/rpy/filtered", Vector3Stamped, cbIMU)
    # subOdom = rospy.Subscriber("/my_filtered_map", Odometry, cbOdom)
    # subOdom = rospy.Subscriber("/landmark_odom", Odometry, cbOdom)
    subOdom = rospy.Subscriber("/outdoor_waypoint_nav/odometry/filtered_map", Odometry, cbOdom)
    rospy.spin()



    

    
