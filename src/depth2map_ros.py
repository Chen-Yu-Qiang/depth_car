#!/usr/bin/wowpython

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
import EKF_localization
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
    tree_data=tree_data_1900
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

    z_depth, x_depth=depth2map.depth_filter(npPointX,npHeight,npDepth,tm,800,1200)


    map=depth2map.into_world_map(map,z_depth, x_depth,car_x*1000,car_y*1000,car_theta)



    # g = depth2map.depth_to_topView(z_depth, x_depth)
    # centre_x_list2,centre_z_list2,radius_r_list2, g2 = depth2map.topView_to_circle(g)
    # centre_x_list2,centre_z_list2,radius_r_list2 = depth2map.circle_filter(centre_x_list2,centre_z_list2,radius_r_list2)

    # map=depth2map.circle_to_world(map,centre_x_list2,centre_z_list2,radius_r_list2,car_x*1000,car_y*1000,car_theta)
    # cv2.imshow("grid2",g2*255)
    # if len(centre_x_list2)>=0:
    #     tree_data_msg=list2ROSmsg_xzr(centre_x_list2,centre_z_list2,radius_r_list2,car_x*1000,car_y*1000,car_theta,AA)
    #     tree_data.publish(tree_data_msg)

    d_list,th_list,r_list,xywh_list,image=depth2map.depth_dir_tree_dthr(npPointX,npDepth,tm,image)
    cv2.putText(image,str(int(car_x*1000)),(0,30), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    cv2.putText(image,str(int(car_y*1000)),(0,60), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    cv2.putText(image,str(int((car_theta)*57.3)),(0,90), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    for i in range(len(d_list)):
        cv2.putText(image,str(int(d_list[i]))+","+str(int((th_list[i])*57.3))+","+str(int(r_list[i])),(0,400+i*30), cv2.FONT_HERSHEY_SIMPLEX,1, 60000, 1, cv2.LINE_AA)
    cv2.imshow("rth",image)
    centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
    centre_x_list,centre_z_list,radius_r_list = depth2map.circle_filter(centre_x_list,centre_z_list,radius_r_list)

    if len(centre_x_list)>=0:
        list2ROSmsg_dthr_each(d_list,th_list,r_list,car_x,car_y,car_theta,AA,tree_data2)
    # map=depth2map.circle_to_world(map,centre_x_list,centre_z_list,radius_r_list,car_x*1000,car_y*1000,car_theta)

    out_msg=bridge.cv2_to_imgmsg(f_img, '16UC1')
    out_msg.header=data.header
    tree_depth_pub.publish(out_msg)


    map_rgb=cv2.cvtColor(map*30, cv2.COLOR_GRAY2BGR)
    map_rgb=depth2map.draw_arrowed(car_x*1000,car_y*1000,car_theta,map_rgb)
    car1.add_car(car_x*1000,car_y*1000,car_theta)
    map_rgb=depth2map.draw_arrowed(car_x*1000,car_y*1000,car_theta,map_rgb)
    map_rgb=car1.add_img(map_rgb)
    
    
    map_rgb_small=cv2.resize(map_rgb, (1000,1000))
    cv2.imshow("map",map_rgb_small)
    
    
    # if AA%30==0:
    #     cv2.imwrite("map/"+str(AA)+".png",map_rgb)
    # if AA%5==0:
    #     cv2.imwrite("each/"+str(AA)+".png",image)     
    print(time.time()-t)
    cv2.waitKey(1)

    
    AA+=1




# def cbIMU(msg):
#     global car_theta
#     car_theta=msg.vector.z*0.0174532
    
def cbOdom(msg):
    global car_x,car_y,car_theta
    car_x=msg.pose.pose.position.x
    car_y=msg.pose.pose.position.y
    z=tf.transformations.euler_from_quaternion([0,0,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    car_theta=z[2]
    # print(car_x,car_y,car_theta*57.3)


if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("depth_to_map", anonymous=True)
    subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth_2,queue_size=1, buff_size=2**24)
    tree_depth_pub=rospy.Publisher("/camera_only_tree/depth/image_rect_raw", Image,queue_size=1)
    tree_data=rospy.Publisher("/tree_data", Float64MultiArray,queue_size=1)
    tree_data2=rospy.Publisher("/tree_data2", Float64MultiArray,queue_size=1)

    # subIMU = rospy.Subscriber("/imu_filter/rpy/filtered", Vector3Stamped, cbIMU)
    # subOdom = rospy.Subscriber("/my_filtered_map", Odometry, cbOdom)
    subOdom = rospy.Subscriber("/landmark_odom", Odometry, cbOdom)

    
    # subOdom = rospy.Subscriber("/outdoor_waypoint_nav/odometry/filtered_map", Odometry, cbOdom)
    rospy.spin()



    
