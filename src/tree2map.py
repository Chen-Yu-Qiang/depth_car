#!/usr/bin/wowpython

import rospy
import sys
import tf
import cv2
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray,MultiArrayDimension
from geometry_msgs.msg import Twist
import depth2map
import max_like_tree

car_x=0
car_y=0
car_theta=0
MAP_OFFSET_X=2767690
MAP_OFFSET_Y=-352874

car_x2=0
car_y2=0
car_theta2=0
# map=depth2map.cerate_map()

# car1=depth2map.trj()
# car2=depth2map.trj()
ARRAY_LAY1=20
AA=0

def cb_tree_each(data):
    global car_x,car_y,car_theta,map
    t=time.time()
    data=list(data.data)
    r_list=[data[2]]
    th_list=[data[4]]
    d_list=[data[3]*1000]
    centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
    # print(centre_x_list,centre_z_list,radius_r_list)
    NnW=depth2map.fromCar2World(centre_x_list,centre_z_list,car_x*1000,car_y*1000,car_theta)
    # tree_data_1900=[[2.5,12.5,0.5],[3.5,5.0,0.5],[4.0,-1.0,0.5],[9.0,10.0,0.5],[9.5,6.0,0.5],[10.0,3.0,0.5],[13.5,8.0,0.5]]
    # tree_data_1726=[[-4.58,25.74,0.5],[-3.27,19.23,0.5],[-3.36,11.97,0.5],[2.9,23.56,0.5],[2.6,19.29,0.5],[3.15,16.94,0.5],[7.27,21.5,0.5]]
    # tree_data=tree_data_1900
    for i in range(len(centre_x_list)):
        a=data
        a[5]=car_x
        a[6]=car_y
        a[7]=car_theta
        a[12]=NnW[0,i]*0.001
        a[13]=NnW[1,i]*0.001
        # like_max_i,like_max=max_like_tree.max_like(car_x,car_y,car_theta,tree_data,d_list[i]*0.001,th_list[i],radius_r_list[i]*0.001)
        # a[11]=like_max_i+1
        # a[12]=like_max
        
        b=Float64MultiArray(data=a)
        
        b.layout.dim=[MultiArrayDimension()]
        b.layout.dim[0].stride=ARRAY_LAY1
        b.layout.dim[0].size=ARRAY_LAY1

        b.layout.dim[0].label="A_Tree_each"
        tree_data_each_pub.publish(b)
    
    # map=depth2map.circle_to_world(map,centre_x_list,centre_z_list,radius_r_list,car_x*1000,car_y*1000,car_theta)

    # print("2",time.time()-t)

def cb_tree_together(data):
    global car_x,car_y,car_theta,map
    t=time.time()
    data=list(data.data)
    n=int(len(data)/20)
    r_list=[]
    th_list=[]
    d_list=[]
    for i in range(n):
        r_list.append(data[ARRAY_LAY1*i+2])
        th_list.append(data[ARRAY_LAY1*i+4])
        d_list.append(data[ARRAY_LAY1*i+3]*1000)
    centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
    # print(centre_x_list,centre_z_list,radius_r_list)
    NnW=depth2map.fromCar2World(centre_x_list,centre_z_list,car_x*1000,car_y*1000,car_theta)
    # tree_data_1900=[[2.5,12.5,0.5],[3.5,5.0,0.5],[4.0,-1.0,0.5],[9.0,10.0,0.5],[9.5,6.0,0.5],[10.0,3.0,0.5],[13.5,8.0,0.5]]
    # tree_data_1726=[[-4.58,25.74,0.5],[-3.27,19.23,0.5],[-3.36,11.97,0.5],[2.9,23.56,0.5],[2.6,19.29,0.5],[3.15,16.94,0.5],[7.27,21.5,0.5]]
    # tree_data=tree_data_1900
    for i in range(len(centre_x_list)):
        data[ARRAY_LAY1*i+5]=car_x
        data[ARRAY_LAY1*i+6]=car_y
        data[ARRAY_LAY1*i+7]=car_theta
        data[ARRAY_LAY1*i+12]=NnW[0,i]*0.001
        data[ARRAY_LAY1*i+13]=NnW[1,i]*0.001
        # like_max_i,like_max=max_like_tree.max_like(car_x,car_y,car_theta,tree_data,d_list[i]*0.001,th_list[i],radius_r_list[i]*0.001)
        # a[11]=like_max_i+1
        # a[12]=like_max
        
    b=Float64MultiArray(data=data)
    
    b.layout.dim=[MultiArrayDimension()]
    b.layout.dim[0].stride=ARRAY_LAY1
    b.layout.dim[0].size=ARRAY_LAY1

    b.layout.dim[0].label="A_Tree_each"
    tree_data_together_pub.publish(b)
    
    # map=depth2map.circle_to_world(map,centre_x_list,centre_z_list,radius_r_list,car_x*1000,car_y*1000,car_theta)

    # print("2",time.time()-t)
    



def cbOdom(msg):
    global car_x,car_y,car_theta,map,AA
    # t=time.time()
    car_x=msg.pose.pose.position.x
    car_y=msg.pose.pose.position.y
    z=tf.transformations.euler_from_quaternion([0,0,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    car_theta=z[2]
    # print(car_x,car_y,car_theta*57.3)



    # print("3",time.time()-t)


def cbTwist(msg):
    global car_x,car_y,car_theta,map,AA
    # t=time.time()
    car_x=msg.linear.x
    car_y=msg.linear.y
    car_theta=msg.angular.z

def cbOdom2(msg):
    global car_x2,car_y2,car_theta2
    t=time.time()
    car_x2=msg.pose.pose.position.x
    car_y2=msg.pose.pose.position.y
    z=tf.transformations.euler_from_quaternion([0,0,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    car_theta2=z[2]
    # print(car_x,car_y,car_theta*57.3)

    print("3",time.time()-t)

ARRAY_LAY1=20
ARRAY_LAY2=40
def cb_landmark_z(data):
    global car_x,car_y,car_theta,map
    data=list(data.data)
    n=int(len(data)/ARRAY_LAY2)
    d_list=[]
    th_list=[]
    r_list=[]
    for i in range(n):
        if data[ARRAY_LAY2*i]<0:
            continue
        d_list.append(data[ARRAY_LAY2*i+2]*1000)
        th_list.append(data[ARRAY_LAY2*i+3])
        r_list.append(data[ARRAY_LAY2*i+4])
    
    # car_x=data[13]
    # car_x=data[14]
    # car_theta=data[15]
    car_x_loc=car_x-MAP_OFFSET_X
    car_y_loc=car_y-MAP_OFFSET_Y
    centre_x_list,centre_z_list,radius_r_list=depth2map.dthr2xyr(d_list,th_list,r_list)
    # map=depth2map.circle_to_world(map,centre_x_list,centre_z_list,radius_r_list,car_x_loc*1000,car_y_loc*1000,car_theta)    

def cb_landmark_xz_z(data):
    global car_x,car_y,car_theta,map
    data=list(data.data)
    if data[0]<0:
        return
    centre_x_list=[data[2]*1000]
    centre_z_list=[data[3]*1000]
    radius_r_list=[0.2*1000]
    car_x_loc=car_x-MAP_OFFSET_X
    car_y_loc=car_y-MAP_OFFSET_Y
    # map=depth2map.circle_to_world(map,centre_x_list,centre_z_list,radius_r_list,car_x_loc*1000,car_y_loc*1000,car_theta)    

if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("tree_to_map", anonymous=True)
    tree_data_sub=rospy.Subscriber("/tree_data2_each", Float64MultiArray,cb_tree_each,queue_size=1)
    tree_data_sub=rospy.Subscriber("/tree_data2_together", Float64MultiArray,cb_tree_together,queue_size=1)
    # landmark_z_sub=rospy.Subscriber("/landmark_z", Float64MultiArray,cb_landmark_z,queue_size=1)
    tree_data_each_pub=rospy.Publisher("/tree_data_each", Float64MultiArray,queue_size=1)
    tree_data_together_pub=rospy.Publisher("/tree_data_together", Float64MultiArray,queue_size=1)

    # subOdom = rospy.Subscriber("/landmark_odom", Odometry, cbOdom)
    subTwist = rospy.Subscriber("/landmark_filtered_offset_local", Twist, cbTwist)
    # subOdom = rospy.Subscriber("/outdoor_waypoint_nav/odometry/filtered_map", Odometry, cbOdom2)

    # rate=rospy.Rate(5)

    while not rospy.is_shutdown():
        
        t=time.time()
        car_x_loc=car_x-MAP_OFFSET_X
        car_y_loc=car_y-MAP_OFFSET_Y

        # map_rgb=cv2.cvtColor(map*30, cv2.COLOR_GRAY2BGR)
        # map_rgb=depth2map.draw_arrowed(car_x_loc*1000,car_y_loc*1000,car_theta,map_rgb)
        # car1.add_car(car_x_loc*1000,car_y_loc*1000,car_theta)

        # map_rgb=car1.add_img(map_rgb)
        
        
        # map_rgb_small=cv2.resize(map_rgb, (1000,1000))
        # cv2.imshow("map",map_rgb_small)
        # if AA%30==0:
        #     cv2.imwrite("map/"+str(AA)+".png",map_rgb)
        # if AA%5==0:
        #     cv2.imwrite("each/"+str(AA)+".png",image)     

        # cv2.waitKey(1)

        
        AA+=1
        # print("4",time.time()-t)
        # rate.sleep()
        rospy.sleep(0.2)

    rospy.spin()

    
