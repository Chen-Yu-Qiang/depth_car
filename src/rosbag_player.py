#!/usr/bin/env wowpython

import rosbag
import rospy
from std_msgs.msg import String
from mapping_explorer.msg import Trunkset, Trunkinfo, Correspondence
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import time

rospy.init_node("rosbag_player_node", anonymous=True)
s_pub=rospy.Publisher("/scan_filtered", LaserScan, queue_size=1)
t_pub=rospy.Publisher("/tree/trunk_info", Trunkset, queue_size=1)
o_pub=rospy.Publisher("/odom", Odometry, queue_size=1)

print("Start read bag")
bag_file="/media/yuqiang/2CFC30D1FC309754/research data/1110222/2022-02-20-15-50-19.bag"

bag = rosbag.Bag(bag_file, 'r')
new_msgs = [{
            'topic' : topic,
            'msg' : msg,
            'time' : t.to_sec()
        } for topic, msg, t in bag.read_messages()]

print("Finish read bag")

t0=new_msgs[0]['time']
delta_t=time.time()-t0

print("Start play")
for i in new_msgs:
    if i['topic']=='/scan_filtered':
        s_pub.publish(i['msg'])
        # time.sleep(0.2)
    if i['topic']=='/tree/trunk_info':
        a=Trunkset()
        for j in i['msg'].aframe:
            a.aframe.append(j)
        t_pub.publish(a)
        time.sleep(0.2)
    
    if i['topic']=='/odom':
        o_pub.publish(i['msg'])
    

print("End play")