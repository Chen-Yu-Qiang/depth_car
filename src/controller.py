#!/usr/bin/wowpython

import time
import rospy
from geometry_msgs.msg import Twist
import numpy as np

now_x,now_y,now_th=0,0,0
def cb_lm(data):
    global now_x,now_y,now_th
    now_x=data.linear.x
    now_y=data.linear.y
    now_th=data.angular.z

goal_x,goal_y=0,0,0
def cb_goalPoint(data):
    global goal_x,goal_y
    goal_x=0
    goal_y=0

if __name__ == '__main__':
    rospy.init_node('controller_node')
    rospy.Subscriber("/landmark", Twist,cb_lm,queue_size=1, buff_size=2**20)
    rospy.Subscriber("/cb_goalPoint", Twist,cb_goalPoint,queue_size=1, buff_size=2**20)
    cmd_pub=rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist,queue_size=1)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        v_kp=1
        v_max=1
        v_min=0.3
        omg_kp=1
        omg_max=1

        dis=np.sqrt((goal_y-now_y)**2+(goal_y-now_y)**2)
        ang=np.arctan2((goal_y-now_y),(goal_y-now_y))-now_th

        cmd_msg=Twist()

        
        if (v_kp*dis)>0.05:
            cmd_msg.linear.x=max(min((v_kp*dis),v_max),v_min)
        elif (v_kp*dis)<-0.05:
            cmd_msg.linear.x=max(min((v_kp*dis),(-1.0)*v_min),(-1.0)*v_max)
        else:
            cmd_msg.linear.x=0

        if (omg_kp*ang)>0:
            cmd_msg.angular.z=min(omg_kp*ang,omg_max)
        else:
            cmd_msg.angular.z=max(omg_kp*ang,(-1.0)*omg_max)

        cmd_pub.publish(cmd_msg)

        rate.sleep()
