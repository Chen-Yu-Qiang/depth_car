#!/usr/bin/wowpython

import time
import rospy
from geometry_msgs.msg import Twist,PoseStamped
import numpy as np

now_x,now_y,now_th=0,0,0
def cb_lm(data):
    global now_x,now_y,now_th
    now_x=data.linear.x
    now_y=data.linear.y
    now_th=data.angular.z

goal_x=2767672.7993394216
goal_y=-352852.9162779033

def cb_goalPoint(data):
    global goal_x,goal_y
    goal_x=data.pose.position.y
    goal_y=data.pose.position.x*(-1.0)

if __name__ == '__main__':
    rospy.init_node('controller_node')
    rospy.Subscriber("/landmark", Twist,cb_lm,queue_size=1, buff_size=2**20)
    rospy.Subscriber("/wow_utm_waypoint", PoseStamped,cb_goalPoint,queue_size=1, buff_size=2**20)
    cmd_pub=rospy.Publisher("/husky_velocity_controller/cmd_vel2",Twist,queue_size=1)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        v_kp=0.1
        v_max=0.6
        v_min=0.3
        omg_kp=0.5
        omg_max=1
        dis=np.sqrt((goal_y-now_y)**2+(goal_x-now_x)**2)
        ang=np.arctan2((goal_y-now_y),(goal_x-now_x))-now_th
        ang=(ang+np.pi)%(2.0*np.pi)-np.pi



        cmd_msg=Twist()

        if abs(ang)<0.1:
            if (v_kp*dis)>0.05:
                cmd_msg.linear.x=max(min((v_kp*dis),v_max),v_min)
            elif (v_kp*dis)<-0.05:
                cmd_msg.linear.x=max(min((v_kp*dis),(-1.0)*v_min),(-1.0)*v_max)
            else:
                cmd_msg.linear.x=0
        else:
            cmd_msg.linear.x=0

        if (omg_kp*ang)>0:
            cmd_msg.angular.z=min(omg_kp*ang,omg_max)
        else:
            cmd_msg.angular.z=max(omg_kp*ang,(-1.0)*omg_max)

        cmd_pub.publish(cmd_msg)
        print(goal_y,now_y,goal_x,now_x,dis,ang,cmd_msg.linear.x,cmd_msg.angular.z)

        rate.sleep()
