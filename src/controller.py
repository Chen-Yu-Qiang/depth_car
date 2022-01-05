#!/usr/bin/wowpython

import time
import rospy
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import UInt8
import numpy as np
s=0
now_x,now_y,now_th=0,0,0
def cb_lm(data):
    global now_x,now_y,now_th
    now_x=data.linear.x
    now_y=data.linear.y
    now_th=data.angular.z

goal_x=2767672.7993394216
goal_y=-352852.9162779033

def cb_goalPoint(data):
    global goal_x,goal_y,s
    goal_x=data.pose.position.y
    goal_y=data.pose.position.x*(-1.0)
    s=1

if __name__ == '__main__':
    rospy.init_node('controller_node')
    rospy.Subscriber("/landmark_filtered_offset", Twist,cb_lm,queue_size=1, buff_size=2**20)
    rospy.Subscriber("/wow_utm_waypoint", PoseStamped,cb_goalPoint,queue_size=1, buff_size=2**20)
    cmd_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)

    error_pub=rospy.Publisher("lm/d_t_error",Twist,queue_size=1)
    achieveGoal_pub=rospy.Publisher('/wow/achieveGoal', UInt8,queue_size=1) 
    rate=rospy.Rate(10)
    
    while not rospy.is_shutdown():
        v_kp=0.1
        v_max=0.5
        v_min=0.3
        omg_kp=6
        omg_max=0.5
        dis=np.sqrt((goal_y-now_y)**2+(goal_x-now_x)**2)
        ang=np.arctan2((goal_y-now_y),(goal_x-now_x))-now_th
        ang=(ang+np.pi)%(2.0*np.pi)-np.pi

        # s=0  Standby
        # s=1  Get a New Way Point and Rotating
        # s=2  Reach the Direction and Moving

        cmd_msg=Twist()

        if s==1 and abs(ang)>0.05:
            if (omg_kp*ang)>0:
                cmd_msg.angular.z=min(omg_kp*ang,omg_max)
            else:
                cmd_msg.angular.z=max(omg_kp*ang,(-1.0)*omg_max)
            cmd_msg.linear.x=0
        elif s==1 and abs(ang)<=0.1:
            cmd_msg.angular.z=0
            s=2
        
        elif s==2 and (dis > 0.1) and (np.cos(ang)>0):
            if (v_kp*dis)>0.001:
                cmd_msg.linear.x=max(min((v_kp*dis),v_max),v_min)
            elif (v_kp*dis)<-0.001:
                cmd_msg.linear.x=max(min((v_kp*dis),(-1.0)*v_min),(-1.0)*v_max)
            else:
                cmd_msg.linear.x=0

            if (omg_kp*ang)>0:
                cmd_msg.angular.z=min(omg_kp*ang,omg_max)
            else:
                cmd_msg.angular.z=max(omg_kp*ang,(-1.0)*omg_max)


        elif s==2 and ((dis< 0.1) or (np.cos(ang)<=0)):
            cmd_msg.linear.x=0
            cmd_msg.angular.z=0
            achieveGoal_msg=UInt8()
            achieveGoal_msg.data=1
            achieveGoal_pub.publish(achieveGoal_msg)
            s=0
        else:
            cmd_msg.linear.x=0
            cmd_msg.angular.z=0

        error_msg=Twist()
        error_msg.linear.x=goal_x-now_x
        error_msg.linear.y=goal_y-now_y
        error_msg.linear.z=dis
        error_msg.angular.z=ang
        error_msg.angular.x=s
        error_pub.publish(error_msg)

        cmd_pub.publish(cmd_msg)
        print(round(goal_y,3),round(now_y,3),round(goal_x,3),round(now_x,3),round(dis,3),ang,3),round(cmd_msg.linear.x,3),round(cmd_msg.angular.z)

        rate.sleep()

        
    cmd_msg.linear.x=0
    cmd_msg.angular.z=0
    cmd_pub.publish(cmd_msg)