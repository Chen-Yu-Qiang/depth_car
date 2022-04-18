#!/usr/bin/wowpython

import time
import rospy
from geometry_msgs.msg import Twist,PoseStamped,Pose
from control_msgs.msg import PidState
from std_msgs.msg import UInt8
import numpy as np
import pid_class
import tf
s=0
now_x,now_y=0,0
goal_x,goal_y=0,0
delta_x,delta_y=0,0
WALK_SPEED=0.5
CMD_SPEED=20
num=-1

def cb_goalPoint(data):
    global goal_x,goal_y,now_x,now_y,delta_x,delta_y,num
    goal_x=data.pose.position.x
    goal_y=data.pose.position.y
    now_point=rospy.wait_for_message("/sim/robot", Pose)
    now_x=now_point.position.x
    now_y=now_point.position.y
    L=np.sqrt((goal_x-now_x)**2+(goal_y-now_y)**2)
    num=int((L/WALK_SPEED)*CMD_SPEED)
    num=max(1,num)
    delta_x=(goal_x-now_x)/num
    delta_y=(goal_y-now_y)/num
    print(goal_x,goal_y,now_x,now_y,delta_x,delta_y,num)

if __name__ == '__main__':
    rospy.init_node('g_way2path_node')
    rospy.Subscriber("/ctrl/wp/g", PoseStamped,cb_goalPoint,queue_size=1, buff_size=2**20)
    path_pub=rospy.Publisher("/ctrl/path/g",PoseStamped,queue_size=1)
    achieveGoal_pub=rospy.Publisher('/ctrl/achieve0', UInt8,queue_size=1) 
    rate=rospy.Rate(CMD_SPEED)
    time.sleep(15)
    print("[gazebo_waypoint2path.py] Start control")
    
    while not rospy.is_shutdown():
        if num>0:
            path_msg=PoseStamped()
            path_msg.pose.position.x=goal_x-1.0*num*delta_x
            path_msg.pose.position.y=goal_y-1.0*num*delta_y
            num=num-1
        elif num==0:

            achieveGoal_msg=UInt8()
            achieveGoal_msg.data=1
            achieveGoal_pub.publish(achieveGoal_msg)
            num=num-1
        else:
            path_msg=PoseStamped()
            path_msg.pose.position.x=goal_x
            path_msg.pose.position.y=goal_y

        path_pub.publish(path_msg)
        
        rate.sleep()

