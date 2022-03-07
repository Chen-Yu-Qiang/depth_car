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
now_x,now_y,now_th=0,0,0
def cb_pose(data):
    global now_x,now_y,now_th
    now_x=data.position.x
    now_y=data.position.y
    now_th=tf.transformations.euler_from_quaternion([0,0,data.orientation.z,data.orientation.w])[2]

goal_x=10
goal_y=10

def cb_goalPoint(data):
    global goal_x,goal_y,s
    goal_x=data.pose.position.x
    goal_y=data.pose.position.y
    if s==2:
        s=2
    else:
        s=1

if __name__ == '__main__':
    rospy.init_node('g_controller_node')

    rospy.Subscriber("/sim/robot", Pose,cb_pose,queue_size=1, buff_size=2**20)
    rospy.Subscriber("/ctrl/path/g", PoseStamped,cb_goalPoint,queue_size=1, buff_size=2**20)
    cmd_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
    pid_v_pub=rospy.Publisher("/ctrl/pid_v",PidState,queue_size=1)
    pid_omg_pub=rospy.Publisher("/ctrl/pid_omg",PidState,queue_size=1)

    pid_v=pid_class.pid_controller()
    pid_omg=pid_class.pid_controller()
    pid_omg.output_max=1.0
    pid_omg.output_min=-1.0
    pid_v.output_max=0.6
    pid_v.output_min=0.3
    pid_v.output_zero=0.01


    error_pub=rospy.Publisher("/ctrl/error",Twist,queue_size=1)
    rate=rospy.Rate(20)
    time.sleep(15)
    print("[gazebo_controller.py] Start control")
    
    while not rospy.is_shutdown():

        dis=np.sqrt((goal_y-now_y)**2+(goal_x-now_x)**2)
        ang=np.arctan2((goal_y-now_y),(goal_x-now_x))-now_th
        ang=(ang+np.pi)%(2.0*np.pi)-np.pi
        if (np.cos(ang)<=0):
            dis=dis*(-1.0)


        # s=0  Standby
        # s=1  Get a New Way Point and Rotating
        # s=2  Reach the Direction and Moving

        if s==0:
            rospy.wait_for_message("/ctrl/wp/g",PoseStamped)

        if s==1 and abs(ang)>0.1:
            pid_omg.setpid(2,0,0)
            pid_v.setpid(0,0,0)
        elif s==1 and abs(ang)<=0.1:
            pid_omg.setpid(0,0,0)
            pid_v.setpid(0,0,0)
            s=2
        elif s==2 and (dis > 0.1) and (np.cos(ang)>0):
            pid_omg.setpid(0.5,0,0)
            pid_v.setpid(1,0,0)
        elif s==2 and ((dis< 0.1) or (np.cos(ang)<=0)):
            pid_omg.setpid(0,0,0)
            pid_v.setpid(0,0,0)
            s=0
        if s==0:
            pid_omg.setpid(0,0,0)
            pid_v.setpid(0,0,0)

        error_msg=Twist()
        error_msg.linear.x=goal_x-now_x
        error_msg.linear.y=goal_y-now_y
        error_msg.linear.z=dis
        error_msg.angular.z=ang
        error_msg.angular.x=s
        error_pub.publish(error_msg)

        pid_omg_msg=pid_omg.update(ang)
        pid_v_msg=pid_v.update(dis)
        pid_v_pub.publish(pid_v_msg)
        pid_omg_pub.publish(pid_omg_msg)

        cmd_msg=Twist()
        cmd_msg.angular.z=pid_omg_msg.output
        cmd_msg.linear.x=pid_v_msg.output

        cmd_pub.publish(cmd_msg)
        # print(round(goal_y,3),round(now_y,3),round(goal_x,3),round(now_x,3),round(dis,3),ang,3),round(cmd_msg.linear.x,3),round(cmd_msg.angular.z)

        rate.sleep()

    cmd_msg=Twist()    
    cmd_msg.linear.x=0
    cmd_msg.angular.z=0
    cmd_pub.publish(cmd_msg)