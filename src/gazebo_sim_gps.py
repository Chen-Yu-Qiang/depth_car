#!/usr/bin/wowpython


import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import tf
import random

gt=Pose()
def cb(data):
    global gt
    gt=data
if __name__=='__main__':
    rospy.init_node("gazebo_sim_gps", anonymous=True)
    gps1_pub=rospy.Publisher("/outdoor_waypoint_nav/odometry/filtered_map", Odometry,queue_size=1)
    gps2_pub=rospy.Publisher("/lm_ekf/gps/utm", Twist,queue_size=1)
    gps3_pub=rospy.Publisher("/lm_ekf/gps/local", Twist,queue_size=1)
    rospy.Subscriber("/sim/robot",Pose,cb,queue_size=1)
    rate=rospy.Rate(20)


    while not rospy.is_shutdown():

        x=gt.position.x
        y=gt.position.y
        th=tf.transformations.euler_from_quaternion([0,0,gt.orientation.z,gt.orientation.w])[2] 
        gps1_msg=Odometry()
        gps1_msg.pose.pose.position.x=x
        gps1_msg.pose.pose.position.y=y
        gps1_msg.pose.pose.orientation=gt.orientation

        gps1_pub.publish(gps1_msg)
        
        gps2_msg=Twist()
        gps2_msg.linear.x=gps1_msg.pose.pose.position.x
        gps2_msg.linear.y=gps1_msg.pose.pose.position.y
        gps2_msg.angular.z=th

        gps2_pub.publish(gps2_msg)
        gps3_pub.publish(gps2_msg)

        rate.sleep()