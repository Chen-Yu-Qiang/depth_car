#!/usr/bin/wowpython


import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose, Twist, Vector3Stamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf
import random

gt=Pose()
gt.position.z=-100
def cb(data):
    global gt
    gt.position.z=0
    gt=data

if __name__=='__main__':
    rospy.init_node("gazebo_sim_gps", anonymous=True)
    gps1_pub=rospy.Publisher("/outdoor_waypoint_nav/odometry/filtered_map", Odometry,queue_size=1)
    gps2_pub=rospy.Publisher("/lm_ekf/gps/utm", Twist,queue_size=1)
    gps3_pub=rospy.Publisher("/lm_ekf/gps/local", Twist,queue_size=1)
    gps4_pub=rospy.Publisher("/navsat/fix", NavSatFix,queue_size=1)
    imu_pub=rospy.Publisher("/imu_filter/rpy/filtered", Vector3Stamped,queue_size=1)
    rospy.Subscriber("/sim/robot",Pose,cb,queue_size=1)
    rate=rospy.Rate(20)


    while not rospy.is_shutdown():
        if gt.position.z<-10:
            continue
        x_offset=0
        y_offset=1
        now_time=rospy.get_time()
        if now_time<50:
            x_offset=1
            y_offset=0
        elif now_time<60:
            x_offset=1-(now_time-50)*0.1
            y_offset=0
        elif now_time<70:
            x_offset=0
            y_offset=(now_time-60)*0.1
        else:
            x_offset=0
            y_offset=1


        x=gt.position.x
        y=gt.position.y
        th=tf.transformations.euler_from_quaternion([0,0,gt.orientation.z,gt.orientation.w])[2] 
        gps1_msg=Odometry()
        gps1_msg.pose.pose.position.x=x+x_offset
        gps1_msg.pose.pose.position.y=y+y_offset
        gps1_msg.pose.pose.orientation=gt.orientation

        gps1_pub.publish(gps1_msg)
        
        gps2_msg=Twist()
        gps2_msg.linear.x=gps1_msg.pose.pose.position.x
        gps2_msg.linear.y=gps1_msg.pose.pose.position.y
        gps2_msg.angular.z=th

        gps2_pub.publish(gps2_msg)
        gps3_pub.publish(gps2_msg)

        gps4_msg=NavSatFix()
        gps4_msg.latitude=gps1_msg.pose.pose.position.x
        gps4_msg.longitude=gps1_msg.pose.pose.position.y
        # gps4_pub.publish(gps4_msg)

        imu_msg=Vector3Stamped()
        imu_msg.vector.x=0
        imu_msg.vector.y=0
        imu_msg.vector.z=(((th*57.296)+180)%360)-180
        imu_pub.publish(imu_msg)

        rate.sleep()