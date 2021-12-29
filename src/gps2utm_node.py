#!/usr/bin/wowpython
from numpy.core.numeric import NaN
from numpy.lib.type_check import nan_to_num
import rospy
from rospy.core import loginfo_once

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import time
from geometry_msgs.msg import Twist
from EKF_localization import gps_2_utm_Z
import tf

x0=0
y0=0
def cb_gps(data):
    global x0,y0,x0_loc,y0_loc
    # t0=time.time()
    b_msg=Twist()
    z=gps_2_utm_Z(data)
    b_msg.linear.x=z[0][0]
    b_msg.linear.y=z[1][0]
    b.publish(b_msg)
    # print(time.time()-t0)

    if x0==0 and y0==0 and not(x0_loc==-1000 or y0_loc==-1000):
        x0=z[0][0]-x0_loc
        y0=z[1][0]-y0_loc

x0_loc=-1000
y0_loc=-1000
x_loc_now=0
y_loc_now=0
th=0
def cb_pos(data):
    global x0_loc,y0_loc,x0,y0,x_loc_now,y_loc_now,th
    x_loc_now=data.pose.pose.position.x
    y_loc_now=data.pose.pose.position.y
    th=tf.transformations.euler_from_quaternion([0,0,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]
    if x0_loc==-1000 and y0_loc==-1000:
        x0_loc=x_loc_now
        y0_loc=y_loc_now
    elif not(x0==0 and y0==0):
        filtered_utm_msg=Twist()
        filtered_utm_msg.linear.x=x_loc_now+x0
        filtered_utm_msg.linear.y=y_loc_now+y0
        filtered_utm_msg.angular.z=th
        filtered_utm.publish(filtered_utm_msg)

        local_org_in_utm_msg=Twist()
        local_org_in_utm_msg.linear.x=x0
        local_org_in_utm_msg.linear.y=y0
        local_org_in_utm_pub.publish(local_org_in_utm_msg)

def cb_offset(data):
    global x0,y0,x_loc_now,y_loc_now,th
    lm_off_loc_pub_msg=Twist()
    lm_off_loc_pub_msg.linear.x=data.linear.x+x_loc_now
    lm_off_loc_pub_msg.linear.y=data.linear.y+y_loc_now
    lm_off_loc_pub_msg.angular.z=th
    lm_off_loc_pub.publish(lm_off_loc_pub_msg)

    lm_off_pub_msg=Twist()
    lm_off_pub_msg.linear.x=data.linear.x+x_loc_now+x0
    lm_off_pub_msg.linear.y=data.linear.y+y_loc_now+y0
    lm_off_pub_msg.angular.z=th
    lm_off_pub.publish(lm_off_pub_msg)

    
if __name__ == '__main__':
    rospy.init_node('tf2topic_gps_in_utm_node')
    b=rospy.Publisher("gps_utm",Twist,queue_size=1)
    filtered_utm=rospy.Publisher("filtered_utm",Twist,queue_size=1)
    local_org_in_utm_pub=rospy.Publisher("local_org_in_utm",Twist,queue_size=1)
    rospy.Subscriber("/outdoor_waypoint_nav/gps/filtered", NavSatFix, cb_gps, buff_size=2**20,queue_size=1)
    rospy.Subscriber("/outdoor_waypoint_nav/odometry/filtered_map", Odometry, cb_pos, buff_size=2**20,queue_size=1)
    rospy.Subscriber("gps_offset", Twist, cb_offset, buff_size=2**20,queue_size=1)
    lm_off_loc_pub=rospy.Publisher("landmark_filtered_offset_local",Twist,queue_size=1)
    lm_off_pub=rospy.Publisher("landmark_filtered_offset",Twist,queue_size=1)

    rospy.spin()
