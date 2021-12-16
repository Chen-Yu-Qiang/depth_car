#!/usr/bin/wowpython
import rospy
from rospy.core import loginfo_once

from sensor_msgs.msg import NavSatFix
import time
from geometry_msgs.msg import Twist
from EKF_localization import gps_2_utm_Z
def cb_gps(data):
    t0=time.time()
    b_msg=Twist()
    z=gps_2_utm_Z(data)
    b_msg.linear.x=z[0][0]
    b_msg.linear.y=z[1][0]
    b.publish(b_msg)
    # print(time.time()-t0)

if __name__ == '__main__':
    rospy.init_node('tf2topic_gps_in_utm_node')
    b=rospy.Publisher("gps_utm",Twist,queue_size=1)
    rospy.Subscriber("/outdoor_waypoint_nav/gps/filtered", NavSatFix, cb_gps, buff_size=2**20,queue_size=1)
    rospy.spin()
