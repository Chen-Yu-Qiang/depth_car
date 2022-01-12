#!/usr/bin/wowpython
import rospy
import tf
from geometry_msgs.msg import Twist
import time
import numpy as np
x=0
y=0
yaw = 0

def cb_offset(data):
    global x,y,yaw
    x=-data.linear.x
    y=-data.linear.y
    yaw=0

if __name__ == '__main__':
    rospy.init_node('map2_tf_pub_node')
    rospy.Subscriber("/gps_offset",Twist,cb_offset,queue_size=1)
    while not rospy.is_shutdown():


        br = tf.TransformBroadcaster()
        br.sendTransform(( x, y, 0), tf.transformations.quaternion_from_euler(0, 0, yaw), rospy.Time.now(), child="map", parent="map2")
        # print(x,y)
        rospy.sleep(0.02)