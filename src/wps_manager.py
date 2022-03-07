#!/usr/bin/wowpython

import time
import rospy
from geometry_msgs.msg import PoseArray,Pose,PoseStamped
from std_msgs.msg import UInt8



if __name__=='__main__':
        
    rospy.init_node("wps_manager", anonymous=True)

    wp_pub=rospy.Publisher("/ctrl/wp/g",PoseStamped,queue_size=1)

    while not rospy.is_shutdown():
        wps=rospy.wait_for_message("/plan/wps",PoseArray)
        print("Get New Waypoints")
        
        for i in wps.poses:
            wp_msg=PoseStamped()
            wp_msg.pose=i
            wp_pub.publish(wp_msg)
            print("===================")
            print(wp_msg)
            a=rospy.wait_for_message('/ctrl/achieve', UInt8)
            time.sleep(1)