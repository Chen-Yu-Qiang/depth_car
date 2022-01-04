#!/usr/bin/wowpython
'''ros utils'''
import rospy
from sensor_msgs.msg import NavSatFix

from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseStamped, Point

from geometry_msgs.msg import Twist

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import os


if __name__ == '__main__':
    rospy.init_node("waypoint_follower", anonymous=True)
    pubGoal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    pubGoal_utm = rospy.Publisher("/wow_utm_waypoint", PoseStamped, queue_size=1)
    '''get start GPS point and set datum'''
    x0y0_msg = rospy.wait_for_message('/local_org_in_utm', Twist)

    utm_y_init=x0y0_msg.linear.x
    utm_x_init=x0y0_msg.linear.y*(-1.0)

    rospy.loginfo( "Get x0:{}, y0:{}".format(utm_x_init,utm_y_init))
    utm_x_waypoints=np.array([352850.6,352850.6,352853.6,352853.6])
    utm_y_waypoints=np.array([2767676,2767679,2767679,2767676])

    offset_x = 0
    offset_y = 0

    mapFrame_set = np.vstack((utm_x_waypoints-utm_x_init+offset_x, utm_y_waypoints-utm_y_init+offset_y))
    utmFrame_set = np.vstack((utm_x_waypoints, utm_y_waypoints))
    print("successfully initialized!")
    
    '''sequentially publish waypoint while receiving achieve signal'''
    eof = mapFrame_set.shape[1]
    index = 0

    waypoint_data = PoseStamped()
    waypoint_data.header.frame_id = 'map'
    waypoint_data.pose.position.x = mapFrame_set[1,index]
    waypoint_data.pose.position.y = -mapFrame_set[0,index]
    waypoint_data.pose.position.z = 0
    waypoint_data.pose.orientation.x = 0
    waypoint_data.pose.orientation.y = 0
    waypoint_data.pose.orientation.z = 0
    waypoint_data.pose.orientation.w = 1
    pubGoal.publish(waypoint_data)

    waypoint_utm = PoseStamped()
    waypoint_utm.pose.position.x = utmFrame_set[0,index]
    waypoint_utm.pose.position.y = utmFrame_set[1,index]
    waypoint_utm.pose.position.z = 0
    waypoint_utm.pose.orientation.x = 0
    waypoint_utm.pose.orientation.y = 0
    waypoint_utm.pose.orientation.z = 0
    waypoint_utm.pose.orientation.w = 1
    pubGoal_utm.publish(waypoint_utm)

    rospy.loginfo( "pub number_{}/{} waypoint x:{}, y:{}".format(index,eof-1,mapFrame_set[1,index],-mapFrame_set[0,index]))

    rospy.sleep(5)
    index += 1

    while (index < eof) and ( not rospy.is_shutdown() ):
        nouse = rospy.wait_for_message('/wow/achieveGoal', UInt8) 

        waypoint_data = PoseStamped()
        waypoint_data.header.frame_id = 'map'
        waypoint_data.pose.position.x = mapFrame_set[1,index]
        waypoint_data.pose.position.y = -mapFrame_set[0,index]
        waypoint_data.pose.position.z = 0
        waypoint_data.pose.orientation.x = 0
        waypoint_data.pose.orientation.y = 0
        waypoint_data.pose.orientation.z = 0
        waypoint_data.pose.orientation.w = 1
        pubGoal.publish(waypoint_data)

        waypoint_utm = PoseStamped()
        waypoint_utm.pose.position.x = utmFrame_set[0,index]
        waypoint_utm.pose.position.y = utmFrame_set[1,index]
        waypoint_utm.pose.position.z = 0
        waypoint_utm.pose.orientation.x = 0
        waypoint_utm.pose.orientation.y = 0
        waypoint_utm.pose.orientation.z = 0
        waypoint_utm.pose.orientation.w = 1
        pubGoal_utm.publish(waypoint_utm)

        rospy.loginfo( "pub number_{}/{} waypoint x:{}, y:{}".format(index,eof-1,mapFrame_set[1,index],-mapFrame_set[0,index]))

        # rospy.sleep(5)
        index += 1

    rospy.loginfo('complete waypoints!!!!!!!!')

    