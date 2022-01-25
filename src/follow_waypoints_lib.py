#!/usr/bin/wowpython
'''ros utils'''
import time
import rospy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPose
from robot_localization.srv import SetDatum
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseStamped, Point,Twist

import utm
from pyproj import Proj
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import os

import more_wp
''' do the following things
'''

def setDatum(navsat_data):
    lat_init, lng_init = navsat_data.latitude, navsat_data.longitude
    _, _, zone, R = utm.from_latlon(lat_init, lng_init)
    proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
    utm_x_init, utm_y_init = proj(lng_init, lat_init)
    rospy.loginfo( "set datum to utm_x:{}, utm_y:{}".format(utm_x_init, utm_y_init) )

    rospy.wait_for_service('/outdoor_waypoint_nav/datum')

    navsat_data = rospy.wait_for_message('/navsat/fix', NavSatFix)

    newMapPose = GeoPose()
    newMapPose.position.latitude = navsat_data.latitude
    newMapPose.position.longitude = navsat_data.longitude
    newMapPose.position.altitude = navsat_data.altitude
    newMapPose.orientation.x = 0
    newMapPose.orientation.y = 0
    newMapPose.orientation.z = 0
    newMapPose.orientation.w = 1

    try: 
        geo_pose = rospy.ServiceProxy('/outdoor_waypoint_nav/datum', SetDatum)
        response = geo_pose(newMapPose)
        rospy.loginfo( "set datum to lat:{}, lng:{}".format(newMapPose.position.latitude, newMapPose.position.longitude) )
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s" % e)

    return utm_x_init, utm_y_init, zone, R

if __name__=="__main__":
    rospy.init_node("waypoint_follower_map2", anonymous=True)
    Use_WOW_controller=int(rospy.get_param('Use_WOW_controller',default=0))

    date_time_folder=rospy.get_param('date_time_folder',default="")

    if Use_WOW_controller==0:
        pubGoal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        pubAchieve = rospy.Publisher("/ctrl/achieve", UInt8, queue_size=1)
    pubGoal_local_2 = rospy.Publisher("/ctrl/wp/local", PoseStamped, queue_size=1)
    pubGoal_utm = rospy.Publisher("/ctrl/wp/utm", PoseStamped, queue_size=1)

    '''get start GPS point and set datum'''
    navsat_data = rospy.wait_for_message('/outdoor_waypoint_nav/gps/filtered', NavSatFix)
    # utm_x_init, utm_y_init, zone, R = setDatum(navsat_data)
    time.sleep(10)
    x0y0_msg = rospy.wait_for_message('/lm_ekf/local_org/utm', Twist)

    utm_y_init=x0y0_msg.linear.x
    utm_x_init=x0y0_msg.linear.y*(-1.0)    
    '''load waypoint.txt -> if exist: do not calculate again'''
    file_name_gps=date_time_folder+"/shapefiles/waypoint/waypoint.txt"
    file_name =file_name_gps[0:-4]+"_utm.txt"
    file_name_more=file_name_gps[0:-4]+"_utm_more.txt"


    if os.path.isfile(file_name_more) is True:
        print('waypoint_utm exist!')
        utm_x_waypoints_list = []
        utm_y_waypoints_list = []
        with open(file_name_more) as f:
            for line in f.readlines():
                s = line.split(' ')
                utm_x_waypoints_list.append(float(s[0]))
                utm_y_waypoints_list.append(float(s[1]))
        utm_x_waypoints = np.asarray(utm_x_waypoints_list)
        utm_y_waypoints = np.asarray(utm_y_waypoints_list)

    else:
        print('waypoint_utm file not exist, generate one!!')
        lat = []
        lng = []
        with open(file_name_gps) as f:
            for line in f.readlines():
                s = line.split(' ')
                lat.append(float(s[0]))
                lng.append(float(s[1])) 

        '''project to UTM and transform to /map frame'''

        _, _, zone, R = utm.from_latlon(lat[0], lng[0])

        proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
        utm_x_waypoints, utm_y_waypoints = proj(lng, lat)
        utm_x_waypoints = np.asarray(utm_x_waypoints)
        utm_y_waypoints = np.asarray(utm_y_waypoints)
        with open(file_name, "w") as f:
            for i in range(len(utm_x_waypoints)):
                f.write("%.8f %.8f\n" % (utm_x_waypoints[i], utm_y_waypoints[i]))
        

        more_wp.div(file_name,file_name_more)

        utm_x_waypoints_list = []
        utm_y_waypoints_list = []
        with open(file_name_more) as f:
            for line in f.readlines():
                s = line.split(' ')
                utm_x_waypoints_list.append(float(s[0]))
                utm_y_waypoints_list.append(float(s[1]))
        utm_x_waypoints = np.asarray(utm_x_waypoints_list)
        utm_y_waypoints = np.asarray(utm_y_waypoints_list)

    mapFrame_set = np.vstack((utm_x_waypoints-utm_x_init, utm_y_waypoints-utm_y_init))
    utmFrame_set = np.vstack((utm_x_waypoints, utm_y_waypoints))


    print("Wait for /lm_ekf/gps_w_offset/utm")
    nouse= rospy.wait_for_message("/lm_ekf/gps_w_offset/utm",Twist)
    time.sleep(5)
    print("successfully initialized!")
    
    '''sequentially publish waypoint while receiving achieve signal'''
    eof = mapFrame_set.shape[1]
    index = 0

    waypoint_data = PoseStamped()
    waypoint_data.header.frame_id = 'map2'
    waypoint_data.pose.position.x = mapFrame_set[1,index]
    waypoint_data.pose.position.y = -mapFrame_set[0,index]
    waypoint_data.pose.position.z = 0
    waypoint_data.pose.orientation.x = 0
    waypoint_data.pose.orientation.y = 0
    waypoint_data.pose.orientation.z = 0
    waypoint_data.pose.orientation.w = 1
    if Use_WOW_controller==0:
        pubGoal.publish(waypoint_data)
    pubGoal_local_2.publish(waypoint_data)

    waypoint_utm = PoseStamped()
    waypoint_utm.pose.position.x = utmFrame_set[1,index]
    waypoint_utm.pose.position.y = -utmFrame_set[0,index]
    waypoint_utm.pose.position.z = 0
    waypoint_utm.pose.orientation.x = 0
    waypoint_utm.pose.orientation.y = 0
    waypoint_utm.pose.orientation.z = 0
    waypoint_utm.pose.orientation.w = 1
    pubGoal_utm.publish(waypoint_utm)

    rospy.loginfo( "pub number_{}/{} waypoint, x:{}, y:{}".format(index,eof,mapFrame_set[1,index],-mapFrame_set[0,index]))

    rospy.sleep(5)
    index += 1

    while (index < eof) and ( not rospy.is_shutdown() ):
        if Use_WOW_controller==0:
            nouse = rospy.wait_for_message('/lateral_error3', Point) 
            Achieve_msg = UInt8()
            Achieve_msg.data=0
            pubAchieve.publish(Achieve_msg)
        else:
            nouse = rospy.wait_for_message("/ctrl/achieve", UInt8) 
        waypoint_data = PoseStamped()
        waypoint_data.header.frame_id = 'map2'
        waypoint_data.pose.position.x = mapFrame_set[1,index]
        waypoint_data.pose.position.y = -mapFrame_set[0,index]
        waypoint_data.pose.position.z = 0
        waypoint_data.pose.orientation.x = 0
        waypoint_data.pose.orientation.y = 0
        waypoint_data.pose.orientation.z = 0
        waypoint_data.pose.orientation.w = 1
        if Use_WOW_controller==0:
            pubGoal.publish(waypoint_data)
        
        pubGoal_local_2.publish(waypoint_data)

        waypoint_utm = PoseStamped()
        waypoint_utm.pose.position.x = utmFrame_set[1,index]
        waypoint_utm.pose.position.y = -utmFrame_set[0,index]
        waypoint_utm.pose.position.z = 0
        waypoint_utm.pose.orientation.x = 0
        waypoint_utm.pose.orientation.y = 0
        waypoint_utm.pose.orientation.z = 0
        waypoint_utm.pose.orientation.w = 1
        pubGoal_utm.publish(waypoint_utm)

        rospy.loginfo( "pub number_{}/{} waypoint x:{}, y:{}".format(index,eof,mapFrame_set[1,index],-mapFrame_set[0,index]))

        # rospy.sleep(5)
        index += 1

    rospy.loginfo('complete waypoints!!!!!!!!')

    