#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from pyproj import Proj,Transformer
import time
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
msgFrom = "/outdoor_waypoint_nav/gps/filtered"
msgFrom = "/navsat/fix"


transformer = Transformer.from_crs("epsg:4326", "epsg:32651")
ang=0
x0=0
y0=0
x_last=0
y_last=0
def cb_ang(data):
    global ang
    ang=data.vector

imu_ang=0
def cb_imu(data):
    global imu_ang
    _,_,imu_ang=tf.transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])

cmd_pn=0
def cb_cmd(data):
    global cmd_pn
    if data.linear.x>0:
        cmd_pn=1
    elif data.linear.x<0:
        cmd_pn=-1
    else:
        cmd_pn=0


def cb_gps(data):
    global transformer,ang,x0,y0,x_last,y_last,imu_ang,cmd_pn
    lat=data.latitude
    lon=data.longitude
    x, y = transformer.transform(lat, lon)
    x=x*(-1.0)
    if x0==0 and y0==0:
        x0=x
        y0=y
    # print(x-x0, y-y0,time.time()-t)
    xx0=x-x0
    yy0=y-y0
    gps_len=(x_last-xx0)*(x_last-xx0)+(y_last-yy0)*(y_last-yy0)
    gps_ang=np.arctan2((xx0-x_last),(yy0-y_last))
    # print(gps_len,gps_ang*57.3,ang.z)
    x_last=xx0
    y_last=yy0
    if gps_len>0.05:
        gps_ang_msg=Imu()
        if cmd_pn<0:
            gps_ang=gps_ang+np.pi
        ox,oy,oz,ow=tf.transformations.quaternion_from_euler(0,0,gps_ang)
        gps_ang_msg.orientation.x=ox
        gps_ang_msg.orientation.y=oy
        gps_ang_msg.orientation.z=oz
        gps_ang_msg.orientation.w=ow

        gps_ang_pub.publish(gps_ang_msg)

rospy.init_node('gps2ang')

rospy.Subscriber(msgFrom,NavSatFix,cb_gps)
rospy.Subscriber("/imu_filter/rpy/filtered",Vector3Stamped,cb_ang)
rospy.Subscriber("/husky_velocity_controller/cmd_vel",Twist,cb_cmd)
gps_ang_pub=rospy.Publisher("/gps_ang",Imu,queue_size=1)
rospy.Subscriber("/imu/data",Imu,cb_imu)
rospy.spin()