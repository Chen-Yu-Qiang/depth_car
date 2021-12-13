#!/usr/bin/wowpython

import rospy
import tf
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from pyproj import Proj,Transformer
import time
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
msgFrom = "/navsat/fix"
Tf_to = "base_link_gps"
TF_from = "map"


transformer = Transformer.from_crs("epsg:4326", "epsg:32651")
ang=0
x0=0
y0=0

def cb_ang(data):
    global ang
    ang=data.vector



def cb_gps(data):
    global transformer,ang,x0,y0
    t=time.time()
    br = tf.TransformBroadcaster()
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
    br.sendTransform(( yy0,xx0, 0),tf.transformations.quaternion_from_euler(ang.x/57.3,ang.y/57.3,ang.z/57.3),data.header.stamp,Tf_to,TF_from)

rospy.init_node('wow_tf')

rospy.Subscriber(msgFrom,NavSatFix,cb_gps)
rospy.Subscriber("/imu_filter/rpy/filtered",Vector3Stamped,cb_ang)
gps_ang_pub=rospy.Publisher("/gps_ang",Imu,queue_size=1)

rospy.spin()