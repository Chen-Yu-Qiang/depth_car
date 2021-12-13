#!/usr/bin/wowpython
#%%

import sys

print("Python version: ",sys.version)
import rospy
from sensor_msgs.msg import Image

num=0
def cbDepth_in(data):
    global tree_depth_in_pub,num
    if num==5:
        tree_depth_in_pub.publish(data)
        num=0
    else:
        num=num+1



if __name__=="__main__":
    print("Python version: ",sys.version)
    rospy.init_node("depthHandler", anonymous=True)
    subDepth1 = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth_in)
    tree_depth_in_pub=rospy.Publisher("/image_rect_raw_in", Image,queue_size=1)
    rospy.spin()
