#!/usr/bin/wowpython
import rospy
from rospy.core import loginfo_once
import tf

from geometry_msgs.msg import Twist




if __name__ == '__main__':
    rospy.init_node('tf2topic_gps_in_utm_node')
    listener = tf.TransformListener()
    b=rospy.Publisher("gps_utm",Twist,queue_size=1)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
            
        try:

            listener.waitForTransform("utm", "base_link", rospy.Time(0), rospy.Duration(3.0))
            (trans,rot) = listener.lookupTransform("utm", "base_link", rospy.Time(0) )
            b_msg=Twist()
            b_msg.linear.x=trans[1]
            b_msg.linear.y=trans[0]*(-1.0)
            b.publish(b_msg)
            rate.sleep()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("cannot get TF from {} to {}".format("utm", "base_link"))
            rate.sleep()
            continue
