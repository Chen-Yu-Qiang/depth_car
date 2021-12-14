#!/usr/bin/wowpython
import rospy
from rospy.core import loginfo_once
import tf

from geometry_msgs.msg import Twist




if __name__ == '__main__':
    rospy.init_node('utm2local_org_node')
    listener = tf.TransformListener()
    a=rospy.Publisher("utm2local_org",Twist,queue_size=1)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
            
        try:
            listener.waitForTransform("utm", "map", rospy.Time(0), rospy.Duration(3.0))
            (trans,rot) = listener.lookupTransform("utm", "map", rospy.Time(0) )
            a_msg=Twist()
            a_msg.linear.x=trans[1]
            a_msg.linear.y=trans[0]*(-1.0)
            a.publish(a_msg)

            rate.sleep()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("cannot get TF from {} to {}".format("utm", "map"))
            rate.sleep()
            continue
