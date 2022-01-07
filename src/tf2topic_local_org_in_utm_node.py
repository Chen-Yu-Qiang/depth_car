#!/usr/bin/wowpython
import rospy
from rospy.core import loginfo_once
import tf

from geometry_msgs.msg import Twist




if __name__ == '__main__':
    rospy.init_node('tf2topic_local_org_in_utm_node')
    listener = tf.TransformListener()
    a=rospy.Publisher("/lm_ekf/local_org/utm",Twist,queue_size=1)
    rate=rospy.Rate(10)

    listener.waitForTransform("utm", "map", rospy.Time(), rospy.Duration(10.0))
    while not rospy.is_shutdown():
            
        try:
            now=rospy.Time.now()
            listener.waitForTransform("utm", "map", now, rospy.Duration(3.0))
            (trans,rot) = listener.lookupTransform("utm", "map", now )
            a_msg=Twist()
            a_msg.linear.x=trans[1]
            a_msg.linear.y=trans[0]*(-1.0)
            a.publish(a_msg)

            rate.sleep()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("cannot get TF from {} to {}".format("utm", "map"))
            rate.sleep()
            continue
