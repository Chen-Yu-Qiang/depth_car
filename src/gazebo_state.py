#!/usr/bin/wowpython


import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
if __name__=='__main__':
    rospy.init_node("gazebo_state_getter", anonymous=True)
    fs=int(rospy.get_param("~Sampling_frequency",default="50"))
    object_name=rospy.get_param("~Object_name",default="tree_1")
    output_name=rospy.get_param("~Output_name",default="tree_1")
    rospy.wait_for_service('/gazebo/get_model_state')
    o_pub=rospy.Publisher(output_name,Pose,queue_size=1)


    rate=rospy.Rate(fs)


    while not rospy.is_shutdown():
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp= model_coordinates(object_name,"world")
        if resp.success:
            o_pub.publish(resp.pose)

        rate.sleep()