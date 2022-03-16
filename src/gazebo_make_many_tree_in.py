#!/usr/bin/wowpython

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose
import TREEDATA
import time

def get_sdf_green(r):

    sdf_all="""
<?xml version='1.0'?>
<sdf version="1.4">
    <model name="wowTree">
        <static>true</static>    
{}
    </model>
</sdf>"""
    sdf_link="""
        <link name="{link_name}">
            <pose frame=''>{x} {y} {z} 0 0 0</pose>
            <inertial>
                <mass>1.0</mass>
                <inertia> <!-- inertias are tricky to compute -->
                <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
                <ixx>0.083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
                <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
                <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
                <iyy>0.083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
                <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
                <izz>0.083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
                </inertia>
            </inertial>
            <collision name="collision">
                <geometry>
                    <cylinder>
                    <radius>{radius}</radius>
                    <length>{length}</length>
                    </cylinder>
                </geometry>
            </collision>
            <visual name="visual">
                <geometry>
                    <cylinder>
                    <radius>{radius}</radius>
                    <length>{length}</length>
                    </cylinder>
                </geometry>
                        <material>

            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
            <specular>0 0 0 1</specular>
            <emissive>0 0 0 1</emissive>
            </material>
            </visual>
        </link>"""


    ans=sdf_all.format(sdf_link.format(radius=str(r),link_name="trunk",length="2",x="0",y="0",z="1",color="0.561 0.349 0.008 1")+\
        sdf_link.format(radius=str(r*2),link_name="leaf",length="0.5",x="0",y="0",z="2.25",color="0.306 0.604 0.024 1")+\
        sdf_link.format(radius=str(r*1.5),link_name="leaf",length="0.5",x="0",y="0",z="2.75",color="0.306 0.604 0.024 1")+\
        sdf_link.format(radius=str(r),link_name="leaf",length="0.5",x="0",y="0",z="3.25",color="0.306 0.604 0.024 1"))


    return ans
j=0
time.sleep(5)
for i in TREEDATA.TREE_DATA:

    pose = Pose()
    pose.position.x=i[0]
    pose.position.y=i[1]
    pose.orientation.w = 1.0

    model_name = "tree_" + str(j)

    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    spawn_model.wait_for_service()
    req = SpawnModelRequest()
    req.model_name = model_name
    req.model_xml = get_sdf_green(i[2])
    req.robot_namespace = "/foo"
    req.initial_pose = pose
    resp = spawn_model(req)
    j=j+1


