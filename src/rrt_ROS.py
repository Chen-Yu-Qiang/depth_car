#!/usr/bin/wowpython


import time

import numpy as np
import rospy
import rrtstar
from geometry_msgs.msg import PoseArray,Pose
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt


def rrt_path(start,goal):

    obstacles,bounds=rrtstar.treedata2BoundsAndObstacles()

    treedata=[]
    for i in obstacles:
        treedata.append([i[0],i[1],0,0])

    print(treedata)
    print(obstacles)
    rrt_star = rrtstar.RRTStar(start=start,
            goal=goal,
            bounds=bounds,
            obstacle_list=obstacles,
            treedata=treedata,
            max_iter=500)
    plt.figure(figsize=(6,6))
    rrt_star.plot_scene()
    plt.show()
    path_rrt_star, min_cost = rrt_star.plan()
    print("Total node "+str(len(rrt_star.node_list)))
    print("Path node "+str(len(path_rrt_star)))
    poses=rrtstar.path2poseArray(path_rrt_star)
    plt.figure(figsize=(6,6))
    rrt_star.plot_scene()
    rrt_star.draw_graph()
    if path_rrt_star is None:
        print("No viable path found")
    else:
        plt.plot([x for (x, y) in path_rrt_star], [y for (x, y) in path_rrt_star], '-r')
    rrt_star.get_cost(str(time.time()))
    # plt.show()
    plt.savefig(str(time.time())+".png",dpi=600)
    return poses


if __name__=='__main__':
        
    rospy.init_node("rrt_planner", anonymous=True)

    path_pub=rospy.Publisher("/plan/wps",PoseArray,queue_size=1)
    time.sleep(30)

    # while not rospy.is_shutdown():
        # print("Wait for goal msg")
        # g=rospy.wait_for_message("/plan/goal", Pose)
        # print("Get goal msg")
        # print("Wait for state msg")
        # p=rospy.wait_for_message("/sim/robot", Pose)
        # poses=rrt_path([p.position.x,p.position.y],[g.position.x,g.position.y])
    if 1:
        x_list=[10,10,9,9,8,8,7,7,6,6,5,5]
        y_list=[0,-3,-3,-4,-4,0,0,-6,-6,0,0,-6]
        poses=PoseArray()
        for i in range(12):

            a=Pose()
            a.position.x=x_list[i]
            a.position.y=y_list[i]
            poses.poses.append(a)
        


        path_pub.publish(poses)
        print(poses)

