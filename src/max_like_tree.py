#!/usr/bin/wowpython

import numpy as np


tree_data=[[1,2,0.3],[2,4,0.2],[3,3,0.1]]


def h(car_x,car_y,car_th,a_tree_data):
    delta_x=a_tree_data[0]-car_x
    delta_y=a_tree_data[1]-car_y
    d=np.sqrt(delta_x**2+delta_y**2)
    th=np.arctan2(delta_y,delta_x)-car_th
    th=(th+np.pi)%(2.0*np.pi)-np.pi
    r=a_tree_data[2]

    return [d,th,r]

def max_like(car_x,car_y,car_th,tree_data,o_d,o_th,o_r):
    like_max=0  # init value as threshold
    like_max_i=-1
    o=[o_d,o_th,o_r]
    for i in range(len(tree_data)):
        hp=h(car_x,car_y,car_th,tree_data[i])
        w=[0.05,1.0,0.01]
        dir=0
        for j in range(3):
            dir=dir+w[j]*(hp[j]-o[j])*(hp[j]-o[j])
        like=np.exp(-dir)
        # print(i,like,hp)
        if like>like_max:
            like_max=like
            like_max_i=i
    return like_max_i,like_max


if __name__=="__main__":
    print(max_like(0,0,np.pi*0.5,tree_data,2.2,-0.4,0.3))