#!/usr/bin/env python

#%%
'''math tool'''
import csv
import numpy as np
from numpy.core.shape_base import block
# from scipy.spatial import distance as dist

'''plot tool'''
import matplotlib.pyplot as plt

'''image tool'''
import cv2
# import statistics as sta

# import utm
# from pyproj import Proj
import sys
if sys.platform.startswith('linux'): # or win
    print("in linux")
    file_path = "/home/yuqiang/catkin_car/src/depth_car/src/syn_rosbag2/"

print(sys.version)
#%%


npDepth_list=[None for i in range(100)]
npColor_list=[None for i in range(100)]
for i in range(10):
    npDepth_list[i]=np.load(file_path+"depth/"+str(i+1)+".npy")
    npColor_list[i]=np.load(file_path+"color/"+str(i+1)+".npy")
    npDepth_list[i] = cv2.convertScaleAbs(npDepth_list[i], alpha=0.02) # 6m
    npDepth_list[i] = cv2.applyColorMap(npDepth_list[i], cv2.COLORMAP_JET)

# fig=plt.figure(figsize=(10,10))
# subplot1 = fig.add_subplot(122)
# subplot2 = fig.add_subplot(121)
# for i in range(10):
#     subplot1.imshow(npColor_list[i])
#     subplot2.imshow(npDepth_list[i])
#     plt.draw()  
#     plt.pause(0.001)
#     subplot1.clear()
#     subplot2.clear()

#%%


AA=350

''' show raw data '''
npDepth = np.load(file_path+"depth/"+str(AA)+".npy")
npColor = np.load(file_path+"color/"+str(AA)+".npy")
# npDepth[npDepth>10000]=np.nan
npDepth=cv2.GaussianBlur(npDepth,(11,11),0)
npDepth = npDepth.astype('float64')
''' to world coordinate '''
cx_d = 320.6562194824219 #424
cy_d = 241.57083129882812 #241
fx_d = 384.31365966796875 #424
fy_d = 384.31365966796875 #424
npPointX = np.asarray(range(640))-cx_d
npPointX = np.diag(npPointX)
npPointX = npDepth.dot(npPointX)/ fx_d * (-1)
npPointX = npPointX.astype('float64')

npPointY = np.asarray(range(480))-cy_d
npPointY = np.diag(npPointY)
theta = 0/180*np.pi
npPointY = npPointY.dot(npDepth)/ fy_d * (-1) 
npPointY = npPointY*np.cos(theta) + npDepth * np.sin(theta) + 410
npPointY = npPointY.astype('float64')
''' depth segmentation: show layers '''
npHeight = np.copy(npPointY)
npHeight[npHeight<=300]=300
npHeight[npHeight>=2000]=2000

# plt.imshow(npDepth)
plt.contourf(range(640),479-np.asarray(range(480)),npDepth,100)
plt.colorbar()
plt.show(block=False)
plt.figure()
plt.imshow(npColor)
plt.show(block=False)
plt.figure()
plt.contourf(range(640),479-np.asarray(range(480)),npHeight,100)
plt.colorbar()
plt.show(block=False)


down_component=np.zeros((480,640))
kernel=np.array([[-0.2,-0.2,-0.2,-0.2,-0.2],[0,0,0,0,0],[0.2,0.2,0.2,0.2,0.2]])
npPointX_d=cv2.filter2D(npPointX,-1,kernel=kernel)
npHeight_d=cv2.filter2D(npHeight,-1,kernel=kernel)
npDepth_d=cv2.filter2D(npDepth,-1,kernel=kernel)
v1_norm=np.sqrt(npPointX_d*npPointX_d+npHeight_d*npHeight_d+npDepth_d*npDepth_d)
down_component=npHeight_d/v1_norm

plt.figure()
plt.contourf(range(640),479-np.asarray(range(480)),down_component,100)
plt.colorbar()
plt.show(block=False)

tree_mask=down_component<-0.6

plt.figure()
plt.imshow(tree_mask)
plt.show(block=False)

tree_mask=tree_mask.astype('float64')

kernel = np.ones((12,12), np.uint8)
tree_mask = cv2.erode(tree_mask, kernel, iterations = 1)
kernel = np.ones((20,20), np.uint8)
tree_mask = cv2.dilate(tree_mask,kernel,iterations = 1)

plt.figure()
plt.imshow(tree_mask)
plt.show(block=False)

#%%

''' top-down view grid '''
def depth_Z(u,v):
    return npDepth[v][u]
height_layer_tmp = np.logical_and(npHeight<2500,npHeight>1000)
height_layer = np.logical_and(height_layer_tmp,npHeight!=410)

plane_l1 = np.zeros((int(10/0.05),int(10/0.05)),dtype=np.uint8)

row, column = npDepth.shape

for v in range(row):
    if height_layer[v].any() == True:
        for u in range(column):
            if height_layer[v][u] == True:
                z_depth = depth_Z(u,v)
                if (z_depth>50 and z_depth<6000):
                    x_depth = (u-cx_d)/fx_d*depth_Z(u,v)
                    plane_l1[200-int(z_depth/50)][int(x_depth/50)+100] += 1
                    
hieght_or = np.zeros((int(10/0.05),int(10/0.05)), dtype=np.uint8)
hieght_or[plane_l1>8]=255
hieght_or = hieght_or.astype('uint8')
kernel = np.ones((2,2), np.uint8)
hieght_or = cv2.dilate(hieght_or,kernel,iterations = 1)
hieght_or = cv2.erode(hieght_or, kernel, iterations = 1)

''' find connected component and push into point array A '''
num_objects, labels = cv2.connectedComponents(hieght_or)
centre_x_list = []
centre_y_list = []
radius_r_list = []
circle_bd = np.zeros(hieght_or.shape, dtype=np.uint8)
# print('>>>>num_objects:',num_objects)
for i in range(num_objects-1):
    A = []
    for x in range(200):
        for y in range(200):
            if labels[x][y] == i+1:
                A.append(np.array([-x/(x*x+y*y), -y/(x*x+y*y), -1/(x*x+y*y)]))
    A = np.asarray(A)
    # print('# of points: ',A.shape)
    if A.shape[0] < 10:
        continue

    k = np.linalg.inv(A.T @ A)
    k = k @ A.T
    k = k @ np.ones((k.shape[1],1))
    centre_x = k[0][0]/(-2)
    centre_y = k[1][0]/(-2)
    radius_r = np.sqrt(centre_x*centre_x+centre_y*centre_y-k[2][0])
    # print('x,y,r: ', int(centre_x+0.5), int(centre_y+0.5), int(radius_r+0.5))
    
    cv2.circle(circle_bd,(int(centre_y+0.5), int(centre_x+0.5)), int(radius_r+0.5), 150, 2)
    
    centre_x_list.append(int(centre_x+0.5))
    centre_y_list.append(int(centre_y+0.5))
    radius_r_list.append(int(radius_r+0.5))

    cv2.putText(circle_bd, #numpy array on which text is written
                str(int(radius_r+0.5)), #text
                (int(centre_y)-20,int(centre_x)-20), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.6, #font size
                255, #font color
                1, cv2.LINE_AA) #font stroke
circle_bd[hieght_or==255]=255
centre_x_list = np.asarray(centre_x_list)
centre_y_list = np.asarray(centre_y_list)
radius_r_list = np.asarray(radius_r_list)

''' load robot current pose '''
file_path = "/home/yuqiang/catkin_car/src/depth_car/src/syn_rosbag/"

with open(file_path + 'cb_pose.csv', 'r') as csvfile:
    robot_pose_gps = list( csv.reader(csvfile, delimiter=',') )
    robot_pose_gps = np.array(robot_pose_gps).astype(float)

# lat = robot_pose_gps[AA,0]
# lng = robot_pose_gps[AA,1]
utm_y_loc_origin = robot_pose_gps[AA,0]
utm_x_loc_origin = robot_pose_gps[AA,1]
imu_yaw = robot_pose_gps[AA,2]

# _, _, zone, R = utm.from_latlon(lat, lng)
# proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
# utm_x_loc_origin, utm_y_loc_origin = proj(lng, lat)

cX_m_loc = (centre_y_list-100)*0.05
cY_m_loc = (200-centre_x_list)*0.05
cX_utm_loc = cX_m_loc*np.cos(imu_yaw)-cY_m_loc*np.sin(imu_yaw) + utm_x_loc_origin
cY_utm_loc = cX_m_loc*np.sin(imu_yaw)+cY_m_loc*np.cos(imu_yaw) + utm_y_loc_origin
center_utm_loc = np.vstack((cX_utm_loc,cY_utm_loc))

# file_path = "/home/ncslaber/110-1/210922_EKF-fusion-test/zigzag_bag/"
# np.save(file_path+"found_center/"+str(AA)+'-x',cX_utm_loc )
# np.save(file_path+"found_center/"+str(AA)+'-y',cY_utm_loc )

# np.save(file_path+"found_center/"+str(AA)+'-q_x',cX_m_loc )
# np.save(file_path+"found_center/"+str(AA)+'-q_y',cY_m_loc )
# np.save(file_path+"found_center/"+str(AA)+'-r',radius_r_list )


# fig3 = plt.figure(figsize=(8,8))
# plt.title('current found trunk')
cv2.imwrite(str(AA)+'.png',circle_bd)

# plt.show()
# %%
