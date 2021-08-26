
# for python2

```
sudo ln -s /usr/bin/python /usr/bin/wowpython
```


# for python3

```
sudo ln -s /usr/bin/python3 /usr/bin/wowpython
```


```python
#!/usr/bin/wowpython
```

# 只播放機器本身發布的話題
```
rosbag play 2021-12-17-18-02-26.bag --topic /camera/color/camera_info /camera/color/image_raw/compressed /camera/depth/camera_info /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /clock /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /husky_velocity_controller/odom /imu/data /imu/mag /imu_filter/rpy/filtered /ctrl/wp/loacl /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /outdoor_waypoint_nav/odometry/gps /rosout /rosout_agg /scan /scan_filtered /tf /tf_static /ctrl/achieve /tree/trunk_info /ctrl/wp/utm  --clock

```
or 
```
export Built_In_Topic="/camera/color/camera_info /camera/color/image_raw/compressed /camera/depth/camera_info /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /clock /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /husky_velocity_controller/odom /imu/data /imu/mag /imu_filter/rpy/filtered /ctrl/wp/loacl /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /outdoor_waypoint_nav/odometry/gps /rosout /rosout_agg /scan /scan_filtered /tf /tf_static /ctrl/achieve /tree/trunk_info /ctrl/wp/utm"



rosbag play 2021-12-20-09-00-54.bag --topic $Built_In_Topic  --clock
rosbag record -a -x "/camera(.*)"
```


## 關於numpy inv 造成大量CpU佔用問題
可以加入此行
```bash
 export OPENBLAS_NUM_THREADS=1
```

## Gazebo

- 開啟 
```bash
roslaunch depth_car gazebo_tree.launch
```
- 紀錄檔
bag檔會自動存在~/.ros
如果空間不夠裡面的.bag都可以刪


- 生成地圖
執行直接用python TREEDATA.py 到68行之前
在61行開始設定樹的xy與半徑
之後會產生一個output.npy把它放到
/home/yuqiang/GA_MAP/shapefiles/neg

裡面存成檔名`center_all.npy`

- 控制器
若要使用控制器可以使用launch檔裡面的34 35行
發送目標點到話題`"/ctrl/path/g`
或是一次發送一堆目標點（PoseArray）到`/plan/wps`


若不使用則直接發送速度命令到`cmd_vel`

- 機器人狀態
在話題`/sim/robot`

