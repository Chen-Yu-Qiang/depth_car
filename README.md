
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


'2022-01-12-12-44-58.bag map1 no div in con
2022-01-12-12-58-12.bag map1 no div in con
2022-01-12-13-15-22.bag map1

2022-01-12-14-22-51.bag my con  no div
