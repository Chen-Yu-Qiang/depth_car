
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
rosbag play 2021-12-17-18-02-26.bag --topic /camera/color/camera_info /camera/color/image_raw/compressed /camera/depth/camera_info /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /clock /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /husky_velocity_controller/odom /imu/data /imu/mag /imu_filter/rpy/filtered /move_base_simple/goal /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /outdoor_waypoint_nav/odometry/gps /rosout /rosout_agg /scan /scan_filtered /tf /tf_static /wow/achieveGoal /wow/trunk_info /wow_utm_waypoint  --clock

```
or 
```
export Built_In_Topic="/camera/color/camera_info /camera/color/image_raw/compressed /camera/depth/camera_info /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /clock /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /husky_velocity_controller/odom /imu/data /imu/mag /imu_filter/rpy/filtered /move_base_simple/goal /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /outdoor_waypoint_nav/odometry/gps /rosout /rosout_agg /scan /scan_filtered /tf /tf_static /wow/achieveGoal /wow/trunk_info /wow_utm_waypoint"



rosbag play 2021-12-17-18-02-26.bag --topic $Built_In_Topic  --clock

```