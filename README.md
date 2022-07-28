# Raspberry PI

```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
source ~/.bashrc
```

## tf2
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 run tf2_ros static_transform_publisher -0.05 0 0.3 0 0 0 base_link laser
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link imu_link
```

## two dc motor control
```
colcon build --packages-select tank_control && . install/setup.sh && ros2 run tank_control robot_control
```

## imu
```
cd ros2_ws
ros2 launch mpu9250 mpu9250.launch.py
```

## odometry
```
ros2 run robot_localization ekf_node --ros-args --params-file $HOME/ekf_params.yaml
```

## lidar
```
cd ros2_ws
ros2 launch sllidar_ros2 sllidar_launch.py 
```

----

# PC

```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
source ~/.bashrc
```

## teleop
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## cartographer
```
ros2 launch tank_cartographer cartographer.launch.py
ros2 launch tank_cartographer cartographer_rviz.launch.py
```

## nav2 map saver

```
cd ~/
ros2 run nav2_map_server map_saver_cli -f map
```

## navigation
```
ros2 launch tank_navigation2 navigation2.launch.py map:=$HOME/map.yaml
ros2 launch tank_navigation2 navigation2_rviz.launch.py
```
